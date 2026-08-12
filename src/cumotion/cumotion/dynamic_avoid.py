#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cumotion/dynamic_avoid.py — 재계획 루프를 돌리는 ROS 2 노드 (실행 진입점)

⚠️ **이 노드는 로봇을 실제로 움직인다.** bench_planning_time.py 와 달리 plan_only 가 아니다.
   첫 실행은 반드시 vel_scale 0.15 로, 사람이 비상정지 버튼에 손을 올린 채로 한다.

전제 (testcommand.md 의 T1~T7 이 전부 떠 있어야 한다)
    T1 camera → T2 bringup → T4 segmenter → T5 nvblox → T6 cumotion_planner → T7 move_group
    하나라도 빠지면 조용히 "장애물 없는 세상"에서 계획한다. 특히 T4·T5 가 빠지면
    계획은 성공하는데 장애물을 통과한다 — 실패가 아니라 **성공처럼 보이는 실패**다.

실행 (컨테이너 T8)
    ros2 launch cumotion dynamic_avoid.launch.py mode:=check
    ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong vel:=0.2
    ros2 run cumotion dynamic_avoid --ros-args -p mode:=joint \
        -p goal_joint_deg:="[0.0, 0.0, 90.0, 0.0, 90.0, 0.0]" -p vel_scale:=0.15

파라미터는 config/dynamic_avoid.yaml 에 전부 주석과 함께 있다.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy

from cumotion.arm import CumotionArm

D2R = math.pi / 180.0

# 왕복 시연용 기본 두 자세 (deg).
# 작업영역 x 0.00~0.70 / y -0.30~0.30 (config/README.md) 을 가로지르도록 잡았다.
# ⚠️ 안전 검증된 값이 아니다. 실기에 걸기 전에 mode:=joint 로 각각 한 번씩 따로 가볼 것.
DEFAULT_A_DEG = [45.0, 0.0, 90.0, 0.0, 90.0, 0.0]
DEFAULT_B_DEG = [-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]


class DynamicAvoid(CumotionArm):
    """CumotionArm 에 "무엇을 향해 갈지"를 ROS 파라미터로 얹은 실행 노드."""

    def __init__(self):
        # cfg=None → ArmConfig 를 ROS 파라미터로 선언·수신한다 (arm.py 참고)
        super().__init__(cfg=None, node_name='dynamic_avoid')

        p = self.declare_parameter
        self.mode = p('mode', 'check').value                  # check | joint | pose | pingpong
        self.goal_joint_deg = list(p('goal_joint_deg', [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]).value)
        self.goal_pose = list(p('goal_pose', [0.45, 0.0, 0.35, 180.0, 0.0, 0.0]).value)
        self.a_deg = list(p('pingpong_a_deg', DEFAULT_A_DEG).value)
        self.b_deg = list(p('pingpong_b_deg', DEFAULT_B_DEG).value)
        self.cycles = int(p('cycles', 0).value)               # 0 = 무한

        # ── 재계획 루프 튜닝 (README 3절: lookahead > 계획시간 + handover) ──
        self.replan_hz = float(p('replan_hz', 3.0).value)
        self.lookahead_s = float(p('lookahead_s', 0.35).value)
        self.handover_s = float(p('handover_s', 0.05).value)
        self.max_start_jump = float(p('max_start_jump', 0.25).value)
        self.max_consecutive_failures = int(p('max_consecutive_failures', 4).value)
        self.timeout_s = float(p('timeout_s', 60.0).value)
        self.static_mode = bool(p('static_mode', False).value)

        # 🔴 nvblox ESDF 서비스가 없으면 움직이기를 거부한다.
        #    없어도 계획은 성공하므로, 이 gate 가 없으면 장애물을 통과한 뒤에야 알게 된다.
        #    pipeline_id:=ompl 로 쓸 땐 nvblox 가 필요 없으니 false 로 내린다.
        self.require_obstacle_pipeline = bool(p('require_obstacle_pipeline', True).value)

    # ── 사전 점검: 로봇을 안 움직인다 ────────────────────────────────────────
    def do_check(self) -> bool:
        """움직이기 전에 확인해야 할 것들. 여기서 걸리는 게 실기에서 걸리는 것보다 싸다."""
        if not self.wait_until_ready(timeout=15.0):
            return False

        pos, vel = self.current_state()
        self.get_logger().info('현재 관절 (deg): ' + ', '.join(f'{v / D2R:7.2f}' for v in pos))
        self.get_logger().info('현재 속도 (rad/s): ' + ', '.join(f'{v:6.3f}' for v in vel))

        # 🔴 nvblox → cuMotion 경로 확인. 이게 죽어 있으면 **계획은 성공하는데 장애물을 통과한다.**
        #    계획 성공/실패로는 절대 안 드러나므로 여기서 따로 본다.
        obstacle_ok = self.check_obstacle_pipeline()

        # 계획만 한 번 던져 본다 (plan_only=True 라 로봇은 안 움직인다).
        self.get_logger().info(f'[{self.cfg.pipeline_id}] 제자리 계획 1회 시험...')
        t0 = time.time()
        traj = self.plan(self.joint_goal(pos), start_pos=pos, start_vel=[0.0] * len(pos))
        if traj is None:
            self.get_logger().error(
                '시험 계획 실패 → T4/T5/T6 중 하나가 문제다. testcommand.md 11절 증상표를 본다.')
            return False

        self.get_logger().info(
            f'시험 계획 OK ({(time.time() - t0) * 1000:.0f} ms, {len(traj.points)} pts)')

        # 계획을 한 번 돌렸으니 이제 /curobo/voxels 가 떴을 것이다 — cuMotion 이 실제로
        # 장 본 장애물 수를 확인한다. nvblox 가 살아 있어도 이게 0 이면 지도가 빈 것이다.
        if self.start_voxel_monitor():
            self.get_logger().info('복셀 수신 대기 (계획 1회 더 던진다)...')
            self.plan(self.joint_goal(pos), start_pos=pos, start_vel=[0.0] * len(pos))
            deadline = time.time() + 3.0
            while self.last_voxel_count < 0 and time.time() < deadline:
                time.sleep(0.05)
            if self.last_voxel_count < 0:
                self.get_logger().warn('⚠️ 복셀 미수신 — 장애물을 봤는지 확인할 방법이 없다')
            elif self.last_voxel_count == 0:
                obstacle_ok = False
                self.get_logger().error(
                    '🔴 cuMotion 이 본 복셀 0개 → nvblox 는 살아 있어도 **지도가 비었다**. '
                    '이대로면 장애물을 통과한다. 카메라 FOV / workspace_bounds_* / '
                    'robot_segmenter 출력(world_depth) 확인.')
            else:
                self.get_logger().info(
                    f'cuMotion 이 장 본 장애물 복셀 {self.last_voxel_count}개 '
                    '(2026-08-06 실측 27,646개)')

        # lookahead 가 실제 계획시간보다 짧으면 루프가 궤적을 계속 폐기한다 — 미리 경고한다.
        plan_ms = self.stat_plan_ms[-1] if self.stat_plan_ms else 0.0
        need = plan_ms / 1000.0 + self.handover_s
        if self.lookahead_s < need:
            self.get_logger().warn(
                f'⚠️ lookahead_s({self.lookahead_s:.2f}) < 계획시간+handover({need:.2f}) → '
                '재계획 궤적이 매번 폐기될 수 있다. lookahead_s 를 올려라.')

        if not obstacle_ok:
            self.get_logger().error(
                '점검 실패 — 계획은 되지만 **장애물을 못 본다**. 이대로 움직이면 통과한다.')
            return False
        return True

    # ── 실제 이동 ───────────────────────────────────────────────────────────
    def _loop_kwargs(self) -> dict:
        return dict(replan_hz=self.replan_hz,
                    lookahead_s=self.lookahead_s,
                    handover_s=self.handover_s,
                    max_start_jump=self.max_start_jump,
                    max_consecutive_failures=self.max_consecutive_failures,
                    timeout_s=self.timeout_s,
                    static_mode=self.static_mode)

    def run(self) -> int:
        if self.mode == 'check':
            return 0 if self.do_check() else 1

        if not self.wait_until_ready(timeout=30.0):
            return 1

        # 🔴 움직이기 전에 장애물 경로가 살아 있는지 확인한다.
        #    nvblox 가 없어도 계획은 성공하므로, 이 gate 가 없으면 통과한 뒤에야 안다.
        if not self.check_obstacle_pipeline():
            if self.require_obstacle_pipeline:
                self.get_logger().error(
                    '장애물 경로가 죽어 있어 이동을 거부한다. nvblox(T5)를 먼저 띄운다. '
                    '(정말 장애물 없이 돌리려면 require_obstacle_pipeline:=false)')
                return 1
            self.get_logger().warn(
                '⚠️ require_obstacle_pipeline:=false — 장애물을 못 보는 채로 움직인다.')

        if self.cfg.vel_scale > 0.3:
            self.get_logger().warn(
                f'⚠️ vel_scale={self.cfg.vel_scale} — 파이프라인 지연이 ~0.6 s 라 '
                '빠를수록 회피 여유가 줄어든다. 처음엔 0.15~0.25 를 권한다.')
        if self.static_mode:
            self.get_logger().warn(
                '🔴 static_mode=true — 재계획을 하지 않는다. 장애물이 들어와도 그대로 밀고 간다. '
                '대조군 실험 외에는 켜지 말 것.')

        if self.mode == 'joint':
            target = [v * D2R for v in self.goal_joint_deg]
            self.get_logger().info(
                '목표(관절 deg): ' + ', '.join(f'{v:.1f}' for v in self.goal_joint_deg))
            ok = self.run_to_goal(self.joint_goal(target), goal_positions=target,
                                  **self._loop_kwargs())
            return 0 if ok else 1

        if self.mode == 'pose':
            xyz = self.goal_pose[:3]
            rpy = [v * D2R for v in self.goal_pose[3:]]
            self.get_logger().info(f'목표(TCP): xyz={xyz} rpy_deg={self.goal_pose[3:]}')
            # pose goal 은 관절 목표가 없으니 도착 판정을 궤적 완료/잔여길이로 한다.
            ok = self.run_to_goal(self.pose_goal(xyz, rpy), goal_positions=None,
                                  **self._loop_kwargs())
            return 0 if ok else 1

        if self.mode == 'pingpong':
            targets = [[v * D2R for v in self.a_deg], [v * D2R for v in self.b_deg]]
            names = ['A', 'B']
            i, n = 0, 0
            mode = '재계획 OFF (대조군)' if self.static_mode else f'재계획 {self.replan_hz} Hz'
            self.get_logger().info(f'왕복 시작 — {mode}, vel={self.cfg.vel_scale}')
            self.get_logger().info(
                '지금 작업영역(x 0.0~0.7, y -0.3~0.3)에 손이나 상자를 넣어 본다.')
            while rclpy.ok():
                t = targets[i]
                self.get_logger().info(f'→ 자세 {names[i]}')
                if not self.run_to_goal(self.joint_goal(t), goal_positions=t,
                                        **self._loop_kwargs()):
                    self.get_logger().error(f'자세 {names[i]} 실패 — 중단')
                    return 1
                i ^= 1
                if i == 0:
                    n += 1
                    self.get_logger().info(f'왕복 {n}회 완료 | {self.summary()}')
                    if self.cycles and n >= self.cycles:
                        return 0
                time.sleep(0.3)
            return 0

        self.get_logger().error(
            f"mode:={self.mode} 는 없다. check | joint | pose | pingpong 중 하나.")
        return 2


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DynamicAvoid()
    node.start_spin()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('사용자 중단')
        rc = 130
    finally:
        try:
            node.get_logger().info(node.summary())
            node.cancel_execution()
        except Exception:
            pass
        node.stop_spin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
