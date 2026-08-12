#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reactive_replan.py — MoveIt 액션클라이언트 + 두산 로봇 인터페이스로 짠
독립형(단일 파일) 재계획 루프. cumotion/arm.py 를 import 하지 않는다.

검증 근거 (도커 마운트 실제 소스를 직접 읽어서 확인 — 추측 아님)
---------------------------------------------------------------
1. isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py
   · execute_callback(): update_world_objects() 가 계획 요청 1건당 1번만 불린다
     → ESDF 는 "계획을 다시 던질 때"만 갱신된다. 지속 재계획 루프가 필수인 이유.
   · start_state 를 CuJointState.from_position(position=...) 로만 만든다
     (velocity 인자 없음) → 시작 velocity 는 항상 버려진다. 새 궤적 첫 점은 언제나 v=0.
   · IK_FAIL 은 이 파일 안에서는 NO_IK_SOLUTION 으로 정확히 매핑된다.

2. isaac_ros_cumotion_moveit/src/cumotion_interface.cpp (75행)
   `response.error_code_.val = PLANNING_FAILED;` — action_client_->success 가 false 면
   위 1번의 진짜 코드(NO_IK_SOLUTION 등)를 무조건 PLANNING_FAILED(-1) 로 덮어쓴다.
   cumotion_move_group_client.cpp 의 success 는 error_code==SUCCESS 만 본다.
   → -1 을 받으면 여기서는 원인을 알 수 없다. 진짜 이유는 T6(cumotion_planner_node)
     로그의 "Motion planning failed wih status:" 줄에만 있다.

3. isaac_ros_cumotion/cumotion_goal_set_client.py (NVIDIA 예제)
   execute_plan() 이 /execute_trajectory (moveit_msgs/ExecuteTrajectory, 즉
   MoveItSimpleControllerManager 경유) 를 쓴다 — 이 경로는 이 노드가 실행 중일 때
   "Robot is still executing previous command" 로 막혀서 선점 교체를 못 한다.
   → NVIDIA 공식 예제에도 지속 재계획+선점교체 패턴은 없다. 아래처럼 MoveItSimpleControllerManager
     를 건너뛰고 컨트롤러의 FollowJointTrajectory 액션을 직접 잡아야만 교체가 된다.

4. dsr_moveit_config_m0609/config/moveit_controllers.yaml
   allowed_start_tolerance: 0.01 — 3번의 이유가 바로 이 값이다. 위 우회가 필요한 근거.

5. src/cobot_rg2/rg2/m0609_rg2_bringup/launch/bringup.launch.py
   · control_node / joint_state_broadcaster_spawner / dsr_controller2 spawner 모두
     namespace='dsr01' → 컨트롤러 액션은 /dsr01/dsr_moveit_controller/follow_joint_trajectory
   · joint_state_publisher 는 namespace 없이 뜨고 source_list=['/dsr01/joint_states',
     '/gripper_joint_states'] 를 합쳐 **전역** /joint_states 로 낸다.
   · publish_default_velocities: True — 이게 없으면 cuMotion 이 velocity 길이 불일치로
     계획을 전부 실패시킨다 (cumotion_planner.py 의 js_buffer velocity 검사).

6. doosan-robot2/dsr_controller2/src/dsr_controller2.cpp (2417행)
   `create_service<MoveStop>("motion/move_stop", ...)` — 이 서비스도 dsr01 네임스페이스의
   컨트롤러 노드 안에서 등록되므로 전체 경로는 /dsr01/motion/move_stop.
   MoveStop.srv: `int32 stop_mode` → `bool success` (필드명 확인됨).

이 파일이 하는 일
------------------
    [계획]  /move_action (MoveGroup, pipeline_id=isaac_ros_cumotion, plan_only=True)
    [실행]  /dsr01/dsr_moveit_controller/follow_joint_trajectory 로 직접(JTC 선점 교체)
    [정지]  /dsr01/motion/move_stop (dsr_msgs2/MoveStop)

⚠️ 로봇을 실제로 움직인다. 첫 실행은 vel_scale 을 낮게, 비상정지 버튼에 손을 올리고 한다.
⚠️ /execute_trajectory 나 moveit_py.execute() 는 쓰지 않는다 — 3·4번 근거.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dsr_msgs2.srv import MoveStop  # motion/move_stop — 근거 6

D2R = math.pi / 180.0
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

DR_SSTOP = 2  # dsr_msgs2 motion/MoveStop.srv 주석의 DR_SSTO(2), Soft stop


def err_name(code: int) -> str:
    for name in dir(MoveItErrorCodes):
        if name.startswith('_') or name == 'val':
            continue
        try:
            if getattr(MoveItErrorCodes, name) == code:
                return f'{name}({code})'
        except TypeError:
            continue
    return f'UNKNOWN({code})'


class ReactiveReplan(Node):
    """MoveGroup(계획) + FollowJointTrajectory(실행) 액션클라이언트로 만든
    최소 재계획 루프. 목표를 향해 계속 다시 계획하고, 실제로 달라졌을 때만
    실행 중인 궤적을 교체해 동적 장애물을 회피한다.
    """

    def __init__(self):
        super().__init__('reactive_replan')
        cb = self._cb = ReentrantCallbackGroup()

        p = self.declare_parameter
        self.group_name = p('group_name', 'manipulator').value
        self.base_frame = p('base_frame', 'base_link').value
        self.pipeline_id = p('pipeline_id', 'isaac_ros_cumotion').value
        self.vel_scale = float(p('vel_scale', 0.15).value)
        self.acc_scale = float(p('acc_scale', 0.15).value)
        self.replan_hz = float(p('replan_hz', 3.0).value)          # 근거1: 재계획=ESDF 재조회
        self.lookahead_s = float(p('lookahead_s', 0.35).value)     # 계획시간(~0.2s)+handover 여유
        self.handover_s = float(p('handover_s', 0.05).value)
        self.swap_threshold_rad = float(p('swap_threshold_rad', 0.05).value)  # 근거1: v=0 재시작 방지
        self.max_start_jump = float(p('max_start_jump', 0.25).value)
        self.max_consecutive_failures = int(p('max_consecutive_failures', 4).value)
        self.joint_goal_tol = float(p('joint_goal_tol', 0.01).value)
        self.timeout_s = float(p('timeout_s', 60.0).value)

        self._js_lock = threading.Lock()
        self._js_pos: dict = {}
        self._js_vel: dict = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states,   # 근거5: 전역 토픽
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       durability=QoSDurabilityPolicy.VOLATILE),
            callback_group=cb)

        self._plan_cli = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self._exec_cli = ActionClient(
            self, FollowJointTrajectory,
            '/dsr01/dsr_moveit_controller/follow_joint_trajectory',  # 근거5
            callback_group=cb)
        self._stop_cli = self.create_client(
            MoveStop, '/dsr01/motion/move_stop', callback_group=cb)  # 근거5·6

        self._active_exec_handle = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None

        self.stat_plans = 0
        self.stat_plan_fail = 0
        self.stat_swaps = 0
        self.stat_skips = 0

        # 2026-08-11: max_start_jump 폐기 진단 결과 — plan() 실제 소요시간(OMPL 실측
        # 평균 0.07s)이 고정 lookahead_s(0.35s, cuMotion 계획시간 ~0.2s 기준 튜닝값) 보다
        # 훨씬 짧아, 시드가 "0.35초 뒤" 위치를 예측했는데 검증 시점엔 0.07초밖에 안 지나
        # 있어 항상 크게 어긋났다(폐기 시 plan_dt 평균 0.073s vs 성공 시 0.071s — 통계적
        # 차이 없음, plan() 지연이 원인이 아니라는 뜻). 고정값 대신 실측 plan() 소요시간의
        # 이동평균(EMA)으로 예측 창을 스스로 맞춘다. 최초 1회는 self.lookahead_s로 시작.
        #
        # ⚠️ cross-review 지적(2026-08-11): 위 진단과 검증은 전부 pipeline_id='ompl'
        # (가상환경)에서 했다. 이 노드의 기본 파이프라인은 'isaac_ros_cumotion'(116행)이고,
        # cuMotion 계획시간(~0.2s+)은 OMPL(~0.07s)보다 느리고 GPU/ESDF 재조회 스파이크로
        # 변동폭도 클 수 있다 — **cuMotion 조건에서 EMA가 실제로 잘 수렴하는지는 미검증.**
        # 또한 EMA가 과거 스파이크로 부풀어 있으면 max_start_jump 폐기가 잦아지는데, 이건
        # 관절 점프 관점에서는 안전이지만 "낡은 궤적을 계속 따라간다"는 뜻이라 회피
        # 반응성 관점에서는 오히려 나쁜 쪽으로 작용할 수 있다 — 무조건 안전 쪽 편향이
        # 아니라 트레이드오프다. cuMotion 파이프라인 재검증 전엔 프로덕션 취급 금지.
        self._plan_dt_ema: Optional[float] = None

    # ── 스핀 (action future 를 폴링으로 기다리기 위해 백그라운드 executor 필요) ──
    def start_spin(self) -> None:
        from rclpy.executors import MultiThreadedExecutor
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop_spin(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.002)
        return True

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        if not self._plan_cli.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('/move_action 없음 → move_group(cuMotion 파이프라인 포함) 미기동')
            return False
        left = max(1.0, timeout - (time.time() - t0))
        if not self._exec_cli.wait_for_server(timeout_sec=left):
            self.get_logger().error(
                '/dsr01/dsr_moveit_controller/follow_joint_trajectory 없음 → '
                'dsr_controller2 spawner 확인 (bringup.launch.py)')
            return False
        deadline = time.time() + max(2.0, timeout - (time.time() - t0))
        while time.time() < deadline:
            with self._js_lock:
                have_pos = all(j in self._js_pos for j in JOINT_NAMES)
                have_vel = all(j in self._js_vel for j in JOINT_NAMES)
            if have_pos and have_vel:
                return True
            time.sleep(0.2)
        self.get_logger().error(
            '/joint_states 에 관절 6개 position+velocity 가 안 옴 → '
            'publish_default_velocities:True 확인 (근거5)')
        return False

    def _on_joint_states(self, msg: JointState) -> None:
        with self._js_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._js_pos[name] = msg.position[i]
                if i < len(msg.velocity):
                    self._js_vel[name] = msg.velocity[i]

    def current_state(self) -> Tuple[List[float], List[float]]:
        with self._js_lock:
            pos = [self._js_pos.get(j, 0.0) for j in JOINT_NAMES]
            vel = [self._js_vel.get(j, 0.0) for j in JOINT_NAMES]
        return pos, vel

    # ── 계획 ────────────────────────────────────────────────────────────────
    def joint_goal(self, positions: Sequence[float], tol: float = 0.01) -> Constraints:
        c = Constraints()
        c.name = 'joint_goal'
        for name, val in zip(JOINT_NAMES, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(val),
                tolerance_above=tol, tolerance_below=tol, weight=1.0))
        return c

    def _build_goal(self, goal: Constraints,
                    start_pos: Sequence[float], start_vel: Sequence[float]) -> MoveGroup.Goal:
        req = MotionPlanRequest()
        req.pipeline_id = self.pipeline_id
        req.group_name = self.group_name
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_frame
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -0.20, -0.50, -0.05
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 0.90, 0.50, 0.70
        req.workspace_parameters = ws

        # 근거1: velocity 를 실어도 cuMotion 이 버린다. 그래도 메시지 규약상 채워 보낸다.
        req.start_state.joint_state.name = list(JOINT_NAMES)
        req.start_state.joint_state.position = [float(v) for v in start_pos]
        req.start_state.joint_state.velocity = [float(v) for v in start_vel]
        req.start_state.is_diff = False

        req.goal_constraints = [goal]

        g = MoveGroup.Goal()
        g.request = req
        opts = PlanningOptions()
        opts.plan_only = True                    # 실행은 우리가 FJT 로 직접 한다 (근거3·4)
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        opts.replan = False
        g.planning_options = opts
        return g

    def plan(self, goal: Constraints, start_pos: Sequence[float], start_vel: Sequence[float],
            timeout: float = 10.0) -> Optional[JointTrajectory]:
        """계획 1회. 이 호출 자체가 nvblox ESDF 재조회 트리거다 (근거1)."""
        g = self._build_goal(goal, start_pos, start_vel)
        self.stat_plans += 1

        send_fut = self._plan_cli.send_goal_async(g)
        if not self._wait(send_fut, timeout):
            self.stat_plan_fail += 1
            self.get_logger().warn('계획 goal 전송 타임아웃')
            return None
        handle = send_fut.result()
        if not handle.accepted:
            self.stat_plan_fail += 1
            self.get_logger().warn('move_group 이 계획 요청 거부')
            return None

        res_fut = handle.get_result_async()
        if not self._wait(res_fut, timeout):
            self.stat_plan_fail += 1
            self.get_logger().warn('계획 결과 타임아웃')
            return None

        res = res_fut.result().result
        code = res.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.stat_plan_fail += 1
            hint = ''
            if code == MoveItErrorCodes.PLANNING_FAILED:
                # 근거2: 여기선 원인을 모른다. T6 로그의 "Motion planning failed wih status:" 확인.
                hint = ' → cuMotion 플러그인이 실제 에러코드를 덮어썼다. T6 로그를 봐야 한다 (근거2).'
            elif code == MoveItErrorCodes.START_STATE_IN_COLLISION:
                hint = ' → 로봇 몸이 nvblox 지도에 찍힘. robot_segmenter distance_threshold 확인.'
            elif code == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
                hint = ' → ESDF pull 실패. nvblox_node 가 죽었거나 esdf_service_name 불일치.'
            elif code == MoveItErrorCodes.NO_IK_SOLUTION:
                hint = ' → 목표 pose 의 IK 해 없음.'
            self.get_logger().warn(f'계획 실패: {err_name(code)}{hint}')
            return None

        traj = res.planned_trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().warn('계획 성공인데 궤적이 비었다')
            return None
        return traj

    # ── 실행 ────────────────────────────────────────────────────────────────
    def _reorder(self, traj: JointTrajectory) -> JointTrajectory:
        if list(traj.joint_names) == JOINT_NAMES:
            return traj
        idx = [traj.joint_names.index(j) for j in JOINT_NAMES]
        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(JOINT_NAMES)
        for pt in traj.points:
            q = JointTrajectoryPoint()
            q.positions = [pt.positions[i] for i in idx] if pt.positions else []
            q.velocities = [pt.velocities[i] for i in idx] if pt.velocities else []
            q.time_from_start = pt.time_from_start
            out.points.append(q)
        return out

    @staticmethod
    def _shift_time(traj: JointTrajectory, dt: float) -> JointTrajectory:
        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(traj.joint_names)
        for pt in traj.points:
            q = JointTrajectoryPoint()
            q.positions = list(pt.positions)
            q.velocities = list(pt.velocities)
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 + dt
            q.time_from_start.sec = int(t)
            q.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            out.points.append(q)
        return out

    def send_trajectory(self, traj: JointTrajectory, handover_s: float) -> Optional[object]:
        """/dsr01/.../follow_joint_trajectory 로 직접 보낸다.
        이미 실행 중인 goal 이 있으면 JTC 가 알아서 선점 교체한다 (근거3·4의 우회가 이걸 가능케 함)."""
        traj = self._reorder(traj)
        if handover_s > 0.0:
            traj = self._shift_time(traj, handover_s)
        g = FollowJointTrajectory.Goal()
        g.trajectory = traj
        g.goal_time_tolerance.sec = 1
        fut = self._exec_cli.send_goal_async(g)
        if not self._wait(fut, 2.0):
            self.get_logger().warn('실행 goal 전송 타임아웃')
            return None
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().warn('JTC 가 궤적을 거부했다')
            return None
        self._active_exec_handle = handle
        return handle

    @staticmethod
    def _duration(traj: JointTrajectory) -> float:
        p = traj.points[-1].time_from_start
        return p.sec + p.nanosec * 1e-9

    @staticmethod
    def _sample(traj: JointTrajectory, t: float) -> List[float]:
        pts = traj.points

        def tfs(p):
            return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9

        if t <= tfs(pts[0]):
            return list(pts[0].positions)
        if t >= tfs(pts[-1]):
            return list(pts[-1].positions)
        for i in range(len(pts) - 1):
            t0, t1 = tfs(pts[i]), tfs(pts[i + 1])
            if t0 <= t <= t1:
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                p0, p1 = pts[i].positions, pts[i + 1].positions
                return [p0[k] + a * (p1[k] - p0[k]) for k in range(len(p0))]
        return list(pts[-1].positions)

    def _same_path(self, cur_traj: JointTrajectory, new_traj: JointTrajectory,
                   base: float) -> bool:
        """새 궤적이 지금 달리는 궤적의 남은 부분과 사실상 같은가.

        cuMotion 은 매번 v=0 에서 출발하는 완결 궤적을 준다(근거1). 장애물이 안 변했으면
        그건 같은 경로를 처음부터 다시 시작하는 것뿐이라, 무조건 교체하면 로봇이 매번
        가속 램프에만 머물러 목표에 못 간다. 그래서 실제로 다를 때만 교체한다.

        `base` 는 cur_traj 위에서 new_traj 의 시작점(t=0)에 대응하는 시각이다. **호출자가
        시드를 채취할 때(run_to_goal 의 `seed_base`) 쓴 값을 그대로 넘겨야 한다** — 여기서
        `time.time()` 을 다시 읽지 않는다. 2026-08-11: 예전엔 이 함수 안에서 새로
        `time.time()` 을 읽었는데, `plan()` 왕복(블로킹, ~lookahead_s 자릿수)만큼 시드
        채취 시점보다 늦은 시각이 섞여 들어가 정렬이 부분적으로만 맞았다(cross-review 지적,
        ⭐-4 1번 버그의 잔여분). 시드와 정확히 같은 시각을 쓰는 것으로 근본 수정한다.
        """
        cur_traj = self._reorder(cur_traj)
        new_traj = self._reorder(new_traj)
        span = min(self._duration(new_traj), self._duration(cur_traj) - base)
        if span <= 0.0:
            return False
        for k in range(9):
            dt = span * k / 8
            p_new = self._sample(new_traj, dt)
            p_old = self._sample(cur_traj, base + dt)
            if max(abs(p_new[i] - p_old[i]) for i in range(len(p_new))) > self.swap_threshold_rad:
                return False
        return True

    def brake(self, brake_time: float = 0.3) -> None:
        pos, vel = self.current_state()
        end = [pos[i] + vel[i] * brake_time * 0.5 for i in range(len(pos))]
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = end
        pt.velocities = [0.0] * len(end)
        pt.time_from_start.sec = int(brake_time)
        pt.time_from_start.nanosec = int(round((brake_time - int(brake_time)) * 1e9))
        traj.points = [pt]
        self.send_trajectory(traj, handover_s=0.0)
        time.sleep(brake_time + 0.1)

    def emergency_stop(self, mode: int = DR_SSTOP) -> bool:
        if not self._stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/dsr01/motion/move_stop 서비스 없음')
            return False
        req = MoveStop.Request()
        req.stop_mode = mode
        fut = self._stop_cli.call_async(req)
        if not self._wait(fut, 2.0):
            return False
        return bool(fut.result().success)

    # ── 핵심: 재계획 루프 ───────────────────────────────────────────────────
    def run_to_goal(self, goal: Constraints, goal_positions: Sequence[float]) -> bool:
        period = 1.0 / max(0.1, self.replan_hz)

        pos, _ = self.current_state()
        traj = self.plan(goal, pos, [0.0] * len(pos))
        if traj is None:
            self.get_logger().error('최초 계획 실패 — 시작하지 않는다')
            return False
        if self.send_trajectory(traj, self.handover_s) is None:
            return False

        exec_t0 = time.time()
        traj_dur = self._duration(traj) + self.handover_s
        loop_t0 = time.time()
        fails = 0
        next_plan = time.time() + period

        try:
            while rclpy.ok():
                now = time.time()
                cur, _ = self.current_state()
                err = max(abs(cur[i] - goal_positions[i]) for i in range(len(cur)))
                if err < self.joint_goal_tol:
                    self.get_logger().info(f'목표 도착 (최대오차 {err:.4f} rad)')
                    return True
                if now - loop_t0 > self.timeout_s:
                    self.get_logger().error(f'타임아웃 {self.timeout_s:.0f}s — 정지')
                    self.brake()
                    return False

                traj_finished = (now - exec_t0) > traj_dur
                if now < next_plan and not traj_finished:
                    time.sleep(0.005)
                    continue
                next_plan = now + period

                # 2026-08-11 2차 시도: 1차(EMA 그대로 대입)는 seed_base = effective_lookahead
                # - handover_s 가 거의 0이 되어 매번 v=0 재출발 → 제자리로 되돌렸다(위 주석
                # 이력 참고, git blame). 이번엔 seed_base 자체가 "plan_dt_ema 뒤 위치"가
                # 되도록 handover_s 를 미리 더해 상쇄시킨다:
                #   seed_base = t_on_traj - handover_s
                #             = (now-exec_t0) + (plan_dt_ema + handover_s) - handover_s
                #             = (now-exec_t0) + plan_dt_ema
                # 즉 "새 궤적이 준비될 즈음(plan_dt_ema 초 후) 로봇이 있을 위치"를 그대로
                # 예측한다 — 고정 lookahead_s(0.35s, 실측보다 5배 큼)보다 정확한 예측이면서,
                # 1차 시도처럼 handover_s 를 이중으로 빼지 않는다. 하한(floor)은 0.02s로
                # 두어 plan_dt_ema 가 아주 작게 튄 경우에도 예측이 완전히 0이 되진 않게 한다.
                predicted_plan_dt = (self._plan_dt_ema if self._plan_dt_ema is not None
                                     else self.lookahead_s - self.handover_s)
                effective_lookahead = max(predicted_plan_dt, 0.02) + self.handover_s
                t_on_traj = (now - exec_t0) + effective_lookahead
                # seed_base: 시드를 cur_traj(=traj) 위에서 실제로 뽑았을 때만 값이 있다.
                # 이 경우에만 "새 계획이 cur_traj 의 그 지점과 같은가"라는 _same_path 비교가
                # 의미가 있다 — 아래(traj_finished/추월) 쪽은 애초에 cur_traj 연속이 아니라
                # 현재 실측 위치에서 새로 시작하는 것이라 비교 대상이 없다.
                if traj_finished or t_on_traj >= traj_dur:
                    s_pos, _ = self.current_state()
                    s_vel = [0.0] * len(s_pos)
                    seed_base = None
                else:
                    seed_base = max(0.0, t_on_traj - self.handover_s)
                    s_pos = self._sample(traj, seed_base)
                    s_vel = [0.0] * len(s_pos)   # 근거1: 어차피 버려진다

                plan_t0 = time.time()
                new_traj = self.plan(goal, s_pos, s_vel)
                plan_dt = time.time() - plan_t0
                # EMA 갱신 — 다음 반복의 effective_lookahead 가 이 실측값을 따라간다.
                self._plan_dt_ema = (plan_dt if self._plan_dt_ema is None
                                     else 0.7 * self._plan_dt_ema + 0.3 * plan_dt)
                if new_traj is None:
                    fails += 1
                    if fails >= self.max_consecutive_failures:
                        self.get_logger().error(f'계획 {fails}회 연속 실패 — 감속 정지')
                        self.brake()
                        return False
                    continue
                fails = 0

                cur, _ = self.current_state()
                first = self._reorder(new_traj).points[0].positions
                jump = max(abs(first[i] - cur[i]) for i in range(len(cur)))
                if jump > self.max_start_jump:
                    self.get_logger().warn(
                        f'새 궤적 시작점이 실측과 {jump:.3f} rad 어긋남 — 폐기 '
                        f'(plan_dt={plan_dt:.3f}s, effective_lookahead={effective_lookahead:.3f}s)')
                    continue

                if seed_base is not None and self._same_path(traj, new_traj, seed_base):
                    self.stat_skips += 1
                    continue

                if self.send_trajectory(new_traj, self.handover_s) is not None:
                    traj = new_traj
                    exec_t0 = time.time()
                    traj_dur = self._duration(new_traj) + self.handover_s
                    self.stat_swaps += 1
                    self.get_logger().info(
                        f'교체 #{self.stat_swaps} (plan_dt={plan_dt:.3f}s)')  # 2026-08-11 진단

        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl+C — 감속 정지')
            self.brake()
            raise
        return False

    def summary(self) -> str:
        return (f'계획 {self.stat_plans}회 (실패 {self.stat_plan_fail}) / '
                f'교체 {self.stat_swaps}회 (생략 {self.stat_skips}회)')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ReactiveReplan()
    node.start_spin()
    rc = 0
    try:
        if not node.wait_until_ready(timeout=30.0):
            return 1

        # ── 데모: A ↔ B 왕복. 작업영역에 손/상자를 넣어 회피를 시험한다 ──
        a_deg = [45.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        b_deg = [-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        targets = [[v * D2R for v in a_deg], [v * D2R for v in b_deg]]
        i = 0
        while rclpy.ok():
            t = targets[i]
            node.get_logger().info(f'목표 {"A" if i == 0 else "B"} 로 이동')
            if not node.run_to_goal(node.joint_goal(t), t):
                node.get_logger().error('이동 실패 — 중단')
                rc = 1
                break
            node.get_logger().info(node.summary())
            i ^= 1
            time.sleep(0.3)
    except KeyboardInterrupt:
        rc = 130
    finally:
        try:
            node.get_logger().info(node.summary())
        except Exception:
            pass
        node.stop_spin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
