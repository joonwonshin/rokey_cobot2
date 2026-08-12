#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cumotion/arm.py — MoveIt(+cuMotion+nvblox) 스택을 파이썬으로 제어하는 라이브러리

이 파일이 하는 일
-----------------
RViz의 MotionPlanning 패널이 하던 일(Plan 버튼 / Execute 버튼)을 코드로 분해해서,
**계획을 반복해서 다시 던지고 실행 중인 궤적을 갈아끼우는 루프**로 만든다.

    [계획]  /move_action (MoveGroup 액션, pipeline_id="isaac_ros_cumotion", plan_only=True)
              → move_group 안의 cuMotion 플러그인 → /cumotion/move_group
              → cumotion_planner_node 가 nvblox 에 get_esdf_and_gradient 로 장애물을 pull
              ↓ trajectory_msgs/JointTrajectory
    [실행]  /dsr01/dsr_moveit_controller/follow_joint_trajectory (JointTrajectoryController)
              → dsr_hw_interface2 write() → Drfl.servoj_rt()

전부 표준 ROS 2 인터페이스다(액션/토픽/서비스). RViz 가 쓰는 것과 같은 진입점이고,
GUI 대신 이 코드가 클라이언트일 뿐이다.

🔴 왜 이 루프가 필요한가
    cumotion_planner_node 는 **계획 요청 1건당 ESDF 를 딱 1번** 읽는다
    (config/cumotion_planner.yaml 의 update_esdf_on_request 주석). 즉 궤적이 한 번 만들어지고
    나면 그 뒤에 사람이 걸어 들어와도 cuMotion 은 아무것도 모른다.
    "실시간 회피"는 플래너가 해주는 게 아니라 **이쪽에서 계획을 계속 다시 시키고
    새 궤적으로 갈아끼워야** 생긴다. 이 파일의 존재 이유가 그것 하나다.

🔴 왜 moveit_py 가 아니라 액션 클라이언트인가  (셋 다 이 루프에 치명적이다)
    ① moveit_py.execute() 는 MoveIt 실행 관리자를 탄다 → allowed_start_tolerance(0.01 rad)
       검사에 걸려 **움직이는 중의 궤적 교체가 매번 거부된다.**
    ② moveit_py 에는 실행 중 궤적을 선점 교체하는 API 가 없다(plan→execute 순차 모델).
    ③ moveit_py 는 프로세스 안에 RobotModel/PlanningScene 을 또 띄운다 → move_group 의
       파라미터 일습(robot_description·SRDF·kinematics·planning_pipelines)을 이 노드에도
       똑같이 먹여야 한다. 액션 클라이언트는 이미 떠 있는 move_group 에 붙기만 하면 된다.
    JointTrajectoryController 액션을 직접 부르면 ① 의 검사가 없고, 새 goal 이 오면
    JTC 가 기존 goal 을 스스로 선점(preempt)해서 끊김 없이 교체된다.

⚠️ movel/movej(두산 네이티브 서비스)를 이 루프가 도는 동안 부르지 말 것.
   MoveIt 경로와 movel/movej 가 같은 DRFL TCP 연결 하나를 공유하기 때문에 모션 모드가 충돌한다.
   비상정지만 /dsr01/motion/move_stop 서비스로 부른다(emergency_stop()).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 두산 비상정지 서비스. 없는 환경(에뮬레이터/개발 PC)에서도 import 는 되게 optional 처리한다.
try:
    from dsr_msgs2.srv import MoveStop
    _HAS_DSR_MSGS = True
except ImportError:  # pragma: no cover
    MoveStop = None
    _HAS_DSR_MSGS = False


# ─────────────────────────────────────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────────────────────────────────────

ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# 두산 stop_mode (dsr_msgs2/srv/motion/MoveStop.srv)
DR_QSTOP_STO = 0   # Stop Category 1, STO 없이 급정지
DR_QSTOP = 1       # Stop Category 2, 급정지
DR_SSTOP = 2       # Soft stop
DR_HOLD = 3        # Hold


@dataclass
class ArmConfig:
    """이름/토픽/속도는 전부 여기서만 바꾼다. 코드 본문에 하드코딩하지 않는다.

    ROS 파라미터로 받으려면 declare_from_node(node) 를 쓴다
    (config/dynamic_avoid.yaml 의 키 이름이 여기 필드명과 1:1이다).
    """

    # ── MoveIt ──
    group_name: str = 'manipulator'          # SRDF 의 planning group
    base_frame: str = 'base_link'            # nvblox global_frame 과 같아야 좌표가 맞는다
    ee_link: str = 'tool0'                   # pose goal 을 걸 링크
    move_action: str = '/move_action'        # move_group 의 진입점
    pipeline_id: str = 'isaac_ros_cumotion'  # 'ompl' 로 바꾸면 octomap 기반 OMPL 로 계획한다

    # ── 컨트롤러 (dsr_controller2.yaml: dsr_moveit_controller = JointTrajectoryController) ──
    # ⚠️ 앞의 /dsr01/ 은 오타가 아니다. bringup 의 controller_manager 가 dsr01 네임스페이스에 있다.
    fjt_action: str = '/dsr01/dsr_moveit_controller/follow_joint_trajectory'
    stop_service: str = '/dsr01/motion/move_stop'
    joint_names: List[str] = field(default_factory=lambda: list(ARM_JOINTS))

    # ── 상태 입력 ──
    # 🔴 /joint_states 에 velocity 가 비어 있으면 cuMotion 계획이 전부 실패한다
    #    (bringup.launch.py 의 publish_default_velocities: True 가 그 대책).
    joint_states_topic: str = '/joint_states'

    # ── 장애물 파이프라인 감시 (이 노드가 구독하는 게 아니다. 아래 주석 참고) ──
    #
    # 🔴 이 노드는 nvblox 를 구독하지 않는다. 장애물은 우리를 안 거치고 들어간다:
    #
    #   nvblox_node ──서비스── /nvblox_node/get_esdf_and_gradient
    #                              ▲ cumotion_planner_node 가 pull (read_esdf_world: true,
    #                                update_esdf_on_request: true — 계획 요청 1건당 1회)
    #                       cumotion_planner_node ── cuRobo 충돌월드
    #                              ▲ /cumotion/move_group
    #                       move_group (cuMotion 플러그인)
    #                              ▲ /move_action  ← 우리는 여기만 잡는다
    #
    # 즉 plan() 호출 **그 자체가** nvblox 에 ESDF 를 물어보는 트리거다.
    # 아래 두 개는 "그 경로가 실제로 살아 있는가"를 확인하기 위한 감시용일 뿐,
    # 장애물 데이터를 받아 쓰는 통로가 아니다.
    #
    # 🔴 nvblox 가 죽어도 계획은 **성공한다** — 장애물이 없는 세상에서 계획할 뿐.
    #    실패가 아니라 성공처럼 보이는 실패라서, 미리 확인하지 않으면 로봇이 통과한 뒤에야 안다.
    esdf_service_name: str = '/nvblox_node/get_esdf_and_gradient'
    # cumotion_planner.yaml 의 publish_curobo_world_as_voxels: true 가 내보내는 토픽.
    # "cuMotion 이 장 본 장애물"이라 계획 결과의 유일한 직접 증거다.
    # ⚠️ 계획 요청을 처리하는 중에만 발행된다 — 대기 중엔 아무것도 안 온다.
    voxel_topic: str = '/curobo/voxels'
    monitor_voxels: bool = True

    # ── 속도 ──
    # cumotion_planner.yaml 이 override_moveit_scaling_factors: false 이므로
    # **여기서 보낸 scaling 이 이긴다**. time_dilation_factor(0.8)가 아니라 이 값이 실제 속도다.
    # ⚠️ 실기 첫 실행은 0.15~0.25 로 시작한다. 재계획 루프는 로봇이 실제로 움직이는 코드다.
    vel_scale: float = 0.2
    acc_scale: float = 0.2

    allowed_planning_time: float = 5.0
    num_planning_attempts: int = 1

    # goal 판정 허용오차 (rad, 관절공간 최대 성분오차)
    joint_goal_tol: float = 0.01

    # 🔴 교체 문턱 (rad). 새 궤적이 지금 달리는 궤적과 이만큼도 안 다르면 **교체하지 않는다.**
    #    이게 없으면 로봇이 목표에 못 간다 — 2026-08-07 실측: 7.7초짜리 이동이 3 Hz 재계획에서
    #    60초 타임아웃까지 도착 실패(교체 155회, 실측속도 최대 0.037 rad/s). 자세한 건
    #    README 0절과 _same_path() 주석.
    #    ⚠️ 아직 실기 튜닝 안 된 추정값이다. 너무 크면 장애물이 와도 교체를 안 해 회피가 죽고,
    #      너무 작으면 위의 기어감이 재현된다. mode:=joint 도착시간이 static(7.7초)에
    #      근접하는지부터 보고 올린다.
    swap_threshold_rad: float = 0.05

    @classmethod
    def declare_from_node(cls, node: Node) -> 'ArmConfig':
        """노드에 ROS 파라미터를 선언하고 그 값으로 채운 ArmConfig 를 돌려준다.

        ⚠️ CumotionArm.__init__ 안에서 **구독 생성 전에** 불려야 한다
        (joint_states_topic 이 구독 시점에 확정돼야 하므로).
        """
        d = cls()

        def p(name, default):
            return node.declare_parameter(name, default).value

        d.group_name = p('group_name', d.group_name)
        d.base_frame = p('base_frame', d.base_frame)
        d.ee_link = p('ee_link', d.ee_link)
        d.move_action = p('move_action', d.move_action)
        d.pipeline_id = p('pipeline_id', d.pipeline_id)
        d.fjt_action = p('fjt_action', d.fjt_action)
        d.stop_service = p('stop_service', d.stop_service)
        d.joint_names = list(p('joint_names', d.joint_names))
        d.joint_states_topic = p('joint_states_topic', d.joint_states_topic)
        d.esdf_service_name = p('esdf_service_name', d.esdf_service_name)
        d.voxel_topic = p('voxel_topic', d.voxel_topic)
        d.monitor_voxels = bool(p('monitor_voxels', d.monitor_voxels))
        d.vel_scale = float(p('vel_scale', d.vel_scale))
        d.acc_scale = float(p('acc_scale', d.acc_scale))
        d.allowed_planning_time = float(p('allowed_planning_time', d.allowed_planning_time))
        d.num_planning_attempts = int(p('num_planning_attempts', d.num_planning_attempts))
        d.joint_goal_tol = float(p('joint_goal_tol', d.joint_goal_tol))
        d.swap_threshold_rad = float(p('swap_threshold_rad', d.swap_threshold_rad))
        return d


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """RPY(rad) → quaternion. tf_transformations 의존을 피하려고 직접 계산한다."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


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


# ─────────────────────────────────────────────────────────────────────────────
# 본체
# ─────────────────────────────────────────────────────────────────────────────

class CumotionArm(Node):
    """계획(MoveGroup 액션) + 실행(FollowJointTrajectory 액션) + 재계획 루프.

    cfg=None 이면 ROS 파라미터에서 읽는다(ros2 run / launch 용).
    cfg 를 직접 주면 파라미터를 선언하지 않는다(라이브러리로 import 해서 쓸 때).
    """

    def __init__(self, cfg: Optional[ArmConfig] = None, node_name: str = 'cumotion_arm'):
        super().__init__(node_name)
        self.cfg = ArmConfig.declare_from_node(self) if cfg is None else cfg
        self._cb = ReentrantCallbackGroup()
        cfg = self.cfg

        # ── 관절 상태 ──
        self._js_lock = threading.Lock()
        self._js_pos: dict = {}
        self._js_vel: dict = {}
        self._js_stamp: float = 0.0
        # /joint_states 는 12개 관절(팔 6 + 그리퍼 계열)이 섞여 온다. 이름으로 골라 쓴다.
        self.create_subscription(
            JointState, cfg.joint_states_topic, self._on_joint_states,
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       durability=QoSDurabilityPolicy.VOLATILE),
            callback_group=self._cb)

        # ── 액션 클라이언트 ──
        self._plan_cli = ActionClient(self, MoveGroup, cfg.move_action, callback_group=self._cb)
        self._exec_cli = ActionClient(self, FollowJointTrajectory, cfg.fjt_action,
                                      callback_group=self._cb)

        # ── 두산 비상정지 ──
        self._stop_cli = None
        if _HAS_DSR_MSGS:
            self._stop_cli = self.create_client(MoveStop, cfg.stop_service,
                                                callback_group=self._cb)

        self._active_exec_handle = None
        self._spin_thread: Optional[threading.Thread] = None
        self._executor = None

        # 감시용 (장애물 데이터 통로가 아니다 — ArmConfig 의 주석 참고)
        self._voxel_sub = None
        self.last_voxel_count: int = -1     # -1 = 아직 한 번도 못 받음
        self.last_voxel_time: float = 0.0

        # 통계 (루프 끝나고 요약 출력용)
        self.stat_plans = 0
        self.stat_plan_fail = 0
        self.stat_swaps = 0
        # 계획은 성공했지만 현재 궤적과 사실상 같아서 교체하지 않은 횟수(_same_path).
        # 장애물이 없으면 이 값이 교체 수보다 훨씬 커야 정상이다 — 그게 로봇이 실제로 나아간다는 뜻.
        self.stat_skips = 0
        self.stat_plan_ms: List[float] = []
        # 교체 순간의 실측 관절속도 최대성분. cuMotion 이 준 새 궤적의 첫 점은 속도가 항상 0이라
        # (_build_request 주석) 이 값이 그대로 인계 불연속의 크기다. 0 에 가까울수록 매끄럽다.
        self.stat_seam_vel: List[float] = []

    # ── 스핀 ────────────────────────────────────────────────────────────────
    def start_spin(self) -> None:
        """액션 future 를 메인 스레드에서 기다릴 수 있게 executor 를 백그라운드로 돌린다."""
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

    # ── 준비 확인 ───────────────────────────────────────────────────────────
    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """T2(bringup)·T6(cuMotion)·T7(move_group)이 다 떠 있는지 한 번에 확인한다."""
        t0 = time.time()

        if not self._plan_cli.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f'{self.cfg.move_action} 액션 서버 없음 → T7 move_group 이 안 떠 있다.')
            return False

        left = max(1.0, timeout - (time.time() - t0))
        if not self._exec_cli.wait_for_server(timeout_sec=left):
            self.get_logger().error(
                f'{self.cfg.fjt_action} 없음 → dsr_moveit_controller 가 spawn/active 가 아니다. '
                '(T7 로그의 "Configured and activated dsr_moveit_controller" 확인)')
            return False

        # /joint_states 에 velocity 가 실려 오는지 — cuMotion 의 전제조건이다.
        deadline = time.time() + max(2.0, timeout - (time.time() - t0))
        warned = False
        while time.time() < deadline:
            with self._js_lock:
                have = all(j in self._js_pos for j in self.cfg.joint_names)
                have_vel = all(j in self._js_vel for j in self.cfg.joint_names)
            if have and have_vel:
                self.get_logger().info('준비 완료 (move_group / JTC / joint_states+velocity)')
                return True
            if have and not have_vel and not warned:
                warned = True
                self.get_logger().warn(
                    '🔴 /joint_states 에 velocity 가 없다 → cuMotion 계획이 전부 실패한다. '
                    'bringup.launch.py 의 publish_default_velocities: True 확인.')
            time.sleep(0.2)

        self.get_logger().error(f'{self.cfg.joint_states_topic} 수신 실패')
        return False

    # ── 🔴 장애물 파이프라인 감시 ────────────────────────────────────────────
    def check_obstacle_pipeline(self) -> bool:
        """nvblox → cuMotion 경로가 살아 있는지 확인한다.

        🔴 이게 왜 따로 필요한가: nvblox_node 가 죽어도 **계획은 성공한다.**
           장애물이 없는 세상에서 계획할 뿐이다. 계획 성공/실패로는 절대 알 수 없고,
           로봇이 장애물을 통과한 뒤에야 안다(testcommand.md 의 "성공처럼 보이는 실패").

        ⚠️ 이 함수는 서비스가 **존재하는지**만 본다. nvblox 가 떠 있어도
           esdf_mode 가 2d 면 cuMotion 첫 요청에 nvblox 가 FATAL 로 죽는다 —
           그건 첫 계획을 실제로 던져 봐야(=plan() 호출) 드러난다.
        """
        cfg = self.cfg
        ok = True

        # ① ESDF 서비스 존재 확인 (T5 nvblox_node)
        found = False
        for _ in range(10):                       # 디스커버리에 시간이 걸린다
            names = dict(self.get_service_names_and_types())
            if cfg.esdf_service_name in names:
                found = True
                self.get_logger().info(
                    f'ESDF 서비스 확인: {cfg.esdf_service_name} '
                    f'({names[cfg.esdf_service_name][0]})')
                break
            time.sleep(0.3)

        if not found:
            ok = False
            self.get_logger().error(
                f'🔴 {cfg.esdf_service_name} 없음 → T5 nvblox_node 가 안 떠 있거나 죽었다.\n'
                '   이 상태로 돌리면 **계획은 성공하는데 장애물을 통과한다** (성공처럼 보이는 실패).\n'
                '   확인: pgrep -f nvblox_node / ros2 service list | grep esdf\n'
                '   자주 죽는 이유: esdf_mode 가 2d (기본값). cuMotion 첫 요청에 FATAL 로 죽는다.')

        # ② cuMotion 이 planner 쪽 액션을 실제로 물고 있는지 (T6)
        actions = [n for n, _ in self.get_topic_names_and_types()
                   if n.startswith('/cumotion/move_group/')]
        if actions:
            self.get_logger().info('cuMotion 플래너 노드 확인: /cumotion/move_group')
        else:
            self.get_logger().warn(
                '⚠️ /cumotion/move_group 액션이 안 보인다 → T6 cumotion_planner_node 확인. '
                '(move_group 의 cuMotion 플러그인은 이 액션으로 계획을 위임한다)')

        return ok

    def start_voxel_monitor(self) -> bool:
        """cuMotion 이 실제로 장 본 장애물 복셀 수를 세는 구독을 건다 (감시 전용).

        ⚠️ 이건 장애물을 **받아서 쓰는** 통로가 아니다. 장애물은 cumotion_planner_node 가
           nvblox 에서 직접 pull 한다. 여기서 세는 건 "그 pull 이 실제로 뭔가를 가져왔나"의
           사후 증거일 뿐이다. 복셀 0개면 nvblox 는 살아 있어도 지도가 비었다는 뜻이다.

        ⚠️ /curobo/voxels 는 **계획 요청을 처리하는 중에만** 발행된다. 대기 중엔 안 온다
           (testcommand.md 8절). 그래서 타입 탐색은 실패해도 치명적이지 않다.
        """
        if not self.cfg.monitor_voxels or self._voxel_sub is not None:
            return False

        # 타입을 추측하지 않고 실제로 조회한다. Marker 로 알려져 있지만 버전에 따라 다를 수 있다.
        types = dict(self.get_topic_names_and_types()).get(self.cfg.voxel_topic)
        if not types:
            self.get_logger().info(
                f'{self.cfg.voxel_topic} 아직 없음 — 복셀 감시 생략. '
                '(cumotion_planner.yaml 의 publish_curobo_world_as_voxels 확인)')
            return False

        if types[0] != 'visualization_msgs/msg/Marker':
            self.get_logger().warn(
                f'{self.cfg.voxel_topic} 타입이 {types[0]} 다 (Marker 를 기대했다) — 감시 생략')
            return False

        from visualization_msgs.msg import Marker
        self._voxel_sub = self.create_subscription(
            Marker, self.cfg.voxel_topic, self._on_voxels, 1, callback_group=self._cb)
        self.get_logger().info(f'복셀 감시 시작: {self.cfg.voxel_topic}')
        return True

    def _on_voxels(self, msg) -> None:
        self.last_voxel_count = len(msg.points)
        self.last_voxel_time = time.time()

    def voxel_report(self) -> str:
        if self.last_voxel_count < 0:
            return '복셀 미수신'
        age = time.time() - self.last_voxel_time
        return f'복셀 {self.last_voxel_count}개({age:.1f}s 전)'

    # ── 관절 상태 ───────────────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        with self._js_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._js_pos[name] = msg.position[i]
                if i < len(msg.velocity):
                    self._js_vel[name] = msg.velocity[i]
            self._js_stamp = time.time()

    def current_state(self) -> Tuple[List[float], List[float]]:
        """팔 6축의 (position, velocity). 이름으로 골라 cfg.joint_names 순서로 정렬해 돌려준다."""
        with self._js_lock:
            pos = [self._js_pos.get(j, 0.0) for j in self.cfg.joint_names]
            vel = [self._js_vel.get(j, 0.0) for j in self.cfg.joint_names]
        return pos, vel

    # ── 계획 ────────────────────────────────────────────────────────────────
    def _build_request(self, goal: Constraints,
                       start_pos: Optional[Sequence[float]],
                       start_vel: Optional[Sequence[float]]) -> MoveGroup.Goal:
        cfg = self.cfg
        req = MotionPlanRequest()
        req.pipeline_id = cfg.pipeline_id     # 'isaac_ros_cumotion' | 'ompl'
        req.group_name = cfg.group_name
        req.num_planning_attempts = cfg.num_planning_attempts
        req.allowed_planning_time = cfg.allowed_planning_time
        req.max_velocity_scaling_factor = cfg.vel_scale
        req.max_acceleration_scaling_factor = cfg.acc_scale

        # workspace_parameters 는 FixWorkspaceBounds 어댑터용이다.
        # ⚠️ cuMotion 이 실제로 보는 장애물 범위는 이 값이 아니라
        #    nvblox_realtime.yaml 의 workspace_bounds_* / cumotion_planner.yaml 의 grid_* 다.
        #    여기 값만 고쳐도 회피 범위는 1 mm 도 안 바뀐다.
        ws = WorkspaceParameters()
        ws.header.frame_id = cfg.base_frame
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -0.20, -0.50, -0.05
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 0.90, 0.50, 0.70
        req.workspace_parameters = ws

        # 시작 상태: 재계획에서는 "지금"이 아니라 "이 궤적이 인계될 미래 시점"을 넣는다.
        #
        # 🔴 cuMotion 은 여기 실은 **velocity 를 버린다.** 소스 확인:
        #    cumotion_planner.py:675 가 CuJointState.from_position(position=, joint_names=) 로만
        #    시작상태를 만든다 — velocity 인자가 없어 0 으로 채워진다. is_diff=False 라
        #    라이브 /joint_states 를 읽는 686~698 분기도 타지 않는다.
        #    ⇒ 계획 결과 궤적의 **첫 점 velocity 는 항상 0** 이다. 로봇이 속도를 갖고 달리는
        #      중에 "정지 상태에서 출발하는" 궤적을 인계받으므로 교체 지점마다 속도 딥이 남는다.
        #      lookahead_s 를 키워도 **위치**만 맞고 이건 안 고쳐진다 (run_to_goal 주석 참고).
        #    velocity 를 그래도 실어 보내는 이유: 메시지 규약상 맞는 값이고, 플래너가 나중에
        #    이걸 보게 되면 그때 자동으로 이득이다. 지금은 no-op 이라고 알고 쓴다.
        #
        #    ⚠️ start_pos 를 아예 안 주면(is_diff=True) cuMotion 이 /joint_states 의 실제
        #      velocity 를 읽는다(cumotion_planner.py:694-698). 대신 lookahead 가 사라져
        #      204 ms 뒤처진 상태에서 계획하게 된다 — 이 루프에서는 그 손해가 더 크다.
        if start_pos is not None:
            req.start_state.joint_state.name = list(cfg.joint_names)
            req.start_state.joint_state.position = [float(v) for v in start_pos]
            if start_vel is not None:
                req.start_state.joint_state.velocity = [float(v) for v in start_vel]
            req.start_state.is_diff = False
        else:
            req.start_state.is_diff = True    # move_group 이 현재 상태를 쓴다

        req.goal_constraints = [goal]

        g = MoveGroup.Goal()
        g.request = req
        opts = PlanningOptions()
        opts.plan_only = True                 # 🔴 실행은 우리가 JTC 로 직접 한다
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        opts.replan = False
        g.planning_options = opts
        return g

    def joint_goal(self, positions: Sequence[float], tol: float = 0.01) -> Constraints:
        c = Constraints()
        c.name = 'joint_goal'
        for name, val in zip(self.cfg.joint_names, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(val),
                tolerance_above=tol, tolerance_below=tol, weight=1.0))
        return c

    def pose_goal(self, xyz: Sequence[float], rpy: Sequence[float],
                  pos_tol: float = 0.01, ori_tol: float = 0.05) -> Constraints:
        cfg = self.cfg
        c = Constraints()
        c.name = 'pose_goal'

        pc = PositionConstraint()
        pc.header.frame_id = cfg.base_frame
        pc.link_name = cfg.ee_link
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(pos_tol)])]
        region_pose = Pose()
        region_pose.position.x, region_pose.position.y, region_pose.position.z = map(float, xyz)
        region_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses = [region_pose]
        pc.weight = 1.0
        c.position_constraints = [pc]

        oc = OrientationConstraint()
        oc.header.frame_id = cfg.base_frame
        oc.link_name = cfg.ee_link
        oc.orientation = quat_from_rpy(*[float(v) for v in rpy])
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    def plan(self, goal: Constraints,
             start_pos: Optional[Sequence[float]] = None,
             start_vel: Optional[Sequence[float]] = None,
             timeout: float = 10.0) -> Optional[JointTrajectory]:
        """계획 1회. 성공하면 JointTrajectory, 실패하면 None.

        🔴 이 호출 하나가 nvblox 에 ESDF 를 물어보는 시점이다(update_esdf_on_request: true).
           즉 "장애물을 다시 본다" = "이 함수를 다시 부른다" 이다.
        """
        g = self._build_request(goal, start_pos, start_vel)
        t0 = time.time()
        self.stat_plans += 1

        send_fut = self._plan_cli.send_goal_async(g)
        if not self._wait(send_fut, timeout):
            self.get_logger().warn('계획 goal 전송 타임아웃')
            self.stat_plan_fail += 1
            return None
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().warn('move_group 이 계획 요청을 거부했다')
            self.stat_plan_fail += 1
            return None

        res_fut = handle.get_result_async()
        if not self._wait(res_fut, timeout):
            self.get_logger().warn('계획 결과 타임아웃')
            self.stat_plan_fail += 1
            return None

        res = res_fut.result().result
        code = res.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.stat_plan_fail += 1
            hint = ''
            # 🔴 0 은 오타가 아니다. cumotion_planner.py:790 이 매핑 안 된 cuRobo status
            #    (TRAJOPT_FAIL 등)에 대해 빈 MoveGroup.Result() 를 그대로 돌려주기 때문에
            #    val 이 0 으로 남는다. MoveItErrorCodes.msg 에 0 상수가 없어서 err_name() 이
            #    이름을 못 붙인다 — 그런데 **cuMotion 의 가장 흔한 실패가 이것이다.**
            if code == 0:
                hint = (' → cuRobo 가 궤적을 못 찾았다(코드 미매핑). 진짜 이유는 T6 로그의 '
                        '"Motion planning failed wih status:" 줄에 있다. '
                        'max_attempts / num_trajopt_seeds 를 올린다.')
            elif code == MoveItErrorCodes.INVALID_MOTION_PLAN:
                # 🔴 이건 플래너가 낸 코드가 아니다. cumotion_planner_node 는 이 값을 안 쓴다.
                #    MoveIt 이 **플래너가 준 궤적을 자기 planning scene 으로 재검증**해서 거부한 것이다
                #    (planning_pipeline.h 의 check_solution_paths_).
                #
                #    2026-08-07 실기 실측 (목표 [0.5,-0.4,1.2,0,0.9,0] rad):
                #      · cuMotion 7/10 → 8/10, OMPL 10/10
                #      · octomap:=false 로 내려도 그대로 발생 → **옥토맵은 원인이 아니다**(배제됨)
                #      · /check_state_validity 로 목표 자세 조회 → valid=True → **목표도 원인이 아니다**
                #      · 같은 시작·목표인데 산발적으로 실패 → cuMotion 이 시드마다 다른 궤적을 내고
                #        그중 일부의 **중간 지점**이 MoveIt 검사에 걸린다는 뜻이다.
                #
                #    유력 용의자: SRDF 와 XRDF 의 자기충돌 면제 목록이 다르다.
                #      XRDF(cuMotion): link_4 → [link_5, link_6, rg2_base_link]  ← 무시
                #      SRDF(MoveIt)  : link_4 → [link_3, link_5, link_6]         ← rg2_base_link 검사
                #    testcommand.md 12절이 "실기 모션 전 재검토 필수"로 이미 적어둔 그 항목이다.
                #    ⚠️ 미확정 — 실패한 궤적의 중간 자세를 실제로 검사해 봐야 한다.
                #
                #    루프 관점에서는 안전한 실패다: MoveIt 이 거부한 궤적은 우리에게 오지 않으므로
                #    JTC 로 나가지 않는다. 대신 재계획 성공률이 떨어져 회피 반응이 느려진다.
                hint = (' → MoveIt 이 궤적을 재검증해 거부했다(check_solution_paths). '
                        '옥토맵·목표자세는 실측으로 배제됨. SRDF/XRDF 자기충돌 면제 목록 불일치 의심 '
                        '(link_4 ↔ rg2_base_link).')
            elif code == MoveItErrorCodes.START_STATE_IN_COLLISION:
                # cuMotion 은 INVALID_START_STATE_{WORLD,SELF}_COLLISION 을 여기로 매핑한다
                # (cumotion_planner.py:800-806). 로그 문자열과 코드 이름이 다르니 헷갈리지 말 것.
                hint = (' → 로봇 몸이 nvblox 지도에 찍혔다. robot_segmenter_node(T4) 확인, '
                        'distance_threshold 를 올려본다.')
            elif code == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
                hint = (' → ESDF pull 실패(update_world_objects). T5 nvblox_node 가 죽었거나 '
                        'esdf_service_name 이 어긋났다. pgrep -f nvblox_node.')
            elif code == MoveItErrorCodes.NO_IK_SOLUTION:
                hint = ' → 목표 pose 의 IK 가 없다. 도달 범위 밖이거나 자세가 뒤집혔다.'
            elif code == MoveItErrorCodes.INVALID_LINK_NAME:
                hint = (f' → pose goal 의 link_name({self.cfg.ee_link}) 이 cuMotion 의 tool_frame'
                        '(XRDF tool_frames) 과 다르다. cumotion_planner.py:765')
            elif code == MoveItErrorCodes.FAILURE:
                hint = (' → cumotion_planner_node 가 이미 다른 계획을 처리 중이다(planner_busy, '
                        'cumotion_planner.py:637). RViz 가 동시에 Plan 을 던지고 있지 않은지 본다.')
            elif code == MoveItErrorCodes.PLANNING_FAILED:
                # 🔴 cuMotion 경로에서 이 코드는 **원인을 알려주지 않는다.**
                #    플러그인(cumotion_interface.cpp)이 실패 시 플래너의 진짜 error_code 를
                #    덮어쓰고 PLANNING_FAILED 로 고정한다. 그래서 T6 이 NO_IK_SOLUTION 을
                #    돌려줘도 우리에겐 -1 로 온다 (2026-08-07 실기에서 실제로 겪음:
                #    T6 로그는 IK_FAIL 인데 여기는 PLANNING_FAILED 였다).
                hint = (' → cuMotion 경로면 이 코드는 원인을 안 알려준다(플러그인이 덮어쓴다). '
                        '진짜 이유는 T6 로그의 "Motion planning failed wih status:" 줄에 있다. '
                        'IK_FAIL 이면 목표 pose 의 IK 해가 전부 ESDF 장애물과 충돌한다는 뜻 — '
                        'T6 grid_center_m/grid_size_m 와 T4 distance_threshold 를 본다.')
            elif code == MoveItErrorCodes.GOAL_IN_COLLISION:
                # ⚠️ cuMotion 경로에서는 안 나온다 — cumotion_planner_node 는 이 코드를 안 쓴다.
                #    pipeline_id:=ompl 일 때만 유효한 힌트다.
                hint = ' → (ompl 전용) 목표 자세가 장애물 안이다. 장애물이 치워질 때까지 못 간다.'
            self.get_logger().warn(f'계획 실패: {err_name(code)}{hint}')
            return None

        # ⚠️ MoveGroup.Result 의 필드명은 planned_trajectory 다 (trajectory 아님).
        #    executed_trajectory 는 plan_only=True 라 항상 비어 있다.
        traj = res.planned_trajectory.joint_trajectory
        dt_ms = (time.time() - t0) * 1000.0
        self.stat_plan_ms.append(dt_ms)
        if not traj.points:
            self.get_logger().warn('계획은 성공인데 궤적이 비었다')
            return None
        self.get_logger().debug(
            f'계획 OK {dt_ms:.0f} ms, {len(traj.points)} pts, {self.duration(traj):.2f} s')
        return traj

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        """백그라운드 executor 가 도는 상태에서 future 를 폴링으로 기다린다."""
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.002)
        return True

    # ── 실행 ────────────────────────────────────────────────────────────────
    def send_trajectory(self, traj: JointTrajectory, handover_s: float = 0.05):
        """JTC 로 궤적을 보낸다. 이미 실행 중인 goal 은 JTC 가 알아서 선점 교체한다.

        handover_s: 궤적 전체를 이만큼 뒤로 민다. JTC 는 첫 점까지 현재 실측값에서
                    보간해 들어가므로, 예측 시작상태와 실제 위치가 조금 어긋나도
                    계단식 점프 대신 부드럽게 올라탄다. 0 으로 두면 즉시 점프한다.
        """
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

    def _reorder(self, traj: JointTrajectory) -> JointTrajectory:
        """궤적의 관절 순서를 컨트롤러가 기대하는 cfg.joint_names 순서로 맞춘다."""
        if list(traj.joint_names) == list(self.cfg.joint_names):
            return traj
        idx = [traj.joint_names.index(j) for j in self.cfg.joint_names]
        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(self.cfg.joint_names)
        for p in traj.points:
            q = JointTrajectoryPoint()
            q.positions = [p.positions[i] for i in idx] if p.positions else []
            q.velocities = [p.velocities[i] for i in idx] if p.velocities else []
            q.accelerations = [p.accelerations[i] for i in idx] if p.accelerations else []
            q.time_from_start = p.time_from_start
            out.points.append(q)
        return out

    @staticmethod
    def _shift_time(traj: JointTrajectory, dt: float) -> JointTrajectory:
        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(traj.joint_names)
        for p in traj.points:
            q = JointTrajectoryPoint()
            q.positions = list(p.positions)
            q.velocities = list(p.velocities)
            q.accelerations = list(p.accelerations)
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 + dt
            q.time_from_start.sec = int(t)
            q.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            out.points.append(q)
        return out

    @staticmethod
    def sample(traj: JointTrajectory, t: float) -> Tuple[List[float], List[float]]:
        """궤적을 시각 t(초, 궤적 시작 기준)에서 선형보간해 (position, velocity) 반환."""
        pts = traj.points

        def tfs(p):
            return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9

        if t <= tfs(pts[0]):
            return list(pts[0].positions), list(pts[0].velocities or [0.0] * len(pts[0].positions))
        if t >= tfs(pts[-1]):
            return list(pts[-1].positions), [0.0] * len(pts[-1].positions)

        for i in range(len(pts) - 1):
            t0, t1 = tfs(pts[i]), tfs(pts[i + 1])
            if t0 <= t <= t1:
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                p0, p1 = pts[i].positions, pts[i + 1].positions
                pos = [p0[k] + a * (p1[k] - p0[k]) for k in range(len(p0))]
                v0 = pts[i].velocities or [0.0] * len(p0)
                v1 = pts[i + 1].velocities or [0.0] * len(p0)
                vel = [v0[k] + a * (v1[k] - v0[k]) for k in range(len(p0))]
                return pos, vel
        return list(pts[-1].positions), [0.0] * len(pts[-1].positions)

    @staticmethod
    def duration(traj: JointTrajectory) -> float:
        p = traj.points[-1].time_from_start
        return p.sec + p.nanosec * 1e-9

    # ── 🔴 교체 판정 ────────────────────────────────────────────────────────
    def _same_path(self, cur_traj: JointTrajectory, cur_t0: float,
                   new_traj: JointTrajectory, handover_s: float,
                   threshold: float, n_samples: int = 8) -> bool:
        """새 궤적이 지금 실행 중인 궤적의 **남은 부분**과 사실상 같은가.

        🔴 이 판정이 루프의 생사를 가른다.
            cuMotion 은 매 계획마다 **정지 상태에서 출발하는 완결된** point-to-point 궤적을 준다.
            장애물이 안 변했으면 그건 지금 달리는 궤적과 같은 경로를 처음부터 다시 시작하는
            것뿐이다. 그걸 교체하면 로봇은 영원히 가속 램프에만 머문다.

            2026-08-07 실기 실측 (목표 45°, vel 0.15, 장애물 없음):
              static(재계획 OFF) → 계획 1회, **7.7초 만에 도착**
              3 Hz 재계획       → 교체 155회, **60초 타임아웃까지 도착 실패**
                                  (교체 순간 실측속도가 끝까지 0.000~0.037 rad/s — 속도가 붙질 못함)
            궤적이 7.7초인데 0.33초마다 갈아끼우니 매번 앞 4%(가속 램프)만 실행하고 버린 것이다.

        그래서 **다를 때만** 교체한다. 계획 자체는 3 Hz 로 계속 던지므로 장애물 감시는 그대로다
        (plan() 호출이 곧 ESDF 재조회다 — 2절).

        비교 방법: 새 궤적은 handover_s 뒤부터 시작하므로, 그 시점의 현재 궤적 위 위치를
        기준점으로 잡고 두 궤적을 같은 경과시간에서 샘플해 관절 최대차를 본다.
        """
        cur_traj = self._reorder(cur_traj)
        new_traj = self._reorder(new_traj)

        base = (time.time() + handover_s) - cur_t0   # 인계 시점의 현재 궤적 위 경과시간
        old_left = self.duration(cur_traj) - base
        span = min(self.duration(new_traj), old_left)
        if span <= 0.0:
            return False        # 현재 궤적이 끝났거나 끝나간다 → 무조건 교체해야 한다

        for k in range(n_samples + 1):
            dt = span * k / n_samples
            p_new, _ = self.sample(new_traj, dt)
            p_old, _ = self.sample(cur_traj, base + dt)
            if max(abs(p_new[i] - p_old[i]) for i in range(len(p_new))) > threshold:
                return False
        return True

    # ── 정지 ────────────────────────────────────────────────────────────────
    def brake(self, brake_time: float = 0.3) -> None:
        """현재 속도에서 0 까지 등감속으로 세우는 짧은 궤적을 쏜다.

        goal 을 그냥 cancel 하면 JTC 가 그 순간 위치를 홀드해서 급정지가 된다.
        속도가 붙어 있을수록 충격이 크므로, 정상 종료 경로에서는 이쪽을 쓴다.
        """
        pos, vel = self.current_state()
        end = [pos[i] + vel[i] * brake_time * 0.5 for i in range(len(pos))]

        traj = JointTrajectory()
        traj.joint_names = list(self.cfg.joint_names)
        p = JointTrajectoryPoint()
        p.positions = end
        p.velocities = [0.0] * len(end)
        p.time_from_start.sec = int(brake_time)
        p.time_from_start.nanosec = int(round((brake_time - int(brake_time)) * 1e9))
        traj.points = [p]
        self.send_trajectory(traj, handover_s=0.0)
        time.sleep(brake_time + 0.1)

    def cancel_execution(self) -> None:
        if self._active_exec_handle is not None:
            try:
                self._active_exec_handle.cancel_goal_async()
            except Exception:
                pass
            self._active_exec_handle = None

    def emergency_stop(self, mode: int = DR_SSTOP) -> bool:
        """두산 컨트롤러에 직접 정지를 건다. 실행 액션이 안 먹을 때의 최후 수단."""
        if self._stop_cli is None:
            self.get_logger().error('dsr_msgs2 없음 — emergency_stop 사용 불가')
            return False
        if not self._stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{self.cfg.stop_service} 서비스 없음')
            return False
        req = MoveStop.Request()
        req.stop_mode = mode
        fut = self._stop_cli.call_async(req)
        if not self._wait(fut, 2.0):
            return False
        return bool(fut.result().success)

    # ── 🔴 핵심: 재계획 루프 ─────────────────────────────────────────────────
    def run_to_goal(self,
                    goal: Constraints,
                    goal_positions: Optional[Sequence[float]] = None,
                    replan_hz: float = 3.0,
                    lookahead_s: float = 0.35,
                    handover_s: float = 0.05,
                    max_start_jump: float = 0.25,
                    max_consecutive_failures: int = 4,
                    timeout_s: float = 60.0,
                    static_mode: bool = False) -> bool:
        """목표까지 이동한다. 이동하는 내내 계획을 다시 던져 궤적을 갈아끼운다.

        파라미터
        --------
        replan_hz : 재계획 주파수.
            ⚠️ 3 Hz 위로 올려도 **새 정보가 없다.** robot_segmenter_node 가 3.7 Hz 라
            nvblox 지도 자체가 그 속도로만 갱신된다(config/README.md 병목 항목).
            대신 GPU 부하와 move_group 큐만 늘어난다.
        lookahead_s : "지금"이 아니라 "지금+lookahead" 시점의 궤적 위 상태에서 계획을 시작한다.
            계획에 wall 200 ms 쯤 걸리므로(testcommand.md 실측 204 ms), 현재 상태에서
            계획하면 결과가 나올 땐 로봇이 이미 그 지점을 지나쳐 있다 → 인계 시 점프.
            lookahead 는 **계획시간 + handover 보다 커야** 한다. 0.35 는 그 여유값.
            🔴 이건 **위치**만 맞춰준다. cuMotion 이 시작 velocity 를 버리기 때문에
               (_build_request 주석) 새 궤적의 첫 점은 언제나 속도 0 이고, 교체 지점의
               속도 불연속은 lookahead 를 아무리 키워도 남는다. 이건 튜닝이 아니라 플래너의
               성질이다. 완화 수단은 handover_s ↑ / replan_hz ↓ / vel_scale ↓ 뿐이고,
               실제 크기는 교체 로그와 summary() 의 "이음새" 값으로 본다.
        max_start_jump : 새 궤적 첫 점이 실측 관절값에서 이만큼(rad) 넘게 떨어져 있으면 버린다.
            예측이 빗나갔다는 뜻이라 그대로 쏘면 급격한 점프가 된다.
        static_mode : True 면 최초 1회만 계획하고 재계획하지 않는다.
            **동적 회피가 실제로 뭘 바꾸는지 비교하는 대조군용**이다.
        """
        cfg = self.cfg
        period = 1.0 / max(0.1, replan_hz)

        # ── 1) 최초 계획: 정지 상태에서, 현재 위치로부터 ──
        pos, _ = self.current_state()
        traj = self.plan(goal, start_pos=pos, start_vel=[0.0] * len(pos))
        if traj is None:
            self.get_logger().error('최초 계획 실패 — 시작하지 않는다')
            return False

        # 복셀 감시는 planner 가 한 번 발행한 뒤라야 토픽 타입이 잡힌다 → 첫 계획 뒤에 건다.
        self.start_voxel_monitor()

        if self.send_trajectory(traj, handover_s=handover_s) is None:
            return False

        exec_t0 = time.time()          # 현재 실행 중인 궤적이 시작된 시각
        traj_dur = self.duration(traj) + handover_s
        loop_t0 = time.time()
        fails = 0
        next_plan = time.time() + period

        try:
            while rclpy.ok():
                now = time.time()

                # ── 도착 판정 ──
                if goal_positions is not None:
                    cur, _ = self.current_state()
                    err = max(abs(cur[i] - goal_positions[i]) for i in range(len(cur)))
                    if err < cfg.joint_goal_tol:
                        self.get_logger().info(f'목표 도착 (최대오차 {err:.4f} rad)')
                        return True
                elif now - exec_t0 > traj_dur + 0.5:
                    # pose goal 이라 관절 목표가 없으면 궤적을 끝까지 실행한 것으로 판정
                    self.get_logger().info('궤적 실행 완료')
                    return True

                if now - loop_t0 > timeout_s:
                    self.get_logger().error(f'타임아웃 {timeout_s:.0f}s — 정지')
                    self.brake()
                    return False

                # 궤적이 끝났는데 목표엔 아직 못 갔으면 다시 계획해야 한다
                traj_finished = (now - exec_t0) > traj_dur

                if static_mode and not traj_finished:
                    time.sleep(0.02)
                    continue

                if now < next_plan and not traj_finished:
                    time.sleep(0.005)
                    continue
                next_plan = now + period

                # ── 2) 인계 시점의 시작 상태를 궤적에서 뽑는다 ──
                t_on_traj = (now - exec_t0) + lookahead_s
                if traj_finished or t_on_traj >= traj_dur:
                    # 이미 끝났거나 곧 끝난다 → 실측 정지 상태에서 계획
                    s_pos, _ = self.current_state()
                    s_vel = [0.0] * len(s_pos)
                else:
                    s_pos, s_vel = self.sample(traj, max(0.0, t_on_traj - handover_s))

                # ── 3) 계획 재발행 = ESDF 재조회 = 새 장애물 반영 ──
                new_traj = self.plan(goal, start_pos=s_pos, start_vel=s_vel)
                if new_traj is None:
                    fails += 1
                    if fails >= max_consecutive_failures:
                        self.get_logger().error(
                            f'계획 {fails}회 연속 실패 — 감속 정지. '
                            '장애물이 목표나 로봇을 덮고 있을 가능성이 크다.')
                        self.brake()
                        return False
                    continue
                fails = 0

                # 새 궤적이 사실상 길이 0 이면 이미 목표에 있다는 뜻이다.
                # pose goal 처럼 관절 목표가 없어 도착 판정을 못 하는 경우, 이게 없으면
                # 짧은 궤적을 무한히 다시 쏘며 제자리에서 맴돈다.
                if self.duration(new_traj) < 0.05:
                    self.get_logger().info('잔여 궤적 ≈ 0 — 목표 도달로 판정')
                    return True

                # ── 4) 안전 검사: 예측이 크게 빗나갔으면 버린다 ──
                cur, cur_vel = self.current_state()
                first = self._reorder(new_traj).points[0].positions
                jump = max(abs(first[i] - cur[i]) for i in range(len(cur)))
                if jump > max_start_jump:
                    self.get_logger().warn(
                        f'새 궤적 시작점이 실측과 {jump:.3f} rad 어긋남 (>{max_start_jump}) — 폐기. '
                        'lookahead_s 를 늘리거나 vel_scale 을 낮춘다.')
                    continue

                # ── 5) 🔴 달라졌을 때만 교체한다 ──
                #    장애물이 안 변했으면 새 궤적은 같은 경로를 v=0 에서 다시 시작하는 것뿐이라,
                #    교체하면 로봇이 가속 램프에서 못 벗어난다(_same_path 주석의 실측).
                #    현재 궤적이 끝나가는 경우(traj_finished)는 반드시 교체해야 하므로 제외한다.
                if not traj_finished and self._same_path(
                        traj, exec_t0, new_traj, handover_s, cfg.swap_threshold_rad):
                    self.stat_skips += 1
                    continue

                # ── 6) 교체. JTC 가 기존 goal 을 선점한다 ──
                if self.send_trajectory(new_traj, handover_s=handover_s) is not None:
                    traj = new_traj
                    exec_t0 = time.time()
                    traj_dur = self.duration(new_traj) + handover_s
                    self.stat_swaps += 1
                    # 새 궤적 첫 점의 속도는 항상 0 이므로(_build_request 주석), 교체 순간의
                    # 실측 속도가 곧 인계 불연속의 크기다. 덜컹임을 눈이 아니라 숫자로 본다.
                    seam = max(abs(v) for v in cur_vel) if cur_vel else 0.0
                    self.stat_seam_vel.append(seam)
                    # 🔴 교체할 때마다 cuMotion 이 그 계획에서 실제로 본 장애물 수를 남긴다.
                    #    사람이 손을 넣었는데 이 숫자가 안 변하면 nvblox 경로가 죽은 것이다.
                    if self.last_voxel_count == 0:
                        self.get_logger().warn(
                            f'교체 #{self.stat_swaps} | 이음새 {seam:.3f} rad/s | 복셀 0개 — '
                            'nvblox 는 살아 있어도 지도가 비었다. 카메라 FOV / workspace_bounds 확인.')
                    else:
                        self.get_logger().info(
                            f'교체 #{self.stat_swaps} | 이음새 {seam:.3f} rad/s | '
                            f'{self.voxel_report()}')

        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl+C — 감속 정지')
            self.brake()
            raise
        return False

    def summary(self) -> str:
        n = len(self.stat_plan_ms)
        avg = sum(self.stat_plan_ms) / n if n else 0.0
        mx = max(self.stat_plan_ms) if n else 0.0
        seam = f'{max(self.stat_seam_vel):.3f}' if self.stat_seam_vel else '-'
        return (f'계획 {self.stat_plans}회 (실패 {self.stat_plan_fail}) / '
                f'궤적 교체 {self.stat_swaps}회 (동일해서 생략 {self.stat_skips}회) / '
                f'계획시간 평균 {avg:.0f} ms, 최대 {mx:.0f} ms / '
                f'이음새 최대 {seam} rad/s / '
                f'마지막 {self.voxel_report()}')
