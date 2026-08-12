#!/usr/bin/env python3
"""음성 지시 pick 상태머신.

    ros2 launch pick_fsm pick_fsm.launch.py
    ros2 service call /pick/start   std_srvs/srv/Trigger {}
    ros2 service call /pick/approve std_srvs/srv/Trigger {}     # ✋ 실행 승인

설계 출처: src/PACKAGES.md#pick_fsm (노드 그래프 · 인터페이스 계약 · §1 상태머신)
(옛 설계 문서는 2026-08-08 ws-cleanup 으로 삭제됐다 — PACKAGES.md#pick_fsm 이 단일 출처.)

이 노드가 존재하는 이유는 하나다: **로봇 명령 경로가 두 개 살아 있기 때문이다.**
`dsr_controller2`(서비스 movej/movel)와 `dsr_moveit_controller`(JTC)에 동시에 명령하면
안 된다. 그 배타권을 한 노드가 소유해야 하고, 이 노드가 그 자리다.
→ 그래서 이 노드는 **MoveIt 경로만** 쓴다. DSR_ROBOT2 의 movej/movel 을 부르지 않는다.

⚠️ **이 노드는 항상 실행한다.** `dry_run`(plan_only) 파라미터는 2026-08-09 제거했다 —
   실기 모션 데이터 수집 단계로 넘어갔고, 팔이 안 움직이는데 그리퍼만 실제로 개폐되는
   반쪽 안전(`_move()`만 게이트되고 `rg2.*`는 안 됨)이 오히려 오해를 낳았다.
   🔴 `require_approval` 기본값은 2026-08-11 사용자 결정으로 **false 로 뒤집혔다**
   (launch·yaml 둘 다). 승인 게이트가 꺼진 지금 남은 실기 안전장치는 **물리 비상정지
   버튼 하나뿐**이다. WAIT_APPROVAL 은 여전히 상태로 남아 있고(관측용), require_approval
   이 false 면 `_st_wait_approval` 이 한 tick 만에 STOW 로 넘어간다. 다시 켜려면
   `require_approval:=true`.
"""

import json
import threading

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from pick_fsm import geometry as geo
from pick_fsm.moveit_bridge import SUCCESS, MoveItBridge, err_name, merge_acm
from pick_fsm.rg2 import (
    RG2_MODEL_WIDTH_M, Rg2Client, fingertip_from_rg2_base_m, grip_target_width_m,
)
from pick_fsm.robot_safety_node import UNSAFE_STATES
from pick_fsm.states import HOLDING_STATES, PAUSE_EXEMPT, State, is_allowed

try:                                    # pick_fsm_msgs 가 없어도 legacy/manual 경로는 돌게 한다
    from pick_fsm_msgs.srv import ComputeGrasp
except ImportError:                     # pragma: no cover
    ComputeGrasp = None

#: 상태별 제한시간 [s]. 넘으면 ABORT. 사람 입력을 기다리는 상태는 여기 없다.
DEFAULT_TIMEOUTS = {
    State.LISTENING: 110.0,     # voice_processing.wait_timeout_sec(100s) + margin(5s) 여유
    State.PERCEIVE: 120.0,      # GPU 추론 + 모델 로딩(첫 호출 수십 초)
    State.SCENE_PREP: 10.0,
    State.PLAN: 30.0,
    State.STOW: 20.0,
    State.APPROACH: 180.0,      # replan 루프가 도는 구간이라 넉넉히
    State.OPEN_GRIPPER: 20.0,
    State.DESCEND: 120.0,
    State.CLOSE: 20.0,
    State.VERIFY: 10.0,
    State.RELEASE_RETRY: 20.0,
    State.LIFT: 120.0,
    State.PLACE: 180.0,
    State.RELEASE: 20.0,
    State.HOME: 180.0,
}

#: SPEAK_FAIL -> LISTENING 을 몇 번 연속으로 돌면 IDLE 로 내려앉는지.
#: 이 왕복은 매번 `_to()` 가 `_entered` 를 리셋해서 LISTENING 제한시간이 영원히 안 걸린다 —
#: 멈추는 조건을 따로 두지 않으면 tick 주기로 무한히 돈다(2026-08-07 실기 로그 폭주).
MAX_FAIL_STREAK = 3

#: `/pick/target`(지시) · `/pick/target_active`(현재값) 용.
#: TRANSIENT_LOCAL 이라야 **늦게 뜨는 쪽**이 마지막 값을 받는다 — rqt 패널은 FSM 과 따로
#: 껐다 켜므로, VOLATILE 이면 패널을 다시 띄울 때마다 타겟 표시가 비어 보이고 사람이
#: "타겟이 풀렸나?" 하고 다시 누르게 된다. 양쪽 다 이 프로파일을 써야 한다(durability 가
#: 어긋나면 아예 연결이 안 된다) — 그래서 여기 한 곳에 두고 rqt_panel 이 import 한다.
TARGET_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: `/pick/place_location` 로 고를 수 있는 값 -> 실제로 쓸 joint 파라미터 이름.
#: '내려놓는 위치' 세 경우(장바구니/작업테이블 지정 자리/작업테이블 바깥 폐기)를
#: 관절각 프리셋 세 개로 나눈다 — grasp pose 처럼 좌표를 계산하지 않고 home/place 와
#: 같은 고정 관절이동이라 IK 가 필요 없다(_joint_move 재사용).
PLACE_LOCATIONS = {
    'basket': 'place_joints_deg',
    'table': 'place_table_joints_deg',
    'discard': 'place_discard_joints_deg',
}


def str_param(name: str, value: str) -> Parameter:
    """rcl_interfaces 문자열 파라미터 하나. `SetParameters` 요청에 넣는다."""
    return Parameter(name=name,
                     value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                          string_value=str(value)))


def float_param(name: str, value: float) -> Parameter:
    """rcl_interfaces DOUBLE 파라미터 하나. `select_by_point` 픽셀 좌표 푸시에 쓴다."""
    return Parameter(name=name,
                     value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                          double_value=float(value)))


#: 파라미터 기본값 = 타입의 정본. `config/pick_fsm.yaml` 의 값은 여기 적힌 타입과
#: 같아야 한다 (float 는 `0` 이 아니라 `0.0`). test_pick_fsm.py 가 이 대조를 자동화한다.
#: ⚠️ 런타임 값의 정본은 `config/pick_fsm.yaml` 이다. 아래 기본값은 yaml 없이 뜰 때만
#:    쓰이며, 일부(home/place 관절각·max_reach 등)는 yaml 의 실기 실측/재-teach 값과
#:    **의도적으로 다르다** — 값을 여기서 yaml 에 맞추려 하지 말 것(2026-08-10 code-audit).
PARAM_DEFAULTS = {
    # 안전
    # `dry_run`(plan_only) 은 없다 — 2026-08-09 제거. 모듈 docstring 참고.
    'require_approval': True,        # false 로 두면 사람 승인 없이 실행한다
    'approval_timeout_sec': 300.0,
    # WAIT_PLACE_TARGET(들어올린 뒤 place 미지정 대기)의 자동 내려놓기 타임아웃.
    # 🔴 **0.0 = 비활성이고 그게 기본이다** (2026-08-12 사용자 결정이 2026-08-11 결정을
    # 뒤집었다). 0 보다 크면 그 시간 뒤 파라미터 기본 위치(place_location)로 자동 이동해
    # 내려놓는다 — 이것이 "시간 경과만으로 팔이 움직이는" 유일한 경로였다.
    # `grip_narrow_retries`/`force_down_steps` 와 같은 "0 = 끔" 관례. 옛 동작이 필요하면
    # 코드가 아니라 yaml 로 되돌린다.
    'wait_place_timeout_sec': 0.0,
    # eye-in-hand 재파지 (스캐폴드, 2026-08-11). true 면 pre-grasp 도착 후 REGRASP 를 거친다
    # — 손목 카메라로 재-graspgenx 하고 사람이 승인. 🔴 카메라·hand-eye 캘리브가 아직 없어
    # 실제 재계산은 미구현(_st_regrasp 의 HOOK 참고). 기본 false = 지금 흐름 그대로.
    'regrasp_enabled': False,
    'regrasp_timeout_sec': 300.0,    # REGRASP 승인 대기 제한(초). 넘으면 ABORT

    # MoveIt
    'planning_group': 'manipulator',
    # 🔴 `tool0` 이 아니다 (2026-08-07 정정). tool0 의 접근축은 +Z 가 아니라 +X 라
    #    (`onrobot_rg2.xacro:40` rpy="1.5708 0 1.5708"), grasp 포즈를 tool0 에
    #    그대로 걸면 그리퍼가 90° 누운 채 진입한다. `rg2_base_link` 는 GraspGenX 의
    #    그리퍼 base 와 같은 프레임이라 브라켓 22 mm 오프셋도 같이 해소된다.
    #    MoveIt 은 solver tip(tool0)에 고정조인트로 붙은 링크를 ik_link 로 받는다.
    'ee_link': 'rg2_base_link',
    'base_frame': 'base_link',       # ⚠️ world 아님. planning scene 이 world 를 모른다
    'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
    'vel_scale': 0.1,                # 실기 첫 시도는 느리게
    'acc_scale': 0.1,
    'planning_time': 5.0,
    'planning_attempts': 10,
    'joint_tolerance': 0.001,
    'ik_timeout_sec': 0.2,
    'ik_avoid_collisions': True,
    # 'ompl' | 'isaac_ros_cumotion' (scripts/bench_planning_time.py 와 같은 이름).
    # IK 는 이 값과 무관 — move_group 의 GetPositionIK 는 파이프라인을 안 탄다.
    'planning_pipeline': 'ompl',
    'planner_id': '',
    'replan': True,                  # 실행 중 씬이 바뀌면 move_group 이 다시 계획한다
    'replan_attempts': 3,
    'replan_delay': 0.5,
    'motion_retries': 2,             # move_group 실패 시 FSM 바깥 재시도 횟수
    # PERCEIVE(재)촬영 실패 시 재시도 횟수. 2026-08-09 실기 로그: 정지된 같은 물체·같은
    # 자리에서 collision-free 비율이 0%~53%로 요동쳤다(depth 노이즈로 OBB 후보 자체가
    # 흔들린다) — 재촬영 한 번으로 5번 중 4번은 살아났다. `motion_retries` 와 같은 뼈대.
    'perceive_retries': 2,

    # 자세
    'approach_offset_m': 0.15,       # pre-grasp: grasp 의 -Z 로 물러나는 거리
    'grasp_standoff_m': 0.0,         # DESCEND 종점을 grasp 의 -Z 로 덜 내리는 양
    'lift_offset_m': 0.15,           # LIFT: 월드 +Z
    # tcp_offset_m 은 더 이상 파라미터가 아니다 — rg2.fingertip_length_m(width_m)이
    # 2026-08-07 실측(폭에 따라 손끝이 짧아지는 비선형 보정표)으로 대체했다.
    'max_reach_m': 0.900,            # M0609 URDF 실측 (shoulder 기준)
    'home_joints_deg': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],     # robot_control JReady
    'place_joints_deg': [4.0, 38.0, 64.0, -0.1, 78.0, 4.0],  # robot_control BUCKET_POS ('basket')
    # 2026-08-11 실기 teach 완료 — 값은 pick_fsm.yaml(정본)의 교시값과 맞춰 둔다. yaml 이
    # 런타임엔 이 기본값을 이기지만, yaml 없이 떴을 때의 fallback 도 안전한 자세라야 한다.
    # 🔴 관절값은 교시됐으나 pick_fsm 통합 사이클은 아직 미검증(yaml 주석 참고).
    'place_table_joints_deg': [-130.07, -1.77, 119.41, 0.0, 62.36, 68.8],
    'place_discard_joints_deg': [37.0, 42.0, 47.0, 180.0, -92.0, 40.0],
    # PLACE_LOCATIONS 의 키 중 하나('basket'|'table'|'discard'). 런타임엔 /pick/place_location
    # (rqt 패널)이 이 값을 이긴다 — /pick/target 과 같은 패턴.
    'place_location': 'basket',

    # 씬
    'object_id': 'pick_target',
    'object_radius_m': 0.04,
    'clear_octomap_before_descend': False,
    # 2026-08-11 실기 확인: table/discard place 목표 자세가 PICK 단계에서 찍힌 옥토맵
    # 포인트(테이블 표면)와 거의 항상 겹친다 — "물체를 표면에 내려놓는 자세"는 정의상
    # 그 표면의 옥토맵 포인트에 붙어 있을 수밖에 없다. move_group 로그로 확인:
    # `Found a contact between '<octomap>' and 'rg2_base_link'` → RRTConnect가
    # goal state를 아예 샘플 못 해 매번 FAILURE(99999). 이 ws에서는 기본 True로 켠다.
    'clear_octomap_before_place': True,
    'allow_gripper_octomap_collision': False,
    'gripper_links': ['rg2_base_link',
                      'rg2_left_outer_knuckle', 'rg2_left_inner_knuckle',
                      'rg2_left_inner_finger', 'rg2_right_outer_knuckle',
                      'rg2_right_inner_knuckle', 'rg2_right_inner_finger'],

    # 그리퍼
    'gripper_backend': 'real',       # real | virtual (숫자 명령의 의미가 다르다)
    'gripper_service': '/onrobot/sendCommand',
    'grip_detected_topic': '/onrobot/grip_detected',
    # 물체 폭에서 **빼는** 조임 여유 [m]. 목표 개구 = 물체 폭 - 이 값 (`_grip_width`).
    'grip_clearance_m': 0.008,       # UNVERIFIED: 실측 튜닝값. 도면값 아님
    'max_grip_width_m': RG2_MODEL_WIDTH_M,
    'force_down_steps': 0,           # 'd' 반복 횟수. 0 = 드라이버 기본(=40 N, RG2 최대)
    'gripper_settle_sec': 1.5,
    'verify_required': False,        # true 면 grip_detected 를 못 받았을 때도 실패 처리
    'grip_retries': 1,
    # 파지 실패 시 RELEASE_RETRY(놓고 재인식)로 가기 전에, 같은 자세에서 이만큼씩
    # 좁혀 다시 닫아보는 횟수. width_m 은 GraspGenX 가 고른 grasp 의 폭이라 병처럼
    # 단면이 급변하는 물체에서는 "최대 폭"에 가깝고, 그보다 얇은 부위(목)를 잡으려 하면
    # 손가락이 접촉 전에 멈춰 힘이 안 걸린다(grip_detected=False, 물체는 그대로 있는데
    # "실패"로 오판). 0 이면 이 재시도를 끈다.
    'grip_narrow_retries': 1,
    'grip_narrow_step_m': 0.015,     # UNVERIFIED: 실측 튜닝값. 좁힐 때마다 이만큼 뺀다

    # 인식
    # `/grasp/compute_grasp`(pick_fsm_msgs/ComputeGrasp) 서버는 2026-08-09 에
    # grasp_bridge_node 에 생겼다 — 그전까지 이 ws 어디에도 없어서 기본값이 legacy_trigger 다.
    # 🔴 **폭(width_m)은 compute_grasp 경로로만 온다.** legacy_trigger 는 std_srvs/Trigger 라
    #    응답에 폭을 담을 필드가 없어 `default_width_m`(UNVERIFIED 상수)로 전부 때운다.
    #    물체마다 폭을 맞추려면 `grasp_source:=compute_grasp` 로 바꿔야 한다 — 단
    #    **실기 미검증이다**(폭 측정·조임 여유 부호 둘 다 2026-08-09 신규).
    'grasp_source': 'legacy_trigger',  # legacy_trigger | compute_grasp | manual
    'grasp_service': '/grasp/compute_grasp',
    'grasp_trigger_service': '/grasp/compute',
    'grasp_best_topic': '/grasp/best',
    'grasp_candidates_topic': '/grasp/candidates',
    'min_confidence': 0.5,
    'default_width_m': 0.06,         # legacy/manual 경로에는 폭 정보가 없다
    'max_alternatives': 5,

    # 인식 브리지 파라미터 푸시 (PERCEIVE 진입 때 1회)
    # 타겟의 정본은 **이 FSM** 이고, 브리지는 그 값을 받아 쓰는 쪽이다. 이 푸시가 없으면
    # FSM 의 target 과 브리지의 target_classes 가 각자 살아 어긋나도 아무도 모른다.
    'bridge_node': '/grasp_bridge_node',   # 비우면 푸시하지 않는다(브리지를 직접 설정할 때)
    # 이 ws 의 기본 파이프라인은 YOLO 세그다 — 클래스 이름으로 타겟을 고르려면 필수다.
    # `geometric` 은 클래스를 모르므로 타겟 지정이 불가능하다. 비우면 브리지 설정을 안 건드린다.
    'bridge_seg_source': 'yolo',           # yolo | geometric | '' (안 건드림)

    # 음성
    'voice_enabled': True,
    'keyword_service': '/get_keyword',
    # voice_enabled=false 일 때의 **초기** 타겟. 콤마로 여러 클래스도 된다('apple,orange').
    # 빈 문자열 = 자동(브리지가 본 것 전부에서 점수 최고). 런타임에는 `/pick/target`(String)
    # 이 이 값을 덮어쓴다 — rqt 패널의 타겟 상자가 그 토픽을 쏜다.
    'target': '',

    'tick_hz': 10.0,
}


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        p = self._declare_params()
        # ⚠️ 여기서 죽는 게 맞다 (vla_command_node.pixel_policy 검증과 같은 이유). 검증을
        # 안 하면 yaml 오타(`place_location: bakset`)가 조용히 통과해 rqt 패널엔 오타 그대로
        # 표시되면서 실제 이동은 _st_place 의 fallback 으로 basket 에 접힌다 — 표시값과
        # 실제 목적지가 갈라진다. `/pick/place_location`(토픽) 쪽은 _on_place_location 이
        # 따로 막는다(2026-08-10 cross-review 지적).
        if p['place_location'] not in PLACE_LOCATIONS:
            raise ValueError(
                f"place_location 파라미터 기본값이 잘못됐다: {p['place_location']!r} "
                f'(허용: {sorted(PLACE_LOCATIONS)})')
        cb = ReentrantCallbackGroup()

        self.moveit = MoveItBridge(self, cb, base_frame=p['base_frame'])
        self.rg2 = Rg2Client(self, cb, backend=p['gripper_backend'],
                             service=p['gripper_service'],
                             grip_topic=p['grip_detected_topic'])
        self.kw_cli = self.create_client(Trigger, p['keyword_service'], callback_group=cb)

        # ── grasp 공급원 ──────────────────────────────────
        # 문서의 정본 계약은 ComputeGrasp 다. 하지만 지금 실제로 도는 건
        # graspgenx_perception 의 grasp_bridge_node 가 내는 Trigger 경로다.
        # 둘 다 지원하지 않으면 이 FSM 은 "언젠가 돌 코드"가 된다.
        self.grasp_cli = None
        if p['grasp_source'] == 'compute_grasp':
            if ComputeGrasp is None:
                raise RuntimeError('grasp_source=compute_grasp 인데 pick_fsm_msgs import 실패')
            self.grasp_cli = self.create_client(ComputeGrasp, p['grasp_service'],
                                                callback_group=cb)
        elif p['grasp_source'] == 'legacy_trigger':
            self.grasp_cli = self.create_client(Trigger, p['grasp_trigger_service'],
                                                callback_group=cb)
        elif p['grasp_source'] != 'manual':
            raise ValueError(f"grasp_source 값이 이상하다: {p['grasp_source']!r}")

        # ── 브리지 파라미터 푸시 ──────────────────────────
        # 🔴 2026-08-09 실기: `pick_fsm.launch.py ... target_classes:=apple,orange,banana` 로
        #    띄웠는데 이 런치엔 그런 인자가 없어 **경고도 없이 무시**됐고, 정작 브리지에는
        #    이전 실행의 target_classes + seg_source=geometric 이 남아 있어서 매번
        #    "target_classes 는 seg_source='yolo' 에서만 쓴다" 로 실패했다.
        #    두 값이 각자 사는 한 같은 사고가 반복된다 → PERCEIVE 마다 여기서 밀어 넣는다.
        # manual 경로는 사람이 /grasp/best 를 직접 쏘므로 브리지를 안 건드린다.
        # 노드 이름은 **여기서 한 번만** 읽고 붙잡는다. 클라이언트는 기동 때의 값으로
        # 만들어지므로, 로그에서만 파라미터를 다시 읽으면 런타임에 그 값이
        # 바뀌었을 때 "실제로 설정한 노드"와 "메시지가 말하는 노드"가 갈라진다 —
        # 이번에 고친 사고(정본이 두 군데)와 같은 부류라 여기서 막는다.
        self._bridge_name = p['bridge_node']
        self.bridge_param_cli = None
        if self._bridge_name and p['grasp_source'] != 'manual':
            self.bridge_param_cli = self.create_client(
                SetParameters, self._bridge_name.rstrip('/') + '/set_parameters',
                callback_group=cb)

        self._best = None
        self._best_seq = 0
        self._seq_at_call = 0
        self._candidates = []
        self.create_subscription(PoseStamped, p['grasp_best_topic'], self._on_best, 10,
                                 callback_group=cb)
        self.create_subscription(PoseArray, p['grasp_candidates_topic'], self._on_candidates,
                                 10, callback_group=cb)
        # robot_safety_node 가 별도 프로세스로 Doosan 로봇상태를 폴링해 발행한다
        # (충돌 등으로 SAFE_STOP/EMERGENCY_STOP 에 들어가면 여기가 값을 받는다).
        # 이 노드는 안 떠 있어도 된다 — 그러면 그냥 이 감시 기능만 빠진다.
        self.create_subscription(Int8, '/pick/robot_state_code', self._on_robot_state, 10,
                                 callback_group=cb)

        # ── 관측·조작 인터페이스 ───────────────────────────
        self.state_pub = self.create_publisher(String, '/pick/state', 10)
        # 타겟은 세 군데서 들어온다: 파라미터(초기값) · 음성(LISTENING) · `/pick/target`(사람).
        # `_active` 는 그 결과를 되돌려주는 표시용이다 — 지시와 현재값을 같은 토픽에 섞으면
        # 패널이 자기가 쏜 값을 다시 받아 되먹임이 된다.
        self.target_pub = self.create_publisher(String, '/pick/target_active', TARGET_QOS)
        self.create_subscription(String, '/pick/target', self._on_target, TARGET_QOS,
                                 callback_group=cb)
        # 내려놓을 위치 — basket(장바구니)/table(작업테이블 지정 자리)/discard(테이블 밖 폐기).
        # /pick/target 과 똑같은 패턴: 사람이 못 바꾸면 파라미터 기본값을 쓴다.
        self.place_pub = self.create_publisher(String, '/pick/place_location_active', TARGET_QOS)
        self.create_subscription(String, '/pick/place_location', self._on_place_location,
                                 TARGET_QOS, callback_group=cb)
        # 이번 pick 이 place 를 지정 안 했는지(True=미지정). vla_command_node 가 pick 마다
        # 쏜다 — place_location(영구 설정)과 달리 "이 요청에 place 가 없었다"는 요청별
        # 사실이라 별도 채널이다. rqt/수동 경로는 이 토픽을 안 써서 항상 기존 동작(자동 place).
        self.create_subscription(Bool, '/pick/place_pending', self._on_place_pending,
                                 TARGET_QOS, callback_group=cb)
        # 개체 선정(select_by_point, 2026-08-11) — target/place 와 다른 성격이다: "현재
        # 설정값"이 아니라 **이번 한 사이클용 데이터**(그 순간 프레임의 픽셀)라 place 처럼
        # 계속 남아 있으면 다음 pick 이 다른 프레임의 좌표를 재사용해 엉뚱한 물체를
        # 가리킨다. 그래서 `_place_override`와 달리 **_push_bridge()가 브리지에 실어
        # 보낸 뒤 바로 지운다**(단발성 소비, 아래 `_pixel_override` 주석 참고).
        # payload: {"x":px,"y":py,"w":기준폭,"h":기준높이} — vla_command_node 가 채운다
        # (설계 출처: md/plans/2026-08-08-vla-integration.md §5, vla-bridge-contract.md §2).
        self.create_subscription(String, '/pick/target_pixel', self._on_target_pixel,
                                 TARGET_QOS, callback_group=cb)
        self.create_service(Trigger, '/pick/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/pick/approve', self._srv_approve, callback_group=cb)
        self.create_service(Trigger, '/pick/abort', self._srv_abort, callback_group=cb)
        self.create_service(Trigger, '/pick/reset', self._srv_reset, callback_group=cb)
        self.create_service(Trigger, '/pick/home', self._srv_home, callback_group=cb)
        self.create_service(Trigger, '/pick/retry_place', self._srv_retry_place,
                            callback_group=cb)
        self.create_service(Trigger, '/pick/release_now', self._srv_release_now,
                            callback_group=cb)
        self.create_service(Trigger, '/pick/pause', self._srv_pause, callback_group=cb)
        self.create_service(Trigger, '/pick/resume', self._srv_resume, callback_group=cb)
        self.create_service(Trigger, '/pick/stow', self._srv_stow, callback_group=cb)

        # ── FSM 내부 상태 ─────────────────────────────────
        self.state = State.IDLE
        self._entered = self.get_clock().now()
        self._fut = None            # 진행 중 서비스 future
        self._call = None           # 진행 중 액션 (ActionCall)
        self._extra = []            # 결과를 기다릴 필요 없는 부수 future (그리퍼 명령 등)
        self._start_req = False
        self._approved = False
        self._abort_req = None      # 사유 문자열
        # _abort_req 는 /pick/abort(서비스 콜백)와 _on_robot_state(구독 콜백) 양쪽에서
        # 쓰고 _tick(타이머 콜백)이 읽어서 비운다 — 셋 다 ReentrantCallbackGroup 이라 다른
        # 스레드에서 동시에 돌 수 있다. 락 없이 두면 robot_state 트리거가 조용히
        # 유실될 수 있다(2026-08-07 cross-review 지적).
        self._abort_lock = threading.Lock()
        # ✋ 일시정지. `_abort_req` 와 **같은 락**을 쓴다 — 둘은 경쟁 관계라(하드웨어
        # E-Stop 과 사람의 "멈춰"가 같은 순간에 올 수 있다) 따로 잠그면 _tick 이 한쪽만
        # 보고 다른 쪽을 덮어쓸 수 있다. 우선순위는 _tick 안에서 abort 먼저다.
        self._pause_req = None      # 사유 문자열
        self._paused_from = None    # 어디서 멈췄나 — resume 복귀 지점의 유일한 근거
        self._stow_req = False      # cmd:"stow" — 안전 종료 시퀀스 요청
        self._octomap_cleared = False
        self._acm = None
        self.target = ''
        self._target_override = None   # `/pick/target` 로 들어온 값. None = 파라미터를 쓴다
        self.place_location = ''
        self._place_override = None    # `/pick/place_location` 로 들어온 값. None = 파라미터를 쓴다
        # `/pick/place_pending` 로 들어온 "이번 요청은 place 미지정" 신호. `_st_idle` 이
        # 사이클 시작 때 `_wait_place` 로 latch(단발성)한다. True 면 LIFT 후 WAIT_PLACE_TARGET.
        self._place_pending_req = False
        self._wait_place = False       # 이번 사이클이 place 대기 경로인지 (idle 에서 확정)
        # `/pick/target_pixel` 로 들어온 (x,y,w,h). None = 지정 없음(점수 최고, 기존 동작).
        # place 와 달리 **단발성**이다 — `_push_bridge()`가 다음 PERCEIVE 에 실어 보내는
        # 순간 None 으로 되돌린다. 그대로 남겨두면 클래스만 지시한 다음 pick 이 이전
        # 프레임의 좌표를 재사용해 엉뚱한 물체를 고른다.
        self._pixel_override = None
        self._push_fut = None          # 브리지 SetParameters future (_fut 과 겹치면 안 된다)
        self._pushed = False           # 이번 PERCEIVE 에서 푸시를 끝냈는지
        # ── grasp 3층 (2026-08-12, 손목 eye-in-hand seam) ──────────────
        # 지금은 셋이 항상 같은 값이다. **동작은 완전히 동일하다.**
        # 나눠 둔 이유: 지금 구조는 DESCEND 가 최초 Top grasp 로 그대로 내려간다.
        # 손목 카메라가 붙은 뒤에 이걸 안 고치면 손목이 계산한 grasp 는 로그에만
        # 남고 로봇은 옛 자세로 내려간다 — 조용히 틀리는 종류라 그때 못 잡는다.
        # 그래서 "실제로 내려갈 자세"를 지금 미리 한 이름으로 분리해 둔다.
        self.global_grasp = None    # Top(고정) GraspGenX 가 준 것. 기록용 원본
        self.committed_grasp = None # 🔴 **모션이 실제로 쓰는 것.** 모든 하강·계획이 이걸 본다
        self.grasp_revision = 0     # committed_grasp 가 바뀔 때마다 +1 (IK 해 무효화 근거)
        self.width_m = 0.0
        self.alternatives = []
        self.alternative_widths = []   # alternatives 와 1:1. _st_next_candidate 가 같이 pop 한다
        self.poses = {}             # 'pre_grasp'|'grasp'|'lift' -> PoseStamped
        self.solutions = {}         # 같은 키 -> JointState
        self._plan_i = 0
        self._retry_motion = 0
        self._retry_grip = 0
        self._retry_narrow = 0      # VERIFY 실패 후 "좁게 재시도" 횟수. _st_descend 가 그랩마다 되돌린다
        self._retry_perceive = 0
        self._fail_streak = 0       # SPEAK_FAIL 연속 횟수. _st_idle 이 start 마다 0 으로 되돌린다
        self._object_added = False
        self._nag = 0
        self._home_next = State.IDLE   # HOME 도착 후 갈 곳. _srv_reset/_st_release_retry 가 덮어쓴다

        self.timer = self.create_timer(1.0 / p['tick_hz'], self._tick, callback_group=cb)
        self._publish_target(p['target'])
        self._publish_place(p['place_location'])
        self.get_logger().info(
            f"준비됨 — require_approval={p['require_approval']}, "
            f"grasp_source={p['grasp_source']}, gripper_backend={p['gripper_backend']}")
        self.get_logger().info(
            f"타겟='{p['target'] or '(자동)'}' — 바꾸려면 /pick/target (rqt 패널의 '타겟' 상자). "
            + (f"PERCEIVE 마다 {p['bridge_node']} 에 target_classes"
               + (f"+seg_source={p['bridge_seg_source']}" if p['bridge_seg_source'] else '')
               + ' 를 밀어 넣는다'
               if self.bridge_param_cli is not None else '브리지 푸시 없음(bridge_node 비어 있음)'))
        self.get_logger().warn(
            '⚠️ 계획만 하는 모드는 없다 — 로봇이 실제로 움직인다. 비상정지 버튼을 손에 둘 것')

    # ────────────────────────────────────────────────────────
    # 파라미터
    # ────────────────────────────────────────────────────────
    def _declare_params(self):
        for k, v in PARAM_DEFAULTS.items():
            self.declare_parameter(k, v)
        return {k: self.get_parameter(k).value for k in PARAM_DEFAULTS}

    def p(self, key):
        return self.get_parameter(key).value

    # ────────────────────────────────────────────────────────
    # 구독·서비스 콜백
    # ────────────────────────────────────────────────────────
    def _on_best(self, msg):
        self._best = msg
        self._best_seq += 1

    def _on_candidates(self, msg):
        self._candidates = [(msg.header, pose) for pose in msg.poses]

    def _on_robot_state(self, msg):
        """충돌 등으로 로봇이 자체적으로 안전정지에 들어가면 하던 작업을 즉시 ABORT.

        IDLE/ABORT/SAFE_STOP/SPEAK_FAIL 에서는 중단할 작업이 없으니 다시 안 건드린다 —
        안 그러면 이미 SAFE_STOP 인데 폴링될 때마다 로그만 쌓인다.
        """
        if int(msg.data) not in UNSAFE_STATES:
            return
        if self.state in (State.IDLE, State.ABORT, State.SAFE_STOP, State.SPEAK_FAIL):
            return
        with self._abort_lock:
            self._abort_req = f'로봇 안전정지 감지 (robot_state={int(msg.data)})'

    def _on_target(self, msg):
        """사람이 잡을 대상을 지정한다. 빈 문자열 = 자동(브리지가 본 것 중 점수 최고).

        콤마로 여러 개도 된다('apple,orange') — 그러면 그 셋 중 점수 최고를 잡는다.
        ⚠️ **진행 중인 작업에는 적용하지 않는다.** PERCEIVE 는 진입할 때 이 값을 브리지에
        밀어넣고 시작하므로, 도중에 바꾸면 로그가 가리키는 대상과 실제로 계산된 대상이
        갈라진다. 다음 `/pick/start` 부터 쓴다.
        """
        self._target_override = msg.data.strip()
        shown = self._target_override or '(자동)'
        if self.state is State.IDLE:
            self.get_logger().info(f'타겟 지정: {shown}')
        else:
            self.get_logger().warn(
                f'타겟 지정 {shown} — 진행 중인 {self.state.name} 에는 적용하지 않는다. '
                '다음 /pick/start 부터다')
        self._publish_target(self._target_override)

    def _publish_target(self, value: str):
        self.target_pub.publish(String(data=str(value)))

    def _on_place_location(self, msg):
        """내려놓을 위치 지정. `PLACE_LOCATIONS` 키가 아니면 무시하고 이전 값을 유지한다.

        ⚠️ target 과 같은 이유로 **진행 중인 작업에는 적용하지 않는다** — PICK 도중에
        바뀌면 로그가 가리키는 목적지와 실제 PLACE 관절이 갈라진다. 다음 `/pick/start`부터.

        **PLACE_RETRY 에서는 예외적으로 즉시 반영한다.** 이 상태는 "PLACE 가 실패해서
        다른 위치로 다시 시도할지 사람에게 묻는" 자리라, 다음 사이클이 아니라 **이번
        재시도**가 곧 "다음"이다 — `_srv_retry_place`가 `self.place_location`을 그대로
        읽어 쓰므로, 여기서 안 바꾸면 재시도 버튼이 실패했던 그 위치로 또 간다.
        """
        value = msg.data.strip()
        if value not in PLACE_LOCATIONS:
            self.get_logger().warn(
                f"잘못된 place_location '{value}' — {list(PLACE_LOCATIONS)} 중 하나만 된다. 무시함")
            return
        self._place_override = value
        if self.state is State.IDLE:
            self.get_logger().info(f'내려놓을 위치 지정: {value}')
        elif self.state is State.PLACE_RETRY:
            self.place_location = value
            self.get_logger().info(f'놓기 재시도 목적지 변경: {value} — [재시도]를 누르면 이 위치로 간다')
        elif self.state is State.WAIT_PLACE_TARGET:
            # 물체를 든 채 놓을 위치를 기다리던 상태 — 값이 오면 곧장 내려놓기로 진행한다
            # (PLACE_RETRY 와 같은 즉시반영. set_place 명령이 이 경로로 들어온다).
            self.place_location = value
            self.get_logger().info(f"놓을 위치 지정: {value} — 내려놓기로 진행한다")
            self._to(State.PLACE, f"놓을 위치 '{value}' 지정됨")
        else:
            self.get_logger().warn(
                f'내려놓을 위치 지정 {value} — 진행 중인 {self.state.name} 에는 적용하지 않는다. '
                '다음 /pick/start 부터다')
        self._publish_place(self._place_override)

    def _publish_place(self, value: str):
        self.place_pub.publish(String(data=str(value)))

    def _on_place_pending(self, msg):
        """이번 pick 이 place 를 지정 안 했는지(True). `_st_idle` 이 사이클 시작 때 latch 한다.

        단발성이라 여기서는 값만 붙잡는다 — place_location 처럼 진행 중 상태를 즉시 바꾸지
        않는다(요청별 사실이라 다음 사이클 경계에서만 의미가 있다). 놓치면 False(기존 동작)로
        떨어지므로 실패 방향이 안전하다(자동 basket place).
        """
        self._place_pending_req = bool(msg.data)

    def _on_target_pixel(self, msg):
        """개체 선정 좌표(픽셀). `{"x":..,"y":..,"w":..,"h":..}` 가 아니면 조용히 버린다.

        ⚠️ **단발성이다.** target/place_location 과 달리 이번 사이클이 쓰고 나면
        `_push_bridge()`가 지운다(성공 소비). 소비 전에 실패로 빠지면 `_to()`가 SPEAK_FAIL/
        ABORT 진입 시 지운다(그 블록 주석 참고) — 둘 다 없으면 다음 사이클로 샌다. 여기서 상태와 무관하게
        항상 받아들이는 이유도 그것이다: 이건 "설정"이 아니라 "이번 지시에 실린 데이터"라
        진행 중 작업에 적용하지 않는다는 target 의 규칙이 적용되지 않는다(다음 PERCEIVE가
        곧 이번 지시의 PERCEIVE 다 — vla_command_node 가 place 와 같은 순서로, pick 지시를
        래치에 넣기 **전에** 이 토픽을 먼저 쏜다).
        """
        try:
            doc = json.loads(msg.data)
            x, y, w, h = (float(doc[k]) for k in ('x', 'y', 'w', 'h'))
        except (ValueError, TypeError, KeyError):
            self.get_logger().warn(f'/pick/target_pixel 형식이 이상하다: {msg.data!r} — 버림')
            return
        if w <= 0 or h <= 0:
            self.get_logger().warn(f'/pick/target_pixel 의 w/h 가 양수가 아니다: {w}x{h} — 버림')
            return
        self._pixel_override = (x, y, w, h)
        self.get_logger().info(f'개체 선정 좌표 수신: ({x:.0f},{y:.0f}) / 기준 {w:.0f}x{h:.0f}')

    def _srv_start(self, _req, res):
        if self.state is not State.IDLE:
            res.success, res.message = False, f'IDLE 이 아니다 (현재 {self.state.name})'
            return res
        self._start_req = True
        res.success, res.message = True, '시작'
        return res

    def _srv_approve(self, _req, res):
        # WAIT_APPROVAL(계획 승인)과 REGRASP(재파지 승인, 스캐폴드) 둘 다 같은 승인 버튼/음성을
        # 쓴다 — rqt '승인'·approve_listener_node 가 이 서비스 하나만 부른다.
        if self.state not in (State.WAIT_APPROVAL, State.REGRASP):
            res.success, res.message = False, f'승인 대기 중이 아니다 (현재 {self.state.name})'
            return res
        self._approved = True
        res.success, res.message = True, '승인됨 — 실행한다'
        return res

    def _srv_abort(self, _req, res):
        # SPEAK_FAIL 도 거부한다 — states.py TRANSITIONS 에 SPEAK_FAIL->ABORT 간선이 없어서,
        # 여기서 받아주면 _tick()이 _abort()를 부르고 _to()가 "허용되지 않은 전이"로 잡아
        # 스스로 만든 걸 스스로 에러 로그로 찍는다(2026-08-10 code-audit 지적, _on_robot_state
        # 의 거부 목록과도 맞춘다).
        if self.state in (State.IDLE, State.SPEAK_FAIL, State.ABORT, State.SAFE_STOP):
            res.success, res.message = False, f'중단할 게 없다 (현재 {self.state.name})'
            return res
        with self._abort_lock:
            self._abort_req = '사용자 abort'
        res.success, res.message = True, '중단 요청'
        return res

    def _srv_reset(self, _req, res):
        if self.state is not State.SAFE_STOP:
            res.success, res.message = False, f'SAFE_STOP 이 아니다 (현재 {self.state.name})'
            return res
        # 곧장 IDLE 로 가지 않는다 — 안전정지가 걸린 자리(테이블/물체 근처일 수 있다)에
        # 팔을 그대로 두면 다음 PERCEIVE 가 그 자리에서 재촬영해 그리퍼 자신을 물체로
        # 오인식한다. HOME 을 거쳐야 한다.
        self._home_next = State.IDLE
        self._to(State.HOME, 'SAFE_STOP 복구 — 홈으로 복귀 후 재개')
        res.success, res.message = True, 'HOME 복귀 후 IDLE'
        return res

    def _srv_home(self, _req, res):
        """홈 관절자세로 복귀시킨다. IDLE 에서만 먹는다 (음성/VLA `cmd:"home"` · rqt '홈').

        SAFE_STOP 복구(`_srv_reset`)와 달리 "정상 대기 중 홈으로 보내달라"는 요청이다 —
        진행 중인 pick 사이클 도중에는 받지 않는다(도중 홈 이동은 물체를 문 채일 수 있어
        위험하다. 그럴 땐 `/pick/abort` 뒤 `/pick/reset` 경로를 쓴다). 도착 후 `_home_next`
        (=IDLE)로 돌아온다. 승인 게이트는 없다 — SAFE_STOP 리셋의 HOME 경유와 같은 성격의
        고정 관절이동이라 `require_approval` 을 태우지 않는다(계약 §10 리셋과 동일).
        """
        if self.state is not State.IDLE:
            res.success, res.message = False, f'IDLE 이 아니다 (현재 {self.state.name})'
            return res
        self._home_next = State.IDLE
        self._to(State.HOME, '홈 복귀 요청')
        res.success, res.message = True, 'HOME 복귀 후 IDLE'
        return res

    def _srv_retry_place(self, _req, res):
        """PLACE 실패로 물체를 문 채 정지(PLACE_RETRY)한 상태에서만 먹는다.

        재촬영·재인식 없이 곧장 PLACE 로 되돌아간다 — 물체는 이미 attach 된 채라
        SCENE_PREP/PLAN 을 다시 돌 필요가 없다. `/pick/place_location`(rqt 패널 '내려놓을
        위치' 콤보)을 이 상태에서 새로 보내면 `_on_place_location`이 즉시 반영해두므로
        (target/place 의 "다음 /pick/start 부터" 규칙과 다르다 — 여기선 이번 재시도가
        곧 그 다음이다), 다른 위치를 골라 여기를 부르면 그 위치로 다시 계획한다.
        """
        if self.state is not State.PLACE_RETRY:
            res.success, res.message = False, f'놓기 재시도 대기 중이 아니다 (현재 {self.state.name})'
            return res
        self._to(State.PLACE, f"놓기 재시도 — 목적지 '{self.place_location}'")
        res.success, res.message = True, f"'{self.place_location}' 로 다시 계획한다"
        return res

    def _srv_pause(self, _req, res):
        """✋ "멈춰". **어떤 상태에서든 받는다** — 거부하면 안 되는 유일한 명령이다.

        멈출 게 없는 상태(`PAUSE_EXEMPT`)에서도 `success=True` 로 답한다. 사람이 "멈춰"
        라고 했는데 "실패"가 돌아오면 다시 말하게 되고, 그 사이에 진짜로 뭔가 시작될 수
        있다. 멱등하게 두는 편이 안전하다.
        """
        if self.state in PAUSE_EXEMPT:
            res.success, res.message = True, f'이미 멈춰 있다 (현재 {self.state.name})'
            return res
        with self._abort_lock:
            self._pause_req = '사용자 정지'
        res.success, res.message = True, '정지 요청 — 다음 명령까지 대기한다'
        return res

    def _srv_resume(self, _req, res):
        """"계속해". `PAUSED` 에서만 먹는다.

        복귀 지점은 `_paused_from` 하나로 정한다:

          보유 중 + 목적지 있음 → `PLACE`               재인식 없이 놓기부터 다시 계획
          보유 중 + 목적지 없음 → `WAIT_PLACE_TARGET`   다시 사람을 기다린다
          비보유                → `PERCEIVE`            멈춘 사이 테이블이 바뀌었을 수 있다

        🔴 **보유 중에는 재인식하지 않는다.** 물체를 문 채로 다시 찍으면 그리퍼가 자기
        물체를 오인식한다(`_st_release_retry` 주석의 2026-08-07 실측과 같은 함정).

        어느 쪽이든 **취소된 옛 trajectory 를 재사용하지 않는다** — `self.solutions` 를
        비워 resume 시점의 최신 상태로 다시 계획하게 한다.
        """
        if self.state is not State.PAUSED:
            res.success, res.message = False, f'멈춰 있지 않다 (현재 {self.state.name})'
            return res

        holding = self._paused_from in HOLDING_STATES
        self.solutions.clear()
        self._retries = 0
        if holding:
            if self.place_location:
                nxt, why = State.PLACE, f"재개 — '{self.place_location}' 로 다시 계획"
            else:
                nxt, why = State.WAIT_PLACE_TARGET, '재개 — 목적지 지정 대기로 복귀'
        else:
            nxt, why = State.PERCEIVE, '재개 — 최신 장면으로 다시 인식'
        self._to(nxt, why)
        res.success, res.message = True, why
        return res

    def _srv_stow(self, _req, res):
        """안전 종료 시퀀스. "정리하고 끝내라".

        ```
        보유 중  → PLACE(현재 place_location) → RELEASE → HOME → IDLE
        비보유   → 그리퍼 open → HOME → IDLE
        ```

        🔴 **순서가 요청과 반대인 것이 핵심이다.** "그리퍼를 열고 홈 복귀"를 글자 그대로
        하면 물체를 **지금 있는 자리에** 떨어뜨린다. 30 cm 상공이면 낙하고, 병·컵이면
        깨진다. 그래서 놓을 자리로 **먼저 가고** 그 다음에 연다.

        🔴 **SIGINT/atexit 안에서 이걸 흉내내면 안 된다.** 거기선 executor 가 이미 빠져
        나와 MoveIt 액션 피드백을 못 받고, `move_group` 이 같은 launch 면 동시에 죽는다.
        종료 훅은 "즉시 끝나는 것"만 하고(§ main), 정리는 **사람이 이 서비스를 부르는**
        것으로 한다. GUI 의 'VLA 정지' 버튼이 이 역할이다.

        `SAFE_STOP` 에서만 거부한다 — 그쪽은 `/pick/reset` 이 정본 복구 경로다.
        """
        if self.state is State.SAFE_STOP:
            res.success, res.message = False, '안전정지 상태다 — /pick/reset 으로 복구할 것'
            return res
        if self.state is State.IDLE:
            res.success, res.message = True, '이미 대기 상태다'
            return res
        with self._abort_lock:
            self._stow_req = True
        res.success, res.message = True, '정리 후 종료 — 놓을 자리로 간 뒤 놓는다'
        return res

    def _srv_release_now(self, _req, res):
        """"들고만 있어줘" 다음의 "됐어, 거기 놔". `WAIT_PLACE_TARGET` 에서만 먹는다.

        **지금 자리에서 그대로 연다** — 목적지로 이동하지 않고, 재촬영도 재계획도 IK 도
        없다. `RELEASE` 가 하는 일(그리퍼 열기 + detach)만 하고 HOME 으로 간다.

        다른 상태에서 거부하는 이유: 이 서비스는 "팔이 이미 멈춰 서서 사람을 기다리는
        중"이라는 것을 전제로 안전하다. 이동 중(`PLACE`·`LIFT`)에 열면 물체가 어디에
        떨어질지 아무도 모른다. 30 cm 상공이면 그게 그대로 낙하다.

        `/pick/abort` 와 다르다: abort 는 `HOLDING_STATES` 에서 그리퍼를 **일부러 안
        연다**("떨어뜨리는 것보다 물고 멈춰 있는 게 안전하다"). 이건 사람이 지금 그
        자리를 보고 "놔도 된다"고 판단해서 부르는 것이라 정반대다.
        """
        # PAUSED 도 받는다(2026-08-12, P5). 두 상태의 공통점이 안전의 근거다 —
        # **팔이 이미 멈춰 서 있고 사람이 그걸 보고 있다.** 멈춰 세워 놓고 "그거 그냥
        # 놔"라고 말할 수 있어야 하는데, 그게 안 되면 탈출구가 abort 뿐이다.
        if self.state not in (State.WAIT_PLACE_TARGET, State.PAUSED):
            res.success, res.message = False, (
                f'지금 자리에서 놓기는 팔이 멈춰 있을 때만 된다 (현재 {self.state.name})')
            return res
        if self.state is State.PAUSED and self._paused_from not in HOLDING_STATES:
            res.success, res.message = False, '들고 있는 물체가 없다'
            return res
        self._to(State.RELEASE, '사용자 요청 — 이동 없이 지금 자리에서 놓는다')
        res.success, res.message = True, '이동 없이 지금 자리에서 놓는다'
        return res

    # ────────────────────────────────────────────────────────
    # 전이
    # ────────────────────────────────────────────────────────
    def _to(self, nxt: State, why: str = ''):
        if not is_allowed(self.state, nxt):
            # 전이표에 없는 전이는 버그다. 조용히 넘어가면 상태머신이 아니라 그냥 함수 호출이다.
            self.get_logger().error(f'허용되지 않은 전이 {self.state.name} -> {nxt.name} — ABORT')
            why = f'잘못된 전이 {self.state.name}->{nxt.name}'
            nxt = State.ABORT
        if nxt is not self.state:
            self.get_logger().info(f'[{self.state.name}] -> [{nxt.name}] {why}')
        self.state = nxt
        self._entered = self.get_clock().now()
        self._fut = None
        self._push_fut = None
        self._pushed = False
        self._extra = []
        self._plan_i = 0
        self._nag = 0
        self._octomap_cleared = False
        if nxt is State.PERCEIVE:
            # `_perceive_failed()` 내부 재시도는 `_to()` 를 안 거치므로(같은 상태에 머문다)
            # 여기서 지워도 그 카운트는 안 지워진다 — 여기는 오직 **새 PERCEIVE 진입**(LISTENING
            # 이후, voice_enabled=false 직행, RELEASE_RETRY 뒤 HOME 경유 재인식)만 잡는다.
            # 안 지우면 frame 불일치 같은 `_perceive_failed()` 밖의 SPEAK_FAIL 경로(_accept_grasp)
            # 나, voice_enabled=true 라 `_st_idle` 리셋을 안 거치는 SPEAK_FAIL->LISTENING 루프에서
            # 이전 시도의 잔여 카운트를 물려받아 재시도 예산이 조용히 줄어든다(cross-review
            # 2026-08-09 지적).
            self._retry_perceive = 0
        if nxt in (State.SPEAK_FAIL, State.ABORT):
            # 단발성 픽셀 override 는 성공 경로(`_push_bridge`, 766)에서만 소비·삭제된다.
            # 소비 전에 실패로 빠지면(키워드 실패, 브리지 push 실패 등) `_pushed` 는 여기서
            # 다시 False 가 되므로(540), 지우지 않으면 **다음 pick 사이클의 PERCEIVE 가
            # 이전 지시의 픽셀을 다시 push** 해 엉뚱한 개체를 고른다(cross-review 2026-08-11).
            # 실패 수렴 상태(SPEAK_FAIL/ABORT)에서 지운다: 픽셀은 PERCEIVE 진입 전 IDLE/
            # LISTENING 에서 세팅되고 소비는 PERCEIVE 이므로(`_on_target_pixel` 주석), 이 두
            # 상태 진입이 **정상 소비 창을 지우지 않는다** — `_st_idle` 시작점에서 지우면
            # vla 가 "픽셀 publish -> start" 순으로 쏘는 legit 픽셀을 경합적으로 지운다(그래서 안 함).
            self._pixel_override = None
        self.state_pub.publish(String(data=nxt.name))

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._entered).nanoseconds * 1e-9

    def _abort(self, why: str):
        # ⚠️ 물체를 들고 있을 수 있는 구간에서는 **그리퍼를 열지 않는다.**
        #    떨어뜨리는 게 멈춰 있는 것보다 위험하다. 판정은 전이 **전에** 해야 한다 —
        #    _to() 뒤에는 self.state 가 이미 ABORT 라 어디서 왔는지 알 수 없다.
        if self.state in HOLDING_STATES:
            self.get_logger().warn('물체 보유 가능 상태 — 그리퍼를 열지 않고 정지한다')
        if self._call is not None:
            self._call.cancel()
            self._call = None
        self.get_logger().error(f'ABORT: {why}')
        self._to(State.ABORT, why)

    def _pause(self, why: str):
        """✋ 되돌릴 수 있는 정지. `_abort()` 와 구조는 같고 **파괴적이지 않다.**

        `_abort()` 와 같은 이유로 판정을 전이 **앞**에서 한다 — `_to()` 뒤에는
        `self.state` 가 이미 `PAUSED` 라 어디서 멈췄는지 알 수 없다. 그런데 `resume`
        은 바로 그 "어디서 멈췄나"로 복귀 지점을 정하므로, 여기서 `_paused_from` 을
        안 남기면 되돌아갈 방법이 없다.

        하는 일은 셋뿐이다:
          - 진행 중 MoveIt goal 취소 (팔이 선다)
          - 어디서 멈췄는지 기록
          - `PAUSED` 로 전이

        하지 않는 일 — **이게 사양이다**: 그리퍼를 안 건드리고(I5), 물체를 안 내려놓고,
        홈으로 안 가고, 타이머를 안 걸고, 재인식을 안 한다. 시간이 지나서 뭔가가 저절로
        일어나면 그건 "다음 명령이 올 때까지 대기"가 아니다.
        """
        self._paused_from = self.state
        if self._call is not None:
            self._call.cancel()
            self._call = None
        # 서비스 future 도 버린다. _to() 가 어차피 _fut=None 으로 만들지만, 여기서
        # 명시적으로 두는 건 "진행 중이던 건 전부 무효, resume 은 최신 씬으로 다시
        # 계획한다"는 규칙을 코드에 남겨두기 위해서다.
        self._fut = None
        held = ' (물체 보유 — 그리퍼 유지)' if self._paused_from in HOLDING_STATES else ''
        self.get_logger().warn(f'✋ PAUSED: {why} — {self._paused_from.name} 에서 멈춤{held}')
        self._to(State.PAUSED, why)

    def _stow(self):
        """정리 후 종료. **먼저 세우고, 놓을 자리로 가고, 그 다음에 연다.**

        순서가 요청("그리퍼를 열고 홈 복귀")과 반대인 것이 요점이다 — 글자 그대로 하면
        물체를 지금 있는 자리에 떨어뜨린다.

        1. 아직 안 멈췄으면 `_pause()` 로 먼저 세운다. `PAUSED` 는 어디서든 갈 수 있어서
           (`PAUSE_EXEMPT` 여집합) 임의의 상태에서 이 시퀀스를 시작할 수 있다. 이게 없으면
           `APPROACH` 같은 데서 `HOME` 으로 가는 간선이 없어 시작조차 못 한다.
        2. 보유 중이면 `PLACE` → `RELEASE` → `HOME` → `IDLE` (기존 사슬 그대로 재사용).
           목적지가 안 정해졌으면 파라미터 기본값을 쓴다 — 종료는 **끝나야** 하므로
           여기서만은 기본값으로 진행하는 게 맞다(들고 서 있는 채로 끝낼 수 없다).
        3. 비보유면 그리퍼만 열고 `HOME` → `IDLE`.
        """
        if self.state is not State.PAUSED:
            self._pause('종료 정리(stow)')
        holding = self._paused_from in HOLDING_STATES
        if holding:
            if not self.place_location:
                self.place_location = self.p('place_location')
                self._publish_place(self.place_location)
            self._to(State.PLACE, f"종료 정리 — '{self.place_location}' 에 놓고 끝낸다")
            return
        # 비보유: 여는 건 안전하다(물 게 없다). 결과를 기다리지 않는 부수 호출로 보낸다 —
        # _to() 가 _extra 를 비우므로 전이 **뒤에** 넣는다.
        self._home_next = State.IDLE
        self._to(State.HOME, '종료 정리 — 그리퍼 열고 홈 복귀')
        self._extra = [self.rg2.open_async()]

    # ────────────────────────────────────────────────────────
    # 메인 루프
    # ────────────────────────────────────────────────────────
    def _tick(self):
        with self._abort_lock:
            why = self._abort_req
            self._abort_req = None
            pause_why = self._pause_req
            self._pause_req = None
            stow = self._stow_req
            self._stow_req = False
        if why and self.state not in (State.ABORT, State.SAFE_STOP):
            self._abort(why)
            return
        # abort 다음, 제한시간 판정 **앞**에 둔다. 순서가 뒤바뀌면 "제한시간이 거의 다 된
        # 상태에서 멈춰"가 pause 대신 abort 로 떨어진다 — 사람 입장에선 멈추라고 했는데
        # 파괴적 정지가 된 것으로 보인다.
        if pause_why and self.state not in PAUSE_EXEMPT:
            self._pause(pause_why)
            return
        if stow:
            self._stow()
            return
        timeout = DEFAULT_TIMEOUTS.get(self.state)
        if timeout is not None and self._elapsed() > timeout:
            self._abort(f'{self.state.name} 제한시간 {timeout:.0f}s 초과')
            return
        try:
            getattr(self, f'_st_{self.state.name.lower()}')()
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().error(f'{self.state.name}: {type(exc).__name__}: {exc}')
            self._abort(f'{type(exc).__name__}: {exc}')

    def _service(self, client, request, name):
        """(상태, 결과). 상태는 'pending' | 'done'.

        타이머 콜백 안에서 `spin_until_future_complete` 를 부르면 재진입으로 엉킨다
        (executor 가 이미 이 노드를 돌리고 있다) → 기다리지 않고 매 tick 확인한다.

        ⚠️ 서비스가 아직 안 떠 있으면 **기다린다**(실패로 처리하지 않는다). 즉시 실패로
        보내면 노드 기동 순서에 의존하게 되고, LISTENING 이 SPEAK_FAIL 과 tick 마다
        왕복하며 제한시간을 리셋해 영원히 안 멈춘다. 끝내 안 뜨면 DEFAULT_TIMEOUTS 가
        그 상태를 ABORT 시킨다 (LISTENING 60s / PERCEIVE 120s).
        """
        if self._fut is None:
            if not client.service_is_ready():
                self.get_logger().warn(f'{name} 서비스를 기다리는 중',
                                       throttle_duration_sec=5.0)
                return 'pending', None
            self._fut = client.call_async(request)
            return 'pending', None
        if not self._fut.done():
            return 'pending', None
        return 'done', self._fut.result()

    # ────────────────────────────────────────────────────────
    # 상태 핸들러
    # ────────────────────────────────────────────────────────
    def _st_idle(self):
        if not self._start_req:
            return
        if self._object_added:
            # desync 를 여기서 자동으로 고치지 않는다(=detach 하지 않는다) — 물체가
            # 아직 실제로 그리퍼에 물려 있을 수 있는데 씬만 지우면 다음 계획이 그
            # 물체를 피하지 못한다. PACKAGES.md#pick_fsm "알려진 자잘한 것" 참고.
            self.get_logger().warn(
                '이전 사이클의 attach 물체(pick_target)가 아직 planning scene 에 '
                '남아 있다 — 재사용/충돌 여부를 확인하라 (2026-08-10 code-audit 지적)')
        self._start_req = False
        self._retry_grip = 0
        self._fail_streak = 0
        self.alternatives = []
        self.solutions.clear()
        self.poses.clear()
        # 이번 사이클이 place 대기 경로인지 확정한다(단발성 신호를 여기서 latch·소비).
        self._wait_place = self._place_pending_req
        self._place_pending_req = False
        if self._wait_place:
            # place 미지정 pick — LIFT 후 set_place 를 기다린다. 타임아웃 시 내려놓을
            # fallback 은 파라미터 기본 위치(override 가 아니라 — 이 요청은 위치를 안 정했다).
            self.place_location = self.p('place_location')
        else:
            # `/pick/place_location` 로 들어온 값이 파라미터를 이긴다 (target 과 같은 패턴).
            self.place_location = (self._place_override if self._place_override is not None
                                   else self.p('place_location'))
        self._publish_place(self.place_location)
        if self.p('voice_enabled'):
            self._to(State.LISTENING)
        else:
            # `/pick/target` 로 들어온 값이 파라미터를 이긴다 (한 번이라도 들어왔다면).
            self.target = (self._target_override if self._target_override is not None
                           else self.p('target'))
            self._publish_target(self.target)
            self._to(State.PERCEIVE, f"타겟 '{self.target or '(자동 — 점수 최고)'}'")

    def _st_listening(self):
        status, res = self._service(self.kw_cli, Trigger.Request(), self.p('keyword_service'))
        if status != 'done':
            return
        if res is None or not res.success:
            self._to(State.SPEAK_FAIL, f'키워드 실패: {getattr(res, "message", "응답 없음")}')
            return
        # get_keyword 는 공백으로 이어붙인 타겟 목록을 message 에 담는다 (robot_control.py:101).
        # 이 FSM 은 한 번에 하나만 집는다 — 여러 개를 큐로 돌리는 건 성공률을 본 뒤에 할 일이다.
        words = res.message.split()
        if not words:
            self._to(State.SPEAK_FAIL, '키워드가 비어 있다')
            return
        self.target = words[0]
        self._publish_target(self.target)
        self._to(State.PERCEIVE, f"타겟 '{self.target}'")

    def _st_perceive(self):
        src = self.p('grasp_source')
        if src == 'manual':
            if self._best is None:
                return                      # 사람이 /grasp/best 를 쏠 때까지 기다린다
            self._accept_grasp(self._best, float(self.p('default_width_m')), 1.0, [])
            return

        # 브리지에 "이번엔 뭘 잡을지"를 먼저 심는다. 끝나기 전에는 계산을 시키지 않는다 —
        # 순서가 뒤집히면 브리지가 **직전 실행의 타겟**으로 수십 초를 계산한다.
        if not self._pushed and not self._push_bridge():
            return

        if src == 'compute_grasp':
            req = ComputeGrasp.Request()
            req.target = self.target
            req.min_confidence = float(self.p('min_confidence'))
            status, res = self._service(self.grasp_cli, req, self.p('grasp_service'))
            if status != 'done':
                return
            if res is None or not res.success:
                self._perceive_failed(f'grasp 없음: {getattr(res, "message", "응답 없음")}')
                return
            self.get_logger().info(f'ComputeGrasp: {res.message}')
            self._accept_grasp(res.grasp_pose, res.width_m, res.confidence,
                               list(res.alternatives),
                               list(getattr(res, 'alternative_widths', [])))
            return

        # legacy_trigger: 지금 graspgenx_perception 의 grasp_bridge_node 가 제공하는 계약.
        # 응답에 포즈가 없고 /grasp/best 로 따로 나온다 → **이번 호출 이후에 들어온** 것만 쓴다.
        # 직전 요청의 포즈를 재활용하면 아무 로그도 없이 엉뚱한 물체를 집는다.
        if self._fut is None:
            self._seq_at_call = self._best_seq
        status, res = self._service(self.grasp_cli, Trigger.Request(),
                                    self.p('grasp_trigger_service'))
        if status != 'done':
            return
        if res is None or not res.success:
            self._perceive_failed(f'grasp 없음: {getattr(res, "message", "응답 없음")}')
            return
        if self._best is None or self._best_seq == self._seq_at_call:
            return                          # 서비스는 끝났지만 포즈가 아직 안 왔다 — 기다린다
        self.get_logger().info(f'/grasp/compute: {res.message}')
        alts = []
        for header, pose in self._candidates[: int(self.p('max_alternatives'))]:
            ps = PoseStamped()
            ps.header, ps.pose = header, pose
            alts.append(ps)
        self._accept_grasp(self._best, float(self.p('default_width_m')), 1.0, alts)

    def _push_bridge(self) -> bool:
        """FSM 타겟 -> 브리지 `target_classes`(+`seg_source`). 끝났으면 True.

        `_service()` 와 달리 브리지가 안 떠 있으면 **기다린다** — PERCEIVE 제한시간(120s)이
        결국 끊는다. 설정 실패는 조용히 넘기지 않는다: 실패하면 브리지가 이전 타겟으로
        계산해 "엉뚱한 물체를 잡았는데 로그는 맞다고 말하는" 상태가 된다.
        """
        if self.bridge_param_cli is None:
            self._pushed = True
            return True
        if self._push_fut is None:
            if not self.bridge_param_cli.service_is_ready():
                self.get_logger().warn(
                    f"{self._bridge_name} 의 set_parameters 를 기다리는 중 "
                    '(grasp_bridge_node 가 떠 있는지 확인할 것)', throttle_duration_sec=5.0)
                return False
            req = SetParameters.Request()
            req.parameters = [str_param('target_classes', self.target)]
            seg = self.p('bridge_seg_source')
            if seg:
                req.parameters.append(str_param('seg_source', seg))
            # 개체 선정 좌표. 지정이 없으면 nan 4개를 보낸다 — 브리지 쪽 select_by_point
            # 는 nan 을 "off"로 읽으므로(grasp_bridge_node.py EXTRA_DEFAULTS 주석) 이걸
            # **항상** 보내야 한다. 안 보내면 이전 호출의 좌표가 브리지 파라미터에 그대로
            # 남아 클래스만 지시한 이번 pick 이 전 프레임 좌표로 잘못 걸릴 수 있다.
            px = self._pixel_override
            for name, val in zip(('pixel_x', 'pixel_y', 'pixel_w', 'pixel_h'),
                                 px if px is not None else (float('nan'),) * 4):
                req.parameters.append(float_param(name, val))
            self._push_fut = self.bridge_param_cli.call_async(req)
            return False
        if not self._push_fut.done():
            return False
        res = self._push_fut.result()
        self._push_fut = None
        names = (['target_classes'] + (['seg_source'] if self.p('bridge_seg_source') else [])
                + ['pixel_x', 'pixel_y', 'pixel_w', 'pixel_h'])
        if res is None or len(res.results) != len(names):
            self._to(State.SPEAK_FAIL, f"{self._bridge_name} 파라미터 설정 응답이 이상하다")
            return False
        bad = [f'{n}: {r.reason}' for n, r in zip(names, res.results) if not r.successful]
        if bad:
            self._to(State.SPEAK_FAIL,
                     f"{self._bridge_name} 파라미터 설정 실패 — " + ' / '.join(bad))
            return False
        self._pushed = True
        seg = self.p('bridge_seg_source')
        pixel_note = f', pixel=({self._pixel_override[0]:.0f},{self._pixel_override[1]:.0f})' \
            if self._pixel_override is not None else ''
        self.get_logger().info(
            f"브리지 설정: target_classes='{self.target or '(전부)'}'"
            + (f", seg_source={seg}" if seg else '') + pixel_note)
        # 단발성 소비 — 다음 PERCEIVE 에서 새 지시가 없으면 nan(off)이 나가야 한다.
        self._pixel_override = None
        return True

    def _perceive_failed(self, why: str):
        """PERCEIVE 요청 실패(grasp 없음 포함) 시 재촬영 재시도. `_motion_failed` 와 같은 뼈대다.

        2026-08-09 실기 로그: 정지된 같은 물체·같은 자리에서 collision-free 비율이
        0%~53%로 요동쳤다(depth 노이즈로 GraspGenX OBB 후보 자체가 흔들린다) — 재촬영
        한 번으로 5번 중 4번은 살아났다. 실패 사유를 가리지 않고 재시도한다: 브리지가
        안 뜬 것 같은 하드 오류라도 몇 초 안에 재시도가 소진되어 결국 SPEAK_FAIL 로
        가는 결과는 같다 — 가려서 얻는 이득이 없다.
        """
        self._retry_perceive += 1
        limit = int(self.p('perceive_retries'))
        if self._retry_perceive <= limit:
            self.get_logger().warn(f'{why} — 재촬영 재시도 {self._retry_perceive}/{limit}')
            self._entered = self.get_clock().now()      # PERCEIVE 제한시간(120s) 갱신
            self._fut = None                             # 다음 tick 에 새 요청을 쏜다
            return
        self._retry_perceive = 0
        self._to(State.SPEAK_FAIL, why)

    # ── 손목 eye-in-hand seam (2026-08-12) — 지금은 동작 불변 ────────
    # 여기 두 함수는 **현재 아무것도 바꾸지 않는다.** 손목 카메라가 붙었을 때
    # 고쳐야 할 지점을 지금 한 곳으로 모아 두는 게 목적이다.
    # 상세 근거: md/plans/2026-08-12-vlm-fsm-integration-handoff.md §P6

    def _commit_grasp(self, pose: PoseStamped, *, source: str = 'GLOBAL'):
        """모션이 실제로 쓸 grasp 를 확정한다. **하강 자세가 바뀌는 유일한 문.**

        지금 호출자는 둘 다 Top GraspGenX 경로다(`_accept_grasp`, `_st_next_candidate`)
        이라 `global_grasp` 와 `committed_grasp` 가 항상 같다.

        손목이 붙으면 `_st_regrasp` 의 HOOK 이 `source='WRIST'` 로 여기를 부른다 —
        그러면 `global_grasp`(원본 기록)는 그대로 두고 `committed_grasp` 만 바뀐다.
        그게 "손목이 계산한 자세로 실제로 내려간다"의 전부다.
        """
        self.committed_grasp = pose
        if source.startswith('GLOBAL'):
            self.global_grasp = pose
        self.grasp_revision += 1

    def _invalidate_final_solutions(self):
        """`committed_grasp` 가 바뀌면 grasp/lift IK 해도 더 이상 유효하지 않다.

        `pre_grasp` 는 남겨 둔다 — 접근점은 물체 자세가 조금 바뀌어도 여전히 유효하고,
        이미 그 자리에 도착해 있을 수도 있다.

        🔴 **지금은 호출처가 없다.** `_commit_grasp` 를 부르는 두 곳은 아직 IK 를 풀기
        전(`PERCEIVE`/`NEXT_CANDIDATE` → `PLAN`)이라 지울 해가 없다. 손목이 붙어
        `APPROACH` **뒤에** grasp 가 바뀌기 시작하면 그때 `_st_regrasp` 의 HOOK 에서
        이걸 부른다 — 안 부르면 옛 IK 해로 옛 자리에 내려간다.
        """
        self.solutions.pop('grasp', None)
        self.solutions.pop('lift', None)

    def _accept_grasp(self, pose: PoseStamped, width_m, confidence, alternatives,
                      alternative_widths=None):
        base = self.p('base_frame')
        if pose.header.frame_id and pose.header.frame_id != base:
            # tf2 로 옮겨줄 수도 있지만, 규약이 어긋난 채 조용히 도는 것보다 멈추는 게 낫다.
            self._to(State.SPEAK_FAIL,
                     f'grasp 프레임이 {pose.header.frame_id} 다 (기대: {base})')
            return
        # 🔴 여기가 grasp 프레임 -> `rg2_base_link`(= ee_link) 로 넘어오는 **유일한 지점**이다.
        #    best 와 alternatives 를 같이 돌린다 — 대안만 빠뜨리면 첫 후보가 실패한 뒤부터
        #    조용히 90° 틀어진다. 이 아래로는 전부 ee_link 목표 자세다.
        self._commit_grasp(geo.to_gripper_base(pose), source='GLOBAL')
        self.width_m = self._grip_width(width_m)
        n = int(self.p('max_alternatives'))
        self.alternatives = [geo.to_gripper_base(a) for a in list(alternatives)[:n]]
        # 폭은 **후보마다 다르다**(닫힘축이 다르면 같은 물체라도 다르게 잰다). 길이가 안 맞거나
        # 아예 없으면(legacy/manual 경로) 1등 폭을 복제한다 — 짝을 못 맞춘 채 인덱스로 꺼내면
        # 조용히 남의 폭으로 닫는다. 대신 그때는 "모른다"가 아니라 "1등과 같다"로 명시한다.
        w = [float(v) for v in list(alternative_widths or [])[:n]]
        self.alternative_widths = (w if len(w) == len(self.alternatives)
                                   else [float(width_m)] * len(self.alternatives))
        # 변환 **후** 포즈로 잰다. 지금은 요 회전이라 +Z 가 같아서 결과가 같지만, 이 변환에
        # 언젠가 평행이동이 붙으면 여기만 조용히 :502/:568 과 갈라진다.
        tcp = geo.tcp_of(self.committed_grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'grasp conf={float(confidence):.2f} '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) '
            f'폭={self.width_m * 1000:.1f} mm 대안={len(self.alternatives)}개')
        self._to(State.SCENE_PREP)

    def _grip_width(self, width_m) -> float:
        """물체 폭 -> 그리퍼 목표 개구 폭 [m]. 부호 규약은 `rg2.grip_target_width_m` 참고.

        여기서 하는 건 파라미터를 읽어 넘기는 것과, 0(=폭 모름)을 기본값으로 대체하는 것뿐이다.
        0 을 그대로 흘리면 완전히 닫혀 물체를 으깬다.
        """
        w = float(width_m)
        if w <= 0.0:
            w = float(self.p('default_width_m'))
            self.get_logger().warn(
                f'폭을 못 받았다(0) — default_width_m={w * 1000:.0f} mm 로 대체한다. '
                'UNVERIFIED 상수라 물체에 맞는다는 보장이 없다')
        return grip_target_width_m(w, float(self.p('grip_clearance_m')),
                                   float(self.p('max_grip_width_m')))

    def _st_scene_prep(self):
        """2단계다: 현재 ACM 을 **읽어서** 거기에 얹은 뒤 적용한다.

        🔴 한 번에 보내면 안 된다. `is_diff=true` 라도 `allowed_collision_matrix` 는
           병합이 아니라 **전체 교체**다 (2026-08-06 실측). 그냥 덮어쓰면 SRDF 의
           `disable_collisions` 34개가 사라져 인접 링크가 자기충돌로 잡히고,
           `avoid_collisions=true` IK 가 **모든 포즈에서** NO_IK_SOLUTION 을 낸다.
        """
        ok, why = self.moveit.ready()
        if not ok:
            if self._nag == 0:
                self._nag = 1
                self.get_logger().warn(f'move_group {why}')
            return
        if self._plan_i == 0:                       # ① ACM 읽기
            if self._fut is None:
                self._fut = self.moveit.get_acm_async()
                return
            if not self._fut.done():
                return
            res = self._fut.result()
            if res is None:
                self._abort('/get_planning_scene 응답 없음')
                return
            self._acm = res.scene.allowed_collision_matrix
            self._fut = None
            self._plan_i = 1
            return
        if self._fut is None:                       # ② 물체 + 병합 ACM 적용
            tcp = geo.tcp_of(self.committed_grasp, fingertip_from_rg2_base_m(self.width_m))
            obj = self.moveit.make_object(self.p('object_id'), tcp,
                                          float(self.p('object_radius_m')))
            acm = merge_acm(self._acm, self.p('object_id'), list(self.p('gripper_links')),
                            allow_octomap=bool(self.p('allow_gripper_octomap_collision')))
            self._fut = self.moveit.add_object_async(obj, acm)
            self._object_added = True
            return
        if not self._fut.done():
            return
        res = self._fut.result()
        if res is None or not res.success:
            self._abort('planning scene 갱신 실패 (/apply_planning_scene)')
            return
        self._to(State.PLAN, f"대상 등록 + ACM {len(self._acm.entry_names)}개 보존")

    def _st_plan(self):
        """pre-grasp → grasp → lift 3점 IK. 하나라도 실패하면 다음 후보로 간다."""
        if not self.poses:
            # `grasp_standoff_m` 은 **이동 목표만** 뒤로 뺀다. `committed_grasp` 는 인식이 준 값
            # 그대로 둔다 — SCENE_PREP 의 CollisionObject 와 로그는 "물체가 있다고 본 자리"를
            # 가리켜야 하고, standoff 는 "손끝 모델이 그만큼 틀렸다"는 보정이라 의미가 다르다.
            # 클램프가 걸릴 수 있으니 **적용된 값**을 찍는다(설정값이 아니라).
            approach = float(self.p('approach_offset_m'))
            self.poses = geo.plan_poses(self.committed_grasp, approach,
                                        float(self.p('grasp_standoff_m')),
                                        float(self.p('lift_offset_m')))
            applied = geo.clamped_standoff(float(self.p('grasp_standoff_m')), approach)
            if applied > 0.0:
                self.get_logger().info(
                    f'그립 시작점을 접근축 -Z 로 {applied * 1000:.1f} mm 뺐다 '
                    f'(하강 {(approach - applied) * 1000:.0f} mm, LIFT 도 이 지점 기준)')
        order = ['pre_grasp', 'grasp', 'lift']
        if self._plan_i >= len(order):
            self._to(State.WAIT_APPROVAL, 'IK 3점 성공')
            return
        key = order[self._plan_i]
        pose = self.poses[key]

        if self._fut is None:
            reach = geo.reach_of(pose)
            if reach > float(self.p('max_reach_m')):
                self._to(State.NEXT_CANDIDATE, f'{key} 도달범위 밖 ({reach:.3f} m)')
                return
            if not self.moveit.ik.service_is_ready():
                return
            # 직전 해를 시드로 넘긴다. 안 주면 pre-grasp 와 grasp 의 해가 다른 IK 분기에
            # 앉을 수 있고, 10 cm 하강이 팔 전체를 뒤집는 궤적이 된다.
            seed = self.solutions.get(order[self._plan_i - 1]) if self._plan_i else None
            self._fut = self.moveit.ik_async(
                pose, self.p('planning_group'), self.p('ee_link'), seed=seed,
                avoid_collisions=bool(self.p('ik_avoid_collisions')),
                timeout_sec=float(self.p('ik_timeout_sec')))
            return
        if not self._fut.done():
            return
        res = self._fut.result()
        self._fut = None
        if res is None or res.error_code.val != SUCCESS:
            code = err_name(res.error_code.val) if res is not None else '응답 없음'
            self._to(State.NEXT_CANDIDATE, f'{key} IK 실패 {code}')
            return
        self.solutions[key] = res.solution.joint_state
        self._plan_i += 1

    def _st_next_candidate(self):
        """GPU 를 다시 부르지 않는다. alternatives 를 미리 받아둔 이유가 이것이다."""
        for k in ('pre_grasp', 'grasp', 'lift'):
            self.solutions.pop(k, None)
        self.poses.clear()
        if not self.alternatives:
            self._to(State.SPEAK_FAIL, '후보 소진')
            return
        self._commit_grasp(self.alternatives.pop(0), source='GLOBAL(alt)')
        # 포즈만 갈아타고 폭을 그대로 두면 새 후보의 닫힘축에 맞지 않는 폭으로 닫는다.
        if self.alternative_widths:
            self.width_m = self._grip_width(self.alternative_widths.pop(0))
        tcp = geo.tcp_of(self.committed_grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'다음 후보 (남은 {len(self.alternatives)}개) '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) '
            f'폭={self.width_m * 1000:.1f} mm')
        self._to(State.PLAN)

    def _st_wait_approval(self):
        if not self.p('require_approval'):
            self._to(State.STOW, '승인 불필요 설정')
            return
        if self._approved:
            self._approved = False
            self._to(State.STOW, '사용자 승인')
            return
        if self._elapsed() > float(self.p('approval_timeout_sec')):
            self._abort('승인 대기 시간 초과')
            return
        if self._elapsed() > self._nag * 10.0:
            self._nag += 1
            self.get_logger().info(
                '✋ 승인 대기 — ros2 service call /pick/approve std_srvs/srv/Trigger {}')

    # ── 이동 4종(pre_grasp/grasp/lift/고정자세)은 같은 뼈대다 ──
    def _move(self, key: str, nxt: State, on_fail: State):
        js = self.solutions.get(key)
        if js is None:
            self._abort(f'{key} 관절해가 없다')
            return
        if self._call is None:
            if not self.moveit.move.server_is_ready():
                return
            self.get_logger().info(f'{key}: 계획+실행 (pipeline={self.p("planning_pipeline")})')
            self._call = self.moveit.move_to_joints_async(
                js, self.p('planning_group'), list(self.p('joint_names')),
                plan_only=False,
                vel_scale=self.p('vel_scale'), acc_scale=self.p('acc_scale'),
                planning_time=self.p('planning_time'), attempts=self.p('planning_attempts'),
                tolerance=self.p('joint_tolerance'),
                replan=bool(self.p('replan')), replan_attempts=self.p('replan_attempts'),
                replan_delay=self.p('replan_delay'),
                pipeline=self.p('planning_pipeline'), planner_id=self.p('planner_id'))
            return
        done, result = self._call.poll()
        if not done:
            return
        rejected = self._call.rejected
        self._call = None
        if rejected or result is None:
            self._motion_failed(key, 'goal 거부됨', on_fail)
            return
        if result.error_code.val == SUCCESS:
            self._retry_motion = 0
            self._to(nxt, f'{key} 완료 (계획 {result.planning_time:.2f}s)')
            return
        self._motion_failed(key, err_name(result.error_code.val), on_fail)

    def _motion_failed(self, key, why, on_fail: State):
        """move_group 이 replan 까지 하고도 실패한 경우의 바깥 재시도."""
        self._retry_motion += 1
        limit = int(self.p('motion_retries'))
        if self._retry_motion <= limit:
            self.get_logger().warn(f'{key} 실패({why}) — 재시도 {self._retry_motion}/{limit}')
            self._entered = self.get_clock().now()      # 제한시간 갱신
            return
        self._retry_motion = 0
        if on_fail is State.ABORT:
            self._abort(f'{key} 실패: {why}')
        else:
            self._to(on_fail, f'{key} 실패: {why}')

    def _st_stow(self):
        """이동 전 그리퍼를 완전히 닫는다 — 벌어진 폭만큼 주변과 부딪힐 여지를 줄인다."""
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = [self.rg2.close_async(0.0)]
            self.get_logger().info('그리퍼 닫기(이동 대비)')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.APPROACH)

    def _st_approach(self):
        # regrasp_enabled 면 pre-grasp 도착 후 eye-in-hand 재파지 인식(+승인)을 거친다.
        nxt = State.REGRASP if bool(self.p('regrasp_enabled')) else State.OPEN_GRIPPER
        self._move('pre_grasp', nxt, State.NEXT_CANDIDATE)

    def _st_regrasp(self):
        """(스캐폴드) pre-grasp 도착 후 eye-in-hand 로 재-graspgenx 하고 사람이 승인하는 자리.

        🔴 2026-08-11 스캐폴드다 — 손목 eye-in-hand RealSense 는 아직 없다(constraints.md,
        팀 컨벤션 문서 2절). 지금은 **실제 재촬영·재계산을 하지 않는다**: 상태 플럼빙과 승인
        재사용(`/pick/approve`, rqt '승인')만 실물이고, 카메라/graspgenx 호출부는 아래
        HOOK 자리에 비워 둔다. 카메라가 붙고 flange->camera extrinsic(TF)이 생기면 HOOK 에서
        (1) eye-in-hand 프레임 촬영 (2) graspgenx 재호출 (3) 결과를 base_frame 으로 변환해
        `committed_grasp` 갱신 (4) grasp/lift 재-IK 를 채운다.
        `regrasp_enabled=false` 면 이 상태에 아예 안 들어온다(`_st_approach`).
        """
        # ── HOOK: eye-in-hand 재-graspgenx (미구현 — 카메라/캘리브 대기) ──
        #    채울 때 이 세 줄이 그 자리다 (2026-08-12 seam):
        #
        #        self._commit_grasp(geo.to_gripper_base(wrist_pose), source='WRIST')
        #        self._invalidate_final_solutions()
        #        self._to(State.PLAN, '손목 재파지 — grasp/lift 재계획')
        #
        #    🔴 `_invalidate_final_solutions()` 를 빠뜨리면 **조용히 틀린다**: 손목이
        #    계산한 자세는 로그에만 남고 로봇은 이미 풀어 둔 옛 IK 해로 옛 자리에
        #    내려간다. 예외도 경고도 없다.
        #
        #    🔴 그리고 그보다 먼저 걸리는 게 있다: 지금 구조는 `PERCEIVE` 에서 Top
        #    grasp 를 못 얻으면 `SPEAK_FAIL` 이고, `_st_plan` 이 pre_grasp/grasp/lift
        #    **3점 IK 를 전부** 성공해야 APPROACH 로 온다. 즉 손목 코드를 완벽히
        #    구현해도 **여기까지 도달하지 못하는 경우가 대부분**이다. 진짜 병목은
        #    REGRASP 가 아니라 PERCEIVE→PLAN 상단 의존성이다(계획 §7 · 인계문서 §P6).
        if self._nag == 0:
            self._nag = 1
            self.get_logger().warn(
                'REGRASP(스캐폴드) — eye-in-hand 재파지 인식은 미구현(카메라 미장착). '
                "pre-grasp 에서 정지해 승인을 기다린다. rqt '승인'(/pick/approve) 을 누르면 하강한다")
        if self._approved:
            self._approved = False
            self._to(State.OPEN_GRIPPER, '재파지 승인 — 하강 진행')
            return
        if self._elapsed() > float(self.p('regrasp_timeout_sec')):
            self._abort('재파지 승인 대기 시간 초과')

    def _st_open_gripper(self):
        """pre-grasp 도착 후, 하강 전에 그리퍼를 연다."""
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = [self.rg2.open_async()]
            self.get_logger().info('그리퍼 열기(그립 준비)')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.DESCEND)

    def _st_descend(self):
        self._retry_narrow = 0      # 새 그랩 시도 — 좁게-재시도 예산을 다시 채운다
        if bool(self.p('clear_octomap_before_descend')) and not self._octomap_cleared:
            self._octomap_cleared = True
            self.moveit.clear_octomap_async()
            self.get_logger().warn('octomap 을 비웠다 — 이 구간은 미모델링 장애물이 안 보인다')
        self._move('grasp', State.CLOSE, State.NEXT_CANDIDATE)

    def _st_close(self):
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = self.rg2.lower_force_async(int(self.p('force_down_steps')))
            self._extra.append(self.rg2.close_async(self.width_m))
            self.get_logger().info(f'그리퍼 닫기 → {self.width_m * 1000:.1f} mm')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.VERIFY)

    def _st_verify(self):
        """힘 센서가 없다. 판정 근거는 드라이버가 주는 grip 비트뿐이다."""
        if self._elapsed() < 0.5:
            return
        ok, why = self.rg2.grip_detected()
        if ok is None:
            if bool(self.p('verify_required')):
                self._to(State.RELEASE_RETRY, why)
                return
            if self._nag == 0:          # attach 가 2 tick 걸려서 안 막으면 두 번 찍힌다
                self._nag = 1
                self.get_logger().warn(f'파지 확인 불가 — 통과시킨다 ({why})')
            self._attach_then_lift()
            return
        if ok:
            self._attach_then_lift()
            return
        limit = int(self.p('grip_narrow_retries'))
        if self._retry_narrow < limit:
            # 놓고 재인식(RELEASE_RETRY)하기 전에, 같은 자세·같은 접근에서 더 좁게 한 번
            # 더 닫아본다 — width_m 이 병처럼 단면이 급변하는 물체의 "최대 폭"이라 얇은
            # 부위(목)를 노렸을 때 손가락이 접촉 전에 멈춘 것일 수 있다(_grip_width 참고).
            self._retry_narrow += 1
            self.width_m = max(0.0, self.width_m - float(self.p('grip_narrow_step_m')))
            self.get_logger().warn(
                f'파지 실패 ({why}) — 좁게 재시도 {self._retry_narrow}/{limit} '
                f'({self.width_m * 1000:.1f} mm)')
            self._to(State.CLOSE, '좁게 재시도')
        else:
            self._to(State.RELEASE_RETRY, f'파지 실패 ({why})')

    def _attach_then_lift(self):
        if self._fut is None:
            self._fut = self.moveit.attach_async(
                self.p('object_id'), self.p('ee_link'), list(self.p('gripper_links')))
            return
        if not self._fut.done():
            return
        self._to(State.LIFT, '물체 attach — 이제 부피가 팔을 따라다닌다')

    def _st_release_retry(self):
        if not self._extra:
            self._extra = [self.rg2.open_async()]
            self._retry_grip += 1
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        if self._retry_grip > int(self.p('grip_retries')):
            self._abort(f'파지 재시도 {self._retry_grip}회 실패')
            return
        for k in ('pre_grasp', 'grasp', 'lift'):
            self.solutions.pop(k, None)
        self.poses.clear()
        # 곧장 PERCEIVE 로 가지 않는다 — 팔이 방금 그랩을 시도한 자리(물체 높이, 작업공간
        # 박스 안)에 그대로 있어, 거기서 재촬영하면 그리퍼 자신이 물체로 오인식된다.
        self._home_next = State.PERCEIVE
        self._to(State.HOME, f'재인식 전 홈 복귀 ({self._retry_grip}회차)')

    def _st_lift(self):
        # place 미지정 pick 이면 자동으로 놓지 않고 set_place 를 기다린다(WAIT_PLACE_TARGET).
        nxt = State.WAIT_PLACE_TARGET if self._wait_place else State.PLACE
        self._move('lift', nxt, State.ABORT)

    def _st_wait_place_target(self):
        """들어올린 뒤 place 미지정 — 물체를 문 채 사람의 결정을 기다린다.

        나가는 길 셋: `set_place`(→PLACE) · `release_now`(→RELEASE, 그 자리에서) ·
        `abort`. 셋 다 **사람이 말해야** 일어난다.

        🔴 **자동 내려놓기는 2026-08-12 사용자 결정으로 기본 꺼졌다** (2026-08-11 결정을
        뒤집는다). 예전엔 60s 뒤 기본 위치(basket)로 혼자 내려놨는데, 그게 "시간이 지나면
        팔이 저절로 움직인다"는 유일한 경로였다. 사람이 자리를 비운 사이 로봇이 스스로
        판단해 물체를 옮기는 것보다, 문 채로 서 있는 편이 예측 가능하다.

        ⚠️ 코드에 남은 옛 주석("무한 대기는 안전하지 않다")을 보고 이걸 되돌리지 말 것 —
        무기한 보유의 위험(발열·토크 부담)은 알고 받아들인 것이고, 대신 물리 비상정지
        버튼과 `cmd:"stow"`(정리 후 종료)가 안전장치다.

        파라미터는 지우지 않고 **`0.0 = 비활성`**(기본)으로 재정의했다 —
        `grip_narrow_retries`/`force_down_steps` 와 같은 관례다. 옛 동작이 필요하면
        코드가 아니라 yaml 로 되돌린다.
        """
        timeout = float(self.p('wait_place_timeout_sec'))
        if timeout > 0.0 and self._elapsed() > timeout:
            self.place_location = self.p('place_location')
            self._publish_place(self.place_location)
            self._to(State.PLACE,
                     f"place 대기 {timeout:.0f}s 초과 — 기본 위치 '{self.place_location}' 에 내려놓는다")
            return
        if self._elapsed() > self._nag * 10.0:
            self._nag += 1
            tail = (f"{timeout:.0f}s 후 기본 '{self.p('place_location')}' 에 내려놓는다"
                    if timeout > 0.0 else '사람이 정할 때까지 이대로 기다린다')
            self.get_logger().info(
                "놓을 위치 대기 중 — set_place/release_now(/vla) 또는 rqt '내려놓을 위치' 로 "
                f"지정. {tail}")

    def _st_paused(self):
        """✋ **아무것도 하지 않는다.** 그게 전부이고, 그게 사양이다.

        여기에 코드를 추가하고 싶어지면 먼저 이걸 읽어라 — 이 상태의 계약은
        "다음 명령이 올 때까지 대기"다. 시간 경과로 다음 중 **무엇도** 일어나면 안 된다:

            자동 재개 · 자동 내려놓기 · 자동 홈 복귀 · 타임아웃 ABORT ·
            mission 의 다음 물체로 진행 · 재인식

        그래서 `DEFAULT_TIMEOUTS` 에도 없다(`test_PAUSED_에는_제한시간이_없다`).

        나가는 길은 전부 **서비스 콜백**이다 — `/pick/resume` · `/pick/release_now` ·
        `/pick/place_location` · `/pick/home` · `/pick/abort` · `cmd:"stow"`.
        이 함수가 하는 일은 가끔 사람에게 상기시키는 로그 한 줄뿐이다.
        """
        if self._elapsed() > self._nag * 30.0:
            self._nag += 1
            held = ''
            if self._paused_from in HOLDING_STATES:
                held = " — 물체를 든 채다('놔줘' 또는 놓을 위치를 말하면 정리한다)"
            self.get_logger().info(
                f'✋ 정지 중 ({self._paused_from.name} 에서 멈춤){held}. '
                '"계속해" 또는 다음 지시를 기다린다')

    def _st_place(self):
        # _st_idle 이 잠근 self.place_location 을 쓴다. 정상 경로로는 여기 도달할 때 항상
        # PLACE_LOCATIONS 의 키다 — __init__ 이 파라미터 기본값을, _on_place_location 이
        # 토픽 오버라이드를 각각 진입 시점에 검증해서 막는다. .get() 의 기본값은 그 두 검증을
        # 모두 우회하는 경로가 생기더라도 조용히 잘못된 곳으로 움직이지 않기 위한 방어선이다.
        if bool(self.p('clear_octomap_before_place')) and not self._octomap_cleared:
            self._octomap_cleared = True
            self.moveit.clear_octomap_async()
            self.get_logger().warn('octomap 을 비웠다 — place 구간은 미모델링 장애물이 안 보인다')
        param_name = PLACE_LOCATIONS.get(self.place_location, PLACE_LOCATIONS['basket'])
        # on_fail=PLACE_RETRY: 여기서 motion_retries 를 소진해도 곧장 ABORT(→SAFE_STOP)로
        # 안 보낸다. 물체를 이미 들고 있어(HOLDING_STATES) 재인식부터 다시 하는 것보다
        # 사람이 다른 place_location 을 골라 재시도하는 편이 훨씬 싸다 — 아래
        # `_st_place_retry`/`_srv_retry_place` 참고.
        self._joint_move(param_name, State.RELEASE, on_fail=State.PLACE_RETRY)

    def _st_place_retry(self):
        """PLACE 모션이 motion_retries 를 소진한 뒤의 정지 대기.

        SAFE_STOP 과 달리 물체를 문 채다(HOLDING_STATES) — 아무것도 안 하고 사람이
        `/pick/place_location`으로 다른 위치를 고르고 `/pick/retry_place`를 부르길
        기다린다. DEFAULT_TIMEOUTS 에 없어 자동으로 안 끊긴다(SAFE_STOP/WAIT_APPROVAL과
        같은 패턴 — 사람 판단을 기다리는 상태는 시간제한을 안 건다).
        """
        if self._nag == 0:
            self._nag = 1
            self.get_logger().error(
                f"PLACE_RETRY — '{self.place_location}' 놓기 실패, 물체를 문 채 정지. "
                "다른 위치로 바꾸려면 /pick/place_location 후 "
                "ros2 service call /pick/retry_place std_srvs/srv/Trigger {}")

    def _st_release(self):
        if not self._extra:
            self._extra = [self.rg2.open_async()]
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        if self._fut is None:
            self._fut = self.moveit.detach_and_remove_async(self.p('object_id'),
                                                            self.p('ee_link'))
            return
        if not self._fut.done():
            return
        self._object_added = False
        self._home_next = State.IDLE
        self._to(State.HOME)

    def _st_home(self):
        self._joint_move('home_joints_deg', self._home_next)

    def _joint_move(self, param_name: str, nxt: State, on_fail: State = State.ABORT):
        """고정 관절자세로 이동. IK 가 필요 없어 해를 직접 만든다."""
        if param_name not in self.solutions:
            names = list(self.p('joint_names'))
            positions = geo.deg2rad(self.p(param_name))
            if len(names) != len(positions):
                self._abort(f'{param_name} 길이({len(positions)})가 '
                            f'joint_names({len(names)})와 다르다')
                return
            js = JointState()
            js.name = names
            js.position = positions
            self.solutions[param_name] = js
        self._move(param_name, nxt, on_fail)

    def _st_speak_fail(self):
        # TTS 는 이 ws 에 아직 없다. 로그 + /pick/state 로 통보한다.
        self.get_logger().warn(f"실패 통보: 타겟 '{self.target}'")
        self._cleanup_scene()
        self._fail_streak += 1
        if not self.p('voice_enabled'):
            self._to(State.IDLE)
        elif self._fail_streak >= MAX_FAIL_STREAK:
            # 여기서 멈추지 않으면 LISTENING 과 tick 주기로 왕복한다. IDLE 은 조용하고,
            # 사람이 /pick/start 를 다시 불러야 재개된다.
            self._to(State.IDLE, f'연속 실패 {self._fail_streak}회 — /pick/start 로 다시 시작')
        else:
            self._to(State.LISTENING)

    def _st_abort(self):
        self._to(State.SAFE_STOP)

    def _st_safe_stop(self):
        if self._nag == 0:
            self._nag = 1
            self.get_logger().error(
                'SAFE_STOP — 상황 확인 후 ros2 service call /pick/reset std_srvs/srv/Trigger {}')

    def _cleanup_scene(self):
        if self._object_added and self.moveit.scene.service_is_ready():
            self.moveit.detach_and_remove_async(self.p('object_id'), self.p('ee_link'))
            self._object_added = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 🔴 여기서 **모션을 시도하지 않는다.** 정리는 `cmd:"stow"`(/pick/stow)가 한다.
        #
        # 종료 훅에서 "그리퍼를 열고 홈 복귀"를 흉내내면 세 가지가 동시에 걸린다:
        #   ① 실행이 보장되지 않는다 — executor 가 이미 빠져나와 MoveIt 액션 피드백을
        #      못 받고, move_group 이 같은 launch 면 동시에 죽는다. SIGKILL·크래시·
        #      전원 차단에는 애초에 안 돈다.
        #   ② 순서가 뒤집힌다 — "열고 나서 복귀"는 물체를 **지금 있는 자리**에 떨어뜨린다.
        #      30 cm 상공이면 낙하고, 병·컵이면 깨진다.
        #   ③ HOLDING_STATES 에서 그리퍼를 안 여는 판단(_abort)과 정면으로 충돌한다.
        #
        # 그래서 여기서는 **즉시 끝나는 것만** 한다: 진행 중 goal 취소 + 경고.
        # 그리퍼는 건드리지 않는다 — 문 채 프로세스가 죽으면 RG2 는 전원이 살아 있는 한
        # 계속 물고 있고, 그게 떨어뜨리는 것보다 안전하다.
        if node._call is not None:
            node._call.cancel()
        if node.state in HOLDING_STATES:
            node.get_logger().error(
                '🔴 물체를 문 채 종료됐다 — 그리퍼는 그대로 둔다(떨어뜨리지 않으려고). '
                '다시 띄운 뒤 /pick/release_now 나 rqt 로 놓을 것. '
                "다음부터는 종료 전에 /pick/stow 를 부르면 놓을 자리로 간 뒤 정리된다")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
