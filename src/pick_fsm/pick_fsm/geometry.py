"""포즈 연산. ROS 메시지에만 의존하고 로봇/네트워크는 안 건드린다 → 단위테스트가 가능하다.

GraspGenX 가 주는 grasp 포즈의 규약 (`robot.py:59-61`):
    +Z = 접근축(approach) · +X = 손가락이 닫히는 방향 · 원점 = 그리퍼 base

🔴 이 프레임은 우리 `tool0` 이 **아니다** (2026-08-07 정정, md/context/constraints.md).
   `tool0` 의 접근축은 +Z 가 아니라 **+X** 다 — `onrobot_rg2.xacro:40` 의
   `rpy="1.5708 0 1.5708"` 이 rg2 의 +Z 를 tool0 의 +X 로 보낸다. 여기 있는 함수들이
   가정하는 "+Z=접근축" 프레임은 `to_gripper_base()` 를 거친 **`rg2_base_link`** 목표다.
   변환 전 포즈를 `ik_link='tool0'` 로 넘기면 그리퍼가 90° 누운 채 진입한다.

여기서 scipy 를 쓰지 않는다. 필요한 건 쿼터니언에서 축 하나 뽑는 것뿐인데
그 한 줄 때문에 노드 실행 의존성을 늘릴 이유가 없다.
"""

import math

from geometry_msgs.msg import Pose, PoseStamped

#: GraspGenX 의 grasp 프레임 -> `rg2_base_link` 로 가는 로컬 요(yaw) 90°.
#: 근거: `x_grippers/onrobot_RG2/gripper.urdf:8-12` 의 `world -> onrobot_rg2_base_link`
#: 가 `rpy="0 0 1.5708"` 이다. grasp 는 그 `world` 프레임에서 주어지므로 그리퍼 base 의
#: 자세는 grasp 자세에 이 요를 **오른쪽에서** 곱한 것이다.
#: 교차확인: `config.json` 의 `sweep_volume.extents[0]=0.102`, `bbox` x 가 ±76 mm
#: → grasp 프레임에서 개구 방향은 X. `rg2_base_link` 안에서는 너클이 ±Y 로 벌어진다
#: (`onrobot_rg2_model_macro.xacro:158` `xyz="0 0.017178 0.125797"`, `axis="1 0 0"`).
_SQRT1_2 = math.sqrt(0.5)


def quat_axis_x(q) -> tuple[float, float, float]:
    """쿼터니언이 나타내는 회전행렬의 1열 = 로컬 +X축을 월드에서 본 방향."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w))


def quat_axis_z(q) -> tuple[float, float, float]:
    """회전행렬의 3열 = 로컬 +Z축. grasp 포즈에서는 **접근축**이다."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y))


def to_gripper_base(grasp: PoseStamped) -> PoseStamped:
    """GraspGenX grasp 포즈 -> `rg2_base_link` 목표 자세. **IK 에 넘기기 전에 반드시 거친다.**

    원점은 그대로다 (양쪽 프레임 원점이 같은 점이다 — 요만 다르다). 자세만 로컬 +Z 축
    기준 90° 돌린다. +Z 를 돌리는 회전이라 **접근축은 안 바뀐다** — 그래서
    `pre_grasp()` · `tcp_of()` 는 변환 전후 어느 쪽에 걸어도 같은 답을 준다.
    바뀌는 건 접근축을 중심으로 한 손가락 방향이고, 그게 틀리면 손가락이 물체를
    잡는 폭이 아니라 긴 축을 물게 된다.
    """
    q = grasp.pose.orientation
    out = PoseStamped()
    out.header = grasp.header
    # 값 복사다 — 대입하면 구독 메시지와 같은 객체를 공유하게 된다 (`translated()` 와 같은 관례)
    out.pose.position.x = grasp.pose.position.x
    out.pose.position.y = grasp.pose.position.y
    out.pose.position.z = grasp.pose.position.z
    out.pose.orientation.x = _SQRT1_2 * (q.x + q.y)
    out.pose.orientation.y = _SQRT1_2 * (q.y - q.x)
    out.pose.orientation.z = _SQRT1_2 * (q.w + q.z)
    out.pose.orientation.w = _SQRT1_2 * (q.w - q.z)
    return out


def translated(pose: Pose, axis: tuple[float, float, float], dist: float) -> Pose:
    """자세는 그대로 두고 위치만 `axis` 방향으로 `dist` [m] 옮긴 새 Pose."""
    out = Pose()
    out.orientation = pose.orientation
    out.position.x = pose.position.x + axis[0] * dist
    out.position.y = pose.position.y + axis[1] * dist
    out.position.z = pose.position.z + axis[2] * dist
    return out


def pre_grasp(grasp: PoseStamped, back_off_m: float) -> PoseStamped:
    """접근축(-Z)으로 `back_off_m` 만큼 물러난 포즈.

    **월드 -Z(수직 위)가 아니라 grasp 자신의 -Z 다.** 비스듬히 접근하는 grasp 에서
    수직으로 물러나면 하강 구간이 직선이 아니게 되어 손가락이 물체 옆구리를 친다.
    """
    out = PoseStamped()
    out.header = grasp.header
    out.pose = translated(grasp.pose, quat_axis_z(grasp.pose.orientation), -abs(back_off_m))
    return out


def lifted(grasp: PoseStamped, up_m: float) -> PoseStamped:
    """월드(base_link) +Z 로 들어올린 포즈. 여기는 접근축이 아니라 **중력 반대**가 맞다."""
    out = PoseStamped()
    out.header = grasp.header
    out.pose = translated(grasp.pose, (0.0, 0.0, 1.0), abs(up_m))
    return out


def clamped_standoff(standoff_m: float, approach_offset_m: float) -> float:
    """실제로 적용되는 standoff [m]. 로그가 설정값이 아니라 이 값을 찍어야 클램프가 보인다."""
    return min(abs(standoff_m), abs(approach_offset_m))


def plan_poses(grasp: PoseStamped, approach_offset_m: float, standoff_m: float,
               lift_offset_m: float) -> dict[str, PoseStamped]:
    """PLAN 이 IK 를 푸는 3점. `standoff_m` = 하강 종점을 접근축 -Z 로 덜 내리는 양.

    🔴 `lift` 는 grasp 원점이 아니라 **하강 종점**에서 올라간다. 그리퍼가 닫힌 자리가
    거기이기 때문이다 — 원점 기준으로 올리면 물체를 문 채로 접근축 +Z 로 standoff 만큼
    **도로 밀고 들어간 뒤** 상승한다(기울어진 grasp 에선 그 성분이 수평이라 물체를 문지른다).

    `standoff_m` 은 `approach_offset_m` 으로 클램프한다. 더 크면 DESCEND 가 pre-grasp 보다
    뒤가 되어 하강이 **후진**이 된다. 부호는 무시한다(`pre_grasp` 규약과 같이 항상 후퇴).
    """
    hold = pre_grasp(grasp, clamped_standoff(standoff_m, approach_offset_m))
    return {
        'pre_grasp': pre_grasp(grasp, approach_offset_m),
        'grasp': hold,
        'lift': lifted(hold, lift_offset_m),
    }


def tcp_of(grasp: PoseStamped, tcp_offset_m: float) -> tuple[float, float, float]:
    """손끝 위치. grasp 원점 + offset x (+Z축).

    grasp 원점은 그리퍼 base 라 **물체 위치가 아니다.** 접근이 30° 기울면 원점은
    물체에서 9 cm 옆에 앉는다 (2026-08-05 에 이걸 "좌표 오차"로 오진한 이력).
    로그·CollisionObject 배치에는 이쪽을 쓴다.
    """
    a = quat_axis_z(grasp.pose.orientation)
    p = grasp.pose.position
    return (p.x + a[0] * tcp_offset_m, p.y + a[1] * tcp_offset_m, p.z + a[2] * tcp_offset_m)


def reach_of(pose: PoseStamped) -> float:
    """base_link 원점에서의 거리 [m]. 도달범위 사전 판정용."""
    p = pose.pose.position
    return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)


def deg2rad(values) -> list[float]:
    """두산 펜던트/`robot_control` 은 도(deg), MoveIt JointConstraint 는 라디안이다."""
    return [math.radians(float(v)) for v in values]
