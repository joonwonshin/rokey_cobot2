"""move_group 과의 통신만 담당한다. 상태 전이 판단은 하지 않는다.

move_group 은 전역 네임스페이스에 뜬다 (`m0609_rg2_moveit/launch/moveit.launch.py:230`,
`name='move_group'`, namespace 지정 없음) → 서비스/액션 이름이 전부 `/…` 절대경로다.

프레임은 `world` 가 아니라 **`base_link`** 다. SRDF 에 virtual_joint(parent="world") 가 있지만
MoveIt 은 fixed 타입 virtual joint 로 모델 프레임을 만들지 않아 플래닝 프레임이 루트 링크로 남는다.
`frame_id='world'` 로 CollisionObject 를 보내면 `Unknown frame: world` 로 **조용히 무시된다**
(md/context/constraints.md, 2026-08-02 실측).
"""

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty

#: MoveIt 이 octomap 을 AllowedCollisionMatrix 에서 부르는 이름.
#: `collision_detection::World::OCTOMAP_NS`. libmoveit_planning_scene.so 의 문자열로 확인함.
OCTOMAP_ACM_NAME = '<octomap>'

SUCCESS = MoveItErrorCodes.SUCCESS

# 에러코드 → 사람 말. 숫자만 찍으면 로그에서 원인이 안 보인다.
# ⚠️ `vars(MoveItErrorCodes)` 로는 안 된다 — rosidl 이 만든 상수는 클래스가 아니라
#    **메타클래스**에 붙어 있어서 `__dict__` 에 없고 `dir()`/`getattr` 로만 보인다.
_ERR: dict[int, str] = {}
for _name in dir(MoveItErrorCodes):
    if _name.isupper() and _name != 'SLOT_TYPES':
        _val = getattr(MoveItErrorCodes, _name)
        if isinstance(_val, int):
            _ERR.setdefault(_val, _name)


def err_name(code: int) -> str:
    return f'{_ERR.get(code, "UNKNOWN")}({code})'


def merge_acm(current: AllowedCollisionMatrix, obj_id: str, links,
              allow_octomap: bool = False) -> AllowedCollisionMatrix:
    """기존 ACM 에 (그리퍼 링크 ↔ 대상물체[, ↔ octomap]) 허용을 **더한** 새 ACM.

    🔴 이 함수가 존재하는 이유 (2026-08-06 실측으로 밝혀짐):
       `PlanningScene.allowed_collision_matrix` 는 `is_diff=true` 여도 **병합이 아니라 전체 교체**다.
       내 7개짜리 행렬만 보냈더니 SRDF 의 `disable_collisions` 34개가 통째로 사라져서
       `rg2_base_link ↔ rg2_left_outer_knuckle` 같은 **인접 링크가 자기충돌**로 잡혔고,
       `avoid_collisions=true` IK 가 모든 포즈에서 NO_IK_SOLUTION 을 냈다.
       진단: `/check_state_validity` 의 contacts 가 전부 인접 링크쌍이었다.
       → 반드시 `/get_planning_scene` 으로 현재 ACM 을 읽어 여기에 얹어서 되돌려준다.

    순수 함수다 — 네트워크를 안 타므로 단위테스트로 고정해 둔다.
    """
    out = AllowedCollisionMatrix()
    out.entry_names = list(current.entry_names)
    out.entry_values = [AllowedCollisionEntry(enabled=list(e.enabled))
                        for e in current.entry_values]
    out.default_entry_names = list(current.default_entry_names)
    out.default_entry_values = list(current.default_entry_values)

    targets = [obj_id] + ([OCTOMAP_ACM_NAME] if allow_octomap else [])
    for name in targets + list(links):
        if name not in out.entry_names:
            out.entry_names.append(name)
            for row in out.entry_values:
                row.enabled.append(False)          # 기존 행에 새 열
            out.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(out.entry_names)))
    # 행 길이가 짧은 게 섞여 있으면(상류가 그렇게 줄 수 있다) 인덱싱이 조용히 어긋난다
    for row in out.entry_values:
        if len(row.enabled) < len(out.entry_names):
            row.enabled.extend([False] * (len(out.entry_names) - len(row.enabled)))

    idx = {n: i for i, n in enumerate(out.entry_names)}
    for link in links:
        for tgt in targets:
            i, j = idx[link], idx[tgt]
            out.entry_values[i].enabled[j] = True
            out.entry_values[j].enabled[i] = True
    return out


class ActionCall:
    """액션 호출 2단계(goal 수락 → result)를 폴링 한 번으로 감싼다.

    타이머 콜백 안에서 `spin_until_future_complete` 를 부르면 재진입으로 엉킨다
    (executor 가 이미 이 노드를 돌리고 있다). 그래서 **기다리지 않고 매 tick 확인**한다.
    """

    def __init__(self, client: ActionClient, goal):
        self.goal_future = client.send_goal_async(goal)
        self.handle = None
        self.result_future = None
        self.rejected = False

    def poll(self):
        """(끝났나, 결과). 거부됐으면 (True, None) 이고 `rejected` 가 선다."""
        if self.result_future is None:
            if not self.goal_future.done():
                return False, None
            self.handle = self.goal_future.result()
            if self.handle is None or not self.handle.accepted:
                self.rejected = True
                return True, None
            self.result_future = self.handle.get_result_async()
            return False, None
        if not self.result_future.done():
            return False, None
        wrapped = self.result_future.result()
        return True, (None if wrapped is None else wrapped.result)

    def cancel(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()


class MoveItBridge:
    def __init__(self, node, callback_group, base_frame='base_link'):
        self._node = node
        self.base_frame = base_frame
        self.ik = node.create_client(GetPositionIK, '/compute_ik',
                                     callback_group=callback_group)
        self.scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                        callback_group=callback_group)
        self.get_scene = node.create_client(GetPlanningScene, '/get_planning_scene',
                                            callback_group=callback_group)
        self.clear_octomap = node.create_client(Empty, '/clear_octomap',
                                                callback_group=callback_group)
        self.move = ActionClient(node, MoveGroup, '/move_action',
                                 callback_group=callback_group)

    def ready(self) -> tuple[bool, str]:
        missing = [n for n, c in (('/compute_ik', self.ik),
                                  ('/apply_planning_scene', self.scene),
                                  ('/get_planning_scene', self.get_scene))
                   if not c.service_is_ready()]
        if not self.move.server_is_ready():
            missing.append('/move_action')
        return (not missing), ('연결됨' if not missing else f'대기 중: {", ".join(missing)}')

    # ── IK ──────────────────────────────────────────────────
    def ik_async(self, pose: PoseStamped, group: str, ik_link: str,
                 seed=None, avoid_collisions=True, timeout_sec=0.2):
        """`ik_link` 가 `pose` 자세를 갖게 하는 관절해 요청.

        seed 는 직전 해의 JointState. **이걸 넘기는 게 중요하다** — 안 주면 매번 현재
        자세에서 풀어서 pre-grasp 와 grasp 의 해가 다른 IK 분기에 앉을 수 있고,
        그러면 10 cm 하강이 팔 전체를 뒤집는 궤적이 된다.
        """
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group
        r.ik_link_name = ik_link
        r.pose_stamped = pose
        r.avoid_collisions = avoid_collisions
        r.timeout = Duration(sec=int(timeout_sec), nanosec=int((timeout_sec % 1) * 1e9))
        r.robot_state = RobotState()
        # is_diff=True + 빈 joint_state = "현재 상태를 그대로 쓴다".
        # move_group 의 kinematics capability 가 robotStateMsgToRobotState 로 병합한다.
        r.robot_state.is_diff = True
        if seed is not None:
            r.robot_state.joint_state = seed
        return self.ik.call_async(req)

    # ── 계획/실행 ────────────────────────────────────────────
    def move_to_joints_async(self, joint_state, group, joint_names, *, plan_only,
                             vel_scale, acc_scale, planning_time, attempts,
                             tolerance, replan, replan_attempts, replan_delay,
                             pipeline='ompl', planner_id=''):
        """관절공간 목표로 계획(+실행). 포즈 목표가 아니라 관절 목표를 쓰는 이유:

        포즈 목표를 주면 IK 를 move_group 이 다시 풀어서 **우리가 도달 가능하다고 판정한
        그 해가 아닌 다른 해**로 갈 수 있다. 3점(pre-grasp/grasp/lift)의 연속성을
        우리가 이미 시드로 확보했으므로, 그 해를 그대로 목표로 준다.
        """
        pos = {n: p for n, p in zip(joint_state.name, joint_state.position)}
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.pipeline_id = pipeline
        req.planner_id = planner_id
        req.num_planning_attempts = int(attempts)
        req.allowed_planning_time = float(planning_time)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)
        req.start_state.is_diff = True          # 현재 상태에서 출발

        c = Constraints()
        c.name = 'joint_goal'
        for name in joint_names:
            if name not in pos:
                raise KeyError(f'IK 해에 관절 {name} 이 없다 (해: {list(pos)})')
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos[name])
            jc.tolerance_above = float(tolerance)
            jc.tolerance_below = float(tolerance)
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]

        opt = goal.planning_options
        opt.plan_only = bool(plan_only)
        # ⚠️ 여기가 문서의 STOP_REPLAN 이다. move_group 의 PlanExecution 이 실행 중
        #    planning scene 갱신을 감시하다가 궤적이 무효가 되면 멈추고 다시 계획한다.
        #    "장애물이 사라지면 재개"가 아니라 "새 경로가 나오면 재개"라 사람이 비켜주지
        #    않아도 돌아간다. FSM 은 이 루프를 다시 구현하지 않고 바깥 재시도만 센다.
        opt.replan = bool(replan)
        opt.replan_attempts = int(replan_attempts)
        opt.replan_delay = float(replan_delay)
        opt.planning_scene_diff.is_diff = True
        opt.planning_scene_diff.robot_state.is_diff = True
        return ActionCall(self.move, goal)

    # ── Planning Scene ──────────────────────────────────────
    def _scene_diff(self) -> PlanningScene:
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        return s

    def make_object(self, obj_id: str, center, radius_m: float) -> CollisionObject:
        """대상 물체를 구 하나로 근사한 CollisionObject.

        구인 이유: 우리가 아는 건 손끝 좌표 하나와 개구 폭뿐이다. 메시가 없는데 박스를
        놓으면 **없는 방향 정보를 지어내는 것**이 된다. 반지름은 파라미터로 노출한다.
        """
        co = CollisionObject()
        co.header.frame_id = self.base_frame
        co.header.stamp = self._node.get_clock().now().to_msg()
        co.id = obj_id
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(radius_m)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
        pose.orientation.w = 1.0
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def get_acm_async(self):
        """현재 ACM 만 읽어온다. `merge_acm` 의 입력이다 (그냥 덮어쓰면 SRDF 가 날아간다)."""
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        return self.get_scene.call_async(req)

    def add_object_async(self, obj: CollisionObject, acm: AllowedCollisionMatrix = None):
        """물체 등록 + (있으면) 병합된 ACM 적용.

        ACM 이 필요한 이유: 넣지 않으면 grasp pose 에서 손가락이 물체와 겹쳐
        **IK 가 collision 으로 실패**한다 — 잡으러 가는 게 목적인데 닿는 걸 금지하는 셈이다.
        `acm` 은 반드시 `merge_acm(현재 ACM, …)` 의 결과여야 한다.
        """
        scene = self._scene_diff()
        scene.world.collision_objects = [obj]
        if acm is not None:
            scene.allowed_collision_matrix = acm
        req = ApplyPlanningScene.Request()
        req.scene = scene
        return self.scene.call_async(req)

    def attach_async(self, obj_id: str, link: str, touch_links) -> object:
        """물체를 그리퍼에 붙인다. 들고 이동할 때 **그 부피가 따라가게** 하는 것이 목적이다."""
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.header.frame_id = self.base_frame
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = list(touch_links)
        scene = self._scene_diff()
        scene.robot_state.attached_collision_objects = [aco]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        return self.scene.call_async(req)

    def detach_and_remove_async(self, obj_id: str, link: str):
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = self.base_frame
        co.operation = CollisionObject.REMOVE
        scene = self._scene_diff()
        scene.robot_state.attached_collision_objects = [aco]
        scene.world.collision_objects = [co]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        return self.scene.call_async(req)

    def clear_octomap_async(self):
        """octomap 전체 삭제. **잔상 때문에 필요할 때가 있지만 공짜가 아니다** —
        사람 팔을 포함한 모든 미모델링 장애물이 같이 사라지고, 다시 관측될 때까지 안 보인다."""
        return self.clear_octomap.call_async(Empty.Request())
