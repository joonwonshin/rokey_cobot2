"""로봇·move_group 없이 검증 가능한 부분만 테스트한다.

여기서 검증하는 것은 넷뿐이고, 전부 **틀리면 조용히 물체를 망가뜨리거나 문서가 거짓말을 하는** 것이다:
  1. 폭 → rgwd 단위 변환 (mm 로 잘못 바꾸면 10배 좁게 명령한다)
  2. grasp 포즈의 접근축 오프셋 (월드 -Z 로 물러나면 비스듬한 grasp 에서 물체를 친다)
  3. 상태 전이표
  4. README 의 mermaid stateDiagram ↔ 전이표 (다이어그램과 코드가 갈라지는 걸 막는다)

    colcon test --packages-select pick_fsm
    python3 -m pytest src/pick_fsm/test/test_pick_fsm.py -q      # ROS 없이도 됨
"""

import math
import re
from pathlib import Path

import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

from pick_fsm import geometry as geo
from pick_fsm.task_manager import PARAM_DEFAULTS
from pick_fsm.moveit_bridge import OCTOMAP_ACM_NAME, merge_acm
from pick_fsm.rg2 import (
    RG2_BRACKET_LENGTH_M,
    RG2_MAX_RGWD,
    RG2_MODEL_WIDTH_M,
    fingertip_from_rg2_base_m,
    fingertip_length_m,
    grip_target_width_m,
    width_to_rgwd,
)
from pick_fsm.states import TRANSITIONS, State, is_allowed


# ── 1. 단위 변환 ─────────────────────────────────────────
def test_width_to_rgwd_는_1_10mm_단위다():
    # 드라이버 주석: "must be provided in 1/10th millimeters"
    assert width_to_rgwd(0.048) == 480          # mm(48)로 바꾸면 10배 좁다 = 으깬다
    assert width_to_rgwd(0.110) == RG2_MAX_RGWD


def test_width_to_rgwd_는_유효범위로_클램프한다():
    assert width_to_rgwd(-1.0) == 0
    assert width_to_rgwd(5.0) == RG2_MAX_RGWD   # 범위를 넘겨 보내면 드라이버가 무시/오동작


def test_grip_target_width_m_은_조임_여유를_뺀다():
    # 🔴 부호가 뒤집히면 손가락이 물체 표면 **앞에서** 멈춘다 — 힘이 안 걸려 grip_detected 가
    #    false 로 남고, 로그는 "그리퍼 닫기 68 mm" 라고 정상처럼 찍힌다. 가장 찾기 어려운 실패다.
    #    2026-08-09 이전 코드가 `+clearance` 였고, default_width_m(0.06)이 실제 물체보다 작은
    #    상수였던 덕에 우연히 조여져서 증상이 안 보였다.
    assert grip_target_width_m(0.075, 0.008) == pytest.approx(0.067)   # 사과 75mm -> 67mm
    assert grip_target_width_m(0.075, 0.008) < 0.075                   # 이게 핵심이다


def test_grip_target_width_m_은_모델_한계와_0_으로_클램프한다():
    assert grip_target_width_m(0.500, 0.008) == pytest.approx(RG2_MODEL_WIDTH_M)
    assert grip_target_width_m(0.003, 0.008) == 0.0    # 여유가 폭보다 커도 음수로 안 나간다


def test_fingertip_length_m_은_실측_보정점을_그대로_재현한다():
    # 2026-08-07 실측 3점 (닫힘/70mm/100mm) — 보간 함수는 이 점들에서 정확히 맞아야 한다
    assert fingertip_length_m(0.000) == pytest.approx(0.240)
    assert fingertip_length_m(0.070) == pytest.approx(0.240 - 0.017)
    assert fingertip_length_m(0.100) == pytest.approx(0.240 - 0.041)


def test_fingertip_length_m_은_구간_내에서_선형보간한다():
    assert fingertip_length_m(0.035) == pytest.approx(0.240 - 0.017 / 2)


def test_fingertip_length_m_은_음수와_표_밖_폭도_처리한다():
    assert fingertip_length_m(-1.0) == fingertip_length_m(0.0)     # 음수는 0으로 클램프
    beyond = fingertip_length_m(0.110)                             # RG2 최대 폭, 외삽 구간
    assert beyond < fingertip_length_m(0.100)                      # 계속 짧아져야 한다(단조)
    assert 0.0 < beyond < 0.240


def test_rg2_base_기준_손끝은_플랜지_기준보다_브라켓만큼_짧다():
    # grasp 포즈 원점은 rg2_base_link 다. 플랜지 기준 240 mm 를 그대로 쓰면 22 mm 더 나간다.
    assert fingertip_from_rg2_base_m(0.0) == pytest.approx(0.218)
    assert (fingertip_length_m(0.07) - fingertip_from_rg2_base_m(0.07)
            == pytest.approx(RG2_BRACKET_LENGTH_M))


# ── 2. 포즈 연산 ─────────────────────────────────────────
def _pose(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    (ps.pose.orientation.x, ps.pose.orientation.y,
     ps.pose.orientation.z, ps.pose.orientation.w) = quat
    return ps


def test_단위_쿼터니언의_축():
    p = _pose()
    assert geo.quat_axis_z(p.pose.orientation) == pytest.approx((0.0, 0.0, 1.0))
    assert geo.quat_axis_x(p.pose.orientation) == pytest.approx((1.0, 0.0, 0.0))


def test_pre_grasp_는_월드가_아니라_grasp_자신의_Z로_물러난다():
    # Y축 기준 90° 회전 → 로컬 +Z 가 월드 +X 를 향한다
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    assert geo.quat_axis_z(tilted.pose.orientation) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)

    pre = geo.pre_grasp(tilted, 0.10)
    # 월드 -Z(수직)로 물러났다면 z 가 0.2 가 됐을 것이다. 접근축 기준이면 x 가 0.4 다.
    assert pre.pose.position.x == pytest.approx(0.40)
    assert pre.pose.position.z == pytest.approx(0.30)
    # 자세는 건드리지 않는다 — 물러나면서 손목이 돌면 그건 다른 grasp 다
    assert pre.pose.orientation.y == pytest.approx(tilted.pose.orientation.y)


def test_lift_는_접근축이_아니라_월드_위쪽이다():
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    up = geo.lifted(tilted, 0.15)
    assert up.pose.position.z == pytest.approx(0.45)
    assert up.pose.position.x == pytest.approx(0.50)


def test_standoff_는_하강_종점과_lift_를_같이_민다():
    """LIFT 를 grasp 원점 기준으로 두면 물체를 문 채 접근축으로 도로 밀고 들어간다."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))     # 로컬 +Z 가 월드 +X
    poses = geo.plan_poses(tilted, approach_offset_m=0.10, standoff_m=0.025,
                           lift_offset_m=0.15)
    assert poses['pre_grasp'].pose.position.x == pytest.approx(0.40)
    # 종점은 grasp 원점(0.50)이 아니라 접근축으로 25 mm 앞
    assert poses['grasp'].pose.position.x == pytest.approx(0.475)
    # 🔴 lift 의 x 가 0.50 이면 물체를 문 채 25 mm 밀고 들어간 것이다
    assert poses['lift'].pose.position.x == pytest.approx(0.475)
    assert poses['lift'].pose.position.z == pytest.approx(0.45)


def test_standoff_는_approach_를_넘지_못하고_부호도_무시한다():
    """하강이 후진이 되는 설정을 만들 수 없어야 한다. 실기에서 튜닝하는 값이다."""
    assert geo.clamped_standoff(0.025, 0.10) == pytest.approx(0.025)
    assert geo.clamped_standoff(-0.025, 0.10) == pytest.approx(0.025)
    assert geo.clamped_standoff(0.30, 0.10) == pytest.approx(0.10)

    p = _pose(0.5, 0.0, 0.3)                                 # 접근축 = 월드 +Z
    poses = geo.plan_poses(p, approach_offset_m=0.10, standoff_m=0.30, lift_offset_m=0.15)
    # 클램프가 없으면 종점 z 가 pre_grasp(0.20)보다 위로 올라가 하강이 뒤집힌다
    assert poses['grasp'].pose.position.z == pytest.approx(poses['pre_grasp'].pose.position.z)


def test_standoff_0_이면_예전_동작_그대로다():
    p = _pose(0.5, 0.0, 0.3)
    poses = geo.plan_poses(p, approach_offset_m=0.10, standoff_m=0.0, lift_offset_m=0.15)
    assert poses['grasp'].pose.position.z == pytest.approx(0.30)
    assert poses['lift'].pose.position.z == pytest.approx(0.45)


def test_tcp_는_grasp_원점이_아니다():
    """접근이 기울면 그리퍼 base 는 물체에서 크게 벗어난다 (2026-08-05 오진 사례)."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    tcp = geo.tcp_of(tilted, 0.18)
    assert tcp == pytest.approx((0.68, 0.0, 0.3))       # 원점(0.5,0,0.3)에서 18 cm 떨어져 있다


def test_to_gripper_base_는_접근축을_보존하고_손가락만_90도_돌린다():
    """이게 깨지면 그리퍼가 물체를 폭이 아니라 긴 축으로 물거나, 90° 누워서 진입한다."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))    # 로컬 +Z 가 월드 +X
    out = geo.to_gripper_base(tilted)

    # ① 접근축(+Z)은 그대로여야 한다 — Z 축 둘레의 회전이므로
    assert geo.quat_axis_z(out.pose.orientation) == pytest.approx(
        geo.quat_axis_z(tilted.pose.orientation), abs=1e-9)
    # ② 손가락 방향(+X)은 원래의 +Y 로 간다 (로컬 요 +90°).
    #    Ry(90°) 에서 원래 +X 는 (0,0,-1), 원래 +Y 는 (0,1,0) 이다.
    assert geo.quat_axis_x(tilted.pose.orientation) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)
    assert geo.quat_axis_x(out.pose.orientation) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    # ③ 원점은 안 움직인다 — 두 프레임은 요만 다르다
    assert (out.pose.position.x, out.pose.position.y, out.pose.position.z) == pytest.approx(
        (0.5, 0.0, 0.3))


def test_to_gripper_base_는_단위쿼터니언에서_정확히_요_90도다():
    out = geo.to_gripper_base(_pose())
    assert geo.quat_axis_x(out.pose.orientation) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert geo.quat_axis_z(out.pose.orientation) == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_deg2rad():
    assert geo.deg2rad([0, 90, -30]) == pytest.approx([0.0, math.pi / 2, -math.pi / 6])


# ── 3. 상태 전이표 ───────────────────────────────────────
def test_모든_상태가_전이표에_있다():
    missing = [s.name for s in State if s not in TRANSITIONS]
    assert not missing, f'전이표에 없는 상태: {missing}'


def test_전이표의_목적지도_전부_정의된_상태다():
    for src, dsts in TRANSITIONS.items():
        for dst in dsts:
            assert isinstance(dst, State), f'{src} -> {dst}'


def test_문서의_주요_경로가_허용된다():
    path = [State.IDLE, State.LISTENING, State.PERCEIVE, State.SCENE_PREP, State.PLAN,
            State.WAIT_APPROVAL, State.STOW, State.APPROACH, State.OPEN_GRIPPER, State.DESCEND,
            State.CLOSE, State.VERIFY,
            State.LIFT, State.PLACE, State.RELEASE, State.HOME, State.IDLE]
    for a, b in zip(path, path[1:]):
        assert is_allowed(a, b), f'{a.name} -> {b.name} 가 막혀 있다'


def test_실패_경로도_허용된다():
    assert is_allowed(State.PLAN, State.NEXT_CANDIDATE)
    assert is_allowed(State.NEXT_CANDIDATE, State.PLAN)
    assert is_allowed(State.NEXT_CANDIDATE, State.SPEAK_FAIL)
    assert is_allowed(State.VERIFY, State.RELEASE_RETRY)
    # RELEASE_RETRY/SAFE_STOP 은 곧장 PERCEIVE/IDLE 로 가지 않고 HOME 을 거친다 —
    # 팔이 물체 높이에 남은 채 재촬영하면 그리퍼가 물체로 오인식된다(2026-08-07).
    assert is_allowed(State.RELEASE_RETRY, State.HOME)
    assert is_allowed(State.HOME, State.PERCEIVE)
    assert is_allowed(State.ABORT, State.SAFE_STOP)
    assert is_allowed(State.SAFE_STOP, State.HOME)
    assert is_allowed(State.HOME, State.IDLE)


def test_지름길은_막혀_있다():
    # 승인을 건너뛰고 바로 실행하는 전이가 있으면 안 된다
    assert not is_allowed(State.PLAN, State.APPROACH)
    # SAFE_STOP 에서 곧바로 동작 상태로 나가면 안 된다 — 사람 리셋을 거쳐야 한다
    assert not is_allowed(State.SAFE_STOP, State.APPROACH)
    # 인식 없이 계획으로 갈 수 없다
    assert not is_allowed(State.IDLE, State.PLAN)


def test_자기_자신으로의_전이는_상태_유지다():
    assert is_allowed(State.PERCEIVE, State.PERCEIVE)


# ── 3-1. README 다이어그램 ↔ 전이표 ──────────────────────
# states.py 가 경고하는 그 실패("다이어그램과 코드가 조용히 갈라진다")를 여기서 막는다.
_MERMAID = re.compile(r'```mermaid\n(.*?)```', re.S)
_EDGE = re.compile(r'^\s*(\w+)\s*-->\s*(\w+)\s*(?::.*)?$')

#: 전이표에 있지만 **일부러** 안 그린 전이. 이유 없이 여기 추가하지 말 것.
#: 모든 `* -> ABORT` 는 16개라 그리면 못 읽는다 → 다이어그램의 note 로 묶었다 (아래 테스트가 예외처리).
_UNDRAWN = {
    # 표에는 있지만 _st_listening 이 실제로 쓰지 않는 경로 (README 에 각주로 적어뒀다)
    (State.LISTENING, State.IDLE),
}


def _readme_state_edges() -> set:
    # README.md 가 아니라 PACKAGES.md 다 — 2026-08-08 문서 개편으로 다이어그램 정본이
    # `src/PACKAGES.md#pick_fsm` 으로 옮겨갔다(README.md 에는 stateDiagram 블록이 없다).
    readme = Path(__file__).resolve().parent.parent.parent / 'PACKAGES.md'
    if not readme.exists():                     # 설치 트리에서 돌 때는 PACKAGES.md 가 없다
        pytest.skip(f'PACKAGES.md 없음: {readme}')
    for block in _MERMAID.findall(readme.read_text(encoding='utf-8')):
        if 'stateDiagram' not in block:
            continue
        edges, unknown = set(), []
        for line in block.splitlines():
            m = _EDGE.match(line)
            if m is None:                       # `[*] --> IDLE`, note 본문, 노드 선언
                continue
            src, dst = m.groups()
            for name in (src, dst):
                if name not in State.__members__:
                    unknown.append(name)
            if not unknown:
                edges.add((State[src], State[dst]))
        assert not unknown, f'PACKAGES.md 다이어그램에 없는 상태 이름: {unknown}'
        return edges
    pytest.fail('PACKAGES.md 에 stateDiagram mermaid 블록이 없다')


def test_README_다이어그램의_전이가_전부_전이표에_있다():
    bogus = [(s.name, d.name) for s, d in _readme_state_edges() if not is_allowed(s, d)]
    assert not bogus, f'그림에만 있고 TRANSITIONS 에 없는 전이: {bogus}'


def test_전이표의_전이가_전부_README_에_그려져_있다():
    drawn = _readme_state_edges()
    # ABORT 와 PAUSED 는 목적지로서 제외한다 — 둘 다 "거의 모든 상태에서" 갈 수 있어
    # 그리면 17줄씩 늘어나 다이어그램이 못 읽게 된다. 대신 note 로 묶여 있다.
    bulk = {State.ABORT, State.PAUSED}
    missing = [(s.name, d.name)
               for s, dsts in TRANSITIONS.items() for d in dsts
               if d not in bulk and (s, d) not in drawn and (s, d) not in _UNDRAWN]
    assert not missing, f'전이표에는 있는데 그림에 없는 전이: {missing}'


# ── 3-1b. release_now — 지금 자리에서 놓기 ────────────────
# "집어서 들고만 있어줘" 다음의 "됐어, 놔". 이게 없을 때의 유일한 탈출구는
# basket 으로 보내기 아니면 abort 였다.

def test_RELEASE_로_가는_길은_팔이_멈춰_선_상태에서만_열린다():
    """세 곳뿐이다. 그리고 **셋 다 팔이 서 있다** — 그게 이 검정의 실제 내용이다.

      PLACE              목적지에 도착해서 놓는 정상 완료
      WAIT_PLACE_TARGET  들고 대기 중에 사람이 "그냥 거기 놔" (release_now)
      PAUSED             멈춰 선 채로 사람이 "그냥 거기 놔"

    나머지가 막혀 있어야 하는 이유: **이동 중**에 그리퍼가 열리면 물체가 어디로
    떨어질지 아무도 모른다. 30 cm 상공이면 그게 그대로 낙하다.

    새 출발지를 여기 추가할 때는 "그 상태에서 팔이 정지해 있는가"를 먼저 답해라.
    답이 '아니오'면 그건 낙하 경로다.
    """
    reaches_release = {s for s, dsts in TRANSITIONS.items() if State.RELEASE in dsts}
    assert reaches_release == {State.PLACE, State.WAIT_PLACE_TARGET, State.PAUSED}, \
        reaches_release


def test_목적지_대기는_여전히_물체를_문_상태다():
    """RELEASE 로 나갈 길이 생겼다고 HOLDING 에서 빠지면 안 된다 —
    abort 가 여기서 그리퍼를 열기 시작한다."""
    from pick_fsm.states import HOLDING_STATES                      # noqa: PLC0415

    assert State.WAIT_PLACE_TARGET in HOLDING_STATES


# ── 3-1c. PAUSED — 되돌릴 수 있는 일시정지 ────────────────
# 2026-08-12 사용자 결정: ① "멈춰"는 즉시 멈추고 다음 명령까지 대기한다.
# ② 다음 명령이 오면 그대로 한다 (재개를 위해 두 번 말하게 하지 않는다).

def test_PAUSED_에는_제한시간이_없다():
    """🔴 이 계획 전체에서 가장 조용히 깨지기 쉬운 것.

    `DEFAULT_TIMEOUTS` 에 PAUSED 가 들어가는 순간 ①이 죽는다 — 사람이 자리를 비운
    사이 타임아웃이 `_abort()` 를 부르고, 사용자 입장에선 "멈춰놨는데 혼자 중단됐다"가
    된다. 사람을 기다리는 상태(WAIT_APPROVAL·SAFE_STOP·PLACE_RETRY)가 전부 같은
    규약을 따른다.
    """
    from pick_fsm.task_manager import DEFAULT_TIMEOUTS              # noqa: PLC0415

    assert State.PAUSED not in DEFAULT_TIMEOUTS


def test_PAUSED_는_물체를_문_상태로_취급한다():
    """물체를 든 채로도 멈출 수 있다. 그때 그리퍼를 놓으면 일시정지가 아니라 낙하다."""
    from pick_fsm.states import HOLDING_STATES                      # noqa: PLC0415

    assert State.PAUSED in HOLDING_STATES


def test_멈출_게_있는_모든_상태에서_멈출_수_있다():
    """"멈춰"는 거부하면 안 되는 유일한 명령이다.

    새 상태를 추가한 사람이 pause 를 빠뜨려 **거기서만 안 멈추는** 구멍이 생기는 것을
    막는다. 그래서 전이를 손으로 적지 않고 `PAUSE_EXEMPT` 의 여집합으로 넣는다.
    """
    from pick_fsm.states import PAUSE_EXEMPT                        # noqa: PLC0415

    for state in State:
        if state in PAUSE_EXEMPT:
            continue
        assert is_allowed(state, State.PAUSED), f'{state.name} 에서 못 멈춘다'


# ── 3-1d. 손목 eye-in-hand seam (2026-08-12) ─────────────
# 구현이 아니라 **자리**다. 이 검정들은 그 자리가 실수로 채워지거나 사라지는 걸 잡는다.

def test_손목은_아직_꺼져_있다():
    """I9. 카메라도 hand-eye 캘리브도 없다. 기본값이 켜지면 REGRASP 에서 승인만
    기다리다 제한시간에 걸린다."""
    assert PARAM_DEFAULTS['regrasp_enabled'] is False
    assert _yaml_params()['regrasp_enabled'] is False


def test_하강_자세를_바꾸는_문은_하나다():
    """`_commit_grasp` 가 그 문이다. `committed_grasp` 에 직접 대입하는 코드가
    생기면 손목이 붙었을 때 `grasp_revision` 이 안 오르고, 그러면 옛 IK 해가
    유효한 것처럼 남는다 — 로그엔 새 자세, 로봇은 옛 자리로 간다."""
    source = (Path(__file__).resolve().parent.parent
              / 'pick_fsm' / 'task_manager.py').read_text(encoding='utf-8')
    # **대입만** 센다. 읽기(`geo.tcp_of(self.committed_grasp, ...)`)는 얼마든지 있어도 된다.
    assigns = [line.strip() for line in source.splitlines()
               if re.match(r'self\.committed_grasp\s*=[^=]', line.strip())]
    # __init__ 의 초기화 1개 + _commit_grasp 안의 1개. 그 외는 없어야 한다.
    assert len(assigns) == 2, f'committed_grasp 직접 대입이 늘었다: {assigns}'


def test_손목_파라미터는_예약만_되어_있다():
    """이름은 못박되 값은 살아 있으면 안 된다 — 받을 노드가 없다."""
    params = _yaml_params()
    for name in ('wrist_camera_ns', 'wrist_grasp_service',
                 'observation_offset_m', 'max_active_views'):
        assert name not in params, f'{name} 가 주석에서 풀렸다 (받을 노드가 없다)'


def test_시간_경과만으로_팔이_움직이는_경로는_기본값에_없다():
    """🔴 불변식 I12. 2026-08-12 사용자 결정.

    `WAIT_PLACE_TARGET` 의 자동 내려놓기가 마지막 하나였고 여기서 꺼진다. 사람이 자리를
    비운 사이 로봇이 스스로 물체를 옮기는 것보다, 문 채 서 있는 편이 예측 가능하다.

    파라미터를 **지우지 않고 0.0=끔**으로 둔 이유: 옛 동작이 필요해지면 코드가 아니라
    yaml 로 되돌릴 수 있어야 한다(`grip_narrow_retries` 와 같은 관례). 그래서 이 검정은
    "코드가 없다"가 아니라 **"기본값이 꺼져 있다"**를 지킨다.

    `DEFAULT_TIMEOUTS` 의 항목들은 전부 *멈추는* 방향(ABORT)이라 이 불변식과 안 부딪힌다.
    """
    assert PARAM_DEFAULTS['wait_place_timeout_sec'] == 0.0
    yaml_value = _yaml_params()['wait_place_timeout_sec']
    assert yaml_value == 0.0, f'yaml 이 자동 내려놓기를 켜 두었다: {yaml_value}'


def test_PAUSED_에서_시간으로_빠져나가는_길은_없다():
    """탈출구는 전부 사람의 명령에 대응한다. 자율 진행에 해당하는 것이 없어야 한다.

    특히 `SAFE_STOP` 이 직접 탈출구에 없는 것이 중요하다 — 있으면 "멈춰"가 시간이
    지나 저절로 파괴적 정지가 되는 길이 열린다. abort 는 **사람이나 하드웨어가**
    부를 때만 `ABORT` 를 거쳐 간다.
    """
    exits = TRANSITIONS[State.PAUSED]
    assert State.SAFE_STOP not in exits
    assert exits == {State.PERCEIVE, State.PLACE, State.WAIT_PLACE_TARGET,
                     State.RELEASE, State.HOME, State.IDLE, State.ABORT}


# ── 3-2. yaml ↔ declare_parameter 타입 ───────────────────
# 🔴 2026-08-08: `max_reach_m: 1` 하나 때문에 노드가 기동 즉시 죽었다.
#    YAML 은 `1` 을 INTEGER 로 읽고 rclpy 는 DOUBLE 기본값과의 불일치를 거부한다
#    (`InvalidParameterTypeException`). PyYAML 로만 열어보면 멀쩡해 보여서 눈으로는 못 잡는다.
#    같은 함정을 replan_delay(2026-08-08 이전)에서도 한 번 밟았다 → 여기서 자동화한다.
_YAML = Path(__file__).resolve().parent.parent / 'config' / 'pick_fsm.yaml'


def _yaml_params() -> dict:
    if not _YAML.exists():                      # 설치 트리에서 돌 때는 경로가 다르다
        pytest.skip(f'config 없음: {_YAML}')
    doc = yaml.safe_load(_YAML.read_text(encoding='utf-8'))
    return doc['task_manager']['ros__parameters']


def _kind(v) -> str:
    """rcl 이 보는 파라미터 타입. bool 이 int 의 서브클래스라 순서가 중요하다."""
    if isinstance(v, bool):
        return 'BOOL'
    if isinstance(v, int):
        return 'INTEGER'
    if isinstance(v, float):
        return 'DOUBLE'
    if isinstance(v, str):
        return 'STRING'
    if isinstance(v, list):
        # rcl 은 원소 타입이 섞인 배열도 거부한다 ([0, 0.0, 90] 같은 것)
        return 'ARRAY[' + '|'.join(sorted({_kind(x) for x in v})) + ']'
    return type(v).__name__


def test_yaml_값의_타입이_declare_parameter_기본값과_같다():
    bad = [f'{k}: {v!r} 는 {_kind(v)} 인데 기본값 {PARAM_DEFAULTS[k]!r} 은 {_kind(PARAM_DEFAULTS[k])}'
           for k, v in _yaml_params().items()
           if k in PARAM_DEFAULTS and _kind(v) != _kind(PARAM_DEFAULTS[k])]
    assert not bad, '노드가 기동 즉시 죽는다 (float 는 `0` 이 아니라 `0.0`):\n  ' + '\n  '.join(bad)


def test_yaml_에_선언되지_않은_파라미터가_없다():
    """오타는 예외 없이 조용히 무시된다 — 값을 바꿨는데 안 먹는 형태로 나타난다."""
    unknown = sorted(set(_yaml_params()) - set(PARAM_DEFAULTS))
    assert not unknown, f'declare_parameter 에 없는 키(무시된다): {unknown}'


# ── 4. ACM 병합 ──────────────────────────────────────────
# 🔴 2026-08-06 실측: PlanningScene 의 allowed_collision_matrix 는 is_diff 여도
#    **전체 교체**다. 내 행렬만 보냈더니 SRDF 의 disable_collisions 34개가 날아가
#    인접 링크가 자기충돌로 잡히고 IK 가 모든 포즈에서 NO_IK_SOLUTION 을 냈다.
#    아래 테스트가 그 회귀를 막는다.
def _acm(names, pairs=()):
    m = AllowedCollisionMatrix()
    m.entry_names = list(names)
    idx = {n: i for i, n in enumerate(names)}
    m.entry_values = [AllowedCollisionEntry(enabled=[False] * len(names)) for _ in names]
    for a, b in pairs:
        m.entry_values[idx[a]].enabled[idx[b]] = True
        m.entry_values[idx[b]].enabled[idx[a]] = True
    return m


def _enabled(m, a, b):
    idx = {n: i for i, n in enumerate(m.entry_names)}
    return m.entry_values[idx[a]].enabled[idx[b]]


def test_merge_acm_은_기존_항목을_보존한다():
    """SRDF 에서 온 disable_collisions 가 살아남아야 한다 — 이게 핵심이다."""
    cur = _acm(['link_6', 'tool0', 'rg2_base_link'],
               pairs=[('link_6', 'rg2_base_link'), ('tool0', 'rg2_base_link')])
    out = merge_acm(cur, 'pick_target', ['rg2_base_link'])
    assert _enabled(out, 'link_6', 'rg2_base_link')      # 날아가면 자기충돌로 IK 가 죽는다
    assert _enabled(out, 'tool0', 'rg2_base_link')


def test_merge_acm_이_대상물체를_추가하고_대칭이다():
    cur = _acm(['link_6', 'rg2_base_link'])
    out = merge_acm(cur, 'pick_target', ['rg2_base_link'])
    assert 'pick_target' in out.entry_names
    assert _enabled(out, 'rg2_base_link', 'pick_target')
    assert _enabled(out, 'pick_target', 'rg2_base_link')
    # 그리퍼가 아닌 링크는 물체와 부딪히면 안 된다 (허용하면 팔뚝으로 물체를 밀고 간다)
    assert not _enabled(out, 'link_6', 'pick_target')


def test_merge_acm_은_행렬을_정사각으로_유지한다():
    cur = _acm(['a', 'b'])
    out = merge_acm(cur, 'obj', ['a', 'c'])          # 'c' 는 기존에 없던 이름
    n = len(out.entry_names)
    assert all(len(r.enabled) == n for r in out.entry_values)


def test_merge_acm_의_octomap_은_옵션이다():
    cur = _acm(['rg2_base_link'])
    off = merge_acm(cur, 'obj', ['rg2_base_link'], allow_octomap=False)
    assert OCTOMAP_ACM_NAME not in off.entry_names   # 기본값에서 켜지면 사람 팔이 안 보인다
    on = merge_acm(cur, 'obj', ['rg2_base_link'], allow_octomap=True)
    assert _enabled(on, 'rg2_base_link', OCTOMAP_ACM_NAME)


def test_merge_acm_은_입력을_변형하지_않는다():
    cur = _acm(['a'])
    merge_acm(cur, 'obj', ['a'])
    assert cur.entry_names == ['a']                  # 원본을 건드리면 재시도 때 누적된다
    assert len(cur.entry_values[0].enabled) == 1
