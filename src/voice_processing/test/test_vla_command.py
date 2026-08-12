"""`parse_command` 스키마 검증 테스트. 로봇도 카메라도 GPU 도 필요 없다.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 -m pytest src/voice_processing/test/test_vla_command.py -q

⚠️ **source 는 필요하다.** `parse_command` 자체는 rclpy 를 안 쓰지만 모듈 최상단이
`import rclpy` 라 ROS 환경 없이는 collection 단계에서 죽는다.

여기서 지키는 계약은 하나다: **필드가 없거나 타입이 다르면 조용히 기본값으로 진행하지
않는다.** `dry_run` 이 제거된 뒤로 잘못 받은 지시는 곧바로 실제 모션이 된다
(`md/plans/2026-08-08-vla-integration.md` §0-B).
"""

import ast
import json
from pathlib import Path

import pytest

from voice_processing.vla_command_node import (
    ABANDON_STATES,
    BYPASS_STATE,
    DONE_STATE,
    FAIL_STATES,
    RELEASE_STATE,
    classify_cmd,
    parse_command,
)

try:
    from pick_fsm.states import State
    from pick_fsm.task_manager import DEFAULT_TIMEOUTS
except ImportError:
    # pick_fsm 이 이 환경에 빌드/설치돼 있지 않다 — voice_processing 은 런타임에도
    # pick_fsm 에 의존하지 않으므로(토픽/서비스로만 통신) package.xml 에 의존성을
    # 추가하지 않는다. 여기서는 skip 으로만 처리한다.
    State = None
    DEFAULT_TIMEOUTS = None

DETECT = {'apple', 'orange', 'banana', 'cup'}

requires_pick_fsm = pytest.mark.skipif(
    State is None, reason='pick_fsm 이 설치돼 있지 않다 (빌드/소스 안 됨)')


def cmd(**fields) -> str:
    payload = {'cmd': 'pick', 'class': 'apple'}
    payload.update(fields)
    return json.dumps(payload)


# ── 받아들이는 것 ───────────────────────────────────────────
def test_minimal_pick():
    out, warn = parse_command(cmd())
    assert out['class'] == 'apple'
    assert out['request_id'] == ''
    assert warn == ''


def test_cmd_defaults_to_pick():
    out, _ = parse_command('{"class": "apple"}')
    assert out is not None and out['cmd'] == 'pick'


def test_pick_and_place_is_the_same_thing():
    # 이 FSM 의 pick 사이클은 어차피 place_joints_deg 에 놓는 것으로 끝난다.
    out, _ = parse_command(cmd(cmd='pick_and_place'))
    assert out is not None


def test_vla_side_field_name_class_name():
    # VLA 의 SceneObject 는 `class_name` 이다. 그쪽 이름 그대로 보내도 받는다.
    out, _ = parse_command('{"cmd": "pick", "class_name": "orange"}')
    assert out['class'] == 'orange'


def test_comma_list_survives():
    # target_classes 가 원래 콤마 목록이고 FSM 이 그대로 브리지에 넘긴다.
    out, _ = parse_command(cmd(**{'class': 'apple,orange'}), allowed_classes=DETECT)
    assert out['class'] == 'apple,orange'


def test_request_id_is_echoed():
    out, _ = parse_command(cmd(request_id='a17-3'))
    assert out['request_id'] == 'a17-3'


# ── 거부하는 것 ─────────────────────────────────────────────
def test_not_json():
    out, why = parse_command('그냥 사과 집어')
    assert out is None and 'JSON' in why


def test_json_but_not_object():
    out, why = parse_command('["apple"]')
    assert out is None and why


def test_unknown_cmd():
    out, why = parse_command(cmd(cmd='drop'))
    assert out is None and 'cmd' in why


def test_empty_class():
    out, why = parse_command('{"cmd": "pick", "class": "  "}')
    assert out is None and 'class' in why


def test_class_with_space_is_rejected():
    # FSM 은 /get_keyword 응답을 공백으로 쪼개 첫 단어만 쓴다 — 뒷부분이 조용히 사라진다.
    out, why = parse_command(cmd(**{'class': 'apple orange'}))
    assert out is None and '공백' in why


def test_class_outside_detect_list():
    out, why = parse_command(cmd(**{'class': 'hammer'}), allowed_classes=DETECT)
    assert out is None and 'hammer' in why


def test_allowed_classes_empty_means_no_check():
    out, _ = parse_command(cmd(**{'class': 'hammer'}))
    assert out is not None


def test_place_wrong_shape_is_rejected():
    # 2026-08-10: place 는 이제 basket/table/discard 이름 하나만 받는다 — 좌표/객체는 여전히 거부.
    out, why = parse_command(cmd(place={'kind': 'named', 'value': 'basket'}))
    assert out is None and 'place' in why


def test_falsy_place_is_also_rejected():
    # truthiness 로 보면 `{}` · `""` · `0` 이 통과한다 — 키의 존재로 판정해야 한다.
    for falsy in ({}, '', 0, []):
        out, why = parse_command(cmd(place=falsy))
        assert out is None and 'place' in why, f'{falsy!r} 가 통과했다'


def test_place_accepted_value_passes_through():
    # pick_fsm.task_manager.PLACE_LOCATIONS 의 키와 같아야 한다 — 어긋나면 이 테스트가 잡는다.
    for value in ('basket', 'table', 'discard'):
        out, warn = parse_command(cmd(place=value))
        assert out is not None and out['place'] == value and warn == ''


def test_place_unknown_name_is_rejected():
    out, why = parse_command(cmd(place='attic'))
    assert out is None and 'place' in why and 'attic' in why


def test_place_absent_means_no_override():
    # place 를 아예 안 보내면 None — task_manager 쪽 파라미터 기본값(basket)이 그대로 쓰인다.
    out, _ = parse_command(cmd())
    assert out['place'] is None


def test_class_is_normalised():
    # 원문을 그대로 넘기면 `'apple,'` 이 target_classes 에 실려 빈 클래스를 찾게 된다.
    out, _ = parse_command(cmd(**{'class': 'apple,,orange,'}))
    assert out['class'] == 'apple,orange'


def test_class_of_only_commas():
    out, why = parse_command(cmd(**{'class': ',,,'}))
    assert out is None and 'class' in why


def test_pixel_without_pixel_wh():
    # 리사이즈된 프레임 위에서 찍은 좌표라면 기준 해상도 없이는 조용히 어긋난다.
    out, why = parse_command(cmd(pixel=[312, 188]))
    assert out is None and 'pixel_wh' in why


def test_pixel_wrong_shape():
    out, why = parse_command(cmd(pixel=[312], pixel_wh=[424, 240]))
    assert out is None and 'pixel' in why


def test_pixel_non_numeric():
    out, why = parse_command(cmd(pixel=['312', 188], pixel_wh=[424, 240]))
    assert out is None and 'pixel' in why


def test_pixel_wh_must_be_positive():
    out, why = parse_command(cmd(pixel=[312, 188], pixel_wh=[0, 240]))
    assert out is None and 'pixel_wh' in why


# ── pixel_policy 세 값 ──────────────────────────────────────
def test_pixel_warn_passes_with_warning():
    """기본값(warn): pixel 은 검증만 통과하고 실제로는 쓰이지 않는다(ignored)."""
    out, warn = parse_command(cmd(pixel=[312, 188], pixel_wh=[424, 240]))
    assert out is not None
    assert out['ignored'] == ['pixel']
    assert out['pixel'] == (312.0, 188.0)
    assert warn


def test_pixel_reject_policy():
    out, why = parse_command(cmd(pixel=[312, 188], pixel_wh=[424, 240]),
                             pixel_policy='reject')
    assert out is None and 'pixel_policy=reject' in why


def test_pixel_select_policy_not_ignored():
    """select: pixel 이 검증만 되는 게 아니라 실제로 쓰인다 — ignored 에 없어야 한다."""
    out, warn = parse_command(cmd(pixel=[312, 188], pixel_wh=[424, 240]),
                              pixel_policy='select')
    assert out is not None
    assert out['ignored'] == []
    assert out['pixel'] == (312.0, 188.0)
    assert out['pixel_wh'] == (424.0, 240.0)
    assert warn == ''


def test_launch_default_is_select_so_the_pixel_is_not_thrown_away():
    """🔴 계획 §0-4 · 테스트 T3/T11. 2026-08-12 에 warn → select 로 올렸다.

    `warn` 이면 VLA 가 "이거"라고 지목해 pixel 을 실어 보내도 **FSM 이 그 값을 버리고**
    클래스만으로 고른다. 사과가 하나면 우연히 맞고, 둘이면 지목과 무관하게 점수 높은
    쪽을 집는다 — 증상이 "가끔 엉뚱한 걸 집는다"로만 나와서 원인을 못 찾는 종류다.

    `parse_command` 의 정책 분기는 전부 `if 'pixel' in doc:` 안에 있으므로, pixel 없는
    평범한 클래스 지시는 select 여도 그대로 동작한다(no-op).
    """
    assert _launch_default('pixel_policy') == 'select'


def test_select_without_a_pixel_is_a_no_op():
    """select 로 올려도 클래스만 보내는 기존 지시가 깨지면 안 된다."""
    out, warn = parse_command(cmd(), pixel_policy='select')
    assert out is not None
    assert out['ignored'] == []
    assert out['pixel'] is None
    assert warn == ''


def test_base_xy_is_ignored_not_used():
    out, warn = parse_command(cmd(base_xy=[0.42, -0.18]))
    assert out is not None
    assert out['ignored'] == ['base_xy']
    assert warn


# ── cmd 라우팅 (rqt 패널 버튼 대응) ──────────────────────────
def test_classify_control_cmds():
    for word in ('start', 'abort', 'reset', 'home'):
        assert classify_cmd(word) == 'control'


def test_classify_approve_is_blocked():
    # rqt 패널의 '승인' 버튼에 해당한다 — 절대 열지 않는다(계획 §0-B).
    assert classify_cmd('approve') == 'blocked'


def test_classify_pick_and_unknown_both_fall_through_to_pick():
    # 'pick'/'pick_and_place'는 물론, 모르는 값도 parse_command 가 최종 거부하도록
    # 'pick' 갈래로 흘려보낸다.
    assert classify_cmd('pick') == 'pick'
    assert classify_cmd('pick_and_place') == 'pick'
    assert classify_cmd('drop') == 'pick'


# ── 경계 원본 이슈 A: LISTENING 타임아웃 3중 복제 대조 ─────────
# (code-audit-vla-integration.md 항목 A)
#
# task_manager.DEFAULT_TIMEOUTS[State.LISTENING] 이 원본이고, 이 값이
# vla_command_node 의 파라미터 기본값·launch 기본값 두 곳에 그대로 복제돼 있다.
# 자동으로 대조하는 코드가 없어 셋 중 하나만 바뀌면 조용히 어긋난다 — 사본이
# 원본보다 크면(margin 계산이 실제 FSM 타임아웃보다 늦게 걸려) SAFE_STOP 오경보로
# 이어진다.

def _node_declare_default(param_name: str):
    """`vla_command_node.py` 안 `self.declare_parameter(param_name, <default>)` 호출에서
    <default> 리터럴을 ast 로 뽑는다. 노드를 실제로 띄우지 않고(rclpy.init 불필요) 값만 본다."""
    src_path = Path(__file__).resolve().parents[1] / 'voice_processing' / 'vla_command_node.py'
    tree = ast.parse(src_path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and len(node.args) >= 2
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == param_name):
            return ast.literal_eval(node.args[1])
    raise AssertionError(f'declare_parameter({param_name!r}, ...) 를 소스에서 못 찾았다')


def _launch_default(arg_name: str):
    """`vla_command.launch.py` 의 `DeclareLaunchArgument(arg_name, default_value=...)` 에서
    default_value 리터럴을 ast 로 뽑는다."""
    src_path = Path(__file__).resolve().parents[1] / 'launch' / 'vla_command.launch.py'
    tree = ast.parse(src_path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == arg_name):
            for kw in node.keywords:
                if kw.arg == 'default_value':
                    return ast.literal_eval(kw.value)
    raise AssertionError(f'DeclareLaunchArgument({arg_name!r}, ...) 를 소스에서 못 찾았다')


@requires_pick_fsm
def test_node_default_matches_fsm_listening_timeout():
    node_default = float(_node_declare_default('fsm_listening_timeout_sec'))
    assert node_default == DEFAULT_TIMEOUTS[State.LISTENING]


@requires_pick_fsm
def test_launch_default_matches_fsm_listening_timeout():
    launch_default = float(_launch_default('fsm_listening_timeout_sec'))
    assert launch_default == DEFAULT_TIMEOUTS[State.LISTENING]


# ── 경계 원본 이슈 C: FSM 상태 이름 문자열 하드코딩 대조 ────────
# (code-audit-vla-integration.md 항목 C)
#
# vla_command_node 는 pick_fsm.states.State 를 import 하지 않고 상태 이름을
# 문자열('RELEASE', 'HOME', ...)로 하드코딩한다. FSM 쪽에서 상태 이름을 리네이밍하면
# 이쪽은 에러 없이 그냥 매칭이 조용히 끊긴다 — 이 테스트는 리네이밍을 막지는
# 못하지만, 리네이밍이 일어났을 때 즉시 fail 하게는 만든다.

@requires_pick_fsm
def test_hardcoded_state_names_still_exist_in_fsm():
    names = set(State.__members__)
    assert RELEASE_STATE in names
    assert DONE_STATE in names
    assert FAIL_STATES <= names
    assert ABANDON_STATES <= names
    assert BYPASS_STATE in names
