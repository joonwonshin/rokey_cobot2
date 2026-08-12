"""GUI 체크박스 -> `ros2 launch` 인자.

이 파일이 있는 이유는 같은 실수가 두 번 났기 때문이다(2026-08-11).

  1. launch가 `skill_tier_enabled`를 선언은 했지만 agent_node의
     `parameters=[]`로 넘기지 않아 `ros2 launch ... skill_tier_enabled:=true`가
     조용히 무시됐다.
  2. 고친 뒤에도 GUI의 "VLA 시작"이 그 인자를 아예 안 보내서, 체크 여부와
     무관하게 launch 기본값 false가 이겼다.

둘 다 예외를 던지지 않는다. 화면에는 규칙 계층이 켜진 것처럼 보이고, 증상은
며칠 뒤 "말한 규칙이 재시작하면 사라진다"로만 나타난다. 조용히 틀리는 배선은
조용히 틀리지 않게 만들어 두는 수밖에 없다.
"""

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

LAUNCH_FILE = (Path(__file__).resolve().parent.parent
               / "launch" / "vla_system.launch.py")


def gui_arguments(**flags) -> dict[str, str]:
    """GUI가 보내는 `이름:=값`을 딕셔너리로."""
    from vla_system.vla_gui import build_launch_command      # noqa: PLC0415

    command = build_launch_command(**flags)
    return dict(part.split(":=", 1) for part in command if ":=" in part)


ALL_ON = {"pick_bridge": True, "skill_tier": True, "perception": True}
ALL_OFF = {key: False for key in ALL_ON}


def test_the_rule_layer_checkbox_reaches_the_launch_command():
    assert gui_arguments(**ALL_ON)["skill_tier_enabled"] == "true"
    assert gui_arguments(**ALL_OFF)["skill_tier_enabled"] == "false"


def test_perception_can_be_turned_off_for_a_fake_stage():
    """dryrun_stage.py가 /vla/scene을 대신 낼 때 카메라 노드는 떠 있으면 안 된다."""
    assert gui_arguments(**ALL_OFF)["enable_perception"] == "false"
    assert gui_arguments(**ALL_ON)["enable_perception"] == "true"


def test_no_camera_means_no_camera():
    """"카메라 인식"을 끄면 RealSense도 안 뜬다.

    이 둘이 갈라져 있으면 가짜 무대로 돌릴 때 launch가 장치를 찾다 죽는다 --
    enable_realsense는 원래 pick_bridge에만 묶여 있어서, 단독 모드로 두고
    카메라만 끄면 오히려 카메라를 여는 조합이 나왔다.
    """
    for pick_bridge in (True, False):
        args = gui_arguments(**{**ALL_OFF, "pick_bridge": pick_bridge})
        assert args["enable_realsense"] == "false"


def test_every_argument_the_gui_sends_is_one_the_launch_file_declares():
    """오타나 이름 변경은 `ros2 launch`에서 에러가 아니라 무시로 나타난다."""
    declared = set(re.findall(r'DeclareLaunchArgument\(\s*"([^"]+)"',
                              LAUNCH_FILE.read_text(encoding="utf-8")))
    assert declared, "launch 파일을 못 읽었다"
    unknown = set(gui_arguments(**ALL_ON)) - declared
    assert not unknown, f"launch가 모르는 인자를 보내고 있다: {sorted(unknown)}"


def test_the_launch_file_forwards_the_rule_flag_into_the_agent():
    """선언만 하고 Node(parameters=[])에 안 넘기면 값이 사라진다 -- 실제로 그랬다.

    ``Node(`` 사이의 글자를 세지 않고 파일 끝까지 훑는 이유: 노드가 추가되거나
    삭제되면(2026-08-11 병합에서 robot_node·wrist_grasp_node가 사라졌다) 구분자에
    기대던 파싱이 검정 자체를 깨뜨린다. 배선이 멀쩡한데 빨간 줄이 뜨는 검정은
    다음 사람이 지워버린다.
    """
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    agent = source[source.index('executable="agent_node"'):]
    assert '"skill_tier_enabled": skill_tier_enabled' in agent
    assert '"rule_store_path": rule_store_path' in agent


def test_every_bridge_status_has_a_korean_label():
    """브리지가 낼 수 있는 status 는 전부 GUI 라벨을 가져야 한다.

    같은 부류의 조용한 배선 실수다(2026-08-12). `fsm_state_view()` 가
    WAIT_PLACE_TARGET 에 `waiting_place` 라는 **새 status** 를 주기 시작했는데
    GUI 의 라벨 맵에는 그 키가 없어서, `.get(status, status)` 폴백이 걸려
    조작자 화면에 영문 `waiting_place` 가 그대로 나갔다.

    예외도 경고도 안 난다. 나머지 상태가 전부 한글이라 한 줄만 영어로 뜨고,
    그게 하필 **사람이 목적지를 말해줘야 하는 순간**이다.

    새 status 를 추가할 때 라벨을 같이 넣게 만드는 것이 이 검정의 전부다.
    """
    from vla_system.bridge.pick_bridge import _FSM_STATE_INFO    # noqa: PLC0415
    from vla_system.vla_gui import robot_state_line              # noqa: PLC0415

    statuses = {status for status, _label in _FSM_STATE_INFO.values()}
    statuses.add("moving")      # 미등록 상태의 폴백 (fsm_state_view)

    missing = []
    for status in sorted(statuses):
        line = robot_state_line({
            "status": status, "holding_class": "", "motion_enabled": True,
            "current_action": "", "details": "", "last_action": "",
            "last_result": "",
        })
        # 라벨이 없으면 폴백으로 status 문자열 자체가 그대로 찍힌다.
        if line.startswith(status):
            missing.append(status)

    assert not missing, f"GUI 라벨이 없는 status: {missing}"


def test_chat_shows_only_the_conversation():
    """대화창에는 사람과 AI 가 주고받은 말만 남는다 (2026-08-12 사용자 결정).

    운영 로그(프로세스 정리·마이크 오류·파이프라인 기동·로봇 상태 변화)는
    `log_system()` 으로 터미널에 찍는다. 섞이면 정작 읽어야 할 "AI 가 뭐라고 했나"가
    스크롤에 묻히는데, 실기 중에 봐야 하는 건 그거 하나다.

    `"system"` 은 `append_chat` 의 role 로는 아직 살아 있다(폴백 role). 그래서 지우는
    대신 **호출자가 없다**를 검사한다 -- 무심코 다시 쓰면 여기서 걸린다.
    """
    source = (Path(__file__).resolve().parent.parent
              / "vla_system" / "vla_gui.py").read_text(encoding="utf-8")
    lines = source.splitlines()
    offenders = []
    for i, line in enumerate(lines):
        if "self.append_chat(" not in line:
            continue
        # 한 줄짜리와 여러 줄짜리 둘 다 본다 -- role 은 다음 줄에 올 수 있다.
        blob = line + " " + (lines[i + 1] if i + 1 < len(lines) else "")
        if re.search(r'append_chat\(\s*"system"', blob):
            offenders.append(f"{i + 1}: {line.strip()}")
    assert not offenders, "대화창에 시스템 메시지가 들어간다 -> log_system() 을 쓸 것:\n" + \
        "\n".join(offenders)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
