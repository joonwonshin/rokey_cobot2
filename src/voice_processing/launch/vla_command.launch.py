"""VLA 지시 브리지(`vla_command_node`) 하나만 띄운다.

`pick_fsm`·`graspgenx_perception`·카메라·로봇은 **일부러 포함하지 않았다.** 이 ws 는 런치를
단위로 쪼개 쓴다 — 여기서 다시 include 하면 같은 노드가 두 번 뜬다. 기동 순서는
`src/PACKAGES.md` 의 pick_fsm §2 가 단일 출처다.

    # VLA 가 지시를 보내고, 사람이 start/approve 를 누른다 (가장 안전한 기본)
    ros2 launch voice_processing vla_command.launch.py

    # VLA 가 start 까지 건다 — 승인(/pick/approve)은 여전히 사람 몫이다
    ros2 launch voice_processing vla_command.launch.py auto_start:=true

    # 같은 클래스 물체가 여럿 놓여 있을 때, 개체 지정 없이 오집만 막는다
    ros2 launch voice_processing vla_command.launch.py pixel_policy:=reject

    # 같은 클래스 물체가 여럿일 때 VLA 가 pixel 로 어느 개체인지 지목한다(2026-08-11 구현)
    ros2 launch voice_processing vla_command.launch.py pixel_policy:=select

이 노드는 rqt 패널(`pick_fsm.rqt_panel`)의 **시작·중단·리셋·홈** 버튼도 같은 채널로 연다 —
`{"cmd":"start"}` / `{"cmd":"abort"}` / `{"cmd":"reset"}` / `{"cmd":"home"}` 을
`/vla/pick_command` 로 보내면 그대로 `/pick/start`·`/pick/abort`·`/pick/reset`·`/pick/home`
을 부른다(`reset`은 SAFE_STOP, `home`은 IDLE 에서만 먹는다). **`승인` 버튼(`/pick/approve`)은
빠졌다** — `cmd:"approve"` 는 무조건 거부한다(계획 §0-B, 실기가 실제로 움직이기 전 마지막
소프트 게이트).

⚠️ `pick_fsm` 은 `voice:=true`(기본) 로 띄워야 한다. `voice:=false` 면 `LISTENING` 을
   건너뛰므로 이 노드의 `/get_keyword` 를 아무도 부르지 않는다 — 단, `start`/`abort`/`reset`/
   `home` 제어 명령은 `voice` 값과 무관하게 항상 동작한다(별도 서비스라 `/get_keyword` 를 안 거친다).

⚠️ 마이크 노드(`get_keyword`)와 **동시에 띄우지 않는다.** 둘 다 `/get_keyword` 를
   제공해서 어느 쪽이 답할지 알 수 없다.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_allowed_classes() -> str:
    """ws 루트 `config/objects.yaml` 의 `detect` 를 콤마 문자열로. 없으면 `''`(검사 끔).

    경로 규칙은 `graspx.launch.py:default_objects_file()`·`pick_fsm.launch.py:default_target()`
    과 같다(`COBOT2_OBJECTS` 로 덮어씀). 여기서 실패해도 런치를 막지 않는다 — 검사만 빠진다.

    이 검사가 있는 이유: VLA 는 자기 C270 과 자기 클래스 목록을 보고 판단한다. 우리 YOLO 가
    애초에 안 찾는 이름(예: `hammer`)을 지시하면 촬영+GPU 왕복 수십 초 뒤에 "해당하는 물체가
    없다"로 실패하고, 그 메시지로는 원인이 안 보인다. 여기서 즉시 거부하면 `/vla/pick_result`
    에 "인식 대상이 아니다 + 허용 목록"이 그대로 돌아간다.
    """
    path = os.environ.get('COBOT2_OBJECTS')
    if not path:
        share = get_package_share_directory('voice_processing')
        path = os.path.abspath(os.path.join(share, '..', '..', '..', '..',
                                            'config', 'objects.yaml'))
    if not os.path.isfile(path):
        print(f'[vla_command.launch] {path} 가 없다 — 클래스 검사를 끈다')
        return ''
    try:
        doc = yaml.safe_load(open(path, encoding='utf-8')) or {}
        return ','.join(str(name).strip() for name in (doc.get('detect') or []))
    except Exception as exc:                                        # noqa: BLE001
        print(f'[vla_command.launch] {path} 를 못 읽었다 ({exc}) — 클래스 검사를 끈다')
        return ''


def generate_launch_description():
    args = [
        DeclareLaunchArgument('command_topic', default_value='/vla/pick_command',
                              description='VLA -> 우리. std_msgs/String (JSON)'),
        DeclareLaunchArgument('result_topic', default_value='/vla/pick_result',
                              description='우리 -> VLA. std_msgs/String (JSON)'),
        DeclareLaunchArgument('keyword_service', default_value='/get_keyword',
                              description='task_manager 의 LISTENING 이 부르는 이름. '
                                          '마이크 노드와 겹치므로 둘 중 하나만 띄운다'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='지시가 오면 /pick/start 까지 부른다. '
                                          '/pick/approve 는 어떤 값에서도 부르지 않는다'),
        # 🔴 2026-08-12: 기본값을 warn → select 로 올렸다.
        #    계획 §0-4 가 `warn` 을 **Phase 0 한정 임시값**으로 뒀고("Phase 2 통과 후 같은
        #    클래스 2개 실기로 별도 검증"), 통합 테스트 T3·T11 이 select 를 전제한다.
        #    warn 이면 VLA 가 어느 개체인지 지목해 보내도 **FSM 이 그 pixel 을 버리고**
        #    클래스만으로 고른다 — 사과가 둘이면 지목과 무관하게 점수 높은 쪽을 집는다.
        #    select 의 실패 방향은 안전하다: 후보가 match_tolerance_m 밖이거나 2등과
        #    ambiguity_margin_m 안으로 붙어 모호하면 grasp_bridge 가 **호출을 실패시킨다**
        #    (틀린 걸 집는 것보다 안 집는 게 낫다). pixel 이 아예 없으면 no-op 이라
        #    클래스만 보내는 기존 지시는 그대로 동작한다(parse_command 의 `if 'pixel' in doc`).
        #    🔴 실기 미검증 — 같은 클래스 2개로 T11 을 통과시킬 것.
        DeclareLaunchArgument('pixel_policy', default_value='select',
                              description="warn | reject | select. warn 은 클래스만으로 진행, "
                                          "reject 는 pixel 오면 거부, select 는 pixel 로 개체를 "
                                          "고른다(2026-08-11 구현, select_by_point)"),
        DeclareLaunchArgument('ttl_sec', default_value='10.0',
                              description='지시 유효시간[s]. 받은 시각 기준(송신 stamp 아님)'),
        DeclareLaunchArgument('wait_timeout_sec', default_value='50.0',
                              description='/get_keyword 를 붙잡고 기다릴 시간[s]. '
                                          'fsm_listening_timeout_sec - listening_margin_sec '
                                          '보다 작아야 한다 (기동 때 검증, 넘으면 안 뜬다)'),
        DeclareLaunchArgument('fsm_listening_timeout_sec', default_value='110.0',
                              description='task_manager.DEFAULT_TIMEOUTS[State.LISTENING] 의 '
                                          '사본. 저쪽 값을 바꿨으면 여기도 맞춘다'),
        DeclareLaunchArgument('listening_margin_sec', default_value='5.0',
                              description='서비스 탐색·왕복 여유[s]'),
        DeclareLaunchArgument('allowed_classes', default_value=default_allowed_classes(),
                              description='콤마 목록. 기본값은 config/objects.yaml 의 detect. '
                                          '비우면 검사 안 함'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    node = Node(
        package='voice_processing',
        executable='vla_command_node',
        name='vla_command_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            # ⚠️ LaunchConfiguration 은 문자열이다. bool/double 로 선언한 파라미터에
            #    "true"/"10.0" 을 그대로 넣으면 타입 불일치로 노드가 뜨자마자 죽는다
            #    (pick_fsm.launch.py 가 같은 함정을 주석으로 남겨 뒀다).
            'command_topic': ParameterValue(LaunchConfiguration('command_topic'),
                                            value_type=str),
            'result_topic': ParameterValue(LaunchConfiguration('result_topic'),
                                           value_type=str),
            'keyword_service': ParameterValue(LaunchConfiguration('keyword_service'),
                                              value_type=str),
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
            'pixel_policy': ParameterValue(LaunchConfiguration('pixel_policy'), value_type=str),
            'ttl_sec': ParameterValue(LaunchConfiguration('ttl_sec'), value_type=float),
            'wait_timeout_sec': ParameterValue(LaunchConfiguration('wait_timeout_sec'),
                                               value_type=float),
            'fsm_listening_timeout_sec': ParameterValue(
                LaunchConfiguration('fsm_listening_timeout_sec'), value_type=float),
            'listening_margin_sec': ParameterValue(
                LaunchConfiguration('listening_margin_sec'), value_type=float),
            'allowed_classes': ParameterValue(LaunchConfiguration('allowed_classes'),
                                              value_type=str),
        }],
    )

    return LaunchDescription(args + [node])
