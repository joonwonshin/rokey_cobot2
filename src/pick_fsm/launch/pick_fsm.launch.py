"""task_manager + robot_safety_node 를 띄운다. 로봇·카메라·MoveIt 은 **일부러 포함하지 않았다.**

이 ws 는 런치를 단위로 쪼개 쓴다 (bringup / camera / moveit). 여기서 다시 include 하면
같은 노드가 두 번 뜨거나(robot_description 충돌) 이미 켜둔 걸 못 쓰게 된다.
기동 순서는 README "실행" 절이 단일 출처다.

`robot_safety_node`(안전정지·backdrive 서비스, 로봇상태 발행)는 **여기 포함시켰다.** UI(rqt
패널)는 있으나 없으나 자동중단(충돌 감지 시 task_manager 가 하던 작업을 abort)은 항상 동작해야
하기 때문이다 — safety 노드는 `rqt:=false` 로 꺼도 항상 뜬다.

`rqt`(기본 `true`)는 이 런치가 `rqt --standalone pick_fsm` 을 같이 띄울지를 정한다.
끄려면:
    ros2 launch pick_fsm pick_fsm.launch.py rqt:=false

⚠️ `dry_run`(계획만) 인자는 2026-08-09 제거했다. **이 런치는 항상 실기를 움직인다.**
🔴 `require_approval` 기본값은 2026-08-11 사용자 결정으로 **false 로 뒤집혔다** — 승인 없이
   곧장 실행한다. **지금 실기 안전장치는 물리 비상정지 버튼 하나뿐이다.** 승인을 다시
   켜려면 `require_approval:=true`.

🔴 **여기 선언되지 않은 인자는 에러도 경고도 없이 조용히 무시된다** (2026-08-09 실측).
   두 번 밟은 자리라 이름을 적어둔다:
   - `dry_run:=true` — 붙여도 로봇은 **움직인다**. "안전하게 켰다"는 착각을 준다
   - `target_classes:=apple` — 이 런치의 이름이 아니다. 잡을 대상은 **`target:=`** 다
     (`target_classes` 는 `grasp_bridge_node` 쪽 이름이고, task_manager 가 `target` 을
     PERCEIVE 마다 거기로 밀어 넣는다)

    ros2 launch pick_fsm pick_fsm.launch.py                    # 실행. 승인은 필요
    ros2 launch pick_fsm pick_fsm.launch.py voice:=false target:=apple
    ros2 launch pick_fsm pick_fsm.launch.py voice:=false target:=apple,orange   # 셋 중 최고점
    ros2 launch pick_fsm pick_fsm.launch.py voice:=false target:=              # 자동(전부)
    ros2 launch pick_fsm pick_fsm.launch.py planning_pipeline:=isaac_ros_cumotion

타겟은 런치 인자로 못 박지 않아도 된다 — 돌고 있는 동안 바꾸는 쪽이 정상 운용이다:

    rqt --standalone pick_fsm                                  # '타겟' 상자 + [자동] 버튼
    ros2 topic pub -1 /pick/target std_msgs/String "data: apple" \
        --qos-durability transient_local
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_target() -> str:
    """ws 루트 `config/objects.yaml` 의 `pick_default`. 없으면 `''`(자동).

    같은 파일의 `detect` 를 `yolo_seg_node` 가 읽는다 — 탐지 대상과 기본 pick 타겟이
    한 파일에 있어야 "YOLO 는 안 찾는데 타겟으로 지정된" 조합이 안 생긴다.
    경로 규칙은 `graspx.launch.py:default_objects_file()` 과 같다(`COBOT2_OBJECTS` 로 덮어씀).
    여기서 실패해도 런치를 막지 않는다 — 타겟은 rqt 패널로도 정할 수 있고, 이 파일 하나
    때문에 로봇을 못 띄우는 게 더 나쁘다. 대신 무엇이 잘못됐는지는 반드시 찍는다.
    """
    path = os.environ.get('COBOT2_OBJECTS')
    if not path:
        share = get_package_share_directory('pick_fsm')
        path = os.path.abspath(os.path.join(share, '..', '..', '..', '..',
                                            'config', 'objects.yaml'))
    if not os.path.isfile(path):
        return ''
    try:
        doc = yaml.safe_load(open(path, encoding='utf-8')) or {}
        pick = str(doc.get('pick_default') or '').strip()
    except Exception as exc:                                    # noqa: BLE001
        print(f'[pick_fsm.launch] {path} 를 못 읽었다 ({exc}) — 타겟을 자동으로 둔다')
        return ''
    # 같은 파일 안에서 어긋나는 조합을 여기서 끊는다. 안 걸러도 결국 실패하지만, 그건
    # 촬영+GPU 왕복 수십 초 뒤이고 메시지도 "해당하는 물체가 없다"라 원인이 안 보인다.
    detect = doc.get('detect') or []
    wanted = [s.strip() for s in pick.split(',') if s.strip()]
    outside = [w for w in wanted if w not in detect]
    if outside:
        print(f'[pick_fsm.launch] ⚠️ {path}: pick_default 의 {outside} 가 detect{list(detect)} '
              '밖이다 — YOLO 가 애초에 안 찾으므로 절대 못 잡는다. 타겟을 자동으로 둔다')
        return ''
    return pick


def generate_launch_description():
    pkg = get_package_share_directory('pick_fsm')
    default_params = os.path.join(pkg, 'config', 'pick_fsm.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='파라미터 yaml. 값의 정본은 이 파일이다'),
        # 🔴 2026-08-11 사용자 결정으로 기본값을 false 로 뒤집었다 — WAIT_APPROVAL 에서
        #    /pick/approve 를 기다리지 않고 곧장 실행한다. 근거: "위험하면 모션플래닝이
        #    막는다"(단 플래너는 모델에 있는 충돌만 막는다 — 인식오류·미모델링 장애물·사람은
        #    못 막는다). 승인 게이트가 없어진 지금 **유일한 실기 안전장치는 물리 비상정지
        #    버튼뿐**이다. 승인을 다시 켜려면: require_approval:=true.
        DeclareLaunchArgument('require_approval', default_value='false',
                              description='true 로 두면 /pick/approve 없이는 APPROACH 로 못 넘어간다. '
                                          '기본 false = 승인 없이 실행 (2026-08-11 사용자 결정)'),
        DeclareLaunchArgument('voice', default_value='true',
                              description='false 면 get_keyword 를 건너뛰고 target 인자를 쓴다'),
        DeclareLaunchArgument('target', default_value=default_target(),
                              description='voice:=false 일 때의 초기 타겟. 기본값은 '
                                          'config/objects.yaml 의 pick_default. 콤마로 여러 개, '
                                          '비우면 자동(점수 최고). 런타임에는 /pick/target 이 이긴다'),
        DeclareLaunchArgument('grasp_source', default_value='legacy_trigger',
                              description='legacy_trigger | compute_grasp | manual. '
                                          'compute_grasp 서버는 이 ws 에 아직 없다'),
        DeclareLaunchArgument('seg_source', default_value='yolo',
                              description="grasp_bridge_node 에 밀어 넣을 세그 방식. "
                                          "yolo | geometric | '' (안 건드림). "
                                          'geometric 은 클래스를 몰라 target 을 못 쓴다'),
        DeclareLaunchArgument('bridge_node', default_value='/grasp_bridge_node',
                              description='타겟을 밀어 넣을 인식 브리지 노드. 비우면 안 민다'),
        DeclareLaunchArgument('planning_pipeline', default_value='ompl',
                              description='ompl | isaac_ros_cumotion — move_group 에 그 파이프라인이 '
                                          '떠 있어야 한다(moveit.launch.py 쪽 설정)'),
        DeclareLaunchArgument('gripper_backend', default_value='real',
                              description='real | virtual. 숫자 명령의 단위가 다르다'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('robot_ns', default_value='dsr01',
                              description='robot_safety_node 가 Doosan 서비스를 찾을 네임스페이스'),
        DeclareLaunchArgument('rqt', default_value='true',
                              description='감시 UI(rqt --standalone pick_fsm)를 같이 띄울지. '
                                          'false 로 끄면 task_manager/robot_safety_node 만 뜬다'),
    ]

    # yaml 을 먼저 깔고 런치 인자를 그 위에 덮는다 (뒤에 오는 dict 가 이긴다).
    # ⚠️ LaunchConfiguration 은 **문자열**이다. 노드가 bool 로 선언한 파라미터에
    #    "true" 라는 문자열을 넣으면 타입 불일치로 노드가 뜨자마자 죽는다.
    #    ParameterValue(value_type=bool) 로 변환을 명시해야 한다.
    def as_bool(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    task_manager = Node(
        package='pick_fsm',
        executable='task_manager',
        name='task_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'require_approval': as_bool('require_approval'),
                'voice_enabled': as_bool('voice'),
                'target': ParameterValue(LaunchConfiguration('target'), value_type=str),
                'grasp_source': ParameterValue(LaunchConfiguration('grasp_source'),
                                               value_type=str),
                'bridge_seg_source': ParameterValue(LaunchConfiguration('seg_source'),
                                                    value_type=str),
                'bridge_node': ParameterValue(LaunchConfiguration('bridge_node'),
                                              value_type=str),
                'planning_pipeline': ParameterValue(LaunchConfiguration('planning_pipeline'),
                                                    value_type=str),
                'gripper_backend': ParameterValue(LaunchConfiguration('gripper_backend'),
                                                  value_type=str),
            },
        ],
    )

    safety = Node(
        package='pick_fsm',
        executable='robot_safety_node',
        name='robot_safety_node',
        output='screen',
        parameters=[{'robot_ns': ParameterValue(LaunchConfiguration('robot_ns'),
                                                 value_type=str)}],
    )

    rqt = ExecuteProcess(
        cmd=['rqt', '--standalone', 'pick_fsm'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rqt')),
    )

    return LaunchDescription(args + [task_manager, safety, rqt])
