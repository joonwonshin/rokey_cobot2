import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    # -----------------------------------------------------------------------
    # Launch Arguments
    # -----------------------------------------------------------------------
    # bringup.launch.py와 겹치는 노드(static_tf/robot_state_publisher/joint_state_publisher)를
    # 끄는 스위치. bringup 위에 얹을 때는 standalone:=false — 안 그러면 robot_description과
    # world→base_link TF가 이중 발행돼 TF가 깜빡이고 실기 관절값이 시뮬값에 덮인다.
    #   ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false
    args = [
        DeclareLaunchArgument('standalone', default_value='true',
                              description='true: 이 launch만으로 완결(로봇 없이 테스트). '
                                          'false: bringup.launch.py 위에 move_group만 얹는다'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='MoveIt RViz spawn 여부 (bringup RViz를 쓸 거면 false)'),
        DeclareLaunchArgument('octomap', default_value='true',
                              description='RealSense 포인트클라우드로 3D 장애물 감지. '
                                          'false면 RViz로 직접 놓은 장애물만 반영(알고리즘 디버깅용)'),
        # rosbag 재생(`ros2 bag play --clock`)으로 디버깅할 때 true. 안 맞추면 move_group이
        # 벽시계를 보는데 bag의 TF·클라우드는 녹화 시각 스탬프라 전부 "너무 낡음"으로 버려진다
        # (octomap이 조용히 비어 있는다). 실기에서는 false 그대로 둔다.
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true: /clock을 따른다 (rosbag play --clock과 한 짝)'),
        # Isaac ROS cuMotion을 두 번째 플래닝 파이프라인으로 등록한다.
        # true로 켜도 default_planning_pipeline은 ompl 그대로다 — RViz MotionPlanning
        # Context 탭 드롭다운에서 골라야 바뀐다. 같은 목표로 둘을 번갈아 계획해
        # 시간을 비교하는 것이 이 스위치의 목적이다.
        # ⚠️ isaac_ros_cumotion_moveit가 설치된 환경(= Isaac ROS 컨테이너)에서만 켤 수 있다.
        # ⚠️ 플러그인은 액션 클라이언트일 뿐이다 — cumotion_planner_node를 따로 띄우지 않으면
        #    계획 요청이 응답 없이 걸린다.
        DeclareLaunchArgument('cumotion', default_value='false',
                              description='true: planning_pipelines에 isaac_ros_cumotion 추가 '
                                          '(Isaac ROS 컨테이너 전용)'),
    ]
    is_standalone = IfCondition(LaunchConfiguration('standalone'))
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # -----------------------------------------------------------------------
    # 패키지 경로
    # -----------------------------------------------------------------------
    bringup_pkg = get_package_share_directory('m0609_rg2_bringup')
    moveit_pkg  = get_package_share_directory('m0609_rg2_moveit')

    # -----------------------------------------------------------------------
    # robot_description: 통합 xacro → URDF 변환
    # -----------------------------------------------------------------------
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'm0609_with_rg2.urdf.xacro')
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    # -----------------------------------------------------------------------
    # robot_description_semantic: SRDF 로드
    # -----------------------------------------------------------------------
    srdf_file = os.path.join(moveit_pkg, 'config', 'm0609_rg2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': f.read()
        }

    # -----------------------------------------------------------------------
    # kinematics: IK 솔버 설정 로드
    # -----------------------------------------------------------------------
    kinematics_yaml = load_yaml('m0609_rg2_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    # -----------------------------------------------------------------------
    # joint_limits: 관절 제한 로드
    # -----------------------------------------------------------------------
    # 이 yaml 전체가 robot_description_planning 네임스페이스로 들어간다.
    # 충돌 여유(default_object_padding / default_robot_padding)도 **yaml 안에 있다** —
    # 여기서 dict 로 덮어쓰지 않는다. 파이썬에 값을 두면 config/ 만 보는 사람이
    # "고쳤는데 왜 안 먹지"로 시간을 버린다 (2026-08-08 이관).
    joint_limits = {
        'robot_description_planning': load_yaml('m0609_rg2_moveit', 'config/joint_limits.yaml')
    }

    # -----------------------------------------------------------------------
    # ompl_planning: 경로 계획 알고리즘 설정 로드
    # -----------------------------------------------------------------------
    ompl_planning_yaml = load_yaml('m0609_rg2_moveit', 'config/ompl_planning.yaml')

    # -----------------------------------------------------------------------
    # planning_pipelines: ompl을 기본 파이프라인으로 명시적 등록
    # -----------------------------------------------------------------------
    # cumotion:=true면 여기에 isaac_ros_cumotion이 하나 더 붙는다. 인자 값을 읽어야 해서
    # 실제 조립은 OpaqueFunction 안(make_nodes)에서 한다 — 호스트처럼 패키지가 없는
    # 환경에서 import 시점에 죽지 않게 하려는 것도 이유다.
    def make_planning_pipelines(context):
        pipelines = {
            'planning_pipelines': ['ompl'],
            'default_planning_pipeline': 'ompl',
            'ompl': ompl_planning_yaml,
        }
        if LaunchConfiguration('cumotion').perform(context).lower() != 'true':
            return pipelines
        # isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml 을 그대로 쓴다.
        # 우리 config/로 복사하지 않는다 — 복사본은 업스트림이 바뀌어도 조용히 낡는다.
        try:
            cumotion_yaml = load_yaml('isaac_ros_cumotion_moveit',
                                      'config/isaac_ros_cumotion_planning.yaml')
        except PackageNotFoundError:
            cumotion_yaml = None
        if not cumotion_yaml:
            raise RuntimeError(
                'cumotion:=true 인데 isaac_ros_cumotion_moveit 의 planning yaml을 못 읽었다. '
                'Isaac ROS 컨테이너 안에서 install/setup.bash 를 source 했는지 확인할 것.')
        pipelines['planning_pipelines'] = ['ompl', 'isaac_ros_cumotion']
        pipelines['isaac_ros_cumotion'] = cumotion_yaml
        return pipelines

    # -----------------------------------------------------------------------
    # moveit_controllers: 컨트롤러 설정 로드
    # -----------------------------------------------------------------------
    moveit_controllers_yaml = load_yaml('m0609_rg2_moveit', 'config/moveit_controllers.yaml')

    # -----------------------------------------------------------------------
    # 3D 장애물 감지 (octomap) — octomap:=false 로 끌 수 있다
    # -----------------------------------------------------------------------
    # RealSense 포인트클라우드를 octomap에 넣어 계획 시 회피하게 한다.
    # 끄는 이유가 있다: 시뮬레이션에서 RViz로 직접 놓은 장애물만으로 알고리즘을
    # 디버깅할 때, 실제 클라우드가 섞이면 무엇이 계획을 막았는지 구분이 안 된다.
    #
    # octomap_frame은 반드시 **고정 프레임**이어야 한다. camera_link 같은 움직이는
    # 프레임을 주면 로봇이 움직일 때마다 지도가 통째로 따라 흔들린다.
    #
    # ⚠️ `world`가 아니라 `base_link`다. SRDF에 virtual_joint(parent_frame="world")가 있어서
    # 플래닝 프레임이 world일 것 같지만 **아니다.** MoveIt은 fixed 타입 virtual joint로는
    # 모델 프레임을 만들지 않아서 플래닝 프레임은 루트 링크(base_link)로 남는다.
    # world는 TF에는 있지만 **planning scene은 모른다** — 2026-08-02 실측:
    #   frame_id='world'  → "[ERROR] moveit_planning_scene: Unknown frame: world", 장애물 무시됨
    #   frame_id='base_link' → /monitored_planning_scene에 정상 등록
    # RViz Scene Objects도 같은 규칙이다. 프레임을 world로 두면 조용히 무시된다.
    # octomap_frame / octomap_resolution 은 **sensors_3d.yaml에 적는다.** 여기 값은
    # yaml에 그 키가 없을 때만 쓰이는 최후 기본값이다(setdefault). update()로 덮으면
    # yaml에 써 넣어도 조용히 무시돼서, 튜닝하는 사람이 "왜 안 먹지"로 시간을 버린다.
    sensors_3d_yaml = load_yaml('m0609_rg2_moveit', 'config/sensors_3d.yaml') or {}
    octomap_params = dict(sensors_3d_yaml)
    octomap_params.setdefault('octomap_frame', 'base_link')
    octomap_params.setdefault('octomap_resolution', 0.02)

    # -----------------------------------------------------------------------
    # Planning Scene Monitor 발행 설정
    # -----------------------------------------------------------------------
    # RViz의 Scene Objects 탭에서 놓은 장애물이 move_group의 계획에 실제로 반영되려면
    # 이 넷이 켜져 있어야 한다. 기본값으로 두면 RViz에 상자는 보이는데 경로는
    # 그 상자를 그냥 통과한다 — "보이는데 안 먹는" 상태가 된다.
    planning_scene_monitor_params = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # -----------------------------------------------------------------------
    # Static TF (world → base_link)
    # -----------------------------------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
        condition=is_standalone,
    )

    # -----------------------------------------------------------------------
    # Robot State Publisher: URDF → TF 트리 퍼블리시
    # -----------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, use_sim_time],
        condition=is_standalone,
    )

    # -----------------------------------------------------------------------
    # Joint State Publisher: 시뮬레이션용 관절 상태 퍼블리시
    # -----------------------------------------------------------------------
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # publish_default_velocities: cuMotion이 /joint_states의 velocity 길이를 position과
        # 대조해서 안 맞으면 계획을 포기한다. 기본값 False면 velocity가 비어 있다.
        # 근거·실측은 bringup.launch.py의 같은 파라미터 주석 참조.
        parameters=[robot_description, use_sim_time, {'publish_default_velocities': True}],
        condition=is_standalone,
    )

    # -----------------------------------------------------------------------
    # dsr_moveit_controller spawner (standalone:=false 일 때만)
    # -----------------------------------------------------------------------
    # 궤적 실행(Execute)에 필요한 JointTrajectoryController. bringup은 이걸 안 띄운다
    # (joint_state_broadcaster + dsr_controller2만 띄움) — 그래서 Plan은 되는데
    # Execute가 ABORTED 났다. MoveIt이 쓰는 컨트롤러이므로 여기서 띄우는 게 맞다.
    #
    # 파라미터는 이미 bringup이 dsr_controller2.yaml로 controller_manager에 넣어놨다
    # (`dsr_moveit_controller:` 블록, position+velocity 인터페이스). spawn만 하면 된다.
    # dsr_controller2와 충돌하지 않는다 — dsr_controller2는 command/state 인터페이스를
    # 하나도 claim하지 않는 순수 서비스 래퍼다 (dsr_controller2.cpp:89-101 빈 config).
    #
    # 네임스페이스는 config/moveit_controllers.yaml의 컨트롤러 이름과 반드시 짝이다.
    # 한쪽만 바꾸면 조용히 Execute만 죽는다.
    dsr_moveit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dsr_moveit_controller', '-c', '/dsr01/controller_manager'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('standalone')),
    )

    # -----------------------------------------------------------------------
    # MoveGroup + RViz — 둘 다 인자 값에 의존해서 OpaqueFunction 안에서 만든다
    # -----------------------------------------------------------------------
    # octomap 파라미터는 "넣느냐 마느냐"라서 IfCondition으로 못 가른다(조건은 노드 전체에만
    # 걸린다). planning_pipelines도 cumotion 인자에 따라 내용이 달라진다. 인자 값을 읽으려면
    # LaunchDescription 생성 이후여야 하므로 OpaqueFunction으로 감싼다.
    # RViz도 같은 딕셔너리를 받아야 한다 — 안 그러면 MotionPlanning 패널 드롭다운에
    # cuMotion이 안 뜬다(move_group만 알고 RViz는 모르는 상태가 된다).
    rviz_config_file = os.path.join(moveit_pkg, 'launch', 'moveit.rviz')

    def make_nodes(context, *_args, **_kwargs):
        planning_pipelines = make_planning_pipelines(context)

        move_group_params = [
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipelines,
            moveit_controllers_yaml,
            planning_scene_monitor_params,
            use_sim_time,
        ]
        if LaunchConfiguration('octomap').perform(context).lower() == 'true':
            move_group_params.append(octomap_params)

        return [
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                output='screen',
                parameters=move_group_params,
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    planning_pipelines,
                    joint_limits,
                    use_sim_time,
                ],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]

    return LaunchDescription(args + [
        static_tf,
        robot_state_publisher,
        joint_state_publisher,
        dsr_moveit_controller_spawner,
        OpaqueFunction(function=make_nodes),
    ])
