"""FoundationPose × d435i rosbag(848x480) 실행 런치.

기성 런치를 왜 안 쓰나 (2026-08-05 소스 확인):
  isaac_ros_foundationpose.launch.py   입력 토픽 인자가 없다. 마스크가 rt_detr_segmentation으로
                                       나가는데 FP는 segmentation을 구독해 연결이 끊겨 있다
  *_realsense.launch.py                RealSense 노드를 띄우고 해상도가 1280x720 하드코딩
  *_core.launch.py                     isaac_ros_examples 패키지 필요 — 워크스페이스에 없음

bag 실측(2026-08-05, apple bag sqlite3 디코딩):
  color  848x480, compressed만 존재            -> republish 필요
  aligned_depth_to_color 848x480 16UC1(mm)     -> ConvertMetric으로 32FC1(m) 변환 필요
  color/image 와 aligned_depth/image 의 stamp가 780개 중 756개 완전 일치 -> sync_threshold=0 성립
  depth/image_rect_raw 는 frame이 camera_depth_optical_frame 이라 쓰면 안 된다

실행:
  ros2 launch scripts/foundationpose_bag.launch.py mesh_file_path:=/abs/path/apple.obj
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.realpath(__file__))

COLOR_COMPRESSED = '/camera/camera/color/image_raw/compressed'
COLOR_RAW = '/camera/camera/color/image_raw'
COLOR_INFO = '/camera/camera/color/camera_info'
DEPTH_RAW = '/camera/camera/aligned_depth_to_color/image_raw'  # depth/image_rect_raw 아님

IMAGE_W, IMAGE_H = 848, 480


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mesh_file_path', description='대상 물체 CAD 메시 절대경로 (.obj)'),
        DeclareLaunchArgument(
            'assets_dir', default_value='/workspaces/isaac_ros-dev/assets/foundationpose',
            description='refine_model.onnx / score_model.onnx 가 있는 디렉토리'),
        DeclareLaunchArgument(
            'engines_dir', default_value='/workspaces/isaac_ros-dev/engines',
            description='TensorRT 엔진 저장 위치. /tmp 는 컨테이너 재시작에 날아간다'),
        DeclareLaunchArgument('center_x', default_value=str(IMAGE_W / 2)),
        DeclareLaunchArgument('center_y', default_value=str(IMAGE_H / 2)),
        DeclareLaunchArgument('size_x', default_value='240.0'),
        DeclareLaunchArgument('size_y', default_value='240.0'),
        DeclareLaunchArgument(
            'sync_threshold', default_value='0',
            description='ns. 0=완전 일치. 프레임이 하나도 안 통과하면 여기부터 의심한다'),
    ]
    assets = LaunchConfiguration('assets_dir')
    engines = LaunchConfiguration('engines_dir')

    # bag에 raw color가 없다. header를 그대로 복사하므로 stamp는 보존된다. 중복 실행 금지
    republish = Node(
        package='image_transport', executable='republish',
        arguments=['compressed', 'raw'],
        remappings=[('in/compressed', COLOR_COMPRESSED), ('out', COLOR_RAW)],
        output='screen')

    bbox_pub = ExecuteProcess(
        cmd=['python3', os.path.join(THIS_DIR, 'fp_bbox_pub.py'), '--ros-args',
             '-p', ['image_topic:=', COLOR_COMPRESSED],
             '-p', ['center_x:=', LaunchConfiguration('center_x')],
             '-p', ['center_y:=', LaunchConfiguration('center_y')],
             '-p', ['size_x:=', LaunchConfiguration('size_x')],
             '-p', ['size_y:=', LaunchConfiguration('size_y')]],
        output='screen')

    # 16UC1 mm -> 32FC1 m. FP는 nitros_image_32FC1만 받는다(foundationpose_node.cpp:56)
    convert_metric = ComposableNode(
        name='convert_metric',
        package='isaac_ros_depth_image_proc',
        plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
        remappings=[('image_raw', DEPTH_RAW), ('image', 'depth_fp32')])

    # Detection2DArrayFilter는 건너뛴다 — results[].hypothesis.score>0 이 없으면 조용히 버린다
    # (detection2_d_array_filter.cpp:72). 박스가 하나뿐이라 고를 것도 없다
    to_mask = ComposableNode(
        name='detection2_d_to_mask',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
        parameters=[{'mask_width': IMAGE_W, 'mask_height': IMAGE_H}])

    # texture_path 는 넣지 않는다 — 노드가 선언하지 않는 파라미터다(기성 런치의 잔재).
    # 텍스처는 .obj 의 .mtl 로 따라온다
    foundationpose = ComposableNode(
        name='foundationpose',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
        parameters=[{
            'mesh_file_path': LaunchConfiguration('mesh_file_path'),
            'sync_threshold': LaunchConfiguration('sync_threshold'),
            'tf_frame_name': 'fp_object',

            'refine_model_file_path': PathJoinSubstitution([assets, 'refine_model.onnx']),
            'refine_engine_file_path': PathJoinSubstitution([engines, 'refine_trt_engine.plan']),
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],

            'score_model_file_path': PathJoinSubstitution([assets, 'score_model.onnx']),
            'score_engine_file_path': PathJoinSubstitution([engines, 'score_trt_engine.plan']),
            'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'score_input_binding_names': ['input1', 'input2'],
            'score_output_tensor_names': ['output_tensor'],
            'score_output_binding_names': ['output1'],
        }],
        remappings=[
            ('pose_estimation/image', COLOR_RAW),
            ('pose_estimation/camera_info', COLOR_INFO),
            ('pose_estimation/depth_image', 'depth_fp32'),
            ('pose_estimation/segmentation', 'segmentation'),
            ('pose_estimation/output', 'fp/output'),
        ])

    container = ComposableNodeContainer(
        name='foundationpose_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[convert_metric, to_mask, foundationpose],
        output='screen')

    return LaunchDescription(args + [republish, bbox_pub, container])
