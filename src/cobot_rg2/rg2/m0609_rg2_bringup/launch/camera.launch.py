"""RealSense 드라이버 + 캘리브 static TF (base_link → camera_link).

로봇과 분리된 단위다. 순서:
  1) ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100   # 로봇만
  2) ros2 launch m0609_rg2_bringup camera.launch.py                                    # 카메라 + TF
  3) ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false                   # move_group + JTC + RViz
     (dsr_moveit_config_m0609 demo.launch.py 아니다 — 패키지명 불일치로 애초에 안 뜨고,
      떠도 그 config URDF엔 RG2가 없다. README "알려진 함정" 참고.)

TF 값을 여기 하드코딩하지 않는다. config/T_cam2base.npy를 읽어 매 launch마다 계산한다.
재캘리브 후엔 npy만 갈아끼운다 (symlink-install이면 rebuild 불필요):
  cp corecode/Calibration_Tutorial/T_cam2base.npy \
     src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy

캘리브 미세보정(dxyz/drpy)은 아래 인자로 준다. 드라이버는 그대로 두고 TF만 다시 띄우며 맞춘다:
  ros2 launch m0609_rg2_bringup camera.launch.py driver:=false drpy:="0 1.5 0"
맞으면 그 값을 아래 DeclareLaunchArgument 기본값에 박아 고정한다(npy는 그대로 둔다).

eye-to-hand 전제다 — 카메라가 로봇에 붙어 있지 않으므로 URDF가 아니라 static TF로 준다.
eye-in-hand(그리퍼 부착)로 바꾸면 이 launch를 쓰면 안 된다. camera_link의 부모가
URDF와 여기 둘로 갈려 TF 트리가 깨진다. → bringup_camera.launch.py 참고.
"""
import os
import sys

import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

PARENT_FRAME = 'base_link'
CHILD_FRAME = 'camera_link'


def _apply_delta(t, q, dxyz, drpy):
    """캘리브 결과에 손보정을 얹는다.

    평행이동은 base_link 축(눈으로 보는 RViz 축), 회전은 camera_link 자기 축
    (x=전방, y=좌, z=상) 기준 — 카메라 원점에서 돌린다. 테이블이 기울어 보이는
    건 보통 pitch 1~3° 문제라 이쪽이 손으로 맞추기 쉽다.
    """
    R = Rotation.from_quat(q) * Rotation.from_euler('xyz', drpy, degrees=True)
    return np.asarray(t) + np.asarray(dxyz), R.as_quat()


def _args():
    # 해상도 기본값을 낮게 잡는다: 848*480*30 = 12.2 M point/s, 424x240x15면 1/8 (약 1.5 M point/s).
    #
    # 🔴 2026-08-05 정정: 여기 있던 "i7-10510U 15W / GPU 없음" 근거는 **이 랩탑 사양이 아니었다**
    #   (개인PC 측정치가 잘못 옮겨온 것). 실측은 i7-13620H 10C/16T + RTX 4060.
    #   그래도 기본값을 그대로 두는 이유는 CPU가 아니라 **MoveIt octomap updater가 단일 스레드**라서다
    #   — 코어가 16개여도 콜백 하나의 처리 시간은 안 줄어든다. 근거는 constraints.md
    #   "이 랩탑 하드웨어 — 2026-08-05 재측정" 절(헤더의 octomap::KeyRay key_ray_).
    #
    # [튜닝] 올릴 때는 sensors_3d.yaml의 max_update_rate/point_subsample과 **곱으로** 붙는다는 점을
    #   기억한다. 한 번에 하나만 올리고 move_group %CPU와 "queue is full" 로그를 보고 판단한다.
    #
    # color도 같이 낮춘다: align_depth.enable=true면 depth를 **color 해상도로 리샘플**하므로
    # color만 크면 낮춘 의미가 없다.
    #
    # [튜닝] GPU PC나 여유 있는 머신에서는 인자로 올린다:
    #   ros2 launch ... camera.launch.py depth_profile:=848x480x30 color_profile:=848x480x30
    #
    # 🔴 2026-08-11: graspgenx(1280x720 요구) vs octomap/nvblox(반응속도 우선, 424x240 요구)를
    #   동시에 만족할 수 없다 — RealSense 드라이버는 depth를 **한 해상도로만** 낸다.
    #   해결: 카메라는 그래스핑이 요구하는 해상도로 한 번만 열고, T4(robot_segmenter) 직전에
    #   depth_downsample_node.py(같은 패키지 scripts/)로 저해상도 사본을 만들어 nvblox
    #   체인에는 그쪽을 먹인다. octomap 쪽은 sensors_3d.yaml의 point_subsample을 해상도
    #   배율만큼 올려서 대응한다(배율의 정확한 지수는 그 파일 UNVERIFIED 각주 참고 — 실측 필요).
    #   🔴 depth_profile:=1280x720x15/x30은 실기에서 "Frames didn't arrived within 5 seconds"로
    #   죽는다(USB 대역폭 초과로 추정, 2026-08-11 확인) — **depth_profile은 그대로 두고
    #   color_profile만 1280x720으로 올린다.** align_depth가 depth를 color 해상도로 리샘플하므로
    #   aligned_depth_to_color는 1280x720이 나온다(실측: ~19~29 Hz 안정). 단 이건 depth_profile
    #   해상도(예: 848x480)를 업샘플한 것이라 실제 공간 분해능의 상한은 depth_profile 쪽이다.
    #   자세한 실행 순서·실측치는 config/testcommand.md "T1.5" 절.
    return [
        DeclareLaunchArgument('dxyz', default_value='0 0 0',
                              description='캘리브 평행이동 보정 "x y z" (m, base_link 축)'),
        DeclareLaunchArgument('drpy', default_value='0 0 0',
                              description='캘리브 회전 보정 "roll pitch yaw" (deg, camera_link 축)'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='RealSense 드라이버 spawn 여부 (false면 TF만)'),
        DeclareLaunchArgument('depth_profile', default_value='424x240x15',
                              description='depth 스트림 WxHxFPS. 올리면 move_group CPU가 같이 오른다'),
        DeclareLaunchArgument('color_profile', default_value='424x240x15',
                              description='color 스트림 WxHxFPS. align_depth가 이 해상도를 따라간다'),
    ]


def _setup(context, *_):
    pkg_share = get_package_share_directory('m0609_rg2_bringup')
    sys.path.insert(0, os.path.join(pkg_share, 'scripts'))
    from calib_npy_to_tf import npy_to_tf_args  # noqa: E402

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'enable_sync': True,
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        }],
        condition=IfCondition(LaunchConfiguration('driver')),
        output='screen',
    )

    # npy가 없으면 TF 노드만 빠지고 카메라는 뜬다 (캘리브 전에도 영상은 봐야 하니까).
    calib_npy = os.path.join(pkg_share, 'config', 'T_cam2base.npy')
    calib_tf = []
    if os.path.exists(calib_npy):
        t, q = npy_to_tf_args(np.load(calib_npy), PARENT_FRAME, CHILD_FRAME)
        dxyz = [float(v) for v in LaunchConfiguration('dxyz').perform(context).split()]
        drpy = [float(v) for v in LaunchConfiguration('drpy').perform(context).split()]
        if any(dxyz) or any(drpy):
            t, q = _apply_delta(t, q, dxyz, drpy)
            print(f'[camera.launch] 캘리브 보정 적용: dxyz={dxyz} m, drpy={drpy} deg')
        calib_tf = [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_calib_tf',
            output='log',
            arguments=[f'{v:.6f}' for v in t] + [f'{v:.8f}' for v in q]
                      + [PARENT_FRAME, CHILD_FRAME],
        )]
    else:
        print(f'[camera.launch] ⚠️ {calib_npy} 없음 — '
              f'{PARENT_FRAME}→{CHILD_FRAME} TF를 발행하지 않는다 (포인트클라우드가 로봇과 안 붙는다)')

    return [realsense_node] + calib_tf


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=_setup)])
