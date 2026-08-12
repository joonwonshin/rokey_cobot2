#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dynamic_avoid.launch.py — 재계획 루프 노드 실행

기본값은 config/dynamic_avoid.yaml 에서 오고, 아래 launch 인자로 자주 바꾸는 것만 덮어쓴다.
(launch 인자 → yaml 순으로 파라미터를 얹으므로 **launch 인자가 이긴다**)

    ros2 launch cumotion dynamic_avoid.launch.py mode:=check
    ros2 launch cumotion dynamic_avoid.launch.py mode:=joint goal_joint_deg:="[0.,0.,90.,0.,90.,0.]"
    ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong vel:=0.2
    ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong static:=true   # 대조군

⚠️ mode:=check 외에는 로봇이 실제로 움직인다. 비상정지 버튼에 손을 올린 채로 실행할 것.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# ⚠️ ParameterValue 는 launch.substitutions 가 아니라 launch_ros 쪽에 있다.
# 문자열로 들어온 launch 인자를 float/bool/list 로 못 바꾸면 rclpy 가 타입 불일치로 죽는다.
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('cumotion')
    params_file = os.path.join(pkg, 'config', 'dynamic_avoid.yaml')

    args = [
        DeclareLaunchArgument(
            'mode', default_value='check',
            description='check(안 움직임) | joint | pose | pingpong'),
        DeclareLaunchArgument(
            'vel', default_value='0.15',
            description='속도 스케일 0~1. cumotion_planner.yaml 이 '
                        'override_moveit_scaling_factors:false 라 이 값이 실제 속도다'),
        DeclareLaunchArgument(
            'replan_hz', default_value='3.0',
            description='재계획 주파수. 세그멘터가 3.7 Hz라 그 위는 새 정보가 없다'),
        DeclareLaunchArgument(
            'lookahead', default_value='0.35',
            description='인계 선행시간(초). 계획시간(~0.2s)+handover 보다 커야 한다'),
        DeclareLaunchArgument(
            'static', default_value='false',
            description='true = 재계획 끄기(대조군). 장애물이 와도 그대로 밀고 간다'),
        DeclareLaunchArgument(
            'pipeline', default_value='isaac_ros_cumotion',
            description='isaac_ros_cumotion(nvblox ESDF) | ompl(MoveIt octomap)'),
        DeclareLaunchArgument(
            'goal_joint_deg', default_value='[0.0, 0.0, 90.0, 0.0, 90.0, 0.0]',
            description='mode:=joint 목표 (deg)'),
        DeclareLaunchArgument(
            'goal_pose', default_value='[0.45, 0.0, 0.35, 180.0, 0.0, 0.0]',
            description='mode:=pose 목표 x y z(m) roll pitch yaw(deg)'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='기본값 묶음. 여기 없는 파라미터는 전부 이 파일에서 온다'),
    ]

    node = Node(
        package='cumotion',
        executable='dynamic_avoid',
        name='dynamic_avoid',
        output='screen',
        emulate_tty=True,          # 한글 로그가 버퍼에 갇히지 않게
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'mode': LaunchConfiguration('mode'),
                'vel_scale': ParameterValue(LaunchConfiguration('vel'), value_type=float),
                'acc_scale': ParameterValue(LaunchConfiguration('vel'), value_type=float),
                'replan_hz': ParameterValue(LaunchConfiguration('replan_hz'), value_type=float),
                'lookahead_s': ParameterValue(LaunchConfiguration('lookahead'), value_type=float),
                'static_mode': ParameterValue(LaunchConfiguration('static'), value_type=bool),
                'pipeline_id': LaunchConfiguration('pipeline'),
                'goal_joint_deg': ParameterValue(
                    LaunchConfiguration('goal_joint_deg'), value_type=List[float]),
                'goal_pose': ParameterValue(
                    LaunchConfiguration('goal_pose'), value_type=List[float]),
            },
        ],
    )

    return LaunchDescription(args + [node])
