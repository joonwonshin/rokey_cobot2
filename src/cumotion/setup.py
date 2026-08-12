import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cumotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일이 get_package_share_directory('cumotion')/config 에서 yaml 을 찾는다.
        # 둘 중 하나라도 빠지면 launch 가 FileNotFound 로 죽는다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon5514@naver.com',
    description='MoveIt(+cuMotion+nvblox) 재계획 루프로 두산 M0609 를 제어한다',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ⚠️ 로봇이 실제로 움직이는 노드다. 항상 mode:=check 를 먼저 돌린다.
            'dynamic_avoid = cumotion.dynamic_avoid:main',
            # ⚠️ 이것도 로봇이 실제로 움직인다. arm.py 를 안 쓰는 독립형 재계획 루프이고,
            #    check 모드가 없으니 vel_scale 을 낮춘 채로 시작한다.
            'reactive_replan = cumotion.reactive_replan:main',
            # NVIDIA 예제(isaac_ros_moveit_goal_setter) 방식. plan_only=False 라
            # move_group 이 실행까지 한다. reactive_replan 과 비교하려고 만든 것.
            'goal_setter_replan = cumotion.goal_setter_replan:main',
        ],
    },
)
