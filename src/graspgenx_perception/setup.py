import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'graspgenx_perception'

# graspx 배선 스크립트(grasp_bridge_node / capture_graspgenx_scene / graspgen_worker)는
# 2026-08-07 에 워크스페이스 `scripts/` 에서 이 패키지 안으로 옮겼다. 이전에는 소스가
# `scripts/` 에 있고 setup.py 가 `scripts=[...]` 로 바깥 경로를 심는 구조였는데, 한 기능의
# 파일이 두 디렉토리에 흩어져 편집·grep 이 번거로웠다. 지금은 전부 패키지 안에 있고
# 진입점도 `console_scripts` 하나로 통일했다 — 실행 파일 이름에서 `.py` 가 빠진다.

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimkh',
    maintainer_email='wook9980@gmail.com',
    description='GraspGenX 파이프라인 인식 패키지 — YOLO/기하 세그멘테이션, grasp 브릿지',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_seg_node = graspgenx_perception.yolo_seg_node:main',
            'grasp_bridge_node = graspgenx_perception.grasp_bridge_node:main',
            'capture_graspgenx_scene = graspgenx_perception.capture_graspgenx_scene:main',
            # graspgen_worker.py 는 여기 없다 — rclpy 가 아니라 GraspGenX venv 에서
            # `uv run python <경로>` 로 도는 별도 프로세스라 ros2 실행 파일이 아니다.
        ],
    },
)
