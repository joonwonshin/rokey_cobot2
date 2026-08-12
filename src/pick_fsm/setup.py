import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pick_fsm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'plugin.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimkh',
    maintainer_email='wook9980@gmail.com',
    description='음성 지시 pick 상태머신 (task_manager)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = pick_fsm.task_manager:main',
            'robot_safety_node = pick_fsm.robot_safety_node:main',
            'planned_tcp_path_node = pick_fsm.planned_tcp_path_node:main',
        ],
    },
)
