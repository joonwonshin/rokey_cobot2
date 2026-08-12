from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'voice_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ⚠️ `.env` 를 리스트에 직접 적지 않는다. `glob('resource/*')` 는 dot 파일을 안 잡아서
        #    예전엔 `+ ['resource/.env']` 로 못박아 뒀는데, `.env` 는 gitignore 대상이라
        #    **파일이 없는 머신에서 빌드가 통째로 실패했다**(그래서 이 패키지가 COLCON_IGNORE
        #    상태였다, 2026-08-08). `glob('resource/.env')` 는 있으면 설치하고 없으면 []
        #    를 돌려주므로 빌드가 안 깨진다. `.env` 는 여전히 `get_keyword` 의 **런타임**
        #    필수 파일이다 — 없으면 그 노드만 못 뜬다(`vla_command_node` 는 안 쓴다).
        (os.path.join('share', package_name, 'resource'),
            glob('resource/*') + glob('resource/.env')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='음성/VLA 지시를 pick_fsm 의 /get_keyword 계약으로 옮기는 지시 입력 층',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 마이크 + STT + LLM. openai/langchain/pyaudio/openwakeword + resource/.env 필요
            'get_keyword = voice_processing.get_keyword:main',
            # VLA(외부 PC) JSON 지시. 표준 ROS 2 만 쓴다 — 추가 의존성도 .env 도 없다
            'vla_command_node = voice_processing.vla_command_node:main',
            # WAIT_APPROVAL 동안만 "승인" 등을 들으면 /pick/approve 를 부른다.
            # get_keyword 와 같은 마이크/웨이크워드/STT 의존성 (openai/pyaudio/openwakeword)
            'approve_listener_node = voice_processing.approve_listener_node:main',
        ],
    },
)
