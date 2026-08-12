<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    nvblox 워크스페이스·udev·Docker 컨테이너 셋업 (§1~§5만. §6 이후는 cumotion-bringup.md 소유)
-->

# Phase 1: Isaac ROS + nvblox 셋업 가이드 (D435i, RTX 4050/4060 노트북)

> ⏸ **보류 — GPU는 있으나(RTX 4060 Laptop 8GB) 도커 경로가 막혀 있다.**
> `kimkh`가 docker 그룹 비멤버(멤버는 `rokey`) + `nvidia-container-toolkit` 미설치 — 이 둘이 풀려야 §4 이후를 진행할 수 있다.
> 충돌 회피는 이미 Octomap이 담당 중이라 nvblox는 여전히 **시각화 전용·우선순위 낮음**이다.
> **nvblox 실행 절차 본체(§6 이후)는 이 문서가 아니라 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6이 단일 출처다** — 가장 최신이고 실패 이력(`std::lerp`, `warp.torch`)까지 있다.
> 이 문서는 §1~§5(워크스페이스·udev·Docker 컨테이너 셋업)만 유효하다. 문서 지도: [[ws/cobot2/README]]

목표: 로봇 없이, D435i만으로 nvblox의 실시간 3D 재구성을 눈으로 확인한다.

---

## 0. 사전 확인

```bash
nvidia-smi   # GPU 인식 확인
lsb_release -a   # Ubuntu 22.04 확인
docker --version   # Docker 설치 여부 확인
```

Docker와 nvidia-container-toolkit이 없다면 먼저 설치해야 합니다
(Isaac ROS는 Docker 기반 개발환경을 강력히 권장합니다).

```bash
# nvidia-container-toolkit 설치 (없는 경우)
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 1. 워크스페이스 및 isaac_ros_common 클론

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

git clone -b release-4.4 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-4.4 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
```

## 2. RealSense 지원을 위한 컨테이너 설정

Docker 컨테이너를 빌드하기 전에, RealSense용 레이어를 포함하도록 설정합니다.

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
```

## 3. RealSense udev 규칙 등록 (호스트 측)

카메라를 뽑아둔 상태에서 진행합니다.

```bash
wget https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> 주의: 이전에 apt로 설치한 `librealsense2-dkms`와 버전(v2.56.3)이 다를 수 있습니다.
> 충돌이 의심되면 `dpkg -l | grep realsense`로 현재 버전을 먼저 확인하세요.

## 4. Docker 컨테이너 실행

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}
```

컨테이너 안에 들어가면 카메라를 연결하고 확인합니다.

```bash
# 컨테이너 내부에서
realsense-viewer
```

## 5. 워크스페이스 빌드

```bash
# 컨테이너 내부에서
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --packages-up-to-regex realsense*
colcon build --symlink-install --packages-up-to isaac_ros_nvblox
source install/setup.bash
```

> 이 ws는 `COLCON_IGNORE`가 루트에 있으므로 실제로는 `--base-paths src`를 붙인다
> (근거는 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §2-1).

### 5-1. ⛔ `isaac_ros_nvblox` 단독 빌드 시 실기에서 걸린 것들 (2026-08-06)

`isaac_ros_cumotion` 계열은 이미 빌드돼 있었지만 **`isaac_ros_nvblox`는 이번이 처음**이었다.
`colcon build --packages-up-to isaac_ros_nvblox`가 3가지에서 순서대로 막혔다:

1. **`isaac_ros_managed_nitros` 등 NITROS 코어 패키지가 src에 없었다.**
   `isaac_ros_common`/`isaac_ros_nvblox`/`isaac_ros_pose_estimation`은 이미 클론돼 있었는데
   `isaac_ros_nitros`(NITROS 프레임워크 본체 — `isaac_ros_managed_nitros`, `isaac_ros_gxf`,
   `isaac_ros_nitros_image_type`, `isaac_ros_nitros_camera_info_type` 등을 담은 별도 레포)는
   클론된 적이 없었다. 버전 정렬을 맞춰 같은 태그로 받는다:
   ```bash
   git clone -b v3.2-14 --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git \
     src/isaac_ros_nitros
   ```
   ⚠️ LFS 대용량 파일(cuVSLAM/cuApriltags 바이너리)이 커서 `git clone`이 2분 타임아웃에 걸릴 수 있다.
   끊기면 재-clone하지 말고 `git checkout HEAD -- .`로 이어받는다(이미 받은 objects는 재다운로드 안 함).
   `cd`로 이동한 뒤 `git checkout -- .`을 쓰면 워크트리 안에서도 간헐적으로
   `pathspec '.' did not match`가 났다 — `git -C <경로> checkout HEAD -- .`처럼 **경로를 명시**하는 쪽이 안정적이었다.

2. **rosdep `magic_enum` 키가 해석 안 됨.** `extra_rosdeps.yaml`엔 `ros-humble-magic-enum`으로
   매핑돼 있지만 이 컨테이너는 rosdep으로 사전 설치되지 않았다. 다만 apt 저장소
   (`isaac.download.nvidia.com/isaac-ros/release-3`)엔 있어서 바로 설치된다:
   ```bash
   sudo apt-get update -qq && sudo apt-get install -y ros-humble-magic-enum
   ```

3. **🔴 `isaac_ros_nitros`/`isaac_ros_nvblox` 레포 다수의 CMakeLists.txt가 `magic_enum`을
   패키지 실제 의존성으로 링크하면서 `find_package(magic_enum)`을 빠뜨렸다.**
   apt로 설치해도 `CMake Error ... links to target "magic_enum::magic_enum" but the target
   was not found`로 계속 막힌다 — **rosdep/apt 문제가 아니라 업스트림 CMakeLists 버그**다.
   `isaac_ros_gxf`처럼 `package.xml`에 `<depend>magic_enum</depend>`이 있는 패키지는
   `ament_auto_find_build_dependencies()`가 알아서 찾아주지만, 그 익스포트를 **전이적으로
   링크만 하는** 패키지들(`isaac_ros_nitros` 자신, `isaac_ros_nitros_*_type` 전종,
   `nvblox_ros` 본체와 `nvblox_ros/test/unit_tests`)은 자기 CMakeLists에 `find_package`가
   없어서 타깃이 안 보인다. 총 29개 CMakeLists.txt에 `find_package(magic_enum REQUIRED)`
   한 줄씩 추가해 해결했다. 패치:
   - `patches/isaac_ros_nitros-cmake-magic_enum.patch` (28개 파일, `isaac_ros_nitros` 레포)
   - `patches/isaac_ros_nvblox-cmake-magic_enum.patch` (`nvblox_ros/CMakeLists.txt` +
     `nvblox_ros/test/unit_tests/CMakeLists.txt`)
   둘 다 소스트리 패치라 **레포를 재-clone하면 사라진다.** 재현:
   ```bash
   git -C src/isaac_ros_nitros apply ~/cobot2_ws/patches/isaac_ros_nitros-cmake-magic_enum.patch
   git -C src/isaac_ros_nvblox apply ~/cobot2_ws/patches/isaac_ros_nvblox-cmake-magic_enum.patch
   ```

이 3개를 해결한 뒤 `colcon build --symlink-install --base-paths src --packages-up-to isaac_ros_nvblox`가
**27개 패키지 전부 성공**(약 7분, 대부분 `nvblox_core` CUDA 컴파일). `ros2 pkg list`에
`isaac_ros_nvblox`, `nvblox_ros`, `nvblox_rviz_plugin` 등이 잡히는 것으로 확인.

## 6. nvblox 실행 · 검증 · 다음 단계 — 여기서부터는 이 문서가 아니라 다른 문서를 본다

> 이 문서가 작성된 뒤(release-4.4 가정, rosbag 대신 라이브 카메라 가정) 실제 환경이 갈렸다:
> `release-3.2` 고정, 카메라 없이 rosbag 재생 기반 검증. 그 최신 상태·확정 명령어·지뢰 목록은
> **[[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6이 단일 출처다.**
> 검증 체크리스트·ESDF→PlanningScene 다음 단계도 그 문서 이후 절에서 다룬다.

## 참고 문서

- RealSense 셋업: https://nvidia-isaac-ros.github.io/getting_started/sensors/realsense_setup.html
- nvblox 개요: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html
- 트러블슈팅(RealSense): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_realsense.html
