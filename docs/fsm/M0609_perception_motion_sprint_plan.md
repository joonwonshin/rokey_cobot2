<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    Day1~5 스프린트 원본 계획 (PC 배치·GPU 의존 항목 분리는 하위 plans/ 문서가 소유)
-->

# Sprint Plan: M0609 Perception-Guided 6DoF 모션 제어 (nvblox + TAMP-lite)

> 📁 문서 지도: [[ws/cobot2/README]] · **어느 PC에서 하느냐**는 [[ws/cobot2/plans/2026-08-01-pc-role-split]]이 나눈다.
> Day1~3의 Octomap 충돌회피는 **2026-08-03에 실기 확인됨** → 결과는 [[ws/cobot2/review_moveit]].
> GPU 의존 항목(Day4·nvblox·cuRobo)은 [[ws/cobot2/plans/2026-08-03-gpu-dependent-candidates]]로 분리됐다.

**기간:** Day 1 – Day 5 (1주, 집중 스프린트) | **팀:** 1인 (본인)
**환경:** ROS 2 Humble / Doosan M0609 (6축) / RealSense D435i (eye-to-hand, 고정) / Logitech C270 (eye-in-hand, 플랜지 부착) / OnRobot RG2 그리퍼 / VoiceProcess

**스프린트 목표:**
> D435i(고정, 전역 3D 재구성)로 depth 기반 Octomap 충돌 회피맵을 만들어 MoveIt2 모션에 연결하고, 물체 인식은 **FoundationPose**(6D pose 추정, ray-plane intersection 대체)로, 그립 지점 생성은 **GraspGenX**(RG2 대응, 재학습 없는 cross-embodiment 그립 생성)로 수행하는 인식→그립→모션 파이프라인의 최소 동작 버전을 M0609 실기에서 검증한다.

> **채택 결정(개발 착수 전 반영):** 원래 계획했던 C270 기반 ray-plane intersection과 하드코딩 그립 지점 방식을 각각 FoundationPose, GraspGenX로 대체한다. C270은 이제 "근접 확인/보조" 역할로 축소되고, 메인 물체 인식은 D435i RGB-D + FoundationPose가 담당한다.

---

## 0. 두 카메라의 역할 분담 (확정)

| 카메라 | 마운트 | 역할 | 원리 | 한계 |
|---|---|---|---|---|
| **D435i** | eye-to-hand (고정) | 전역 3D 재구성 → nvblox 충돌 회피맵 | GPU 가속 TSDF/ESDF 누적 | 팔이 최종 접근 시 self-occlusion 발생 |
| **D435i** | (위와 동일) | **물체 6D pose 추정 (FoundationPose)** | RGB-D + 첫 프레임 세그멘테이션 마스크 → CAD 모델 있으면 model-based, 없으면 model-free(참조 이미지 몇 장)로 6D pose 실시간 추적. 평면/단일 레이어 가정 불필요 | 세그멘테이션 마스크 준비 필요(간단한 색상 기반 또는 수동 박스로 대체 가능) |
| **C270** | eye-in-hand (플랜지 부착) | 근접 확인·보조 (기존 ray-plane 방식은 폐기, 보류) | 그립 직전 시각적 확인용 보조 카메라로 역할 축소 | 이번 스프린트에서는 메인 파이프라인에서 제외, Day4 스코프 아님 |

**핵심 변경 근거:** ray-plane intersection은 물체가 평평한 단일 레이어 위에 있다는 가정이 필요한 임시방편이었다. FoundationPose는 이 가정 없이 D435i RGB-D만으로 실제 6D pose(위치+회전)를 추정하므로, 개발 착수 전 시점에 처음부터 이 방식으로 설계하는 게 재작업을 줄인다. C270은 hand-eye 캘리브레이션 인프라(Day2)는 유지하되, 메인 인식 경로에서는 빠지고 향후 그립 직전 근접 확인용으로만 남긴다.

**그립 생성:** RG2(2핑거 병렬 그리퍼)는 원래 GraspGen의 사전학습 그리퍼 3종(Franka, Robotiq 2F-140, 흡착)에 포함되지 않는다. 대신 **GraspGenX**(cross-embodiment, 그리퍼 URDF만으로 재학습 없이 그립 생성)를 사용한다.

---

## 1. 가정 및 확인 필요 사항

| 항목 | 가정 | 비고 |
|---|---|---|
| D435i 마운트 | 고정형(eye-to-hand), 작업공간 내려다보는 배치 | Day1 확정 |
| C270 마운트 | M0609 플랜지 부착(eye-in-hand) | Day1 확정, 그리퍼와 간섭 없는 위치 선정 |
| 픽업 대상 | 단일 레이어 가정 불필요(FoundationPose 채택으로 완화) | 다층/적재도 원칙적으로 가능하나 이번 스프린트는 단일 물체로 검증 범위 한정 |
| 그리퍼 | OnRobot RG2 (2핑거 병렬, 최대 스트로크 110mm) | GraspGenX 미지원 시 원래 GraspGen의 Robotiq 2F-140 체크포인트를 폭 오프셋 보정해 임시 대체 |
| GPU | ~~x86_64 + NVIDIA GPU(CUDA), nvidia-docker 런타임 설치됨~~ → **🔴 거짓으로 확인 (2026-08-02)** | 이 랩탑은 **Intel UHD 내장뿐, NVIDIA GPU 없음**(i7-10510U / 4C8T 1.8GHz 15W). `nvidia-smi` 미설치, `lspci`에 외장 GPU 없음. **nvblox·FoundationPose·GraspGenX 전제가 전부 깨진다** — 별도 GPU 머신 확보 또는 스코프 재조정 필요. 상세: `md/context/constraints.md` |
| VoiceProcess | 음성 명령 → 문자열 토픽 발행 가능 | Day4 어댑터로 흡수 |
| M0609 패키지 | `doosan-robotics/doosan_robot2` 기반 MoveIt2 설정 완료 | 본인 확인 사항 반영 |

---

## 1.5 빠른 초안 검증 경로 (정확도 튜닝 이전, 최소 동작만 확인)

> Day1~3 전체를 순서대로 다 하지 않고, "장애물을 놓으면 M0609가 궤적을 바꾼다"는 것만 빠르게 보여주고 싶을 때 쓰는 압축 경로. 여기서는 **정밀 캘리브레이션(`corecode/Calibration_Tutorial`), C270, ray-plane, TAMP-lite, 플래너 튜닝을 전부 생략**하고, 카메라-로봇 TF는 줄자/CAD 기반 대략값으로 대체한다. 정확도가 필요해지면 그때 Day2의 정식 캘리브레이션(`eye2hand_calibration.py`)으로 교체.

**Step 1. RealSense 실행 + depth → 포인트클라우드**
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true align_depth.enable:=true \
  camera_name:=camera pointcloud.enable:=false   # 여기선 depth_image_proc이 변환 담당
# align_depth는 point_cloud_xyz_node가 raw depth를 쓰므로 이 경로에선 불필요. RViz 확인용으로만 켠다.

ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz
```

**Step 2. 카메라→로봇 TF, 대략값으로 임시 발행 (정밀 캘리브레이션 생략)**
```bash
# 줄자/CAD로 잰 대략적인 x y z (m) + roll pitch yaw (rad) 를 base_link 기준으로 입력
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y -0.5 --z 0.6 --roll 0 --pitch 0.6 --yaw 1.57 \
  --frame-id base_link --child-frame-id camera_link
```
**주의:** 이 TF는 "임시"임을 명확히 표시해두고, 정밀도 튜닝 단계(Day2)에서 반드시 `eye2hand_calibration.py` → `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py` 결과로 교체할 것.

> ⚠️ **rosbag 녹화 중에는 이 임시 TF를 띄우지 않는다.** bag의 `/tf_static`에 가짜 값이 박히면 나중에 진짜 캘리브 값과 충돌해 bag을 못 쓰게 된다. 출근 후 순서는 **rosbag 녹화 → 캘리브 → Day1.5**이며, 상세 절차는 `md/state.md`의 "출근 후 D435i 세션" 절이 기준이다.

**Step 3. Octomap → MoveIt2 PlanningScene 연동**
```bash
sudo apt install ros-humble-octomap-server
# ⚠️ 기본 설정(848x480x30 + resolution 0.02 + max_range 무제한)은 이 랩탑에서 안 돈다.
#    octomap_server는 단일 스레드이고 ros2_control이 이미 2코어를 먹는다. 근거: constraints.md
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/camera/camera/depth/points_xyz \
  -p frame_id:=base_link \
  -p resolution:=0.03 \
  -p sensor_model.max_range:=1.5 \
  -p pointcloud_min_z:=-0.1 -p pointcloud_max_z:=1.2

ros2 launch <m0609_moveit_config> demo.launch.py   # 실기 대신 fake execution으로 먼저
rviz2   # PointCloud2 display on /octomap_point_cloud_centers (octomap_rviz_plugins 미설치)
```
> **디버깅은 반드시 이 순서로**: ① `tf2_echo base_link camera_depth_optical_frame` → ② `topic hz .../points_xyz`
> → ③ RViz PointCloud2 on `.../points_xyz` (Fixed Frame=base_link) → ④ 그제서야 octomap.
> `/projected_map`은 2D 투영이라 매니퓰레이터 디버깅에 쓸 수 없다. 상세는 `md/context/constraints.md`.

**Step 4. 궤적 확인 (fake execution)**
- 카메라 앞에 박스 등 장애물을 놓고 RViz MoveIt 플러그인에서 임의 목표 pose로 드래그 → Plan
- 궤적이 장애물을 피해가면 성공. 이 단계는 아직 실기를 움직이지 않음.

**Step 5. (확인되면) 실기로 전환**
```bash
ros2 param set /move_group velocity_scaling_factor 0.2
ros2 param set /move_group acceleration_scaling_factor 0.2
ros2 launch <m0609_moveit_config> move_group.launch.py   # 실기 드라이버 포함 launch
```
그리퍼 없이, 여유 공간 크게 두고 저속으로 먼저 확인.

**DoD (빠른 초안):** 장애물 유무에 따라 계획된 궤적이 달라지는 것을 RViz(fake) 및 실기(저속)에서 육안 확인. 좌표/그립 정밀도는 이 단계의 목표가 아님 — Day2 정식 캘리브레이션에서 다룸.

---

## 2. 데일리 백로그 (터미널 명령어 포함)

### Day1 — 카메라 파이프라인 구성

**P0. D435i → isaac_ros_nvblox 파이프라인 구성**

> ⚠️ **버전 고정 필수:** 최신 `release-4.4`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 사실상 Jazzy 중심으로 재편되어 `run_dev.sh`가 없다. Humble 환경을 유지하려면 `run_dev.sh`가 살아있는 **`release-3.2`** 태그로 관련 저장소를 모두 통일해서 클론한다.

> 아래 블록은 `scripts/setup_isaac_ros.sh`로 스크립트화되어 있다. 실제 실행은 스크립트를 쓰고, 이 블록은 설명용으로만 본다 (둘이 어긋나면 스크립트가 기준).

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

# 버전 고정: isaac_ros_common, isaac_ros_nvblox 모두 release-3.2로 통일
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-3.2 --recursive https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
# realsense-ros는 클론하지 않는다 — apt ros-humble-realsense2-camera 4.58.2 설치 확인됨 (2026-08-01)

# RealSense 지원 이미지 키 설정
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}   # Isaac ROS dev docker 컨테이너 진입 (nvidia-docker runtime 필요)

# 컨테이너 내부
sudo apt update && rosdep update
rosdep install -i -r --from-paths src --rosdistro humble -y
colcon build --symlink-install --packages-up-to isaac_ros_nvblox realsense2_camera
source install/setup.bash

# 실행 (별도 터미널들, 모두 컨테이너 attach 후)
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true align_depth.enable:=true \
  camera_name:=d435i pointcloud.enable:=true

ros2 launch isaac_ros_nvblox isaac_ros_nvblox.launch.py
rviz2   # nvblox mesh/voxel 토픽 추가해 재구성 확인 (이번 스프린트에서는 시각화·향후 확장용으로만 사용)
```
**DoD:** RViz에서 3D mesh/voxel 재구성 실시간 확인, `tf2_ros` 트리에 `camera_link(d435i) → base_link` 정상 표시.

**P1. C270 웹캠 노드 등록**
```bash
sudo apt install ros-humble-usb-cam
# 또는
sudo apt install ros-humble-v4l2-camera

v4l2-ctl --list-devices    # C270 장치 노드(/dev/videoX) 확인, D435i와 충돌 없는지 체크
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/video2" \
  -p image_size:="[1280,720]" \
  -r image_raw:=/webcam/image_raw

ros2 topic hz /webcam/image_raw   # 프레임레이트 확인, D435i 동시 구동 시 드랍 여부 체크
```
**DoD:** `/webcam/image_raw`, `/webcam/camera_info` 정상 퍼블리시, USB 대역폭 분리 확인(D435i·C270 각각 다른 USB 컨트롤러 포트에 연결 권장).

---

### Day2 — 이중 캘리브레이션 + PlanningScene 연동

> **방식 변경(2026-08-02): `easy_handeye2` → `corecode/Calibration_Tutorial` 스크립트.**
> 이유는 성능이 아니라 리스크다. `easy_handeye2`는 Humble 브랜치·의존성·`aruco` 검출 파이프라인을 새로 세워야 하는 미검증 외부 패키지인데, `corecode/`에는 이미 이 로봇(M0609, ZYZ posx)과 이 카메라를 전제로 쓰인 스크립트가 들어 있다. ROS 노드가 아니라 오프라인 파이썬이라 실패해도 스택 전체를 흔들지 않는다.
> **알고리즘은 합성 데이터로 검증했다**(2026-08-02): 정답 변환을 심고 되찾는 테스트에서 두 스크립트 모두 오차 `~1e-13`. 아래 "검증 결과" 절 참고.

**공통 1단계 — 데이터 수집 (`data_recording.py`)**
```bash
# dsr_bringup2가 떠 있어야 한다 (posx를 읽는다)
cd corecode/Calibration_Tutorial
python3 data_recording.py     # 카메라 창에서 'q' = 저장. 자세 15~20개. 종료는 Ctrl+C
```
파일 상단 설정 플래그(2026-08-02 추가):
- `USE_REALSENSE_TOPIC` — `True`면 D435i ROS 토픽(`/camera/camera/color/image_raw`), `False`면 V4L2 `/dev/video<DEVICE_NUMBER>`. **eye-to-hand(D435i)=True, eye-in-hand(C270)=False.** C270은 `ls /dev/video*`로 번호 확인.
- `RECORD_IN_FLANGE_FRAME` — 기본 `True`(= `set_tcp` 미적용). **`set_tcp`가 걸린 `posx`는 flange가 아니라 TCP 기준**이라 결과의 부모 프레임도 TCP가 된다. `True`로 두면 부모가 flange라서 CAD/줄자로 검산할 수 있다. `False`로 쓸 거면 `TOOL_NAME`/`TCP_NAME`을 **RG2 등록명으로 교체**할 것 — 티치펜던트에 없는 이름이면 원점이 어긋난다.
- 조작: `q` = 저장, **`ESC` = 정상 종료**(예전엔 탈출 코드가 없어 Ctrl+C를 써야 했다).
- **체커보드는 내부 코너 10x7 (칸 11x8) / 한 칸 24mm** → 코드에 `checkerboard_size = (10, 7)`, `square_size = 24.0`으로 반영 완료(2026-08-02). 인쇄물 칸을 캘리퍼스로 재서 갱신할 것.
- 자세마다 **회전을 충분히 섞을 것.** 평행이동만 하면 `logR()`이 0으로 나눠 NaN이 되어 eye-to-hand가 통째로 죽는다(합성 테스트로 재현 확인).

**P0-a. D435i eye-to-hand (카메라 고정 · 체커보드를 그리퍼에 부착)**
```bash
python3 eye2hand_calibration.py        # → T_cam2base.npy  (base_link 기준 카메라 pose, mm)

# npy(mm) → ROS static TF(m) 변환. 인자 그대로 실행하면 TF가 뜬다.
cd ../.. && ros2 run m0609_rg2_bringup calib_npy_to_tf.py \
  corecode/Calibration_Tutorial/T_cam2base.npy base_link camera_link

ros2 run tf2_ros tf2_echo base_link camera_link
```
**DoD:** 알려진 좌표 물체 배치 후 D435i가 인식한 3D 위치와 실측값 오차 < 1cm.

**P0-b. C270 eye-in-hand (카메라가 그리퍼에 · 체커보드는 작업공간에 고정)**
```bash
python3 handeye_calibration.py         # → T_gripper2camera.npy  (TCP 기준 카메라 pose, mm)

cd ../.. && ros2 run m0609_rg2_bringup calib_npy_to_tf.py \
  corecode/Calibration_Tutorial/T_gripper2camera.npy link_6 camera_link_webcam

ros2 run tf2_ros tf2_echo link_6 camera_link_webcam   # 고정 오프셋 확인
```
**DoD:** `link_6(또는 TCP) → camera_link_webcam` 정적 TF 확보. 이후 실시간 `camera_link_webcam → base_link`는 FK로 자동 계산됨을 `tf2_echo base_link camera_link_webcam`로 확인.

> 저장소에 이미 들어 있는 `T_gripper2camera.npy`는 **튜토리얼 잔재**다(평행이동 247.8mm, z=-233mm). 이 ws의 실측값이 아니므로 반드시 새로 만들어 덮어쓴다.

**검증 결과 (2026-08-02, 합성 데이터)**

| 항목 | 결과 |
|---|---|
| `handeye_calibration.py` (eye-in-hand, `cv2.calibrateHandEye` PARK) | ✅ 정답 복원, max\|err\| 1.1e-13 |
| `eye2hand_calibration.py` (eye-to-hand, Park-Martin 자체 구현) | ✅ 정답 복원, max\|err\| 2.0e-13 |
| A/B 쌍 구성 방향(`A=G_i G_{i+1}^{-1}`, `B=P_i P_{i+1}^{-1}`) | ✅ `AX=XB`, `X`=베이스 기준 카메라 pose로 성립 |
| 회전 규약 ZYZ = 두산 `posx` | ✅ 일치 |
| `logR()` 회전≈0에서 NaN | ⚠️ 재현됨. 수집 시 회전 섞는 것으로 회피 |
| `find_checkerboard_pose()`의 `square_size` 하드코딩(`* 25`) | ✅ **2026-08-02 수정.** 두 파일 모두 `square_size` 사용. 합성 보드 렌더링으로 거리 복원 확인(24mm/내부코너 10x7 → Z 500.00mm) |
| `sqrtm` 출력의 직교성 | ✅ 이번 데이터에선 실수·직교. `calib_npy_to_tf.py`가 매번 검사 후 SVD 정규화 |
| 내부 파라미터를 `camera_info` 대신 `calibrateCamera`로 재추정 | ⚠️ D435i 공장 intrinsic보다 나쁠 수 있음. 오차 1cm 초과 시 여기부터 의심 |

**P0. 실제 3D 포인트클라우드 → MoveIt2 충돌 회피 연동 (선택 A: cuMotion 제외, 표준 Humble 조합)**

> ⚠️ **방향 전환:** cuMotion(`isaac_ros_cumotion_moveit`)은 현재 사실상 Jazzy 전용으로 재편되어 Humble 지원이 불확실하다. M0609 MoveIt2 스택 전체가 Humble 기반이므로, 이번 스프린트는 cuMotion 없이 **수년간 검증된 표준 조합**(`depth_image_proc` → `octomap_server` → MoveIt2 OMPL)으로 충돌 회피를 구현한다. nvblox(release-3.2)는 예쁜 3D 시각화·향후 확장용으로만 병행 실행하고, 실제 충돌 회피 판단에는 관여시키지 않는다.

```bash
sudo apt install ros-humble-depth-image-proc ros-humble-octomap-server ros-humble-moveit-msgs

# D435i 정렬된 depth 이미지 → 3D 포인트클라우드 변환
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz

# 포인트클라우드를 octomap_server에 연결해 3D occupancy map 생성
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/camera/camera/depth/points_xyz \
  -p frame_id:=base_link \
  -p resolution:=0.03 \
  -p sensor_model.max_range:=1.5 \
  -p pointcloud_min_z:=-0.1 -p pointcloud_max_z:=1.2
# resolution/max_range 근거는 constraints.md "octomap_server — 리소스" 절. 기본값은 이 랩탑에서 안 돈다.

# MoveIt2 move_group 설정(sensors_3d.yaml)에 PointCloudOctomapUpdater 플러그인 등록
#   point_cloud_topic: /camera/camera/depth/points_xyz  (octomap_server 없이 MoveIt이 직접 구독하는 방식도 가능)
ros2 launch <m0609_moveit_config> move_group.launch.py

rviz2   # MoveIt 플러그인에서 Octomap 충돌 지오메트리 확인
```
**DoD:** RViz MoveIt 플러그인에서 실제 depth 기반 3D 충돌 지오메트리(Octomap)가 반영되고, OMPL이 이를 반영해 계획된 궤적이 장애물을 회피함.
**비고:** MoveIt2의 `occupancy_map_monitor/PointCloudOctomapUpdater` 플러그인을 쓰면 별도 `octomap_server` 노드 없이도 MoveIt이 포인트클라우드를 직접 구독해 내부 Octomap을 구성할 수 있다 — 리소스를 아끼고 싶으면 이 방식으로 단순화 가능(Day1 저녁 스파이크로 둘 중 하나 확정).

---

### Day3 — 충돌 회피 모션 실기 검증 및 튜닝

**P0. 장애물 회피 실기 테스트**
```bash
ros2 launch <m0609_moveit_config> demo.launch.py   # 또는 실기 드라이버 launch
ros2 run moveit_commander moveit_commander_cmdline.py   # 대화형 목표 pose 테스트(선택)

# 속도 제한 필수 — 실기 안전
ros2 param set /move_group velocity_scaling_factor 0.2
ros2 param set /move_group acceleration_scaling_factor 0.2
```
**DoD:** 임의 배치 장애물 3종 각각에서 충돌 없이 목표 pose 도달, 실패 시 재계획 로그 확보.

**P1. 플래너 튜닝 비교**
```bash
# ompl_planning.yaml에서 planner_id 교체하며 비교: RRTConnect, RRTstar, LBKPIECE 등
ros2 param get /move_group planning_pipeline
# 각 플래너별 성공률/평균 계획시간 기록 (rosbag으로 planning_scene, trajectory 기록)
ros2 bag record -o day3_tuning /move_group/monitored_planning_scene /joint_states
```
**DoD:** 튜닝 로그 표(성공률, 평균 계획 시간) 작성.

**P2. 협소 입구 통과 전용 테스트 케이스 + narrow-passage 샘플러**
— 항목명만 있고 내용 미작성. `state.md`에 잘못 섞여 들어가 있던 조각을 2026-08-03에 여기로 회수했다.
시나리오 설계는 [[ws/cobot2/review_moveit]] §4.3(고정 장면 3종) 참고.

---

### Day4 — TAMP-lite 상태머신 + FoundationPose + GraspGenX + 음성 트리거

**P0. FoundationPose 기반 6D 물체 pose 추정 (ray-plane intersection 대체)**
```bash
# 버전 정책 준수: release-3.2 계열로 통일 (isaac_ros_common과 동일 태그 확인 필수)
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git isaac_ros_pose_estimation
# ⚠️ release-3.2에 FoundationPose 패키지가 없으면 해당 시점의 최신 Humble 호환 태그로 대체 확인 필요

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}

# 컨테이너 내부
cd ${ISAAC_ROS_WS}
rosdep install -i -r --from-paths src --rosdistro humble -y
colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
source install/setup.bash

# 실행 명령은 gpu-rental-checklist §8-2 하나로 단일화했다. 3단계로 간다:
#   1) NVIDIA 퀵스타트(Mustard 메시 + NVIDIA bag)로 스택 자체를 검증
#   2) 우리 D435i bag/카메라 + iPhone LiDAR 스캔 메시  (interface_specs_file 을 848x480으로 복제)
#   3) GraspGenX 연동 — 단, GraspGenX는 pose가 아니라 instance_mask를 받는다(§8-2 단계 3)
sudo apt install -y ros-humble-isaac-ros-examples     # core.launch.py 는 clone이 아니라 apt다
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=foundationpose ...
```
> ❌ **이전 버전의 이 블록에 있던 아래 명령은 틀렸다 (2026-08-05 확인).**
> ```
> ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose.launch.py \
>   input_depth_topic:=... input_rgb_topic:=...
> ```
> `input_depth_topic`/`input_rgb_topic`은 **선언되지 않은 launch 인자**다(런치 파일에 없음).
> ros2 launch는 미선언 인자를 **에러 없이 무시**하므로(로컬 실행 확인) 토픽이 기본값에 남고
> 노드는 아무것도 못 받은 채 조용히 대기한다. 게다가 `depth/image_rect_raw`는 frame이
> `camera_depth_optical_frame`이라 컬러 내부파라미터와 맞지 않는다 — `aligned_depth_to_color`를 쓴다.
> **"CAD 없으면 model-free"도 거짓이다** — `isaac_ros_foundationpose`에 model-free 모드는 없다(`constraints.md:423`).

**DoD:** 물체 1종에 대해 FoundationPose가 추정한 6D pose와 실측 좌표 오차 확인(허용 오차 사전 정의, 예 <5mm, 회전 오차도 함께 기록).

**P0. GraspGenX 기반 RG2 그립 지점 생성**

> ⚠️ **UNVERIFIED:** 아래 URL·스크립트명·인자는 실재 확인이 안 된 상태다(`NVlabs/GraspGen`은 확인, `GraspGenX`는 미확인). Day0에 URL부터 열어보고, 없으면 리스크표의 대체안으로 간다. 이 블록을 그대로 실행해서 안 되면 그건 환경 문제가 아니라 문서 문제다.

```bash
git clone https://github.com/NVlabs/GraspGenX.git
cd GraspGenX
# RG2 URDF로 신규 그리퍼 통합 (인터랙티브 마법사가 config.json 자동 생성)
# 정확한 CLI는 저장소의 "Integrating a New Gripper" 섹션 참고
python integrate_gripper.py --urdf <RG2_URDF_경로>   # 예시, 실제 스크립트명 저장소에서 확인 필요

# FoundationPose가 준 물체 pose + 물체 메시(또는 점군)를 입력으로 그립 후보 생성
python run_graspgenx.py --gripper rg2 --object_pose <foundationpose_output>
```
**DoD:** 물체 1종에 대해 GraspGenX가 생성한 그립 후보 중 실행 가능한(충돌 없는) 그립 1개 이상 확보. GraspGenX 저장소 상태(설치 난이도, 문서화 수준)가 예상보다 미성숙하면 원래 GraspGen의 Robotiq 2F-140 체크포인트를 RG2 스트로크에 맞게 오프셋 보정해 임시 대체(리스크 참고).

**P0. TAMP-lite 상태머신 (인식→그립→서브골→모션)**
```bash
ros2 pkg create --build-type ament_python tamp_lite_statemachine \
  --dependencies rclpy moveit_msgs geometry_msgs std_msgs

# 상태: IDLE → APPROACH(D435i/Octomap 회피 경로) → PERCEIVE(FoundationPose 6D pose)
#      → GRASP_SELECT(GraspGenX 후보 중 선택) → GRASP → LIFT → DONE/FAIL
ros2 run tamp_lite_statemachine sm_node
ros2 topic pub /tamp/start std_msgs/msg/Empty "{}"   # 수동 트리거 테스트
```
**DoD:** 상태머신 노드 1개, 물체 1종 대상 pick 서브골 시퀀스(인식→그립 선택→모션)가 로그로 확인됨.

**P1. VoiceProcess 트리거 어댑터**
```bash
# VoiceProcess가 발행하는 실제 토픽/메시지 타입 확인 후 매핑
ros2 topic list | grep -i voice
ros2 topic echo /voice/command   # 예시, 실제 토픽명 확인 필요

# 어댑터 노드: /voice/command(String) 수신 시 특정 키워드 매칭 → /tamp/start 발행
ros2 run tamp_lite_statemachine voice_trigger_adapter
```
**DoD:** 음성 명령 1개("잡아" 등)로 파이프라인 트리거 확인.

---

### Day5 — 통합 검증, 회고, 백로그

**P0. 통합 반복 시행**
```bash
# 전체 파이프라인 동시 기동 (launch 파일로 통합 권장)
ros2 launch tamp_lite_statemachine full_pipeline.launch.py

# 반복 시행 로그 기록
ros2 bag record -o day5_integration_test -a
```
**DoD:** 10회 반복 시행, 성공률·실패 유형(인식 오탐/충돌맵 노이즈/IK 실패/ray-plane 오차) 분류 기록.

**P1. 회고 및 다음 스프린트 백로그 정리** — 문서 작업, 명령어 없음.

**P2(Stretch). 실패 케이스 rosbag 보관**
```bash
mkdir -p ~/failure_cases
ros2 bag record -o ~/failure_cases/case_$(date +%s) -a
```

---

## 3. 리스크 (업데이트)

| 리스크 | 영향 | 완화책 |
|---|---|---|
| **🔴 GPU PC 미확인 (`nvidia-smi`, `docker info \| grep -i runtime`)** | **스프린트 단일 실패점.** 개인 노트북엔 NVIDIA GPU가 없어(2026-08-01 확인) nvblox·FoundationPose 둘 다 실행 불가. GPU PC가 없거나 nvidia-docker가 없으면 Day4 전체가 무산된다 | **Day0에 최우선 확인.** 실패 시 대안: 인식을 CPU 경로(색상/AprilTag 기반 위치 추정 + 하드코딩 그립)로 되돌리고 FoundationPose·GraspGenX는 다음 스프린트로 이월. Day1.5~Day3(Octomap 충돌 회피)는 GPU 없이도 개인PC에서 그대로 가능 |
| **FoundationPose는 TensorRT/CUDA 필수** | nvblox를 "GPU 없음"을 이유로 Octomap으로 강등했는데, FoundationPose는 그보다 무거운 GPU 의존을 갖는다. Day4는 **GPU PC 전용 작업**이 된다 | Day4 작업 장소를 GPU PC로 명시 고정. 개인PC에서는 rosbag 재생 + Octomap/플래너/상태머신 골격만 개발하고, 인식 노드는 인터페이스(토픽·메시지 타입)만 먼저 확정해 나중에 갈아끼운다 |
| isaac_ros_common/nvblox 버전 불일치 (release-3.2 vs 최신 태그 혼용) | 빌드 실패, Day1~2 지연 | 모든 Isaac ROS 저장소를 release-3.2로 통일해서 클론, 다른 릴리스 태그와 섞지 않기. 빌드 에러 시 `.isaac_ros_common-config`의 이미지 키와 태그 조합 재확인 |
| MoveIt2 sensors_3d.yaml 플러그인 설정 경험 부재로 Day2 지연 | Day2 지연 | Day1 저녁에 `depth_image_proc`+`octomap_server` 조합을 M0609 없이 데스크탑에서 먼저 단독 테스트해 토픽 흐름부터 검증. 막히면 임시로 RViz에 수동 박스 충돌 오브젝트만 넣고 진행, 실제 depth 연동은 Day3로 이월 |
| eye-to-hand(D435i) 캘리브레이션 오차 누적 | 충돌 회피 정확도 저하 | 알려진 좌표 물체로 오차 측정 후 진행, 1cm 초과 시 Day3 보류 |
| **캘리브 결과의 부모 프레임이 flange가 아니라 TCP** (`data_recording.py`가 `set_tcp` 후의 `posx`를 기록) | TF 체인이 TCP 오프셋만큼 통째로 틀어진다. 궤적은 멀쩡해 보이는데 그립만 계속 빗나가는, 제일 찾기 어려운 형태로 나타난다 | 수집 전 `set_tcp`를 0으로 두거나, TF 부모를 TCP 프레임으로 명시. **Day2 첫 검증은 "알려진 좌표 물체"가 아니라 "TF 부모가 무엇인지"부터 확인** |
| **단위 불일치 (corecode는 전 구간 mm, ROS·FoundationPose는 m)** | 1000배 오차. 즉시 드러나므로 치명적이진 않지만 통합 시 반복 발생 | 변환은 `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py` **한 곳에서만** 한다. 다른 노드에서 `/1000`을 재차 하지 않는다 |
| `logR()`이 회전≈0인 자세쌍에서 NaN (합성 테스트로 재현) | eye-to-hand 결과가 통째로 NaN | 수집 시 자세마다 회전을 충분히 섞는다. 결과에 NaN이 보이면 데이터 문제지 코드 문제가 아니다 |
| eye-in-hand(C270) hand-eye 오프셋 오차 | 향후 C270을 근접 확인 용도로 쓸 때 오차 누적 | Day2에 캘리브레이션 인프라는 유지하되, 이번 스프린트 메인 경로에서는 C270 정밀도가 결과에 영향 없음(사용 안 함) |
| **FoundationPose 세그멘테이션 마스크 품질 의존성** | 마스크 부정확 시 6D pose 추정 오차/실패 | Day4 초반에 간단한 색상 기반 마스크로 먼저 검증, 필요 시 수동 박스 지정으로 폴백 |
| **isaac_ros_foundationpose가 release-3.2에 없거나 Humble 미지원일 가능성** | Day4 전체 지연 | Day1 저녁에 저장소 태그/브랜치를 미리 확인. 없으면 해당 시점 최신 Humble 호환 태그로 대체하거나, 최악의 경우 FoundationPose 컨테이너를 별도 이미지로 분리 실행(트레이드오프: 통합 복잡도↑) |
| **GraspGenX는 저장소 실재 자체가 미확인** (`NVlabs/GraspGen`은 확인되나 `GraspGenX`는 아님. 아래 Day4 블록의 `integrate_gripper.py`/`run_graspgenx.py`도 문서 자체가 "스크립트명 확인 필요"로 적어둠) | Day4 그립 생성 전면 재설계 | **Day0에 URL 접속 한 번으로 끝나는 확인이므로 즉시 한다.** 없으면 `NVlabs/GraspGen`의 Robotiq 2F-140 체크포인트를 RG2 스트로크(110mm)에 맞게 오프셋 보정해 쓰거나, 물체 1종만 다루므로 그립 지점 하드코딩으로 스코프 축소 |
| D435i 최종 접근 시 self-occlusion (팔이 시야 가림) | FoundationPose 추적이 그립 직전 끊길 수 있음 | 접근 마지막 단계는 미리 계획된 궤적(open-loop)으로 처리하고, C270을 필요시 근접 확인용 폴백으로 재도입 검토(다음 스프린트) |
| VoiceProcess 인터페이스 사양 불명확 | Day4 P1 지연 | P1로 낮춰뒀으므로 지연 시 이월, 최악의 경우 키보드 트리거로 대체 |
| D435i + C270 동시 구동 시 USB 대역폭 이슈 | 인식 프레임 드랍 | Day1에 별도 USB 컨트롤러 분리 연결, 필요 시 C270 저해상도 다운그레이드 |
| 실기 충돌/안전 | 하드웨어 손상, 안전사고 | Day3부터 속도 스케일링 20~30% 제한, protective stop 여유 확보, 비상정지 상시 대기 |

---

## 4. Definition of Done (스프린트 전체)

- [ ] nvblox 3D 재구성이 RViz에서 실시간 확인됨 (D435i, eye-to-hand)
- [ ] D435i depth 기반 실제 3D 포인트클라우드가 MoveIt2 PlanningScene의 Octomap 충돌 지오메트리로 반영됨 (nvblox는 시각화 용도로만 병행)
- [ ] 임의 배치 장애물에 대해 M0609가 실기에서 충돌 없이 회피 경로로 도달
- [ ] FoundationPose가 D435i RGB-D로 물체 1종의 6D pose를 추정, 위치/회전 오차가 허용 범위 이내
- [ ] GraspGenX(또는 임시 대체)가 RG2 기준 실행 가능한 그립 후보를 생성
- [ ] 인식(FoundationPose) → 그립 선택(GraspGenX) → 서브골 → 모션 실행 상태머신이 실기에서 최소 1개 물체에 대해 동작
- [ ] 10회 반복 시행 결과와 실패 유형이 기록됨
- [ ] 다음 스프린트 백로그(다층 대응, TAMP 확장, GPU 가속 등) 문서화 완료

## 5. Key Dates

| Day | 이벤트 |
|---|---|
| Day0 | **GPU PC 확인(`nvidia-smi`, docker runtime) + GraspGenX URL 실재 확인** — 둘 다 몇 분이면 끝나고, 실패 시 Day4 설계가 통째로 바뀐다 |
| Day1 | 스프린트 시작, D435i/C270 파이프라인 구성 |
| Day1.5 | 빠른 초안 검증(장애물 회피 궤적 시연) — GPU 불필요, 개인PC 가능 |
| Day2 | 이중 캘리브레이션 + PlanningScene 연동 (최대 난관 구간) |
| Day3 | 충돌 회피 실기 검증 및 튜닝 |
| Day4 | FoundationPose + GraspGenX + TAMP-lite 상태머신 + 음성 트리거 |
| Day5 | 통합 검증, 회고, 차기 백로그 |

---

## 6. 다음 스프린트 후보

> 3절 리스크와 constraints.md가 이미 확정한 사실이 우선한다: **이 랩탑엔 NVIDIA GPU가 없다**(i7-10510U, Intel UHD뿐). nvblox·FoundationPose·GraspGenX·cuMotion·cuRobo는 전부 CUDA 전제라 **GPU 머신을 새로 확보하기 전엔 후보에서 뺀다** — 여기 다시 올리면 3절과 같은 거짓 가정을 반복하는 것. 아래는 현재 CPU-only 랩탑 + 이미 실기 검증된 MoveIt/Octomap 스택(state.md, constraints.md) 위에서 실제로 착수 가능한 것만 추렸다.

### 6-1. GPU 없이 바로 착수 가능 (우선순위 순)

| 후보 | 왜 지금 가능한가 | 가치 |
|---|---|---|
| **narrow-passage 유효 상태 샘플러 튜닝** (`ObstacleBasedValidStateSampler` 등) | `ros-humble-ompl`에 이미 포함, 신규 패키지·GPU 불필요. Day3 플래너 튜닝의 연장 | 좁은 틈 그립 접근 시 RRTConnect 실패율을 직접 낮춤 — 이번 스프린트에서 이미 검증된 충돌회피 스택의 신뢰도를 높이는 가장 저비용 개선 |
| **색상/AprilTag 기반 pose 추정 + 하드코딩 그립으로 pick 서브골 완성** | FoundationPose/GraspGenX 자리에 CPU 대체재를 끼우면 Day4 상태머신(`tamp_lite_statemachine`)을 GPU 없이 끝까지 돌릴 수 있음. 인터페이스(토픽/메시지 타입)만 맞으면 나중에 GPU 모델로 교체 가능 | 이번 스프린트 최우선 DoD인 "인식→그립→모션 최소 동작"을 GPU 없이도 완주 — 이월시키지 않아도 됨 |
| **PDDLStream + FastDownward 기반 TAMP-lite 교체** | 순수 Python/CPU, Docker·GPU 불필요. 위 CPU 인식/그립을 stream으로 그대로 재사용 가능 | 겹침/차단 물체(occlusion-aware pick)에 필요한 "방해물 먼저 치우기" 같은 순서 계획을 상태머신 하드코딩 없이 얻음 — 다층/적재 대응의 전제 조건 |
| **C270 근접 확인 역할 재도입** | 이미 확보된 eye-in-hand 캘리브레이션(Day2) 재사용, 신규 하드웨어·GPU 없음 | self-occlusion 구간 폴백 — 위 CPU 인식 경로의 약점을 바로 보완 |
| **실패 케이스 rosbag → 실패 유형 분류 문서화** | Day5에 이미 녹화하는 데이터, 학습 없이 사람이 분류만 하면 됨(Fail2Progress의 "재학습" 부분은 제외) | 다음 스프린트 우선순위를 실측 실패율로 정하게 해줌 — 다른 모든 후보의 근거 데이터 |

### 6-2. GPU 머신 확보가 선행 조건 (착수 보류)

| 후보 | 보류 사유 |
|---|---|
| FoundationPose 6D pose, GraspGenX 그립 생성 | CUDA/TensorRT 필수, 이 랩탑에서 실행 불가 (constraints.md) |
| nvblox 3D 재구성 | 원래도 시각화 용도일 뿐 충돌 회피는 이미 Octomap이 담당 — GPU 있어도 후순위 |
| cuTAMP 스타일 GPU 배치 최적화, cuRobo/cuMotion 모션 플래닝 | 6-1의 CPU OMPL 스택이 이미 실기 검증됨 — GPU 버전은 그 스택이 병목일 때만 재검토 |
| People/dynamic reconstruction nvblox 모드 | nvblox 자체가 보류 상태라 선행 조건 미충족 |
| VLM 기반 자연어 지시 → 서브골 생성 | 로컬 GPU 없이도 API 호출로는 가능하나(OWL-TAMP류), TAMP-lite 골격(6-1의 PDDLStream)이 먼저 서야 붙일 대상이 생김 — 순서상 후순위 |

> **참고:** 위 명령어들은 패키지/저장소 버전에 따라 인자명이나 launch 파일명이 다를 수 있습니다(특히 `doosan_robot2`, VoiceProcess 인터페이스는 본인 환경의 실제 저장소 문서로 재확인 필요). 실행 전 각 저장소 README의 Humble 대응 브랜치를 먼저 확인하세요.
>
> **버전 정책:** 이번 스프린트는 Humble 유지를 위해 Isaac ROS 관련 저장소를 모두 `release-3.2` 태그로 고정한다. 최신 `release-4.x`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 cuMotion 등 신규 패키지가 사실상 Jazzy 중심으로 재편되어, 이번 스코프(Humble/M0609)에서는 사용하지 않는다.
