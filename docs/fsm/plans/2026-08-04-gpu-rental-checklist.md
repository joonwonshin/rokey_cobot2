<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    GPU 대여 절차(§1~5) · 밟은 지뢰 목록(§6) · 확정 명령어(§7)
-->

# GPU 대여 준비/작업 체크리스트

**작성일:** 2026-08-04
**성격:** 부가 문서. 본 계획은 [[ws/cobot2/M0609_perception_motion_sprint_plan]] §6-2, [[ws/cobot2/plans/2026-08-03-gpu-dependent-candidates]]에 있음. 이 문서는 "GPU를 어떻게 확보하느냐"의 실행 절차만 다룬다 — 무엇을 테스트할지는 위 두 문서가 기준.
**배경:** 팀 실물 GPU(RTX 4070)를 팀원들과 공유 중이라 자리를 못 잡을 때, 대여 GPU로 FoundationPose/GraspGenX 소프트웨어 스택(빌드·노드 구동·알고리즘 플로우)을 먼저 검증해두는 용도. **최종 시연은 이 랩탑의 로컬 RTX 4060 Laptop 8GB에서 진행**([[ws/cobot2/context/constraints]] — 2026-08-05 확인. 이전 판의 "4070" 가정은 낡았다).

> ⛔ **인스턴스에 붙으면 §6 「밟은 지뢰 목록」부터 읽는다.** 여기 있는 10건은 전부 실제로 한 번씩 당한 것이고,
> 대부분 **에러 메시지가 원인을 안 가리킨다**. 새 세션의 나는 이 대화를 기억하지 못한다.

---

## 0. 이 방식으로 확인되는 것 / 안 되는 것 (매번 되새길 것)

✅ 패키지 빌드, 노드 기동, 인터페이스(토픽/메시지) 정합성, 알고리즘 플로우
❌ 정확도 DoD(실측 좌표 오차), 실기 그립/모션, self-occlusion 같은 동적 장면, RTX 4060 8GB 실물 성능

원격 GPU와는 **실시간 통신을 하지 않는다.** rosbag 파일을 통째로 복사해 원격 머신 안에서 혼자 재생·처리하는 방식이다 (네트워크가 개입하는 지점은 파일 전송 시점뿐).

---

## 1. 대여 전 준비물

- [ ] **rosbag 파일 확보** — 기존 계획에 이미 있는 녹화 지점(Day3 P1 플래너 튜닝, Day5 통합 테스트) 재활용, 없으면 D435i로 짧게 새로 녹화
  ```bash
  ros2 bag record -o gpu_test_bag /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info /camera/camera/color/image_raw
  ```
- [ ] **정확도 검증까지 하고 싶으면** bag 촬영 시점에 물체의 실제 좌표를 줄자/CAD로 재서 따로 기록해둘 것 (안 해두면 이 세션에서 정확도 DoD는 확인 불가)
- [ ] **GraspGenX 저장소 실재 확인** — GPU 없이 지금 브라우저로 URL 확인 가능, Day0 항목 그대로
- [ ] SSH 키 페어 준비 (`ssh-keygen`), 서비스 콘솔에 퍼블릭 키 등록
- [ ] 서비스 계정 + 결제수단 등록 (사용자 직접 — 대행 불가)
- [ ] 예산 한도 인지: 세션당 대략 $2~10 예상(서비스·GPU·시간에 따라 변동, 대여 직전 콘솔에서 현재가 재확인)

---

## 2. 서비스 선택 순서 (비용 낮은 순으로 시도, 막히면 다음)

| 순서 | 서비스 | 이유 |
|---|---|---|
| 1 | **Lightning AI 무료 티어** | 신용카드 불필요, 월 15 크레딧(~80 GPU-시간, 저사양 기준). 먼저 시도 |
| 2 | **Lambda Cloud** | 진짜 VM(루트 SSH), nested docker 문제 없음. Lightning/RunPod에서 docker 막히면 이쪽 |
| 3 | **Vast.ai / RunPod** | 저렴하나 인스턴스가 컨테이너일 수 있어 nested docker 확인 필수 |

## 3. 대여 시작 직후 — 가장 먼저 할 것 (모든 서비스 공통)

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```
두 번째 명령이 실패하면 (컨테이너 안에서 docker 중첩 불가) → **Docker/Isaac ROS 워크플로우는 이 인스턴스에서 불가**, ROS 없이 순수 PyTorch(YOLO 등)만 가능하거나 서비스를 바꿔야 함.

**✅ 검증됨 (2026-08-04, Lightning AI 무료 티어, Tesla T4):** 두 명령 모두 통과. `nvidia-smi`에서 Tesla T4 15360MiB 인식, `docker run --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`도 컨테이너 안에서 정상적으로 같은 GPU를 잡음 — nested docker 문제 없음. Isaac ROS Day4 P0 블록(`run_dev.sh` 포함) 그대로 진행 가능.
**주의:** T4는 Turing 아키텍처(로컬 RTX 4060=Ada Lovelace와 세대 다름) — 빌드·노드 구동·알고리즘 플로우 검증엔 문제 없으나, TensorRT 엔진은 RTX 4060 실물에서 반드시 재빌드해야 함(§0 원칙 그대로). VRAM도 T4 15GB보다 **좁다**(8GB) — `num_grasps` 등은 로컬 실행 시 낮춰야 한다.

---

## 4. 대여 중 작업 순서

**✅ 진행 상황 (2026-08-04):** bag 4종(`obstacle1`, `hand`, `robot_moving`, `apple`) 업로드 완료, SSH 접속 완료, `isaac_ros_common`/`isaac_ros_pose_estimation` 클론 완료. 다음은 §4-2(`run_dev.sh`)부터.

**⚠️ bag 재생 시 주의 — `ros2 bag info`로 4종 전부 실측 확인함:**
- 컬러가 **compressed로만** 녹화돼 있음(`/camera/camera/color/image_raw/compressed`), Day4 P0가 기대하는 raw `/camera/camera/color/image_raw`는 bag에 없음 → 재생 시 압축 해제 노드를 반드시 같이 띄울 것(아래 3번)
- depth(`/camera/camera/depth/image_rect_raw`)는 raw로 존재, 그대로 사용 가능
- `/camera/camera/extrinsics/depth_to_color`(`realsense2_camera_msgs/msg/Extrinsics`) 토픽 존재 — apt `ros-humble-realsense2-camera`(4.58.2, 2026-08-01 설치 확인)가 의존성으로 `realsense2_camera_msgs`도 깔아주므로 추가 조치 불필요
- 헤드리스 인스턴스라 RViz2 시각화 불가 — pose는 `ros2 topic echo`로 텍스트 확인

1. bag 파일 전송 — **완료 (2026-08-04)**
   ```bash
   scp -i <key> -r rosbag/bag_0803calibed/* <user>@<instance-ip>:~/
   ```
2. Docker: **켜야 함.** Isaac ROS는 컨테이너 안에서 빌드/실행하는 게 표준 워크플로우(`run_dev.sh`가 dev 컨테이너를 띄움) — §3에서 nested docker는 이미 검증 통과했으니 그대로 진행
   ```bash
   cd ${ISAAC_ROS_WS}/src/isaac_ros_common
   ./scripts/run_dev.sh ${ISAAC_ROS_WS}
   # 컨테이너 내부
   rosdep install -i -r --from-paths src --rosdistro humble -y
   colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
   source install/setup.bash
   ```
3~4. **bag 재생 + FoundationPose → §8-2로 단일화했다 (2026-08-05).** 여기 있던 명령 2개는 전부 틀렸었다:
   `--loop`는 지뢰 5(sim time 역행)와 정면 충돌하고, `input_depth_topic`/`input_rgb_topic`은
   **존재하지 않는 launch 인자**라 에러 없이 무시된다. republish도 §8-2 런치가 직접 띄우므로 따로 실행하면 지뢰 9(중복)다.
5. GraspGenX 블록 이어서 실행 (저장소 실재 시), 실행 가능한 그립 후보 로그 확인
6. 위 2~4를 나머지 3개 bag(`obstacle1`, `hand`, `robot_moving`)에도 반복
7. **결과물 회수** — 로그, 스크린샷, 빌드 산출물 중 필요한 것만 로컬로 scp
   ```bash
   scp -i <key> <user>@<instance-ip>:~/results/* ./
   ```

---

## 5. 종료 체크리스트 (과금 방지, 필수)

- [ ] 결과물 로컬로 다 받았는지 확인 (persistent storage 안 붙였으면 종료 즉시 디스크 삭제됨)
- [ ] 인스턴스 **Terminate/Stop** — 콘솔에서 직접 확인, 켜둔 채로 방치하면 계속 과금
- [ ] 이번 세션에서 확인된 사실(빌드 성공 여부, release-3.2 호환성, GraspGenX 실제 CLI명 등)을 `M0609_perception_motion_sprint_plan.md`의 해당 Day4 블록 또는 `md/context/constraints.md`에 반영 — 다음 세션이 이 정보를 기억하지 못하므로

---

## 6. 밟은 지뢰 목록 — 같은 실수 반복 방지 (최우선)

**전부 실제로 당한 것.** 굵은 글씨가 "왜 못 알아챘나"다.

| # | 증상 | 진짜 원인 | 해결 | 지문 |
|---|---|---|---|---|
| 1 | `Couldn't parse params file: '--params-file /share/...'` | `$(ros2 pkg prefix nvblox_examples_bringup)`가 **Package not found를 stderr로 뱉고 빈 문자열을 반환** → 경로 앞이 잘림. `nvblox_examples_bringup`은 `visual_slam`/`triton`/`unet`/`detectnet`에 의존해 `--packages-up-to isaac_ros_nvblox`로는 **안 깔린다** | `--params-file`에 **소스 절대경로**를 쓴다 (§7) | 경로가 `/share/`로 시작한다 = `$(...)`가 빈 값 |
| 2 | depth 토픽이 0.2~0.7 Hz, `camera_info`는 14 Hz 정상 | **Fast DDS가 848×480 depth(814 KB)를 못 흘린다.** 메시지 크기 의존이 지문 | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` → 15.2 Hz 회복 | 큰 메시지만 느림. `/dev/shm`·`rmem_max`·CPU는 **전부 배제됨** |
| 3 | `Lookup transform failed for frame base_link` + 재구성 전무 | nvblox `global_frame` 기본이 **`odom`**인데 우리 TF엔 없다. **로그가 이름을 대는 `base_link`는 원인이 아니라 `map_clearing_frame_id`다** | `-p global_frame:=base_link` | "콜백은 도는데 `ros/depth/integrate` 타이머가 없다" |
| 4 | `parameter 'esdf_slice_height' cannot be set because it was not declared` | 매퍼 파라미터는 **`static_mapper.` 접두사**가 붙는다 | `-p static_mapper.esdf_slice_height:=0.0` | yaml이 `static_mapper:`로 중첩돼 있음 |
| 5 | 프레임 96% 폐기 (`callback` 15641 vs `integrate` 633) | `ros2 bag play -l`(루프)로 **sim time이 뒤로 점프** → TF 버퍼 클리어 | **`-l` 금지. `-r 0.25` 감속으로 대체** | `[WARN] [tf2_buffer]: Detected jump back in time` |
| 6 | Foxglove `400 Bad Request / Missing sec-websocket-protocol` | bridge 3.4.2(Rust)는 서브프로토콜이 **`foxglove.sdk.v1`**. 구버전 이름 아님 | 클라이언트를 최신으로(브라우저 `app.foxglove.dev`) | **localhost `curl -H "Sec-WebSocket-Protocol: foxglove.sdk.v1"` 한 번이면 프록시 탓/클라 탓이 갈린다. 나는 프록시를 먼저 의심해 틀렸다** |
| 7 | `ssh root@10.192.15.146` 무한 timeout | `10.x`는 **AWS VPC 사설 IP**, 인터넷에서 라우팅 불가 | Lightning 포트 공유 `wss://8765-<studio-id>.cloudspaces.litng.ai` (포트 번호 뒤에 안 붙임) | timeout이지 refused가 아니다 |
| 8 | `static_esdf_pointcloud`가 `topic hz` 0 | **구독자가 0이면 계산 자체를 안 한다** (`get_subscription_count() > 0` 게이트) | Foxglove에서 구독하면 그때 타이머가 생긴다 | 정상 동작이다. 버그로 오진 말 것 |
| 9 | nvblox가 컬러 프레임을 중복 적분 | `republish`를 **두 번 띄웠다** (PID 13309, 13337 동시 존재) | `ros2 topic info /camera/camera/color/image_raw` → `Publisher count: 1` 확인 | `ps`에 `republish`가 2줄 |
| 10 | 명령이 중간에서 끊김 | 붙여넣기에 **`\ `(백슬래시+공백)**가 섞임 → 개행이 아니라 공백을 이스케이프 | 줄 끝 공백 제거 | 뒤쪽 `-r` 인자가 별도 명령으로 실행됨 |

### 이 목록에서 뽑은 규칙

1. **`$(...)`를 인자 안에 넣지 않는다.** 실패해도 조용히 빈 문자열이 되어 경로만 이상해진다. 경로는 미리 `ls`로 확인하고 절대경로를 박는다. *(지뢰 1 — 이건 내가 만든 것이다)*
2. **에러 메시지가 대는 이름을 믿지 않는다.** 지뢰 3이 대표. 로그의 `base_link`는 범인이 아니었다.
3. **원격/네트워크를 먼저 의심하지 않는다.** 지뢰 6에서 localhost curl 한 번이 답이었다.
4. **`docker exec`로 여는 셸마다 환경변수가 초기화된다.** 지뢰 2의 `RMW_IMPLEMENTATION`은 **컨테이너 `~/.bashrc`에 넣어야** 한다. 한 터미널만 CycloneDDS면 그 노드만 안 보인다.
5. **`--params-file`이 실렸는지는 타이밍 표로 확인한다.** `ros/update_esdf`가 5.0(≠10 Hz), `ros/tick`이 400(≠100 Hz)이면 안 실린 것이다.

---

## 7. 확정 명령어 — nvblox + bag (Lightning AI 인스턴스, cwd `/workspaces/isaac_ros-dev`)

**모든 셸에서 먼저:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 지뢰 2. 컨테이너 ~/.bashrc에 박아둘 것
source /opt/ros/humble/setup.bash && source install/setup.bash
```

**터미널 1 — nvblox** (`/tf_static`을 놓치지 않게 **가장 먼저**)
```bash
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p use_sim_time:=true \
  -p global_frame:=base_link \
  -p use_lidar:=false \
  -p num_cameras:=1 \
  -p static_mapper.esdf_slice_min_height:=-0.3 \
  -p static_mapper.esdf_slice_max_height:=0.5 \
  -p static_mapper.esdf_slice_height:=0.0 \
  -r camera_0/depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info
```
- `--params-file`은 **소스 절대경로**다. `$(ros2 pkg prefix ...)` 쓰지 말 것(지뢰 1).
- **`nvblox_realsense.yaml` specialization을 얹지 말 것** — `map_clearing_frame_id: camera0_link`를 넣는데 우리 bag TF엔 `camera_link`뿐이다. base yaml 기본값 `base_link`가 맞다.

**터미널 2 — 컬러 압축 해제** (bag에 raw color가 없다, §8 참조)
```bash
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/camera/camera/color/image_raw/compressed \
  -r out:=/camera/camera/color/image_raw
```
`use_sim_time`은 불필요 — republish는 clock을 읽지 않고 header를 그대로 복사한다. **중복 실행 금지**(지뢰 9).

**터미널 3 — Foxglove**
```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true
```

**터미널 4 — bag 재생 (맨 마지막)**
```bash
ros2 bag play <bag경로>/d435i_0803_2143_robot_moving --clock -r 0.25 --disable-keyboard-controls
```
`-l`(루프) 금지(지뢰 5). `--clock`은 노드 쪽 `use_sim_time:=true`와 반드시 짝.

### 토픽 연동 관계

```
bag play ──┬─ /clock ─────────────────────────────────────► 전 노드 (use_sim_time:=true)
           ├─ /tf_static (4) + /tf (10Hz) ────────────────► nvblox transformer_
           │                                                base_link → camera_color_optical_frame
           ├─ .../aligned_depth_to_color/image_raw ──┐ 16UC1 848x480
           ├─ .../aligned_depth_to_color/camera_info ┴──► camera_0/depth/*  [ExactTime sync]
           ├─ .../color/camera_info ────────────────────┐
           └─ .../color/image_raw/compressed            │
                    └─►[republish]─► .../color/image_raw┴──► camera_0/color/*  [ExactTime sync]
```

**bag 실측 (2026-08-04, `d435i_0803_2143_robot_moving`을 sqlite3로 직접 디코딩):**
- 컬러는 **compressed만** (1720개, `rgb8; jpeg compressed bgr8`) → republish 필수
- depth는 **raw로 존재** (`aligned_depth_to_color/image_raw`, 16UC1, 848×480, 1706개) → republish 불필요
- TF: `world→base_link→camera_link→camera_color_frame→camera_color_optical_frame` **전부 tf_static**. `global_frame:=base_link` 성립
- nvblox는 image↔camera_info를 **ExactTime**으로 묶는다(`image_exact_sync`). bag의 stamp가 나노초까지 일치함을 확인 — 근사 동기화가 아니라 **완전 일치**가 필요하다
- 14.8 Hz → `-r 0.25`면 3.7 Hz. yaml의 `integrate_depth_rate_hz: 40` / `integrate_color_rate_hz: 5` 아래라 프레임 안 버려짐

### Foxglove에서 볼 토픽 (`~/` = `/nvblox_node/`)

| 토픽 | 타입 | 렌더 |
|---|---|---|
| `~/static_esdf_pointcloud` | `PointCloud2` | ✅ |
| `~/tsdf_layer_marker`, `~/color_layer_marker` | `Marker` | ✅ 주력 |
| `~/esdf_slice_bounds` | `Marker` | ✅ 슬라이스 높이 튜닝용 |
| `~/static_occupancy_grid` | `OccupancyGrid` | ✅ |
| `~/mesh`, `~/tsdf_layer` | `nvblox_msgs/*` | ❌ 커스텀 타입 |

### 아직 안 건드린 튜닝 노브

- `voxel_size:=0.02` — 기본 0.05는 코봇 작업영역엔 거칠다. VRAM/속도가 대가.
- `static_mapper.tsdf_decay_factor:=0.9` — **nvblox엔 self-filter가 없다.** 움직이는 로봇 팔이 TSDF에 구워지므로 잔상이 남으면 낮춘다.

---

## 8. 실측 사실 — Isaac ROS 컨테이너 (constraints.md에서 이관, 2026-08-04)

환경: Lightning AI Studio(`ip-10-192-15-146`) 위 Isaac ROS 3.2 dev 컨테이너,
`/workspaces/isaac_ros-dev` 바인드 마운트. **카메라 없음 — 전부 rosbag 재생으로 검증한다.**

### Fast DDS / nvblox `global_frame` / `bag play -l` / 매퍼 파라미터 접두사 — 대여 GPU와 무관한 보편 사실

> 📤 **2026-08-05에 [[ws/cobot2/context/constraints]] "nvblox / DDS"로 이관했다.** 어느 머신에서
> nvblox를 돌리든 재현되는 소프트웨어 속성이라 대여 GPU 전용 문서에 둘 이유가 없다.
> `use_lidar` 기본값(`true`)과 `--params-file` 타이밍 검증법도 그쪽에 함께 있다.

### `static_esdf_pointcloud`가 안 나오는 두 가지 이유

1. **구독자가 0이면 아예 계산도 안 한다** — `nvblox_node.cpp:819` `get_subscription_count() > 0` 게이트.
   `ros2 topic hz`로 붙는 순간 `ros/static/esdf/output/pointcloud` 타이머가 생긴다.
2. **`esdf_slice_height` 기본이 1.0 m** — `esdf_slice_conversions.cu:66`이 모든 점의 `point.z`를
   슬라이스 높이로 덮어쓴다. 즉 `base_link` **위 1 m 평면**에 납작하게 그려진다. 씬이 테이블 위면 안 보인다.
   (기본값 근거: `esdf_integrator_params.h:32-43`)

### Foxglove (헤드리스 시각화)

- apt에 **`3.4.2-1jammy` 하나뿐**이다(`apt-cache madison`). 다운그레이드 불가 → **클라이언트를 최신으로**.
- 3.4.2는 Rust 구현이라 서브프로토콜이 **`foxglove.sdk.v1`**다(구버전 `foxglove.websocket.v1` 아님).
  구클라이언트는 `400 Bad Request / Missing expected sec-websocket-protocol header`로 떨어진다.
  ```bash
  curl -i -N -H "Sec-WebSocket-Protocol: foxglove.sdk.v1" ... # → 101 Switching Protocols
  ```
  이 localhost curl 한 번으로 **프록시 탓인지 클라이언트 탓인지가 갈린다.** 나는 프록시를 먼저 의심해서 틀렸다.
- **SSH 터널 필요 없다.** `10.x`는 AWS VPC 사설 IP라 인터넷에서 라우팅 불가 —
  `ssh root@10.192.15.146`은 무조건 timeout이다. Lightning 포트 공유를 쓴다:
  ```
  wss://8765-<studio-id>.cloudspaces.litng.ai      ← 포트 번호를 뒤에 붙이지 않는다
  ```
  브라우저 `app.foxglove.dev` 권장(항상 최신, `wss://`라 mixed-content 차단도 없음).
- 렌더 가능 여부는 **메시지 타입**이 가른다. `nvblox_msgs/Mesh`·`VoxelBlockLayer`는 커스텀이라 ❌.
  `sensor_msgs/PointCloud2`(`static_esdf_pointcloud`, `back_projected_depth/*`),
  `visualization_msgs/MarkerArray`(`tsdf_layer_marker`), `nav_msgs/OccupancyGrid` ✅.
- ✅ **부수 소득**: `/camera/camera/color/image_raw`를 3D 패널에서 카메라 pose에 투영했더니
  실제 씬과 일치했다 — **핸드아이 캘리브와 TF 체인이 맞다는 독립 검증**이다.

### bag에 raw color가 없다 — republish는 계속 필요하다

`md/rosbag-d435i.md:273`에 기록된 대로 `color/image_raw`(raw)는 **의도적으로 녹화에서 뺐다**
(848x480@30fps = 37 MB/s). compressed만 있으므로:
```bash
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/camera/color/image_raw/compressed \
             -r out:=/camera/camera/color/image_raw
```

### Lightning 영속성 / 백업

- `/teamspace` = 영구(코드·bag·`build/install`). **도커 이미지는 `/var/lib/docker`** — 영속 여부 미확인.
- 백업: `~/docker_backup/isaac_ros_dev-x86_64.tar` (35 G, **`docker save`로 생성** — 사용자 확인 2026-08-04. 따라서 `docker load`로 복구된다).
  ⚠️ **이 tar는 foundationpose 빌드와 cyclonedds·foxglove-bridge apt 설치 이전 시점이다.**
  컨테이너가 죽으면 재빌드 20분+. `docker commit` 후 `docker save`로 갱신할 것.

### 도커 이미지가 날아갔을 때 — tar에서 복구 (3줄)

```bash
docker load -i ~/docker_backup/isaac_ros_dev-built.tar   # 35G, 수 분 소요
docker image ls | grep isaac_ros_dev                       # isaac_ros_dev-x86_64:latest 인지 확인
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh -b -d ${ISAAC_ROS_WS}
```

**`-b`(`--skip_image_build`)가 핵심이다.** 안 붙이면 `run_dev.sh`가 Dockerfile부터 다시 빌드해
방금 load한 이미지를 덮어쓴다 — tar를 푼 의미가 없어진다
(`run_dev.sh:203-205`, `-b`는 `:59`에서 `SKIP_IMAGE_BUILD=1`).
이미지 이름은 `BASE_NAME="isaac_ros_dev-$PLATFORM"`로 고정이라(`:179`)
**태그가 정확히 `isaac_ros_dev-x86_64`여야 `docker image ls --quiet`가 찾는다**(`:219`).

| 상황 | 조치 |
|---|---|
| `docker image ls`에 `<none>` 으로 뜸 | `docker tag <IMAGE_ID> isaac_ros_dev-x86_64:latest` |
| `no space left on device` | tar 35G + 언팩 35G = **70G 필요.** `docker system prune -a` 후 재시도 |
| `open ...tar: no such file` | tar가 `/teamspace` 밖(`~/`)이면 인스턴스와 함께 날아갔을 수 있다. §5대로 로컬로 받아뒀는지 확인 |
| load는 됐는데 `run_dev.sh`가 또 빌드함 | `-b` 빠뜨린 것 |

**복구 후 반드시 다시 해야 하는 것** (tar가 그 이전 시점이라 — 위 ⚠️):
```bash
# 컨테이너 내부
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc   # 지뢰 2, 지뢰 규칙 4
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge
colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
```

**다음번엔 이 짓 안 하려면 — 지금 백업 갱신:**
```bash
docker commit isaac_ros_dev-x86_64-container isaac_ros_dev-x86_64:latest
docker save -o ~/docker_backup/isaac_ros_dev-x86_64.tar isaac_ros_dev-x86_64:latest
```
- 컨테이너명은 `$BASE_NAME-container`로 고정(`run_dev.sh:183`) = `isaac_ros_dev-x86_64-container`.
- ⛔ `docker export`/`import`를 쓰지 말 것. 컨테이너 파일시스템만 뜨고 **`ENV`/`CMD`/`WORKDIR`이 날아가** `run_dev.sh`가 띄운 컨테이너의 ROS 환경이 안 잡힌다. `save`/`load` 짝만 쓴다.
- `docker save`는 **바인드 마운트를 담지 않는다.** `/workspaces/isaac_ros-dev`(=`/teamspace` 쪽 소스·bag·`build/install`)는 이미지에 안 들어가므로 별도로 챙길 것.

> 참고: §4-2의 `./scripts/run_dev.sh ${ISAAC_ROS_WS}`(위치인자)는 **틀린 형식이다.**
> `run_dev.sh:51`은 디렉토리를 `-d`로만 받고, 위치인자는 `docker exec ... /bin/bash $@`로 흘러간다(`:195`).

### 8-2. FoundationPose + bag — 확정 명령 (2026-08-05 갱신)

**기성 런치 3개는 전부 bag에 못 쓴다. 소스 확인 결과다:**

| 런치 | 왜 안 되나 |
|---|---|
| `isaac_ros_foundationpose.launch.py` | **입력 토픽 인자가 없다**(선언된 건 mesh/texture/model/engine/mask/rviz 8개뿐). 게다가 `launch_bbox_to_mask:=True`를 켜도 마스크가 `rt_detr_segmentation`으로 나가는데 FP는 `segmentation`을 구독한다 — **런치 안에서 연결이 끊겨 있다**(`:156` vs `:132`) |
| `*_realsense.launch.py` | `:137` RealSense 노드를 띄운다. 게다가 해상도가 **1280×720 하드코딩**(`:36-37`)이라 848×480 bag과 resize 노드가 안 맞는다 |
| `*_core.launch.py` | `isaac_ros_examples`를 import(`:20`)하는데 그 패키지가 워크스페이스에 없다 → **import 불가**. §8-2 이전 판의 `input_images_expect_freq:=15`도 이 런치 인자라 지금은 못 쓴다 |

**⛔ 이전 판(§4-4, 스프린트 계획 Day4 P0)의 명령은 틀렸다:**
```bash
ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose.launch.py \
  input_depth_topic:=... input_rgb_topic:=...      # ← 둘 다 미선언 인자
```
ros2 launch는 **미선언 인자를 에러 없이 무시한다**(로컬 더미 런치로 실행 확인, 2026-08-05).
즉 토픽은 기본값에 남고 노드는 조용히 아무것도 안 받는다. **가장 안 좋은 실패 형태다.**

**대신 NVIDIA 퀵스타트부터 간다.** `*_core.launch.py`가 요구하는 `isaac_ros_examples`는 **clone이 아니라 apt 패키지**다
(`ros-humble-isaac-ros-examples`) — 컨테이너 안에서 설치하면 §8-2 위 표의 3번째 블로커는 사라진다.
그리고 core fragment는 해상도를 **`interface_specs_file`(JSON)에서 읽으므로**(`isaac_ros_foundationpose_core.launch.py:58-59`)
848×480도 JSON 한 줄로 해결된다 — realsense 런치의 1280×720 하드코딩 문제도 여기선 없다.

#### 단계 1 — 퀵스타트 (NVIDIA 자산 + NVIDIA bag). 우리 bag·우리 물체는 아직 안 쓴다

```bash
# 컨테이너 안
sudo apt install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-rtdetr

# 자산 (⚠️ 아래는 4.5 문서 기준. 우리는 release-3.2 이므로 MAJOR=3 MINOR=2 로 바꿔 확인할 것)
#   NGC_RESOURCE=isaac_ros_foundationpose_assets, NGC_FILENAME=quickstart.tar.gz
#   -> ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/{quickstart.bag, Mustard/textured_simple.obj,
#      quickstart_interface_specs.json}
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose && cd $_
wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.1_onnx/files/refine_model.onnx' -O refine_model.onnx
wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.1_onnx/files/score_model.onnx'  -O score_model.onnx

# TensorRT 엔진 (수 분 소요. 이게 끝나기 전에 bag을 틀면 프레임을 통째로 잃는다)
/usr/src/tensorrt/bin/trtexec --onnx=refine_model.onnx --saveEngine=refine_trt_engine.plan \
  --minShapes=input1:1x160x160x6,input2:1x160x160x6 --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --maxShapes=input1:42x160x160x6,input2:42x160x160x6
/usr/src/tensorrt/bin/trtexec --onnx=score_model.onnx  --saveEngine=score_trt_engine.plan \
  --minShapes=input1:1x160x160x6,input2:1x160x160x6 --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --maxShapes=input1:252x160x160x6,input2:252x160x160x6

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=foundationpose \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/quickstart_interface_specs.json \
  mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mustard/textured_simple.obj \
  refine_engine_file_path:=.../refine_trt_engine.plan \
  score_engine_file_path:=.../score_trt_engine.plan \
  rt_detr_engine_file_path:=.../sdetr_grasp.plan
```

**단계 1에서 반드시 기록할 것 (단계 2 설계가 여기 달려 있다):**
```bash
ros2 topic echo /depth --field encoding --once        # 16UC1 인가 32FC1 인가 ← 우리 bag 변환 여부가 갈린다
ros2 topic echo /image_rect --field encoding --once   # rgb8 인가 bgr8 인가
ros2 topic list | grep -E "segmentation|detections"   # sdetr 마스크 토픽 이름
cat .../quickstart_interface_specs.json               # 우리 848x480용 JSON을 이 형식으로 복제한다
```
> core fragment는 `depth`를 drop node(`depth_format_string: nitros_image_mono16`)로 받아 **ConvertMetric 없이**
> FP(`nitros_image_32FC1`)에 넣는다. 소스만 봐서는 앞뒤가 안 맞으므로 **퀵스타트 실행으로 실제 인코딩을 확인**한다.
> 여기서 확인되기 전에 우리 bag 변환 노드를 짜지 않는다.

#### 단계 2 — 우리 D435i bag/카메라 + 스캔 메시

- `interface_specs_file`을 848×480으로 복제
- bag → fragment 입력 3개로 브리지: `image_rect`(republish 필요, compressed만 있음) / `camera_info_rect` / `depth`
- `input_images_expect_freq:=15`, `input_images_drop_freq:=0` (bag 14.7 Hz)
- **마스크는 SyntheticaDETR(`sdetr_grasp`)가 만든다** — 고정 bbox 노드는 필요 없다

#### 단계 3 — GraspGenX 연동

`infer_scene_depth(depth, intrinsics, instance_mask, sweep_volume)`는 **pose를 안 받는다.**
연동 지점은 둘 중 하나이고, 어느 쪽인지 정하고 시작해야 한다:
1. **마스크 재사용** — FP 파이프라인의 `segmentation`을 GraspGenX `instance_mask`로 그대로 넣는다.
   `2026-08-05-graspgenx-gpu-sprint.md` §2-A의 DBSCAN 노드가 통째로 불필요해진다. **가장 싸다**
2. **완전 점군** — FP의 6D pose + 메시로 가려진 면까지 포함한 점군을 만들어 넣는다. 단일 시점 depth보다 낫다

**설계상 반드시 알아야 할 것 (단계 2 이후):**
1. **stamp가 전부다.** FP는 4입력을 GXF sync로 묶고 기본 `sync_threshold=0`(완전 일치, `foundationpose_node.cpp:199`).
   bag 실측(apple, 2026-08-05): color·camera_info·depth **3중 교집합 752/780** → 약 3.6%는 원래 버려진다.
   한 프레임도 안 나오면 `sync_threshold:=<ns>`부터 올려본다.
2. `texture_path`는 **FoundationPoseNode가 선언하지 않는 파라미터다**(런치에만 있음). 텍스처는 `.obj`의 `.mtl`로 따라온다.
3. TensorRT 엔진을 `/tmp` 기본값에 두지 않는다 — 컨테이너 재시작에 날아간다.
4. `run_dev.sh:288`은 **`isaac_ros-dev`만 마운트한다.** `rosbag/`·`scripts/`는 컨테이너에서 안 보인다 →
   추가 마운트(`~/.isaac_ros_dev-dockerargs`)를 넣거나 `isaac_ros-dev/` 아래로 옮겨야 한다.

#### 스캔 메시(iPhone LiDAR) 준비 시 확인할 것 — ⚠️ 전부 미검증

- **단위 m.** 스캔 앱이 cm/mm로 내보내면 포즈가 100배/1000배 어긋난다
- **텍스처 필수.** FoundationPose는 RGB 렌더를 비교해 점수를 매긴다. 무텍스처 메시는 score 단계가 약해진다
- **메시 로컬 원점 = pose 원점.** 스캔 원점은 임의이므로, 파지 좌표를 논하려면 원점·축을 물체 기준으로 정렬해둔다
- **폴리곤 수.** 매 프레임 렌더링하므로 스캔 원본(수십만 면)은 그대로 쓰지 말고 decimate
- **iPhone LiDAR 정확도는 cm급**이다. 스프린트 DoD의 `<5mm`와 충돌할 수 있다 — 실측 후 DoD를 재조정할 것

**대안 구현 (지금은 안 씀):** `ammar-n-abbas/FoundationPoseROS2` — NVlabs 원본 + **SAM2 자동 마스크**라
마스크 배선이 통째로 필요없고 기본 구독 토픽이 우리 bag과 그대로 일치한다. 대신 conda 환경 소스 빌드
(pybind11·Eigen·nvdiffrast·mycuda)와 **시작 시 Tkinter/cv2 GUI 필수**(헤드리스 불가), 카메라→베이스
외부파라미터가 남의 값으로 하드코딩(`cam_2_base_transform.py`)이다. 출력은 `/Current_OBJ_position_N`
(`PoseStamped`, base 프레임인데 `frame_id`는 `object_N_frame`으로 잘못 찍힘, TF 브로드캐스트 없음).
**메시 요구는 이쪽도 동일하다.**

**남은 선행 블로커: 대상 물체 CAD 메시.** `isaac_ros_foundationpose`에 model-free 모드는 없다
(`constraints.md:423`). 메시가 없으면 위 명령은 못 돌린다 → NVIDIA quickstart의 머스터드병 자산으로 먼저 배선만 검증하는 것도 방법.
컨테이너에 `isaac_ros_assets`가 있는지도 미확인.

---
⚠️ **「도커 이미지가 날아갔을 때」절은 미검증이다** — `docker load` 복구를 실제로 해본 적은 없다.
다만 이미지명·컨테이너명·`-b` 플래그 동작은 로컬 `isaac_ros-dev/src/isaac_ros_common/scripts/run_dev.sh`를
직접 읽어 확인했다(행번호 명시). `save`/`export` 차이는 docker 문서 기준.

확신도: **검증됨** — §6 지뢰 1~10, §8은 전부 이 인스턴스에서 실제로 재현·해결한 것. §7의 bag 실측(토픽 구성·stamp 일치·TF 프레임)은 `d435i_0803_2143_robot_moving`의 `.db3`를 sqlite3로 직접 디코딩해 확인. **§7 명령어 4종 자체는 아직 통째로 완주하지 않았다(미검증)** — 지뢰 1로 죽은 직후 고친 버전이다.
내가 채워넣은 가정: (1) bag은 인스턴스 안에 이미 올라와 있고 경로만 채우면 된다 (2) `use_lidar:=false` 이므로 LiDAR 관련 파라미터는 전부 무시해도 된다 (3) nvblox는 시각화·검증 목적이며 MoveIt 충돌회피는 여전히 Octomap이 담당한다
확인 요청: §7을 완주하면 `ros/update_esdf`가 10 Hz로 찍히는지(= params-file이 실렸는지) 알려주세요.
