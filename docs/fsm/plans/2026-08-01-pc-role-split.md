<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    개인PC(CPU) vs GPU PC 작업 배치표
-->

# PC 역할 분담 계획 — 개인PC(노트북) vs GPU PC

**작성:** 2026-08-01 | **관련:** [[ws/cobot2/M0609_perception_motion_sprint_plan]]

> 📁 문서 지도: [[ws/cobot2/README]] · GPU 항목의 착수 조건은 [[ws/cobot2/plans/2026-08-03-gpu-dependent-candidates]].
>
> 스프린트 계획을 **어느 PC에서 / 실기 없이 되는지** 기준으로 재분류한다.
> 스프린트 내용 자체는 원본 문서를 보고, 여기서는 배치와 동기화만 다룬다.

---

## 0. 두 PC의 확정된 차이

| | 개인PC (이 노트북, `rokey`/`kimkh`) | GPU PC |
|---|---|---|
| NVIDIA GPU | **없음** (`nvidia-smi` 없음, `/dev/nvidia*` 없음) | 있음 (미확인 — Day0에 확인) |
| docker 런타임 | `runc` (nvidia 런타임 아님) | nvidia-docker 필요 (미확인) |
| realsense2_camera | apt 설치됨 (`4.58.2`) | 동일하다고 사용자 진술 (미확인) |
| 실기(M0609/RG2/D435i/C270) 연결 | 상황에 따라 | 상황에 따라 |

**결론:** GPU/CUDA가 필요한 것만 GPU PC로 보내고, 나머지는 전부 개인PC에서 미리 끝낸다.

---

## 1. 작업 배치표

`실기` = 실물 로봇/카메라 필요, `SIM` = 시뮬레이션으로 대체 가능

| 스프린트 항목 | PC | 실기 | 근거 |
|---|---|---|---|
| **Day1** isaac_ros_common/nvblox release-3.2 소스 정리 | 개인PC ✅완료 | — | 소스 배치는 GPU 무관 |
| **Day1** nvblox 도커 빌드 (`run_dev.sh`) | **GPU PC** | — | nvblox core는 CUDA 필수 |
| **Day1** nvblox 실행 + RViz 재구성 확인 | **GPU PC** | 실기 | D435i depth 입력 필요 |
| **Day1** C270 `v4l2_camera` 노드 등록 | 개인PC | 실기(C270만) | GPU 무관, USB 웹캠만 있으면 됨 |
| **Day2** 캘리브 알고리즘 검증(`corecode/Calibration_Tutorial`) | 개인PC | 오프라인 | 로봇·카메라 없이 합성 데이터로 검증 완료(2026-08-02). easy_handeye2는 채택하지 않음 |
| **Day2** D435i eye-to-hand 캘리브 수행 | 어느 쪽이든 | **실기** | 로봇+카메라 동시 필요 |
| **Day2** C270 eye-in-hand 캘리브 수행 | 어느 쪽이든 | **실기** | 동일 |
| **Day2** `depth_image_proc`→`octomap_server` 토픽 흐름 | 개인PC | SIM(rosbag) | GPU 무관, CPU만 씀 |
| **Day2** `sensors_3d.yaml` 작성 + MoveIt Octomap 반영 | 개인PC | SIM | 아래 2절 참고 — **가장 큰 선물** |
| **Day3** 장애물 회피 모션 | 개인PC(SIM) → 실기 | SIM 후 실기 | Gazebo/virtual에서 먼저 |
| **Day3** OMPL 플래너 튜닝 비교 | **개인PC** | SIM | 플래너 성공률·계획시간은 시뮬로 충분, 실기 시간 낭비 |
| **Day4** FoundationPose 6D pose 추정 | **GPU PC 전용** | **실기** | TensorRT/CUDA 필수. ray-plane 방식을 대체(2026-08-02 변경) — 개인PC에서 불가 |
| **Day4** GraspGenX 그립 생성 | **GPU PC 전용** | SIM | ✅ 저장소 실재 확인 완료. 2026-08-05 실기 파이프라인 관통 → [[ws/cobot2/state]] |
| **Day4** 인식 노드 **인터페이스**(토픽·msg 타입) 확정 | **개인PC** | SIM | GPU 없이 가능. 껍데기만 먼저 만들고 나중에 알맹이 교체 |
| **Day4** TAMP-lite 상태머신 작성 | **개인PC** | SIM | 상태 전이 로직, 로봇 불필요 |
| **Day4** VoiceProcess 퍼블리셔 작성 | **개인PC** | — | 마이크만 있으면 됨 |
| **Day5** 통합 10회 반복 | GPU PC | **실기** | 전체 스택 |

**요약(2026-08-02 개정):** 원래는 "GPU PC가 필요한 건 nvblox 뿐"이었고 nvblox는 시각화로 강등돼서 GPU 의존이 사라지는 듯했다. 그런데 **Day4 인식이 ray-plane → FoundationPose로 바뀌면서 GPU 의존이 오히려 커졌다.** nvblox는 없어도 되지만 FoundationPose는 없으면 Day4가 성립하지 않는다.

- **개인PC(GPU 없음)**: Day1.5 ~ Day3 전부 + 상태머신·인터페이스 껍데기. rosbag만 있으면 실기 없이도 진행 가능 — 여기가 여전히 대부분이다.
- **GPU PC**: Day4 인식·그립 + Day5 통합. **미확인 상태이므로 Day0에 `nvidia-smi` / `docker info | grep -i runtime` 먼저.**
- GPU PC가 없다면: 인식을 색상/AprilTag 기반 CPU 경로로 되돌리고 그립 지점은 물체 1종 하드코딩. 시연 목표(장애물 회피 + pick)는 그래도 달성된다.

---

## 2. 시뮬레이션으로 되는 것 (cobot1_ws 방식)

레포에 이미 세 가지 시뮬 경로가 있다:

| 방식 | launch | 용도 |
|---|---|---|
| **virtual 모드**(DRCF 에뮬레이터) | `dsr_bringup2_rviz.launch.py mode:=virtual` | 로봇 없이 조인트 상태·FK·TF 트리 확인. `install_emulator.sh` 먼저 실행 필요 |
| **Gazebo** | `dsr_bringup2_gazebo.launch.py` | 물리 시뮬 + 장애물 배치 |
| **MuJoCo** | `dsr_bringup2_mujoco.launch.py` | 대안 물리 시뮬 |

> 주의: 세 launch 모두 `model` 기본값이 **`m1013`**이다. M0609를 쓰려면 매번 `model:=m0609`를 명시해야 한다.

### 시뮬로 커버되는 범위
- ✅ TF 트리 (`base_0 → flange`), FK — ray-plane 노드가 필요로 하는 `lookup_transform`이 실제로 동작하는지
- ✅ MoveIt2 플래닝·OMPL 튜닝·Octomap 충돌 반영 (rosbag의 depth를 재생하면 카메라도 불필요)
- ✅ TAMP-lite 상태 전이 전 구간 (GRASP만 mock)
- ❌ hand-eye 캘리브레이션 정확도 — 실기 전용
- ❌ 그립 성공/실패, 힘·접촉 — 실기 전용

### 개인PC에서 실기 없이 Day2~4를 진행하는 순서
1. `install_emulator.sh` 실행 → virtual 모드 확보
2. 실기 접근 가능한 날에 **D435i depth를 rosbag으로 한 번 녹화** — 정확한 토픽 목록·런치 인자·순서는 `md/state.md`의 "출근 후 D435i 세션" 절에 있다 (네임스페이스는 `/camera/camera/...`)
3. 이후 개인PC에서 rosbag 재생 + virtual 로봇으로 Octomap·플래너·상태머신을 전부 개발

> **이 rosbag 한 개가 이 계획의 핵심 자산이다.** 이게 있으면 개인PC 혼자서 Day2 P0의 대부분과 Day3 P1 전체를 실기 없이 끝낼 수 있다. 없으면 모든 게 실기 대기 상태가 된다.

---

## 3. git push/pull로 GPU PC에서 바로 쓰기

### 3-1. 지금 상태의 문제

`isaac_ros-dev/`(136MB)가 `.gitignore`에 없어서 **그대로 커밋된다.**

- NVIDIA 공식 저장소를 통째로 vendoring하는 것 → 레포가 136MB 늘고, 나중에 태그 바꿀 때마다 diff가 수천 파일
- `.gitignore`에 `**/.git/`가 있어서 **중첩 저장소의 git 히스토리는 안 따라간다** → GPU PC에서 pull하면 파일은 있는데 `git describe`도 안 되고 태그도 모르는 "출처 불명 소스 덩어리"가 된다
- nvblox submodule(`nvblox_core`)도 같은 문제

**→ vendoring하지 말고 재현 스크립트로 대체한다.** 어차피 GPU PC에서 `git clone` 한 줄이면 끝난다.

### 3-2. 해야 할 준비 (순서대로)

**(a) `.gitignore`에 추가**
```
# NVIDIA Isaac ROS 원본 저장소 — vendoring 대신 scripts/setup_isaac_ros.sh로 재현
isaac_ros-dev/
isaas_ros-dev/
```
> `isaas_ros-dev/`(오타 경로)도 함께 — 커밋 `11800ba`의 이전 시도 잔재가 다시 잡히는 것 방지

**(b) `scripts/setup_isaac_ros.sh` 작성** — 이번 세션에 개인PC에서 손으로 한 작업을 그대로 스크립트화
```bash
#!/usr/bin/env bash
set -euo pipefail
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
mkdir -p "$ISAAC_ROS_WS/src" && cd "$ISAAC_ROS_WS/src"
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-3.2 --recursive https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > isaac_ros_common/scripts/.isaac_ros_common-config
# realsense-ros는 apt(ros-humble-realsense2-camera)로 설치돼 있으므로 클론하지 않는다
```
GPU PC에서는 `git pull && ./scripts/setup_isaac_ros.sh` 두 줄이면 개인PC와 동일한 상태가 된다.

**(c) 이미 커밋된 게 있으면 인덱스에서 제거**
```bash
git rm -r --cached isaac_ros-dev isaas_ros-dev
```
(작업 디렉토리 파일은 남는다)

**(d) 캘리브레이션 결과는 반드시 커밋 대상에 넣는다**

캘리브 산출물(`corecode/Calibration_Tutorial/T_*.npy`)은 **`.gitignore` 여부를 확인하고 반드시 커밋한다.** 실기에서 힘들게 딴 오프셋이 한 PC에만 남으면 다른 PC에서 전부 다시 해야 한다.
→ `config/handeye/` 디렉토리를 만들어 복사해 커밋하고, launch에서 그 경로를 읽게 한다.

**(e) `.gitignore`의 `docs/` 항목 확인**

현재 `docs/`가 ignore되어 있는데 실제 문서는 `md/`에 있다. `md/`는 정상 커밋되지만, 나중에 `docs/`로 옮기면 조용히 사라진다. 지금 규칙과 실제 위치가 어긋나 있으니 정리 필요.

### 3-3. 커밋해야 할 것 / 하지 말 것

| | 대상 |
|---|---|
| ✅ 커밋 | `src/` 자작 패키지, `md/`, `scripts/setup_isaac_ros.sh`, `sensors_3d.yaml`, `config/handeye/*.calib`, launch 파일 |
| ❌ 커밋 금지 | `isaac_ros-dev/`, `build/ install/ log/`, rosbag(`*.db3`,`*.mcap` — 이미 ignore됨) |
| ⚠️ 별도 전달 | **검증용 rosbag** — git 말고 USB/공유폴더로 옮긴다 (2절의 핵심 자산인데 용량 때문에 git에 못 넣음) |

---

## 4. Day0 (스프린트 시작 전, 30분) — 지금 해야 할 것

- [ ] **GPU PC에서 `nvidia-smi` + `docker info | grep -i runtime` 확인** — 이게 실패하면 스프린트 계획 자체를 재작성해야 한다
- [ ] `.gitignore`에 `isaac_ros-dev/` 추가 후 `git rm -r --cached`
- [ ] `scripts/setup_isaac_ros.sh` 작성·커밋·push
- [ ] GPU PC에서 pull → 스크립트 실행 → `run_dev.sh` 진입까지 확인
- [ ] 개인PC에서 `install_emulator.sh` 실행 → `dsr_bringup2_rviz.launch.py mode:=virtual model:=m0609` 동작 확인
- [ ] 실기 접근 가능할 때 D435i rosbag 녹화 (2절)

---

## 5. 부수적으로 확인된 사실 (스프린트 계획 갱신 필요)

1. **`dsr_moveit_config_m0609/config/sensors_3d.yaml`이 이미 존재한다** — 내용은 `sensors: []`로 비어 있음. 즉 Day2 P0는 "파일 신규 작성"이 아니라 "빈 파일에 `PointCloudOctomapUpdater` 항목 채우기"다. 리스크 표의 "sensors_3d.yaml 설정 경험 부재로 Day2 지연"은 실제보다 과대평가돼 있다.
2. **`realsense-ros` 클론 불필요** — apt `ros-humble-realsense2-camera 4.58.2` 설치 확인. 스프린트 문서 51행의 `git clone realsense-ros`는 삭제해야 한다.
3. **doosan-robot2 launch의 `model` 기본값이 `m1013`** — M0609 쓸 때마다 `model:=m0609` 명시 필요. 빠뜨리면 엉뚱한 로봇으로 뜬다.
4. **`md/state.md`가 낡았다** — "SSH 키 GitHub 등록 대기 중, push 불가"라고 적혀 있으나 실제 remote는 HTTPS `personal`이고 사용자는 VS Code로 push 중이다. 갱신 필요.
