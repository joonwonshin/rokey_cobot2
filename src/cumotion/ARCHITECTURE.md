<!-- meta
updated: 2026-08-07
status:  live (실행 중인 컨테이너를 직접 조사해 작성 — 추정 아님)
owns:    도커 환경의 구조 · 호스트↔컨테이너 경계 · 왜 그렇게 나뉘어 있는가
scope:   GPU PC(hostname rokey)의 Isaac ROS 컨테이너. 실행 명령은 config/testcommand.md,
         이 패키지의 코드 설계는 README.md 가 각각 단일 출처다. 여기엔 **환경**만 둔다.
-->

# ARCHITECTURE — 이 프로젝트의 도커 환경

**CPU PC 에서 읽는 사람을 위한 문서다.** GPU PC 에 접속하지 않고도 "무엇이 어디서 도는가"를
이해할 수 있게 쓴다. 아래 값은 전부 **2026-08-07 에 실행 중이던 컨테이너를 직접 조사한 것**이고,
추정한 부분은 그렇다고 표시했다.

---

## 1. 한 장 요약 — 왜 컨테이너가 필요한가

이 프로젝트는 **두 개의 ROS 2 Humble 환경이 하나의 도메인에서 통신**하는 구조다.

```
┌───────────────────────── 같은 물리 PC (hostname: rokey) ─────────────────────────┐
│                                                                                  │
│  호스트 (Ubuntu 22.04 + ROS 2 Humble)      컨테이너 (Isaac ROS, ROS 2 Humble)     │
│  ────────────────────────────────────      ──────────────────────────────────    │
│  · 로봇 드라이버 (dsr_hw_interface2)         · cuRobo / cuMotion  (CUDA)          │
│  · RealSense 드라이버                        · nvblox            (CUDA)          │
│  · ros2_control 컨트롤러                     · MoveIt move_group                  │
│  · cumotion 패키지 (T8, 이 패키지)                                                │
│                                                                                  │
│         └──────────── ROS 2 DDS (ROS_DOMAIN_ID=93, --network host) ────────┘      │
└──────────────────────────────────────────────────────────────────────────────────┘
```

**나누는 기준은 GPU 가 아니라 "설치 지옥"이다.**

- 컨테이너 쪽: CUDA 12.6 + PyTorch + warp + cuRobo + nvblox. 이걸 호스트에 직접 깔면
  시스템 파이썬·numpy·cv2 가 전부 꼬인다. 실제로 numpy 버전 하나 때문에 오늘 두 번 막혔다(6절).
- 호스트 쪽: 로봇 하드웨어(TCP/USB), 그리고 **GPU 를 안 쓰는 것들**.

> 🔴 **`cumotion` 패키지(T8)는 GPU 를 전혀 안 쓴다.** 순수 ROS 2 액션 클라이언트라
> CUDA·cuRobo·nvblox 를 하나도 import 하지 않는다(전수 확인). 그래서 **호스트에 둔다** —
> 컨테이너에 두면 컨테이너를 새로 만들 때마다 재빌드해야 하고, 비상정지용 `dsr_msgs2` 도
> 컨테이너엔 없다.

---

## 2. 물리 구성 (2026-08-07 실측)

| | 값 |
|---|---|
| 호스트 OS | Ubuntu 22.04, ROS 2 **Humble** (`/opt/ros/humble`) |
| GPU | **NVIDIA GeForce RTX 4060 Laptop, 8188 MiB**, 드라이버 595.84 |
| 로봇 | 두산 M0609, 네임스페이스 `dsr01`, IP `192.168.1.100` |
| 그리퍼 | OnRobot RG2 (Modbus TCP) |
| 카메라 | RealSense D435i (USB) |
| 계정 | 팀 공유 랩탑. 계정마다 자기 컨테이너를 띄운다 (`cumotion-joonwon` 등) |

**GPU 사용량 실측(full-up)**: cuMotion 1,508 MiB + segmenter 660 MiB + nvblox 334 MiB
≈ **2.5 GB / 8 GB**. 세 계정이 동시에 써도 여유가 있다.

---

## 3. 컨테이너의 정체

| | 값 |
|---|---|
| 이미지 | `isaac_ros_dev-x86_64` |
| 컨테이너 이름 | `isaac_ros_dev-x86_64-container` |
| 이미지 키 | `ros2_humble` (기본값 — `.isaac_ros_common-config` 가 없어서) |
| 내부 ROS | **Humble** (호스트와 동일 — 이게 DDS 통신의 전제다) |
| CUDA | nvcc **12.6**, PyTorch **2.13.0+cu130**, `torch.cuda.is_available() == True` |
| 빌드된 패키지 | `/workspaces/isaac_ros-dev/install` 에 **44개** (`curobo_core`, `isaac_ros_cumotion*`, `nvblox*` 등) |

### `run_dev.sh` 가 실제로 실행하는 것

```bash
docker run -it --rm \
    --privileged  --network host  --ipc=host  --runtime nvidia \
    -v <isaac_ros-dev>:/workspaces/isaac_ros-dev \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
    -e DISPLAY -e ROS_DOMAIN_ID -e USER \
    -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
    -e HOST_USER_UID=$(id -u)  -e HOST_USER_GID=$(id -g) \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --name isaac_ros_dev-x86_64-container \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /workspaces/isaac_ros-dev \
    isaac_ros_dev-x86_64  /bin/bash
```

우리는 여기에 `-a` 로 마운트를 하나 더 붙인다:
```bash
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

각 플래그의 의미:

| 플래그 | 왜 |
|---|---|
| `--network host` | 호스트와 **같은 네트워크 스택**. DDS 디스커버리가 그냥 된다 |
| `--ipc=host` | 공유메모리 네임스페이스 공유 (DDS SHM 전송용) |
| `--runtime nvidia` | GPU 패스스루 |
| `--privileged` | USB(카메라)·장치 접근 |
| **`--rm`** | 🔴 **PID 1(`/bin/bash`)이 끝나면 컨테이너가 삭제된다.** 11절 참고 |

---

## 4. 🔴 파일시스템 — 같은 디렉토리를 두 경로로 본다

이 프로젝트에서 **가장 많은 사고를 낸 지점**이다.

```
호스트 경로                                컨테이너 경로
─────────────────────────────────────────  ────────────────────────────
/home/kimkh/cobot2_ws/isaac_ros-dev   →   /workspaces/isaac_ros-dev
/home/kimkh/cobot2_ws                 →   /workspaces/cobot2_ws          ← -a 로 추가
/tmp/.X11-unix                        →   /tmp/.X11-unix                 (RViz)
/home/kimkh/.Xauthority               →   /home/admin/.Xauthority        (RViz)
/etc/localtime                        →   /etc/localtime  (ro)           (시계 일치)
$SSH_AUTH_SOCK                        →   /ssh-agent
```

**같은 파일인데 절대경로가 다르다.** 그런데 colcon 은 생성하는 셸 스크립트에
**절대경로를 구워 넣는다.** 그래서:

> 🔴 **하나의 `install/` 은 호스트와 컨테이너 중 한쪽에만 맞을 수 있다.**

실제로 물린 사례(2026-08-07): 컨테이너에서 `dsr_msgs2` 를 기본 `install/` 에 빌드해 버려서
`install/dsr_msgs2/share/dsr_msgs2/package.sh` 에 `/workspaces/cobot2_ws/...` 가 박혔고,
**호스트에서 `dsr_msgs2` 를 의존하는 패키지의 colcon 빌드가 전부 깨졌다.**
(런타임은 멀쩡했다 — 경고만 찍고 넘어간다. 그래서 몇 달도 모를 수 있다)

### 그래서 빌드 디렉토리를 분리한다

| | 호스트 | 컨테이너 |
|---|---|---|
| build | `build/` | `build_container/` |
| install | `install/` | `install_container/` |

```bash
# 컨테이너에서 빌드할 때
colcon build --symlink-install \
  --build-base build_container --install-base install_container \
  --packages-select <pkg>
```

⚠️ 컨테이너 안에서 `--build-base`/`--install-base` 를 **빠뜨리면 그 순간 오염된다.**

---

## 5. 노드 배치 — 무엇이 어디서 도는가

| | 노드 | 위치 | GPU |
|---|---|---|---|
| T1 | `camera.launch.py` (RealSense + 캘리브 TF) | 호스트 | — |
| T2 | `bringup.launch.py` (로봇 + ros2_control + RG2) | 호스트 | — |
| T3 | 컨테이너 기동 (`run_dev.sh`) | — | — |
| T4 | `robot_segmenter_node` (로봇 몸을 depth 에서 지움) | 컨테이너 | ✅ 660 MiB |
| T5 | `nvblox_node` (ESDF 지도) | 컨테이너 | ✅ 334 MiB |
| T6 | `cumotion_planner_node` (cuRobo 플래너) | 컨테이너 | ✅ 1,508 MiB |
| T7 | `move_group` + RViz | 컨테이너 | — |
| **T8** | **`cumotion` (이 패키지)** | **호스트** | — |

### 경계를 넘는 통신 3가지 (전부 실기 검증됨)

```
① 호스트 → 컨테이너   /camera/... (이미지)      T1 → T4
                      /joint_states             T2 → T4, T6
② 컨테이너 → 호스트   controller_manager 서비스  T7 이 컨트롤러 spawn
                      FollowJointTrajectory      T7/T8 → T2 의 JTC
③ 컨테이너 ↔ 호스트   /move_action (액션)        T8 → T7
```

②가 **서비스 호출**이라는 게 중요하다 — 6절의 RMW 주의사항이 여기서 나온다.

---

## 6. 🔴 사용자 — `admin` 과 `root` 는 다른 파이썬을 본다

컨테이너의 entrypoint(`workspace-entrypoint.sh`)가 **호스트 uid/gid 로 `admin` 사용자를
만든다**:

```bash
useradd --uid ${HOST_USER_UID} --gid ${HOST_USER_GID} -m admin
echo "admin ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/admin
```

실측: `uid=1002(admin) gid=1002(admin) groups=sudo,video,plugdev`
호스트 `kimkh` 의 uid 도 1002 → **마운트된 파일의 소유권이 그대로 맞는다.** 이게 목적이다.

`run_dev.sh` 는 붙을 때 **반드시 `-u admin`** 을 쓴다:
```bash
docker exec -i -t -u admin --workdir $ISAAC_ROS_WS $CONTAINER_NAME /bin/bash
```

### 🔴 여기서 오늘 물렸다

```
docker exec -it <container> bash          →  root  로 들어간다 (기본값)
run_dev.sh / docker exec -it -u admin     →  admin 으로 들어간다 (정상)
```

`container_setup.sh` 는 pip 설치를 하는데, **pip 은 사용자별로 설치된다**:

```
admin  →  numpy 1.26.4   /home/admin/.local/lib/python3.10/site-packages   ✅
root   →  numpy 2.2.6    /usr/local/lib/python3.10/dist-packages           ❌ 이미지 기본값
```

증상: `container_setup.sh` 를 분명히 돌렸는데 `root` 로 노드를 띄우면
`import cv2 → ImportError: numpy.core.multiarray failed to import` 로 **뜨자마자 죽는다.**
`robot_segmenter.py:19` 가 모듈 최상단에서 cv2 를 import 하기 때문이다.

> **결론: 컨테이너에는 항상 `admin` 으로 들어간다.**

---

## 7. 컨테이너 초기 설정 — `scripts/container_setup.sh`

**컨테이너를 새로 만들 때마다 한 번씩** 돌려야 한다. 이미지 밖 변경(pip)은 `--rm` 으로
매번 날아가기 때문이다.

| 항목 | 이미지 기본 | 필요한 값 | 안 하면 |
|---|---|---|---|
| warp-lang | 1.16.0 | **1.5.0** | T6 이 `module 'warp' has no attribute 'torch'` 로 죽는다 |
| numpy | 2.2.6 | **1.26.4** | T4 가 `import cv2 → _ARRAY_API not found` 로 죽는다 |

- warp: cuRobo 커밋 36ea382(2024-11)는 warp 1.2.1 까지만 안다. 1.16.0 엔 `warp.torch`
  모듈이 아예 없어서 `read_esdf_world:=True` 경로가 임포트 단계에서 죽는다.
- numpy: 이미지의 2.2.6 이 apt `cv2`(numpy1 빌드)를 깬다. 대가로 `cupy-cuda12x` 가 깨지지만
  쓰는 건 `isaac_ros_cumotion_object_attachment` 뿐이라 무해하다.

**바인드 마운트 안의 것(curobo 소스 패치, colcon 산출물)은 살아남는다** — 다시 할 필요 없다.

> ⚠️ 스크립트의 `🔴 패치가 없다` 줄은 git `dubious ownership` 때문에 나는 **오탐**이다.
> `git config --global --add safe.directory <curobo>` 를 한 번 해주면 정상 보고된다.

---

## 8. 네트워크와 DDS

```
--network host  +  ROS_DOMAIN_ID=93 (호스트에서 -e 로 전달)
```

- `run_dev.sh` 는 **호스트 셸의 `ROS_DOMAIN_ID` 를 그대로 넘긴다.** 그래서 컨테이너를 띄우기
  **전에** `export ROS_DOMAIN_ID=93` 을 해야 한다. 안 하면 컨테이너가 도메인 0 이 되어
  로봇을 못 본다.
- 호스트 터미널마다도 `export ROS_DOMAIN_ID=93` 이 필요하다. 빠뜨리면
  `/move_action 액션 서버 없음` 같은 **엉뚱한 증상**으로 나타난다(오늘 실제로 겪음).

### 🔴 `RMW_IMPLEMENTATION` 을 건드리지 말 것

기본값 **Fast DDS**(`rmw_fastrtps_cpp`)를 그대로 쓴다. cyclonedds 로 바꾸면
**컨테이너↔호스트 서비스가 안 붙는다**(토픽만 됨). 그러면 T7 의 컨트롤러 spawner 가
호스트 `controller_manager` 서비스를 못 불러 멈춘다.

> 참고: 워크스페이스 루트에 `fastdds_udp_only.xml` 이 있다. 컨테이너↔호스트 SHM 전송이
> 안 될 때 UDP 전용으로 강제하는 프로파일이다. **2026-08-07 현재는 필요 없었다** —
> 설정 없이도 이미지·서비스가 모두 흘렀다. 데이터가 안 올 때만 꺼내 쓴다
> (호스트·컨테이너 **양쪽 다** export 해야 효과가 있다).

---

## 9. GPU

```
--runtime nvidia  +  NVIDIA_VISIBLE_DEVICES=all  +  NVIDIA_DRIVER_CAPABILITIES=all
```

컨테이너 안에서 `torch.cuda.is_available() == True` 로 확인됨.

**공유 랩탑이라 GPU 반납 절차가 중요하다:**
```bash
nvidia-smi --query-compute-apps=pid,used_memory --format=csv   # 누가 쥐고 있나
ps -o pid,user,cmd -p <pid>                                    # 🔴 남의 것이면 절대 kill 금지
nvidia-smi --query-gpu=memory.used --format=csv,noheader        # ~15 MiB 면 반납 완료
```

🔴 **`pkill -f` 금지.** `docker exec bash -c` 에서 자기 명령줄에도 매칭돼 자기 셸을 먼저
죽인다. 출력은 깨끗해서 "정리됨"으로 오독하게 된다. PID 로 죽인다.

---

## 10. X11 / RViz

```
-v /tmp/.X11-unix:/tmp/.X11-unix
-v $HOME/.Xauthority:/home/admin/.Xauthority:rw
-e DISPLAY
```

컨테이너의 RViz 가 호스트 X 서버에 그린다. `DISPLAY` 는 **컨테이너 생성 시점의 호스트 값**이
고정된다 — 실측에서 컨테이너는 `DISPLAY=:2`, 호스트는 `:1` 이었다(다른 로그인 세션에서
띄운 컨테이너). RViz 가 안 뜨면 이 불일치를 의심한다.

---

## 11. 🔴 수명주기 — 컨테이너는 쉽게 사라진다

```
docker run -it --rm ... /bin/bash
              ↑ PID 1 인 이 bash 가 끝나면 컨테이너가 통째로 삭제된다
```

| 상황 | 결과 |
|---|---|
| `run_dev.sh` 를 띄운 터미널을 닫는다 | **컨테이너 삭제.** `docker ps -a` 에도 안 남는다 |
| `run_dev.sh` 를 다시 실행 (이미 떠 있음) | 새로 만들지 않고 `docker exec -u admin` 으로 **attach** |
| `run_dev.sh` 를 다시 실행 (없음) | **새 컨테이너 생성** → `container_setup.sh` 다시 필요 |
| `docker exec -it <c> bash` | attach 되지만 **root** 다 (6절 함정) |

**권장**: `run_dev.sh` 를 띄운 터미널은 그대로 두고, 추가 셸은
`docker exec -it -u admin isaac_ros_dev-x86_64-container bash` 로 붙는다.

---

## 12. 🔴 컨테이너 쪽 소스 트리 — 어떤 파일에 무엇이 있나

**CPU PC 에서 코드를 읽거나 짤 때 필요한 지도다.** `isaac_ros-dev/` 는 호스트에 있는
실제 디렉토리이므로 **CPU PC 로 복사해 오면 그대로 읽을 수 있다**(빌드·실행만 못 한다).

### 12-1. 최상위

```
isaac_ros-dev/
├── src/          ← 소스 (읽을 것은 전부 여기)
├── build/        ← colcon 중간 산출물. ⚠️ 파이썬은 여기서 실행된다 (아래 주의)
├── install/      ← 🔴 심볼릭 링크 덩어리. 호스트에서 안 열린다 (12-6)
├── m0609/        ← 우리 로봇 기술 파일 (URDF/XRDF)
├── log/
└── COLCON_IGNORE ← 이 디렉토리를 상위 워크스페이스 빌드에서 제외
```

⚠️ **트레이스백에 `build/` 경로가 찍힌다.** `--symlink-install` 이라 실행되는 파이썬이
`build/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py` 로 나온다. 이건 `src/` 의
같은 파일을 가리키는 링크다 — **놀라지 말고 `src/` 를 고치면 된다.**

### 12-2. `src/` — 업스트림 repo 6개

| 디렉토리 | 무엇 | 우리가 쓰나 |
|---|---|---|
| `isaac_ros_cumotion/` | cuRobo 기반 모션 플래너 + 로봇 세그멘터 | ✅ **핵심** |
| `isaac_ros_nvblox/` | GPU 복셀 지도(TSDF/ESDF) | ✅ **핵심** |
| `isaac_ros_common/` | 컨테이너 빌드/기동 스크립트(`run_dev.sh`) | ✅ 환경용 |
| `isaac_ros_nitros/` | 타입 어댑테이션(GPU 메모리 zero-copy) | 간접 (nvblox 가 씀) |
| `isaac_ros_pose_estimation/` | FoundationPose 등 | ❌ 이 작업엔 미사용 |
| `GraspGenX/` | 파지 생성 | ❌ 이 작업엔 미사용 |

> 잡파일: `src/yoloseg.py`(691 B), `src/r343q31f43/`(빈 디렉토리, **소유자 `joonwon`**).
> 공유 랩탑이라 남의 흔적이 섞인다 — 지우지 말 것.

### 12-3. `isaac_ros_cumotion/` — 하위 패키지 11개

```
isaac_ros_cumotion/
├── isaac_ros_cumotion/              ✅ 파이썬 노드 본체 (T4·T6)
├── isaac_ros_cumotion_moveit/       ✅ MoveIt 플래닝 플러그인 (C++) — T7 이 로드
├── isaac_ros_cumotion_robot_description/  ✅ XRDF 저장소 (robot:= 가 여기서 찾는다)
├── curobo_core/                     ✅ cuRobo 본체 (서브모듈) — 로컬 패치 2개 있음
├── isaac_ros_cumotion_python_utils/    보조
├── isaac_ros_cumotion_interfaces/      메시지/액션 정의
├── isaac_ros_cumotion_object_attachment/  물체 부착(그리퍼가 쥔 물체) — 미사용
├── isaac_ros_esdf_visualizer/          ESDF 시각화 — 미사용
├── isaac_ros_moveit_goal_setter/       목표 설정 예제 — 미사용
├── isaac_ros_goal_setter_interfaces/    위의 인터페이스
└── isaac_ros_cumotion_examples/         예제
```

### 12-4. 🔴 핵심 소스 파일과 봐야 할 지점

**`isaac_ros_cumotion/isaac_ros_cumotion/cumotion_planner.py` (891줄)** — T6 의 전부

| 줄 | 내용 |
|---|---|
| 62~95 | `declare_parameter` 전체 목록 — **파라미터 이름을 여기서 확인한다** |
| 105 | `/curobo/voxels` publisher 생성 (**노드 시작 시** — 계획 전에도 토픽 타입은 잡힌다) |
| 623 | 🔴 복셀은 **구독자가 있을 때만** 발행 |
| 637 | `planner_busy` — 동시 요청을 abort (`FAILURE=99999`) |
| 649~651 | `time_dilation_factor = min(1.0, min(vel_scale, acc_scale))` — **acc 는 별도 스케일이 아니다** |
| 660 | `goal_handle.succeed()` 를 **계획 전에** 부른다 |
| 675 | 🔴 `CuJointState.from_position(...)` — **시작 velocity 를 버린다** (README 4절의 근거) |
| 686~698 | `is_diff` 분기 — start_state 를 안 주면 **라이브 `/joint_states` 의 velocity 를 쓴다** |
| 713~731 | 관절 목표 → FK → `goal_pose`. **관절 목표도 결국 pose 로 푼다** (그래서 IK_FAIL 이 난다) |
| 754~774 | pose 목표. `link_name` 이 XRDF `tool_frames` 와 다르면 `INVALID_LINK_NAME` |
| 790~806 | cuRobo status → `MoveItErrorCodes` 매핑. **매핑 안 된 status 는 val=0 으로 나간다** |
| 833~873 | `publish_voxels()` — Marker 로 발행 |

**`isaac_ros_cumotion/isaac_ros_cumotion/robot_segmenter.py` (414줄)** — T4

| 줄 | 내용 |
|---|---|
| 19 | `import cv2` — 🔴 모듈 최상단. numpy 가 어긋나면 **여기서 즉사**(6절) |
| 54~63 | 파라미터(`time_sync_slop`, `joint_states_topic`, publish 토픽들) |
| 146~152 | 🔴 `ApproximateTimeSynchronizer(depth + /joint_states, slop=0.1)` — **둘의 타임스탬프가 0.1초 안에 안 맞으면 콜백이 아예 안 돈다** |
| 241~248 | `is_subscribed()` — 🔴 **구독자가 없으면 아무것도 발행 안 한다** |
| 285~294 | `on_timer` 가드 (intrinsics·camera_headers·timestamp) |

**`isaac_ros_cumotion_moveit/src/` (C++ 4개)** — T7 안에서 도는 플러그인

| 파일 | 역할 |
|---|---|
| `cumotion_planner_manager.cpp` | 플러그인 등록 (`isaac_ros_cumotion_moveit/CumotionPlanner`) |
| `cumotion_planning_context.cpp` | `solve()` → interface 로 위임 |
| `cumotion_interface.cpp` | 🔴 74~78: **실패 시 플래너의 진짜 error_code 를 버리고 `PLANNING_FAILED(-1)` 로 고정** — cuMotion 의 `-1` 이 원인을 안 알려주는 이유 |
| `cumotion_move_group_client.cpp` | 88~94: `/cumotion/move_group` 액션으로 goal 전달 |

`isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml` — 파이프라인 정의
```yaml
planning_plugin: isaac_ros_cumotion_moveit/CumotionPlanner
request_adapters: FixWorkspaceBounds FixStartStateBounds
                  FixStartStateCollision FixStartStatePathConstraints
```
🔴 여기엔 **재시간화(TOTG) 어댑터가 없다** → cuMotion 이 낸 타이밍이 그대로 살아남는다.

**`curobo_core/curobo/`** — cuRobo 본체

```
src/curobo/
├── wrap/reacher/motion_gen.py    ← plan_single() 진입점 (645행 부근에서 충돌체커 생성)
├── geom/sdf/world_voxel.py       ← ESDF 충돌월드
├── geom/sdf/world_mesh.py        ← 🔧 로컬 패치 ②
├── curobolib/cpp/helper_math.h   ← 🔧 로컬 패치 ①
├── cuda_robot_model/  graph/  opt/  rollout/  types/  util/
```

로컬 패치 2개 (`git status` 로 확인됨, 되돌리면 안 된다):
- `helper_math.h` — C++20 `std::lerp` 충돌 회피
- `world_mesh.py` — `warp.torch` 임포트 무해화

### 12-5. `m0609/` — 우리 로봇 기술 파일

| 파일 | 무엇 | 누가 읽나 |
|---|---|---|
| `m0609_kinematics.urdf` (12 KB) | 팔만 (RG2 없음) | T4·T6 의 `urdf_path:=` |
| `m0609_with_rg2.urdf` (20 KB) | 팔 + RG2 | 참고용 |
| `m0609_rg2.xrdf` (13 KB) | 🔴 cuRobo 용 — 충돌구, 자기충돌 면제, `tool_frames`, 관절한계 | T4·T6 의 `robot:=` |
| `gate_c.py` (3 KB) | 보조 스크립트 | — |

### 12-6. 🔴 같은 파일의 사본이 여러 곳에 있다

`m0609_rg2.xrdf` 는 **7곳**에 있다. 실제로 로드되는 것은 하나뿐이다.

```
isaac_ros-dev/m0609/m0609_rg2.xrdf                          md5 dd902877  ← 작업본(사람이 고치는 곳)
.../isaac_ros_cumotion_robot_description/xrdf/m0609_rg2.xrdf md5 dd902877  ← 🔴 robot:= 가 찾는 곳
src/cobot_rg2/rg2/m0609_rg2_moveit/config/m0609_rg2.xrdf     md5 dd902877  ← ws 쪽 사본
isaac_ros-dev/{build,install}/…                                            ← 빌드 산출물(링크)
cobot2_ws/{install,install_container}/…                                    ← 빌드 산출물
```

🔴 **`robot:=m0609_rg2.xrdf` 는 경로가 아니라 파일명이다.**
`isaac_ros_cumotion_robot_description` 패키지의 share 디렉토리에서 찾는다.
→ **`m0609/` 만 고치면 반영되지 않는다.** 세 사본을 같이 맞춰야 한다(동기화는 수동 `cp` 뿐).

현재는 세 개가 전부 같다(`dd902877`, 2026-08-06). **어긋나면 에러 없이 조용히 다르게 동작한다.**

### 12-7. 🔴 `isaac_ros-dev/install/` 은 호스트에서 안 열린다

```bash
$ ls -la isaac_ros-dev/install/.../xrdf/m0609_rg2.xrdf
… -> /workspaces/isaac_ros-dev/build/isaac_ros_cumotion_robot_description/xrdf/m0609_rg2.xrdf
$ readlink -f <위 경로>
(빈 출력 — 호스트엔 /workspaces 가 없다)
```

**컨테이너에서 `--symlink-install` 로 빌드했기 때문에 링크가 컨테이너 절대경로로 박혀 있다.**
컨테이너 안에서는 정상 동작하고, 호스트/CPU PC 에서는 **깨진 링크**로 보인다.

> **코드를 읽을 땐 `install/` 이 아니라 항상 `src/` 를 본다.** 4절의 "같은 디렉토리를 두
> 경로로 본다" 문제가 파일 단위로 드러난 사례다.

### 12-8. nvblox 쪽 구조

```
isaac_ros_nvblox/
├── nvblox_ros/src/lib/
│   ├── nvblox_node.cpp            ← 노드 본체 (틱·통합·발행 스케줄)
│   ├── node_params.cpp            ← 🔴 파라미터 이름의 단일 출처. static_mapper.* 접두어가 여기서 온다
│   ├── mapper_initialization.cpp  ← 매퍼 설정 적용 (여기서 yaml 값이 덮인다 → -p 가 안 먹는 이유로 추정)
│   ├── conversions/  layer_publishing.cpp  transformer.cpp  camera_cache.cpp
├── nvblox_msgs/                   ← 🔴 호스트엔 없다. hz/echo 하려면 컨테이너에서
├── nvblox_rviz_plugin/            ← NvbloxMesh / NvbloxVoxelBlockLayer 디스플레이
└── nvblox_examples/nvblox_examples_bringup/config/nvblox/
    ├── nvblox_base.yaml           ← 🔴 우리가 얹는 기준 설정
    └── specializations/nvblox_realsense.yaml   ← ⚠️ 얹지 않는다 (map_clearing_frame_id 가 우리 TF 와 안 맞음)
```

우리 쪽 설정은 `cobot2_ws/config/nvblox_realtime.yaml` 이 `nvblox_base.yaml` **위에** 얹힌다
(뒤에 오는 `--params-file` 이 이긴다).

---

## 13. CPU PC 에서 이 문서를 읽을 때

**재현 가능한 것** — 코드 읽기·리뷰, 로그 분석, 파라미터 정합성 확인,
`src/cumotion` 의 순수 로직(궤적 보간·재정렬·`_same_path`) 단위 검증.

**재현 불가능한 것** — cuRobo/nvblox 실행(CUDA 필요), 계획 시간·복셀 수 측정,
실기 모션. **이 문서의 수치는 전부 GPU PC 실측이라, CPU PC 에서 다시 재려 하면 안 된다.**

CPU PC 에서 `src/cumotion` 만 빌드하려면 GPU 가 필요 없다(1절). 필요한 건 메시지 패키지뿐:
```bash
ros2 pkg list | grep -E "^(moveit_msgs|control_msgs|visualization_msgs|dsr_msgs2)$"
```

---

## 13. 함정 요약 (전부 오늘 실제로 당한 것)

| 함정 | 증상 | 대처 |
|---|---|---|
| `docker exec` 가 root | `container_setup.sh` 를 돌렸는데도 cv2/warp 에러 | **`-u admin`** (6절) |
| `container_setup.sh` 누락 | T4/T6 이 뜨자마자 죽는다 | 컨테이너 새로 만들 때마다 실행 (7절) |
| 터미널 닫아서 컨테이너 삭제 | `No such container` | `--rm` 특성. 11절 |
| `install/` 오염 | 호스트 colcon 빌드가 AssertionError | `build_container/`·`install_container/` 분리 (4절) |
| `ROS_DOMAIN_ID` 누락 | 노드가 하나도 안 보인다 | 호스트 터미널마다 export, 컨테이너 띄우기 **전에도** (8절) |
| `RMW_IMPLEMENTATION` 변경 | 토픽은 되는데 **서비스만** 안 붙는다 | 기본값(Fast DDS) 유지 (8절) |
| `pkill -f` | 자기 셸을 먼저 죽인다 | PID 로 kill (9절) |

---

## 참고

- `config/testcommand.md` — 실행 명령(T1~T7)의 단일 출처
- `src/cumotion/README.md` — 이 패키지 코드 설계와 실기 실측 결과
- `scripts/container_setup.sh` — 컨테이너 초기 설정
- `isaac_ros-dev/src/isaac_ros_common/scripts/run_dev.sh` — 컨테이너 기동 스크립트(업스트림)
