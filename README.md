# M0609 VLA Picking System

자연어로 지시하면 협동로봇이 물체를 집어 옮기는 시스템. **VLM(GPT-5-mini)** 이
*무엇을 · 몇 개를 · 어디에* 를 정하고, **deterministic FSM** 이 *어떻게 움직이고
언제 멈추고 무엇을 놓지 않을지* 를 소유한다.

> 핵심 설계: 판단(LLM)과 안전(FSM)을 **한 프로세스에 섞지 않는다.** 두 계층은
> JSON 3채널로만 붙고, 모션 취소·그리퍼 개폐·물체 보유·충돌 씬은 전부 FSM 쪽에 있다.
> LLM 이 느려지거나 틀려도 팔의 안전 동작은 영향을 받지 않는다.

```
사람 ──▶ vla_gui ──▶ agent_node                 판단 계층
                     ├ Tier 1 규칙 (LLM 없이 처리 가능한 것)
                     └ Tier 2 대화 (GPT-5-mini + 카메라 사진)
                          │ RobotAction
                          ▼
                   vla_pick_bridge_node
                          │ /vla/pick_command  (JSON)
        ══════════════════╪══════════════════   ← 워크스페이스 경계
                          ▼
                   vla_command_node ──▶ task_manager (pick_fsm)   안전 계층
                                            │  PERCEIVE → PLAN → APPROACH
                                            │  → DESCEND → CLOSE → LIFT
                                            │  → WAIT_PLACE_TARGET → PLACE
                                            ▼
                                   MoveIt / cuMotion ──▶ M0609 + RG2
```

---

## 무엇을 할 수 있나

| 지시 | 동작 |
|---|---|
| "사과 바구니에 담아줘" | 인식 → 파지 계획 → 집기 → 바구니에 놓기 |
| "이거 집어줘" **(손가락으로 가리키며)** | 카메라 사진에서 손가락이 가리키는 물체를 골라 집는다 |
| "사과 집어줘" (목적지 없이) | 집어서 **든 채로 대기** → "테이블에 놔" / "그냥 거기 놔" |
| "보이는 과일 다 담아줘" | 여러 개를 **하나씩 순차로**. 중간에 "나머지는 테이블로" 로 수정 가능 |
| "멈춰" | LLM 을 거치지 않고 **즉시** 정지. "계속해" 한 마디로 이어서 한다 |
| "컵은 앞으로 담지 마" | 규칙으로 기억. 다음부터 "다 담아줘" 에서도 컵을 뺀다 |

---

## 구성

```
.
├── src/
│   ├── pick_fsm/              상태머신 — 로봇 동작의 단일 주인
│   ├── pick_fsm_msgs/         ComputeGrasp / AcquireTarget 인터페이스
│   ├── voice_processing/      경계 수신부. VLA JSON → FSM 서비스/토픽
│   ├── graspgenx_perception/  인식 + 파지자세 계산 (YOLO-seg + GraspGen)
│   ├── cumotion/              GPU 플래닝 파이프라인 (선택)
│   ├── object_detection/      YOLO 가중치 share 경로
│   ├── vla_system/            판단 계층 — GUI · 에이전트 · 규칙 · 브리지
│   └── vla_interfaces/        판단 계층 내부 메시지 (경계를 넘지 않는다)
├── config/                    objects.yaml · cumotion · nvblox 설정
├── docker/                    GraspGenX 컨테이너 재구성
├── docs/                      계약 · 실기 제약 · 실행 절차 · 인계 문서
├── scripts/
│   ├── build.sh               빌드 (fsm / vla 분리)
│   ├── fetch_externals.sh     외부 저장소 받아오기
│   ├── fsm/                   로봇 쪽 스크립트
│   └── vla/                   판단 쪽 스크립트
├── requirements-vla.txt       판단 계층 파이썬 의존성 (실측 393개)
└── .env.example               API 키 자리
```

### 저장소에 없는 것과 그 이유

| 없는 것 | 크기 | 어떻게 |
|---|---|---|
| `isaac_ros-dev/` (cuMotion · nvblox · GraspGenX) | 20G | `scripts/fetch_externals.sh` |
| Doosan / OnRobot 드라이버 | 273M | 〃 |
| `.venv/` | 6.6G | `requirements-vla.txt` |
| `build/ install/ log/` | 1.7G | **절대경로가 박혀 옮기면 어차피 안 돈다** |
| `data/graspgenx_scene/` | 2.3G | 실행 중 캡처되는 **출력**. 입력이 아니다 |
| 도커 이미지 | 7G | `docker/Dockerfile.graspx` (빌드 검증 완료) |
| `.env` | — | 🔴 **API 키.** `.env.example` 참고 |

---

## 요구 환경

- **Ubuntu 22.04 · ROS 2 Humble · Python 3.10**
- **NVIDIA GPU + CUDA** — GraspGenX·cuMotion 은 CPU 로 안 돈다 (개발기: RTX 4060 Laptop)
- Docker + `nvidia-container-toolkit` (`--gpus all` 이 되어야 한다)
- 하드웨어
  - Doosan **M0609** (네임스페이스 `dsr01`, IP `192.168.1.100`)
  - OnRobot **RG2** 그리퍼
  - Intel RealSense **D435i** 1대 — 🔴 **고정(eye-to-hand)**. 작업대 옆에 세우고
    팔에 달지 않는다. 손목 카메라는 없다.

---

## 설치

```bash
git clone <이 저장소> ~/m0609_vla_ws && cd ~/m0609_vla_ws

# ① 외부 저장소 (GraspGenX · Isaac ROS · Doosan 드라이버)
./scripts/fetch_externals.sh

# ② API 키
cp .env.example .env && chmod 600 .env && $EDITOR .env

# ③ 판단 계층 파이썬 환경
#    🔴 --system-site-packages 필수 — 이게 없으면 rclpy 가 안 보인다
python3 -m venv --system-site-packages .venv
source .venv/bin/activate && pip install -r requirements-vla.txt && deactivate

# ④ 컨테이너 (YOLO-seg + GraspGenX)
#    🔴 마운트 경로가 호스트와 같아야 한다 — 스크립트가 호스트 경로를
#       컨테이너 안에서 그대로 source 하기 때문이다
docker build -f docker/Dockerfile.graspx -t od_kimkh:rebuilt docker
docker run -d --name od_kimkh \
  --gpus all --network host --ipc host \
  -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=93 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $PWD:$PWD \
  od_kimkh:rebuilt sleep infinity

# ⑤ 빌드
./scripts/build.sh
```

### 🔴 빌드에서 반드시 지킬 것

- **`colcon build` 를 맨손으로 돌리지 말고 `./scripts/build.sh` 를 쓴다.**
  `vla_system` 은 `.venv` 안의 torch/openai 가 필요한데, apt 로 깔린 `/usr/bin/colcon` 은
  venv 를 켜도 항상 `/usr/bin/python3` 로 돌아 셰뱅에 그 경로를 박는다. 그러면 노드가
  런타임에 `ModuleNotFoundError: torch` 로 죽는다. 스크립트가 셰뱅까지 검사한다.
- **yaml 만 고쳐도 다시 빌드한다.** `ament_python` 패키지는 share 가 `build/` 를 가리켜
  `src` 수정이 안 넘어간다. `.py` 는 반영돼서 착각하기 쉽다.
- **`ROS_DOMAIN_ID=93`** — 호스트·컨테이너 전부. 하나라도 0이면 토픽이 서로 안 보이는데,
  증상은 `perception_node` 가 "no frames processed yet" 만 찍는 것뿐이라 진단이 오래 걸린다.

### 금지 (전부 실기 사고에서 나온 규칙)

| 하지 말 것 | 대신 | 왜 |
|---|---|---|
| **호스트**에 `pip install opencv-python` | `apt install python3-opencv` | rclpy·Qt 가 같은 프로세스에 뜨면 segfault. **컨테이너는 GUI 를 안 띄우므로 예외** — 거기선 pip `opencv-python==4.11.0.86` 을 쓴다 |
| `numpy>=2.0` | `numpy<2` | Humble `cv_bridge` 의 컴파일된 확장이 numpy 1 ABI 로 빌드돼 있어 `AttributeError: _ARRAY_API not found` 로 죽는다 |
| `pip install --user` | venv | apt pytest 와 충돌해 전 패키지 테스트 붕괴 |

---

## 실행

터미널 배치와 전체 순서는 **[docs/RUNBOOK.md](docs/RUNBOOK.md)** 가 정본이다. 요약:

```
호스트     bringup(로봇) → RealSense
컨테이너   cumotion segmenter → nvblox → cumotion planner → move_group + RViz
호스트     graspx(YOLO) → graspgenx → pick_fsm → vla_command
판단       vla_gui
```

### 자주 쓰는 서비스

```bash
source install/setup.bash && export ROS_DOMAIN_ID=93

ros2 service call /pick/pause       std_srvs/srv/Trigger {}   # ✋ 되돌릴 수 있는 정지
ros2 service call /pick/resume      std_srvs/srv/Trigger {}   # 이어서
ros2 service call /pick/release_now std_srvs/srv/Trigger {}   # 그 자리에 놓기
ros2 service call /pick/stow        std_srvs/srv/Trigger {}   # 🔴 끄기 전 정리
ros2 service call /pick/abort       std_srvs/srv/Trigger {}   # 파괴적 중단(SAFE_STOP)
```

🔴 **끄기 전에 `/pick/stow` 를 부른다.** 물체를 든 채 `Ctrl-C` 하면 그리퍼가 문 채로
남는다 — 일부러 그렇게 뒀다(떨어뜨리는 것보다 안전). `stow` 는 놓을 자리로 **먼저 가서**
놓고 홈으로 간다. "그리퍼 열고 홈 복귀" 를 글자대로 하면 지금 있는 자리에 떨어뜨린다.

---

## 검증

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/pick_fsm/test src/voice_processing/test -q      # 79 passed

source .venv/bin/activate
python3 -m pytest src/vla_system/test -q                              # 246 passed
```

테스트는 **로봇도 카메라도 API 키도 없이** 돈다. 규칙 계층은 표 기반 가짜 파서로,
경계 JSON 은 순수 함수로 검사한다 — 실패하면 LLM 의 기분이 아니라 우리 로직이 틀린 것이다.

---

## 설계에서 절대 깨면 안 되는 것

| | 불변식 |
|---|---|
| I1 | 경계는 **JSON 3채널뿐**. `vla_interfaces` 메시지가 FSM 쪽으로 넘어가지 않는다 |
| I2 | VLM 은 `/pick/approve`(승인)를 호출하지 않는다 — 코드 경로 자체가 없다 |
| I4 | **정지 경로에 LLM 이 끼지 않는다.** "멈춰"는 GUI → 로봇 직행 |
| I5 | 물체 보정 중 자동 그리퍼 개방 금지 — 떨어뜨리는 게 멈추는 것보다 위험하다 |
| I6 | 동시 in-flight action 은 1개. 팔은 하나다 |
| I7 | 상태 전이는 `states.py` **한 곳**에만 |
| I11 | `PAUSED` 에서 **자율 동작 금지.** 시간이 지나도 아무 일도 안 일어난다 |
| I12 | **시간 경과만으로 팔이 움직이는 경로가 없다** |
| I13 | 그리퍼는 팔이 **멈춰 선** 상태에서만 열린다 (출발지 3곳뿐, 검정이 고정) |

---

## 더 읽을 것

| 문서 | 무엇 |
|---|---|
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | 터미널 배치와 실행 순서 (정본) |
| [docs/fsm/vla-bridge-contract.md](docs/fsm/vla-bridge-contract.md) | **경계 계약.** 두 계층을 잇는 JSON 스키마 |
| [docs/fsm/context/constraints.md](docs/fsm/context/constraints.md) | **실기로 알아낸 사실.** 도면과 다른 것들 |
| [src/PACKAGES.md](src/PACKAGES.md) | 패키지별 상세 · FSM 상태도 정본 |
| [docs/fsm/plans/](docs/fsm/plans/) | 통합 작업 인계 문서 |
