<!-- meta
updated: 2026-08-11
status:  live
owns:    지금 상태 · 다음 할 일 · 열려 있는 이슈
-->

# 세션 상태

> 현재 상태로 덮어쓴다. 로그처럼 쌓지 않는다.

**최종 갱신:** 2026-08-11

## 🟢 지금 어디까지 왔나 (2026-08-05) — GraspGenX 실기 파이프라인 관통

**명령 한 번에 촬영→세그멘테이션→GPU 추론→grasp 선택까지 돈다. 로봇은 아직 안 움직인다.**

```
ros2 service call /grasp/compute std_srvs/srv/Trigger
  [grasp_bridge_node.py] 시스템 파이썬·rclpy          [graspgen_worker.py] uv venv·GPU
    depth 10프레임 중앙값 → 세그멘테이션 → 씬 4파일 ──파이프──▶ GraspGenX (모델 1회 로드)
    ◀── grasps(base_link) ── 점수·도달·접근축 필터 ──▶ /grasp/best, /grasp/best_tcp, /grasp/candidates
```

| 실측 결과 (사과) | 값 |
|---|---|
| 손끝(TCP) vs 자로 잰 사과 위치 | **1.1 cm** |
| 세그멘테이션 표면중심 vs 실측 | **0.1 cm** |
| 접근축 `R[2,2]` | −1.00 (수직) |
| grasp 후보 | 12~32개 (씬에 따라) |

- 파일: `src/graspgenx_perception/graspgenx_perception/{capture_graspgenx_scene,graspgen_worker,grasp_bridge_node}.py`
  + 수동 검증 3종(`test/manual_{capture_scene,grasp_bridge,scene_roundtrip}.py`).
  **2026-08-07 부터 ROS 패키지다** — 이전에는 워크스페이스 `scripts/` 단독 실행이었고
  `setup.py` 가 바깥 경로를 `scripts=[...]` 로 심었다. 지금은 소스가 전부 패키지 안에 있고
  진입점이 `console_scripts` 라 **실행 파일 이름에서 `.py` 가 빠졌다**:
  `ros2 run graspgenx_perception {grasp_bridge_node,capture_graspgenx_scene}`.
  이전↔현재 경로 대조표는 `md/graspgenx-perception-notes.md`(2026-08-09 이전엔
  `src/graspgenx_perception/README.md`였다 — 그날 패키지 README 통합으로 이관됨)에 있다
  (vault 밖이라 위키링크가 아니라 저장소 경로로 적는다).
- 계약·튜닝값·함정은 [[ws/cobot2/detect_graspx]]가 단일 출처다(출력 규약 §3, 폭 계산·1/10mm 함수 §5, 상류 버그 §6). 실측 사실(체크포인트 sha256, VRAM 측정치)만 [[ws/cobot2/context/constraints]] "GraspGenX 관련"에 있다.
- ✅ **`tool0` → RG2 손끝 실측 완료 (2026-08-07)**: 닫힘 240 mm = 브라켓+퀵커넥터 22 mm +
  그리퍼 자체 218 mm. 개구 70/100 mm 에서 손끝이 17/41 mm 후퇴(힌지 회전, 비선형).
  실측 근거·두 오차 성분 분리는 [[ws/cobot2/context/constraints]] "GraspGenX 관련"이 단일 출처.
  - 폭에 따른 가변 성분은 `pick_fsm/pick_fsm/rg2.py`의 `fingertip_length_m(width_m)`로 배선
    완료 — `tcp_offset_m` 상수(0.18) 대체. `colcon test --packages-select pick_fsm` PASS(21/21).
  - ✅ **고정 오프셋(브라켓 22 mm)도 xacro에 반영함(2026-08-07, 사용자 승인 후 진행)**.
    `onrobot_rg2.xacro`의 `has_bracket` 분기 값을 0.004→0.022로, `m0609_with_rg2.urdf.xacro`가
    그 스위치를 켜도록 수정. 축은 추측 없이 FK 재계산으로 확인(rg2_base_link 접근축 +Z가
    tool0 프레임의 +X로 매핑됨). `colcon build` PASS. **미검증**: 실기 RViz 육안 확인,
    self-collision 경계 재검토 — 로봇 세션 필요.

> 📁 **문서 전체 지도는 [[ws/cobot2/README]]에 있다.** 어느 문서가 무엇의 단일 출처인지 거기서 본다.

## 🟢 cuMotion + nvblox 실기 파이프라인 전 구간 관통 (2026-08-06) — 로봇은 아직 안 움직였다

> 📌 **실행 명령어·노드 지도·단계별 검증은 `config/testcommand.md`가 단일 출처다.**
> 여기서 명령을 다시 적지 않는다.
> ⚠️ 이 문서는 vault(`md/`) 밖에 있어 **위키링크로 못 건다** — 아래 "열려 있는 이슈" 참고.

```
카메라 ─▶ robot_segmenter ─▶ nvblox(esdf_mode:=3d) ─(서비스)─▶ cumotion_planner ─▶ move_group
         (로봇 몸 지움)                                        (read_esdf_world:=True)
```

실기 실측 (로봇+카메라+nvblox 전부 살아 있는 상태, 관절목표, 각 10회):

| | server 중앙값 | 성공 |
|---|---|---|
| OMPL | 42.4 ms | 10/10 |
| cuMotion | 110.6 ms | **10/10** |

cuMotion이 쥔 장애물 복셀 **27,646개**(`/curobo/voxels`) — nvblox 세계가 실제로 실렸다.

🔴 **이 숫자로 cuMotion을 판정하지 말 것.** 관절공간 목표는 OMPL(RRTConnect)에 가장 유리하다.
(a)/(b) 결정 근거는 **장애물이 궤적을 실제로 막는 씬**에서의 재측정이다.

**하루에 걸린 함정 6개(전부 [[ws/cobot2/context/constraints]]에 기록):**
`/joint_states` velocity 필수 · XRDF 구 과대추정 6쌍 · nvblox `esdf_mode` 기본값 2d가 프로세스를
죽임 · **`robot_segmenter_node` 없으면 로봇이 자기 몸을 장애물로 봄** · 이미지 numpy 2.2.6이 cv2를
깸 · `run_dev.sh`가 컨테이너를 새로 만들어 pip 설치 유실(→ `scripts/container_setup.sh`).
공통점: **OMPL은 멀쩡한데 cuMotion만 죽는다.**

- ⛔ **미해결**: ① 장애물이 궤적을 실제로 바꾸는지 미검증(계획 성공까지만) ② XRDF에서
  `link_4 ↔ rg2_base_link` 자기충돌 검사를 껐다 — 실기 모션 전 재검토 필수 ③ 세그멘터 3.7 Hz 병목
  ④ 그리퍼 Modbus 연결 실패 ⑤ depth가 요청 15 Hz 대비 9.65 Hz.

## README에서 이관 — 현재 상태 / 결합점 / 알려진 함정 (2026-08-09)

> README는 재현 절차(0절)만 남기고, 검증 상태·결합점·함정은 여기로 옮겼다.
> 대상: `m0609_rg2_bringup`, `m0609_rg2_moveit`.

### 기능별 검증 상태

| 기능 | 상태 | 근거 |
|---|---|---|
| 로봇 bringup (virtual) | ✅ 검증됨 | 컨트롤러 3개 active 확인 (2026-08-02) |
| 로봇 bringup (real) | ✅ 검증됨 | 실기 연결 후 MoveIt Plan/Execute까지 확인 (2026-08-02) |
| 카메라 드라이버 (`reals` alias) | ✅ 검증됨 | `/camera/camera/...` 토픽 실측 (2026-08-01) |
| 카메라 드라이버 (`camera.launch.py`) | ✅ 검증됨 | 실기 D435i로 기동 (2026-08-03). `/camera/camera` + `/camera_calib_tf` 노드 기동, `/camera/camera/depth/color/points` 18~20 Hz 발행, `/tf_static`의 `base_link→camera_link`가 `calib_npy_to_tf.py` 출력과 소수점까지 일치 |
| 캘리브 TF (`base_link→camera_link`) | ✅ 전파 검증됨 / 🔴 **값은 불합격** | **전파는 확인됨(2026-08-09)**: 실기 `tf2_echo base_link camera_link` = `[1.435, -0.053, 0.822]`, RPY `[-4.98, 30.22, -177.53]°` 로 새 npy 계산값과 소수점까지 일치. npy를 직접 읽는 노드는 없고 전부 tf2 lookup이라 주입점은 `camera_calib_tf` 하나뿐. **그러나 값 자체가 자체 진단 불합격**: `data/` 32자세를 **424x240**으로 수집해 코너가 **5장**만 검출(쌍 4개), 병진잔차 **41.1 mm** + 회전 2.80°(1.65 m에서 **횡 81 mm**). 근거 `corecode/Calibration_Tutorial/calib_report.json`. **카메라를 실제로 옮겼으므로 되돌리기는 불가 — 1280x720 재수집만이 해법.** 상세는 [[ws/cobot2/context/constraints]] "재캘리브 (2026-08-09)" |
| MoveIt 경로 계획 | ✅ 검증됨 | RRTConnect, 0.019 s |
| MoveIt 궤적 실행(Execute) | ✅ **실기 검증됨** | 실제 로봇으로 Plan → Execute 확인 (2026-08-02) |
| RViz 수동 장애물 회피 | ✅ 설정 완료 | `publish_geometry_updates` 등 4개 활성. [README 5절](README.md#5-시뮬레이션에서-장애물-놓고-회피-디버깅) |
| **3D 장애물 감지 (octomap)** | ✅ **실기 검증됨** (2026-08-03) | `moveit-ros-perception` 설치·self-filter·장애물 회피 확인. 상세 근거는 `md/review_moveit.md`가 단일 출처 |
| 그리퍼 MoveIt 제어 | ❌ 미지원 | RG2는 `/onrobot/sendCommand` 서비스로 직접 제어. MoveIt 컨트롤러 없음 |
| YOLO-seg 인식 (컨테이너→호스트) | ✅ 검증됨 | `/yolo_seg/labels` 호스트 수신 25.6 Hz (2026-08-07 21:15). 이전의 "데이터 안 흐름"은 해소됨 |
| **yolo 경로 테이블 높이 필터링·돌출 계산 + 클래스별 실측 치수** | ✅ 구현·합성테스트 12건 PASS / ⚠️ **실기 미검증** | 2026-08-09. `segment_from_labels()`가 물체마다 국소 테이블 기준(±`obj_radius_m`/`yolo_table_ring_m`)으로 `obj_min_h` 미만 픽셀을 다듬고 돌출높이를 로그에 남긴다 — 전엔 yolo 경로에 이 필터가 아예 없었다. **`class_dims`**(신설): `config/objects.yaml`의 `dimensions`에 형태 고정 물체(병·컵)의 실측 반경·높이를 적으면 전역값 대신 그 값을 쓴다 — 상한도 안전하게 켤 수 있다(전역 12cm 대신 그 클래스 실측 높이). 자연물엔 적용 안 함(개체마다 형태가 다름). depth 결손(반사면 등) 자체는 못 고친다 — 마스크만 다듬는다. → [`src/PACKAGES.md#graspgenx_perception`](../src/PACKAGES.md#graspgenx_perception) "yolo 경로에도 테이블 높이 필터링이 생겼다" / "클래스별 실측 치수" |
| **물체 종류 선정** (`target_classes`) | ✅ 구현·PASS / ⚠️ 실기 미검증 | 빌드 PASS + 순수함수 24개 PASS + 저장된 실기 씬 이미지로 확인(8검출 → `apple` 1개). 라이브 파이프라인 실행은 안 해봄 (YOLO 주 파이프라인, README 0절·6절) |
| **물체 개체 선정** (사과 2개 중 하나) | ✅ 구현·빌드·단위테스트 PASS(2026-08-11) / 🔴 **실기 미검증** | `select_by_point()` — base XY 최근접 매칭, 워커 호출 전에 후보를 하나로 좁힌다. 정본은 [`md/vla-bridge-contract.md`](vla-bridge-contract.md) §8, 설계 근거는 [`md/plans/2026-08-08-vla-integration.md`](plans/2026-08-08-vla-integration.md) §5. `graspgenx_perception/README.md` "다음 방향" 절의 옛 2단계 안(`scene_id` 핸들)은 이걸로 대체됐다 |
| **VLA 지시 채널** (`/vla/pick_command`) | ✅ 구현·빌드 PASS / ⚠️ **노드 단독 실측만** | `voice_processing/vla_command_node` → `task_manager` 의 기존 `/get_keyword` 자리. **`pick_fsm` 코드 0줄·새 msg 0개·새 패키지 0개.** 스키마 27건 PASS + 왕복/TTL/거부/결과판정 실측. 같은 채널로 rqt 패널의 **시작·중단·리셋**(`cmd:"start"/"abort"/"reset"`)도 대신한다 — `승인`만은 코드 경로 자체가 없어 `cmd:"approve"`가 항상 거부됨을 실측 확인. 🔴 **FSM·로봇과 연결한 한 사이클은 미검증**, VLA PC 실연결도 미검증. 실행·테스트 명령 → [README 7-3절](../README.md#7-3-음성vla로-rqt-패널-버튼-대신하기--실행-및-테스트-2026-08-09-구현), 상세 계약 → [`src/PACKAGES.md#voice_processing`](../src/PACKAGES.md#voice_processing) |
| **VLA(`~/M0609_VLA_system`) 나머지 통합** | ❌ 미착수 (범위 확정) | **로봇 행동은 이 ws 가 그대로 유지**하고, VLA 는 "어떤 물체를 집을지"만 **외부 PC**(휴대폰 핫스팟 링크) 에서 전달한다. **D435i 영상은 압축 컬러(`color/image_raw/compressed`)만 넘긴다** — 포인트클라우드 160 Mbps(실측)·raw 컬러 36.8 Mbps 는 핫스팟에서 불가. 남은 것: 핫스팟 대역폭 실측, 두 PC DDS 도달성, `pixel` 개체 선정(§5) |

> 3D 장애물 감지(octomap) 검증 결과·채택 설정값 스냅샷·cuRobo 비교 설계는 `md/review_moveit.md`가 단일 출처다.

### 결합점 — 한 곳만 바꾸면 조용히 깨지는 것들

**에러 없이 기능만 죽는다.** 건드리기 전에 짝을 확인하라.

| # | 값 | 나오는 곳 | 어긋나면 |
|---|---|---|---|
| ① | 네임스페이스 `dsr01` | `bringup.launch.py`(`namespace=`), `moveit_controllers.yaml`(컨트롤러 이름 `/dsr01/...`), `moveit.launch.py`(`-c /dsr01/controller_manager`) — **3곳** | **Plan은 되고 Execute만 ABORTED.** 실제로 겪은 버그다 |
| ② | 캘리브 `T_cam2base.npy` | `corecode/Calibration_Tutorial/`(**정본, 생성**) → `m0609_rg2_bringup/config/`(symlink) → `install/.../config/`(symlink). 2026-08-03부터 **`cp` 불필요 — 정본만 갈아끼우면 install까지 자동 반영**(2026-08-09 md5로 3곳 일치 재확인) | 반대로 **나쁜 캘리브도 즉시 실기에 반영된다**. 갈아끼우기 전에 `calib_report.json`의 `verdict`를 볼 것 |
| ③ | xacro 파일명 `m0609_with_rg2.urdf.xacro` | `bringup.launch.py`, `moveit.launch.py` 양쪽이 경로로 직접 읽는다 | moveit이 런타임에 깨진다 |
| ④ | 관절 이름 `joint_1..6` | SRDF, `dsr_controller2.yaml`의 JTC 설정, `moveit_controllers.yaml` | 궤적이 컨트롤러에 거부된다 |
| ⑤ | `seg_source` **기본값 `yolo`** (2026-08-08 변경, `capture_graspgenx_scene.py:91`) | `grasp_bridge_node`(호스트, 소비) ← `yolo_seg_node`(**컨테이너**, 발행). 기동 명령이 서로 다른 두 터미널이다 | 컨테이너 쪽을 안 띄우면 `/grasp/compute`가 **`seg_source=yolo 인데 라벨맵을 못 받았다`**로 실패한다. 기동 순서는 **`src/PACKAGES.md`** "pick_fsm §2 실행" 3.5/4번이 정본 (2026-08-09 보강. 패키지 README는 같은 날 포인터만 남기고 이관됐다) |
| ⑥ | **`ROS_DOMAIN_ID` = 93** (2026-08-09 신설 — VLA 로 **PC 가 둘이 되면서** 대표 사례가 됐다) | 호스트 셸(`rdm` alias) · 컨테이너 이미지(93 박혀 있음) · **외부 VLA PC(지정 0건 → 기본 `0`)** | **토픽이 아예 안 보인다.** 노드는 멀쩡히 떠 있어서 "죽었나?"로 오진하기 쉽다. 판별: **도메인이 탐색을, DDS 프로파일이 데이터를 결정한다** — 토픽 자체가 안 보이면 도메인, 토픽은 보이는데 데이터가 0 이면 프로파일/방화벽 (2026-08-07 A/B 실측) |
| ⑦ | **카메라 프로파일 `424x240x15`** (`camera.launch.py:74-77` 기본값) | `align_depth.enable=true` 라 **color 프로파일 하나가 octomap·nvblox 두 경로를 동시에** 지배한다 | VLA 대역폭 때문에 올리면 **우리 octomap 이 먼저 밀린다**(updater 단일 스레드). ⚠️ **`480x320` 은 D435i 미지원** — 주면 스트림이 안 열린다. 지원 목록은 [[ws/cobot2/context/constraints]] |

### 알려진 함정

**포인트클라우드가 로봇 옆에 90° 돌아가서 뜬다**
npy는 OpenCV **optical** 규약(z=전방), ROS `camera_link`는 REP-103 **body** 규약(x=전방)이다.
`calib_npy_to_tf.py`가 기본으로 보정한다. `inv(T)` 문제가 아니다 — **부호를 만지지 말 것.**
지문: 출력 RPY의 roll ≈ ±90°. 자세한 채점표는 [[ws/cobot2/context/constraints]].

**`ros2 launch dsr_moveit_config_m0609 demo.launch.py`가 안 된다 — 이건 쓰는 게 아니다**
① `MoveItConfigsBuilder("m0609")`가 관례상 `m0609_moveit_config` 패키지를 찾는데 실제 이름은
`dsr_moveit_config_m0609`다 (업스트림 버그). ② 고쳐도 그 config의 URDF엔 **RG2가 없다.**
→ 이 ws에서는 `m0609_rg2_moveit`을 쓴다.

**Ctrl-C로 끌 때 segfault + 스택트레이스가 쏟아진다**
MoveIt 2.5.9의 알려진 종료 순서 버그(`class_loader` 언로드 타이밍). 기능에 영향 없다. 무시.

**RViz 창이 두 개 뜬다**
bringup RViz(관측: RobotModel/TF/PointCloud2)와 MoveIt RViz(조작: MotionPlanning)는 목적이 다르다.
다만 노드 이름이 둘 다 `rviz2`라 충돌 경고가 뜬다. 하나만 쓰려면 `moveit.launch.py rviz:=false`.

**realsense-viewer와 ROS 드라이버는 동시에 못 쓴다** (USB 장치를 독점한다)

**virtual 모드에서 에뮬레이터가 안 뜬다**
이전 run의 Docker 컨테이너가 `Exited`로 남은 경우다. bringup이 자동으로 지우지만, 수동은
`docker rm -f dsr01_emulator`.

## 계정/환경
- 공유 랩탑(`rokey`)의 `kimkh` 계정, `cobot2_ws`.
- git: remote 이름 `personal` = `https://github.com/gwanhuiGIM/0730_cobo2_personal` (**HTTPS**), 브랜치 `init_sett`(main보다 앞섬, 아직 미머지).
- git identity는 **repo-local**로 설정됨(`user.name=kimkh`, `user.email=wook9980@gmail.com`). repo-local이라 **다른 PC에서 clone하면 다시 설정해야 한다.**
- push는 **VS Code Source Control**로 한다(터미널 git 아님). 정상 동작 확인 — 커밋 `6a78c78`까지 push 완료, 워킹트리 clean.
- `~/.claude/CLAUDE.md`(전역 공통 규칙) 복원 완료.

## 두 PC 체제 — ⚠️ **hostname·계정으로는 구분되지 않는다** (2026-08-05 정정)

두 PC 모두 hostname `rokey`, 계정 `kimkh`, `/home` 구성까지 같다. **`nvidia-smi` 유무로 구분한다.**

- **GPU PC (지금 이 머신)**: **RTX 4060 Laptop 8GB**, 드라이버 595.84. 로봇(`192.168.1.100`)·D435i **직결**.
  ⚠️ VRAM이 계획서들이 가정한 12GB가 아니라 **8GB**다 — `--num_grasps`를 64로 시작한다.
  ✅ **Isaac ROS 컨테이너 경로 열림(2026-08-06)** — `kimkh`가 docker 그룹 멤버이고 컨테이너에서
  `nvidia-smi`가 RTX 4060을 본다. 기동: `./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"`.
  ⚠️ **컨테이너를 새로 만들면 `pip3 install 'warp-lang==1.5.0'`을 다시 해야 한다**(이미지 밖 변경).
- **개인PC(노트북)**: NVIDIA GPU 없음. rosbag으로 개발.
- 상세 하드웨어 실측표는 [[ws/cobot2/context/constraints]] "🔴 이 랩탑 하드웨어".
- 역할 분담·시뮬레이션 범위·동기화 방법은 [[ws/cobot2/plans/2026-08-01-pc-role-split]] 참조.

### GPU PC 최초 세팅 절차
```bash
git clone https://github.com/gwanhuiGIM/0730_cobo2_personal.git cobot2_ws
cd cobot2_ws && git checkout init_sett          # 기본 브랜치가 main이라 필수
git config user.name "kimkh" && git config user.email "wook9980@gmail.com"
./scripts/setup_isaac_ros.sh                    # isaac_ros-dev/ 복원 (release-3.2)
code .                                          # 이후 VS Code Source Control로 커밋/푸시
```

## Isaac ROS 소스 상태
- `isaac_ros-dev/`는 **커밋하지 않는다**(.gitignore). `scripts/setup_isaac_ros.sh`로 재현한다 — 136MB·수천 파일이라 VS Code Source Control이 마비되고, `**/.git/` 규칙 때문에 pull한 쪽에서 태그·히스토리를 알 수 없게 되기 때문.
- 개인PC에 `release-3.2`로 클론 완료: `isaac_ros_common`(`scripts/run_dev.sh` 존재 확인), `isaac_ros_nvblox` `v3.2-14`(submodule `nvblox_core` 포함).
- **release-4.x 금지** — `run_dev.sh`가 Isaac ROS CLI로 이전되어 사라졌고 사실상 Jazzy 중심. 4.4로 받았다가 막혀서 3.2로 재클론한 이력 있음(2026-08-01).
- `.isaac_ros_common-config` = `CONFIG_IMAGE_KEY=ros2_humble.realsense`
- ⚠️ **`setup_isaac_ros.sh`는 GraspGenX를 클론하지 않는다.** 새 PC에서는 따로 받아야 한다:
  `git clone https://github.com/NVlabs/GraspGenX.git isaac_ros-dev/src/GraspGenX` → `uv sync` (venv, torch).
  체크포인트(HF `adithyamurali/GraspGenXModel`)는 **1.7 GB LFS**이고, 매체 복사로 옮기면
  **크기가 맞아도 내용이 0으로 채워질 수 있다** — 반드시 `sha256sum` 검증(해시는 constraints.md).
- `isaac_ros-dev/`에 `COLCON_IGNORE` 있음. **없으면 루트 `colcon build`가 CUDA 없이 nvblox를 빌드하려 든다**
  (colcon은 `base_paths=['.']`로 repo 전체를 훑는다).
- `realsense-ros` 클론 **불필요** — apt `ros-humble-realsense2-camera 4.58.2` 설치됨(GPU PC도 동일하다는 사용자 진술, 미검증).

## 열려 있는 이슈
- 🟢 **VLA 통합 — 지시 채널(§2)은 2026-08-09 구현·빌드·단독실측 완료.**
  `src/voice_processing/vla_command_node` 가 `/vla/pick_command`(JSON) 를 받아
  `task_manager` 가 이미 갖고 있던 `/get_keyword` 자리로 넘긴다. 결과는 `/vla/pick_result`
  로 되돌린다(`/pick/state` 만 보고 판정). 🚨 **`/pick/approve` 는 부르지 않는다** —
  `auto_start` 는 `/pick/start` 까지만이다.
  🟢 **2026-08-11: `pixel`(어느 개체인가)도 이제 쓴다.** `pixel_policy:=select` 로 띄우면
  `grasp_bridge_node.select_by_point()`(신설)가 base XY 최근접 매칭으로 개체 하나를
  고른다 — 이제 `pick_fsm`(`task_manager.py`)·`voice_processing`·`graspgenx_perception`
  세 패키지 모두 코드가 늘었다("pick_fsm 코드 0줄" 원칙은 이 기능부터 깨졌다, 의도된
  것 — 개체 선정은 애초에 그 원칙의 범위 밖이다). 같은 클래스 물체가 2개 이상인데
  `select`를 안 쓰면 여전히 `pixel_policy:=reject` 로 막는 게 유일한 방어다.
  🔴 **미검증: PERCEIVE 한 사이클(픽셀 지정 → 개체 선정 → grasp) 실기 관통, VLA PC 실연결.**
  빌드·순수함수 단위테스트만 PASS — 상세는 [[ws/cobot2/vla-bridge-contract]] §8.
  레퍼런스는 [[ws/cobot2/src/PACKAGES]] `#voice_processing`.
- 🟡 **VLA 통합 — 범위 확정됨(2026-08-08 사용자), §3-3 이하는 미착수.** 정본은
  [[ws/cobot2/plans/2026-08-08-vla-integration]]. 여기엔 값을 베끼지 않는다.
  **로봇 행동(pick_fsm+MoveIt+graspgenx)은 우리가 유지하고, VLA 는 "무엇을 집을지"(나중에
  "어디에 놓을지")만 전달한다. VLA 는 아예 다른 외부 PC 에서 돈다** — 웹캠·homography·LLM 은
  우리 역할이 아니다. 이 확정으로 카메라 전제 충돌·`DR_init` 경합·`vla_wrist` 대체안이
  전부 폐기됐다. **3차 개정(같은 날): 링크는 개인 휴대폰 핫스팟이고 D435i 영상을 VLA PC 로
  넘긴다** → 대역폭이 첫 제약이 됐고(포인트클라우드·raw 컬러는 못 넘긴다, 압축 컬러만),
  대신 지시를 **픽셀 좌표**로 보낼 수 있어 좌표계 합의(D3) 문제가 강등됐다.
  **4차 개정(2026-08-09, 실기 `rokey` 실측)**: ✅ **D5 해결 — `480x320` 미지원 확정,
  `424x240` 그대로 둔다.** 대역폭도 실측으로 교체 — 클라우드 **160 Mbps**(추정 245보다 작음),
  raw 컬러 36.8 / depth 24.5(추정 정확), 🔴 **압축 컬러는 5.7 Mbps 로 추정(1.5)의 3.8배**
  (원인: `jpeg_quality` 미설정 → 기본 95. 80 으로 낮추는 것이 첫 손잡이).
  **포인트클라우드는 아직 쓴다 — 소비자는 `move_group` 정확히 하나**(`Subscription count: 1`
  실측). cuMotion 전환 완료 전엔 못 끈다.
  남은 것: 핫스팟 실효 대역폭 실측(`iperf3`), 두 PC 간 DDS 도달성(우리 93 vs VLA 0),
  ⚠️ **`~/M0609_VLA_system` 이 실기 머신엔 없다** — VLA 측 값은 전부 개인PC 에서 읽은 스냅샷.
- 🟡 **두 패키지가 서로 다른 손끝 모델을 쓴다** (2026-08-08 감사에서 발견).
  `pick_fsm` 은 `rg2.fingertip_from_rg2_base_m()`(실측 **218 mm**, 폭에 따라 가변)로
  2026-08-07 에 갈아탔는데, `graspgenx_perception/grasp_bridge_node.py:56` 은 아직
  `tcp_offset_m: 0.18` 상수다. VLA 도 180 mm 를 쓴다 → 통합 시 **38 mm 얕게 잡는다.**
  결정(D4)과 근거는 [[ws/cobot2/plans/2026-08-08-vla-integration]] §3. 실측 출처는
  [[ws/cobot2/context/constraints]]:900-906.
- 🟡 **`~/.local` 오염 — 개인PC 만의 문제다. 실기 `rokey` 는 깨끗하다** (2026-08-09 정정).
  08-08 에 "다시 오염됐다"고 적은 것은 **개인PC 에서 잰 값**이었다(머신을 안 적었다).
  머신별 실측:
  - **개인PC (2026-08-08)**: VLA `requirements.txt` 가 `torch 2.7.1`·`opencv-python 4.10`·
    `ultralytics 8.4.76`·`numpy 1.24.4`·`anyio 4.13` 을 `~/.local` 에 깔았다. `import cv2` 가
    apt 4.5.4 가 아니라 4.10.0 을 잡고, `pytest` → `ModuleNotFoundError: _pytest.scope`.
    우회 `-p no:anyio` 로 24개 PASS.
  - **실기 `rokey` (2026-08-09)**: 🟢 `~/.local` 에 해당 패키지 **0개**. `cv2` **4.5.4**,
    `numpy` **1.21.5** 둘 다 `/usr/lib/python3/dist-packages`(apt). `pytest` **24 PASS,
    우회 없이 0.39s**.
  → **`-p no:anyio` 는 개인PC 전용 우회**로 읽는다. graspgenx README 의 "우회 불필요(2026-08-07)"
  문장은 rokey 에서는 여전히 맞고 개인PC 에서만 틀리다.
- ⚪ **개인PC(CPU)에는 YOLO 자산이 없다 — 정상이다, 이슈가 아니다** (2026-08-08).
  `nvidia-smi` 없는 머신에서 확인한 것이므로 **GPU PC 상태에 대해서는 아무것도 말해주지
  않는다**(판별 규칙은 아래 "두 PC 체제"). 여기서 관측된 것: 가중치 `yolo11n-seg.pt`
  `find` 0건(`src`·`build`·`install`), 컨테이너 `od_kimkh` 없음(`docker ps -a` 에
  `object_detection`(Exited)·`portainer` 뿐).
  → **개인PC 에서 YOLO 경로를 띄우려 하지 말 것.** 이 PC 의 용도는 rosbag 개발이다.
  ⚠️ 2026-08-08 세션에서 이걸 "YOLO 경로 기동 불가"라는 **전역 사실로 잘못 적었다가 정정함**
  (사용자 지적). CPU PC 에서 `find`/`docker ps` 결과로 GPU PC 를 판단할 수 없다.
  GPU PC 의 실제 상태는 그 머신에서 재확인이 필요하다 — **미확인**.
- 🟡 **`build/`·`install/` 에 삭제된 패키지 6개가 남아 있다** (2026-08-08). `src/` 에서
  지웠지만 `ros2 pkg list` 에 `pick_and_place_text`·`pick_and_place_voice`·`robot_control`·
  `rokey`·`usb_cam`·`webcam_perception` 이 여전히 뜬다. `scripts/graspx_container.sh` 가
  컨테이너 안에서 같은 `install/setup.bash` 를 source 하므로 컨테이너도 죽은 패키지를 본다.
  정리하려면 `rm -rf build install` 후 전체 재빌드.
  ⚠️ **그 전에**: `install/pick_and_place_text/share/.../yolov8n_tools_0122.pt` 가 이 파일의
  **유일 사본**이다(`.gitignore` 의 `*.pt` 로 git 히스토리에 없음). 공구 5종 가중치이고
  2026-08-08 시점 "안 쓸 예정"으로 확인받았으므로 소실을 수용하고 진행해도 된다.
- ✅ ~~**`voice_processing` 이 `COLCON_IGNORE` 다**~~ **해소 (2026-08-09).** `setup.py` 가
  gitignore 된 `resource/.env` 를 `data_files` 에 **못박아** 파일 없는 머신에서 빌드가 통째로
  실패하던 것이었다. `['resource/.env']` → **`glob('resource/.env')`** 로 바꿔 고쳤다 —
  있으면 설치하고 없으면 `[]` 라 빌드가 안 깨진다. `.env` 는 여전히 `get_keyword`(마이크·STT·LLM)
  의 **런타임** 필수 파일이고, 새로 넣은 `vla_command_node` 는 `.env` 도 추가 의존성도 안 쓴다.
  `COLCON_IGNORE` 삭제 + `package.xml` 에 `rclpy`/`std_msgs`/`std_srvs` 등록 완료, 빌드 PASS.
- 🟡 **`testcommand.md`가 vault 밖에 있다** (2026-08-07). 저장소 루트 → `config/testcommand.md`로
  옮겨졌는데, 이 문서는 **md/ 문서 형식으로 쓰여 있다** — meta 헤더(`status: live`,
  `owns: 실행 명령어 · 노드 지도 · 단계별 검증 명령`)가 있고 본문이
  `[[ws/cobot2/context/constraints]]`·`[[ws/cobot2/plans/2026-08-05-cumotion-bringup]]`을 건다.
  `md/` 밖에 있으면 **세 가지가 동시에 깨진다**: ① 이 문서를 `ws/cobot2/testcommand` 위키링크로
  못 부른다(위 "cuMotion" 절이 그래서 저장소 경로로 바뀌었다) ② 이 문서가 내보내는 위키링크 2개가
  Obsidian에서 안 걸린다 ③ `doc_check.sh`가 md/만 훑으므로 이 문서는 **검사 대상에서 빠진다**
  (meta·문서 지도·live 방치 전부). 정본 해결은 `md/testcommand.md`로 옮기고 `md/README.md`
  문서 지도에 등재하는 것. 옮기지 않기로 정했다면 위 세 가지를 감수하는 결정임을 여기 적어둔다.
- 🔴 **세 계정이 동시 로그인해 같은 ROS 도메인(93)과 같은 GPU를 쓴다** (2026-08-06 실측).
  `joonwon`이 띄운 `move_group`이 내 `ros2 node list`에 그대로 나오고, `kill`은 uid가 달라 실패한다.
  **`ps`에 `user`를 넣지 않으면 남의 프로세스를 내 것으로 착각한다.**
  도메인 분리가 협의된 적이 없다 — 정하지 않으면 이 충돌은 계속된다.
  절차·판별법은 [[ws/cobot2/context/constraints]] "세 계정이 동시에 로그인해…"가 단일 출처.
- ~~GPU PC 도커 경로가 막혀 있다~~ ⤴ **해소.** `kimkh`는 이제 docker 그룹 멤버이고
  (`id`로 확인, 2026-08-06) 컨테이너 안에서 `nvidia-smi`가 RTX 4060을 본다.
  GPU 사양의 단일 출처는 [[ws/cobot2/context/constraints]](RTX 4060 Laptop 8GB).
- ~~D435i depth rosbag 미확보~~ ⤴ **해소(2026-08-04).** `rosbag/bag_0803calibed/` 4개 검증 통과.
  남은 건 장면 2개(빈 테이블 / 장애물 여러 개) 추가 뿐이다 → [[ws/cobot2/rosbag-d435i]] "다시 찍어야 하는 것".
- **카메라 마운트 강성 미확보** — 견고한 고정이 아직 어려움. 캘리브는 **잠정(provisional)**으로 취급하고, Day4 인식 정확도 실측 검증은 마운트 확정 후로 미룬다. 개발용 TF로는 잠정값으로 충분하다.
- **Day4 인식 방식 변경(2026-08-02)**: ray-plane intersection → **FoundationPose**(6D pose), 하드코딩 그립 → **GraspGenX**. 평면 가정이 사라지는 건 이득이지만 **GPU 의존이 커졌다** — Day4는 GPU PC 전용이 된다.
- **C270**은 아직 실기 등록·실행 이력이 없다(`.bashrc` alias만 존재). M0609·RG2·D435i는 실기 확인 완료 → [[ws/cobot2/context/constraints]].
- **캘리브 방식 변경(2026-08-02)**: `easy_handeye2` → `corecode/Calibration_Tutorial`(`eye2hand_calibration.py` / `handeye_calibration.py`). 두 알고리즘 모두 합성 데이터로 정답 복원 확인(오차 ~1e-13). npy(mm) → static TF(m) 변환은 `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`. ~~미해결: `data_recording.py`가 `set_tcp` 후의 `posx`를 기록하므로 결과의 부모 프레임이 flange가 아니라 TCP다~~ → **해소(2026-08-03).** eye-to-hand에서는 판↔그리퍼 변환 G가 AX=XB 유도에서 소거되므로, "그리퍼"를 flange로 잡든 TCP로 잡든 **`T_cam2base` 결과는 같다**(X는 base·camera 쪽 변환이라 팔 쪽 기준 프레임 선택과 무관). `--selfcheck`가 이걸 합성 데이터로 확인한다. 현재 `RECORD_IN_FLANGE_FRAME = False`(TCP 기준)로 두어도 문제 없다.
- **Day1.5 압축 경로 신설** — 캘리브·인식 전부 생략하고 "장애물 놓으면 궤적이 바뀐다"만 보여주는 시연용 경로. GPU 불필요, 개인PC 가능. 임시 static TF를 쓰므로 **rosbag 녹화와 절대 겹치면 안 된다.**
- **좌표 규약 버그 발견·수정 (2026-08-02)** — npy는 OpenCV **optical** 규약인데 ROS `camera_link`(body 규약)로 발행해 클라우드가 로봇 옆으로 90° 튀었다. `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`가 이제 기본 보정한다. **`inv(T)` 문제가 아니었다** — 규약을 먼저 의심할 것. 채점표는 [[ws/cobot2/context/constraints]].
- ~~보류: `sensors_3d.yaml`의 `max_range: 1.5`~~ → **해소 (2026-08-09 확인).** 파일은 이미
  `max_range: 2.0`이고 변경 이력·근거가 `sensors_3d.yaml:49-56`에 남아 있다. 08-09 재캘리브 후
  카메라 거리는 **1.655 m**라 2.0 안에 들어온다 — 손댈 것 없다. 거리는 문서에 적지 않는다
  (yaml 주석의 `np.linalg.norm(T[:3,3])/1000` 한 줄로 그때그때 읽는다).
- **캘리브 품질: 현행값은 자체 진단 불합격 (2026-08-09 재캘리브, `data/` 32자세 / 실제 표본 5장)**.
  AX=XB 병진잔차 중앙값 **41.1 mm** (4쌍 중 2쌍이 30 mm 초과). **원인은 수집 해상도 424x240** —
  `camera.launch.py` 기본값 그대로 띄우고 수집하면 코너가 안 잡힌다.
  이전 최고값은 08-05 `data2`(1280x720, 18장)의 **6.0 mm**이고 그게 직전까지 커밋돼 있던 값이다.
  **다음 캘리브는 이 6.0 mm와 비교한다.** 복구/재수집 절차와 두 값의 전체 비교표는
  [[ws/cobot2/context/constraints]] "재캘리브 (2026-08-09)"가 단일 출처다.
  ~~2026-08-03 기록(34장, 40.1 mm): 그 `data/`는 08-09 수집으로 덮어써져 더 이상 재현 불가~~
  - **되돌리기 전에 카메라가 08-05 이후 물리적으로 움직였는지 확인할 것.** 두 값이 198/170/38 mm,
    pitch 48°→30° 차이라 잔차만으로는 설명되지 않는다. 움직였다면 이전 값도 틀린 것이므로
    되돌리기가 아니라 **1280x720 재수집만이 답이다.**
  - RMS 재투영오차는 공장 내부파라미터를 쓰는 동안 **결과 품질이 아니다**(참고값).
  - **LOO 수치를 재현성으로 읽지 말 것** — 리스트 순서만 섞어도 같은 크기로 움직이고,
    계통오차에는 눈이 멀다. 잔차 큰 쌍을 한꺼번에 빼면 80 mm 옮겨간 반면 LOO는 3.6 mm였다.
    (2026-08-03 cross-review에서 반증. 그 전 세션에 내가 "LOO 1.5 mm = 양호"로 적은 건 오판)
- ✅ **`max_range` 1.5 → 2.0으로 되돌림 (2026-08-03, 사용자 결정).** 1.5는 카메라~base 거리 **1.684 m**보다
  작아 베이스 부근을 잘라내고 있었다. 원래 목적이던 CPU 절감은 체감되지 않았고, `max_range`는 애초에
  CPU 손잡이가 아니다(비용은 거리가 아니라 점 개수에 비례 → `point_subsample`·카메라 프로파일이 실효 손잡이).
  **아직 실기 미검증**: 2.0에서 ① 뒷벽이 장애물로 잡히지 않는지 ② 장애물 경계가 또렷해지는지
  ③ `move_group` CPU가 견디는지 — 다음 실기에서 확인. 경위: [[ws/cobot2/errors-log]] §7
- **octomap_rviz_plugins 미설치** — `/octomap_binary`·`/octomap_full`을 RViz에서 볼 수 없다. 당장은 `/octomap_point_cloud_centers`(PointCloud2)로 우회 중. `topic_tools`도 미설치(throttle 불가 → 카메라 프로파일에서 줄인다).

## ⏭ GPU PC 확인 항목 — ① 해결, ② 미확인 (2026-08-09)

### ✅ ① `480x320` 은 D435i 지원 프로파일이 **아니다** (2026-08-09 `rs-enumerate-devices` 실측)

| | 지원 목록 |
|---|---|
| **Color** | `320x180` `320x240` `424x240` `640x360` `640x480` `848x480` `960x540` `1280x720` `1920x1080` |
| **Depth** | `256x144` `424x240` `480x270` `640x360` `640x480` `848x100` `848x480` `1280x720` |

혼동 주의: depth 에 `480x270` 이 있지 **`480x320` 이 아니다.**
→ **`config/testcommand.md` T1 이 `480x320` 으로 적혀 있으면 `424x240`(현 기본)으로 고친다.**
  현 기본을 그대로 두면 바꿀 것이 없다. `config/testcommand.md` 의 "합치면서 드러난 파라미터
  불일치" 표에서 해당 칸도 지운다.
⚠️ **재캘리브 때는 예외로 `1280x720`** — `data_recording.py` 가 해상도를 지정하지 않고
구독만 해서 낮은 해상도로 찍으면 코너 정밀도가 무너진다. **2026-08-09 불합격(41.1 mm)이
바로 `424x240` 으로 수집해서 난 것이다.**

### ⛔ ② GPU PC 의 alias 정의 — **아직 확인 안 함**

```bash
alias reals; alias br; alias realsense
```
`.bashrc` 가 머신 로컬이라 개인PC 에서 확인할 방법이 없었다. [[ws/cobot2/errors-log]] #19.
(⚠️ Bash 툴은 비대화형 셸이라 `.bashrc` 의 alias 가 안 잡힌다 — **사용자가 터미널에서 직접**
쳐야 답이 나온다.)

## 다음 할 일 (순서 고정 — 위가 막히면 아래로 내려가지 않는다)
> 이 절이 "다음에 뭐 하지"의 단일 출처다. 끝난 항목은 지우고 Day 진행 절로 옮긴다.
> 08-03 상세 계획: [[ws/cobot2/plans/2026-08-03-octomap-integration]]

> ✅ 1~3번(플러그인 설치 / 프로파일 축소 / self-filter / 장애물 회피)은 **2026-08-03에 끝났다.**
> 결과·채택값·검증 상태는 [[ws/cobot2/review_moveit]]로 옮겼다. 아래는 남은 것만이다.

0. ⛔ **`tool0` 플랜지면 → RG2 손끝 거리 실측 (줄자)** — 실기 모션의 전제.
   URDF 190 mm vs 매뉴얼 220+10 mm. 차이가 있으면 **`tcp_offset_m`이 아니라
   `onrobot_rg2.xacro`의 `origin xyz`에 어댑터 두께를 넣는다.** 근거는 constraints.md.
0-b. **물체 이름 지정** — 지금 선택 정책은 "점수 최고"라 컵·노이즈를 집을 수 있다.
   실제로 2026-08-05에 558 px 노이즈를 고르고 사과는 후보 0이었던 사례가 있다.
   `-p min_pixels:=1000`이 임시 방편이고, 정본은 YOLO-seg + 음성 타겟팅
   ([[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]] §1).
   **2026-08-07 코드 감사로 재확인**: 여전히 배선 안 됨(6개 근거·남은 일 3개는
   [[ws/cobot2/plans/2026-08-07-graspgenx-target-matching]]).
   ⚠️ **FoundationPose는 마스크를 만들어주지 않는다** — 마스크를 **입력으로 요구**하고 CAD 메시도 필요하다.
   ROI/마스크 출처는 여전히 검출기(YOLO-seg 또는 RT-DETR)다.
   - **2026-08-09 진행**: 배선(`target_classes`)은 이미 있다 — `seg_source` 기본값이 `yolo`가 되면서
     클래스 이름으로 대상을 좁히는 경로가 살아 있다(`grasp_bridge_node.py:243-245`).
   - **2026-08-09 (2차)**: **사람이 타겟을 고르는 UI/인터페이스까지 배선 완료.** 정본을
     `task_manager` 하나로 옮겼다 — PERCEIVE 진입 때 `SetParameters`로 브리지의
     `target_classes`+`seg_source(=yolo)`를 밀어 넣는다. 사람은 `/pick/target`(String,
     TRANSIENT_LOCAL)이나 rqt 패널의 타겟 상자(YOLO가 보고 있는 클래스 목록 + [적용]/[자동])로
     고른다. 빈 문자열 = 자동(점수 최고). `grasp_source` 기본값도 `legacy_trigger`로 바꿨다
     (`compute_grasp` 서버가 이 ws에 없어 120s 기다리다 ABORT했다).
     계기가 된 실기 실패(런치 인자 조용히 무시 + 정본 2개)는 constraints.md 맨 끝에 기록.
     ⚠️ **가중치 블로커는 그대로다** — UI에서 고를 수 있는 건 COCO 80종뿐이다.
     **남은 블로커는 가중치 하나다**: `yolo11n-seg.pt`는 COCO 80종뿐이라 우리 물체를 못 잡는다.
     → 파인튜닝 계획 [[ws/cobot2/plans/2026-08-09-yolo-seg-finetune]] (방법·필요 요소·미결정 4건).
     어노테이션은 **새 툴 없이** `capture_graspgenx_scene.py`의 depth 세그를 그대로 라벨로 쓴다
     (`scripts/seg_to_yolo.py`, `--self-check` PASS). 캡처·학습 모두 **`rokey` 머신 필요**.
0-c. **RG2 개구 폭(`rgwd`) 계산** — grasp에서 폭을 뽑아 그리퍼 명령으로.
   ~~아직 미구현~~ → **정정(2026-08-06 코드 감사)**: 알고리즘은 이미 있다
   (`corecode/GraspSelection/grasp_selector.py`, 442줄, 테스트 통과). 못 한 건 **배선**이다 —
   `grasp_bridge_node.py`가 이 파일을 안 쓰고 폭 계산 없는 자체 `select()`를 따로 쓴다.
   남은 일(import 교체 · `/grasp/best` width_m 추가 · `OnRobotRGOutput` 퍼블리시)은
   [[ws/cobot2/detect_graspx]] §7-10이 단일 출처.
0-d. **GraspMoE 분기별 점수 캘리브레이션 확인 — 로봇 없이 데이터만으로 된다 (2026-08-07 추가)**
   판별기가 확산 분기(`diff`)와 기하 분기(`obb`)를 같은 척도로 채점한다는 보장이 없다.
   (판별기는 데이터셋 음성·on-policy 음성으로도 학습되지만 — 소스 확인 — OBB 격자 후보가
   그 분포에 있는지는 미확인.)
   **최소 실험**: 순위 1·2위가 `obb`, 3·4위가 `diff`인 **순위 역전** 프레임을 골라, 상위 `obb`
   후보를 `corecode/GraspSelection/grasp_selector.py`의 재충돌 필터에 통과시킨다. 점수 0.9+인데
   충돌로 탈락하면 → **그 분기 점수를 못 믿는다는 증거**다. `branch_tags`는 이미 출력에 있다.
   근거·배경은 [[ws/cobot2/2026-08-07-nvblox-curobo-digest]] §9-2(E).
0-e. **`planning_options.replan`이 OMPL/cuMotion 각각에서 실제로 걸리는지 실기 검증 (2026-08-07 추가)**
   `pick_fsm.yaml`엔 `replan: true`로 켜져 있고 `pick_fsm/README.md:340`엔 "⚠️ 추론, 관측한 적
   없음"으로 적혀 있다. 구조적으로 의심되는 지점: move_group의 궤적-무효화 감시는 move_group
   자신의 `PlanningSceneMonitor`(octomap + collision object)를 보는데, cuMotion의 장애물은
   nvblox에서 계획 시점에 서비스로 당겨오는 것(`read_esdf_world:=True`)이라 **그 공유
   PlanningSceneMonitor에 안 들어가 있을 수 있다.** 맞다면 OMPL 경로는 replan이 걸리는데
   cuMotion 경로는 안 걸릴 수 있다 — 소스가 apt 바이너리라 로컬에서 확정 못 했다(2026-08-07).
   **최소 실험**: 실행 중(`APPROACH` 등) 궤적 경로에 손을 넣어 OMPL·cuMotion 각각 재계획되는지
   관찰. `testcommand.md` §12 "장애물이 궤적을 실제로 바꾸는지 미검증"과 같은 실기 세션에 묶어서 확인.

1. **README 4절 체크1~3 명령 실측** — `tf2_echo base_link camera_link`, `topic hz .../points`,
   `topic echo /dsr01/joint_states`는 **아직 한 번도 실행 안 됨**(README에 ⚠️ 미검증 표기해 둠).
   실기 켠 김에 돌려서 기대값과 대조하고 경고를 지운다.
2. **D435i depth rosbag 장면 2개 추가 녹화**(빈 테이블 / 장애물 여러 개) — 기존 4개(2026-08-04)는 검증 통과.
   ⚠️ **녹화 절차의 단일 출처는 [[ws/cobot2/rosbag-d435i]] §6, 재생 절차는 §3이다.** 아래 "출근 후 D435i 세션"에는 순서만 남겼다.
3. **캘리브 오차 정량 측정** — 알려진 좌표의 물체로 cm 단위. `padding_offset` 해석과 cuRobo 비교의 전제.
   - 🔴 **여기에 오차예산 판정이 걸려 있다 (2026-08-07 추가).** 계통오차는 TSDF 가중평균·베이즈
     갱신으로 **원리적으로 제거되지 않는다**(무작위만 상쇄된다). 합성 규칙은 무작위=RSS,
     계통·양자화=선형 합. 대입하면 `필요 마진 ≳ 40 mm(캘리브) + 25 mm(voxel/2) = 65 mm`인데
     현재 cuRobo `activation_distance`는 **25 mm**다 → **마진 부족 가능성.**
     ⚠️ **계산상의 결론이지 실측이 아니다.** 전제 2개가 미검증: ① 40 mm가 정말 bias인지
     (무작위 성분과 분리 안 됨) ② `activation_distance`가 유일한 마진인지(XRDF 구가 이미
     부풀려져 6쌍이 겹쳤으므로 실효 마진은 더 클 수 있다).
     **이 측정이 그 계산의 입력이다.** 근거·유도는 [[ws/cobot2/2026-08-07-nvblox-curobo-digest]] §9-1.
4. **OMPL 플래너 성공률·계획시간 로그** — 스프린트 Day3 P1. cuRobo 비교의 기준선.
5. **`planned_tcp_path_node` 실기 검증 (2026-08-10 추가)** — execute가 따라갈 TCP 경로를
   RViz에서 라인으로 보고 싶다는 요청으로 만듦. `src/pick_fsm/pick_fsm/planned_tcp_path_node.py`
   (신규), `/move_group/display_planned_path`(DisplayTrajectory) 구독 → 각 waypoint를
   `/compute_fk`로 풀어 `/pick/planned_tcp_path`(Marker LINE_STRIP) 발행. `moveit_bridge.py`/
   `task_manager.py`는 안 건드림. `colcon build --packages-select pick_fsm` PASS(2026-08-10,
   `rokey`), **실기 미검증**.
   - ⚠️ **가장 먼저 확인할 것**: `task_manager`가 `move_to_joints_async(plan_only=False)`로
     이동 명령을 보낼 때(현재 유일한 호출 방식) `/move_group/display_planned_path`가 실제로
     퍼블리시되는지. `ros2 topic hz /move_group/display_planned_path`를 이동 중에 같이 띄워서
     히트 찍히는지 확인 — move_group 소스 기반 추정이라 이 ws에서 실기로 검증된 적 없음.
     안 찍히면 이 노드는 빈 토픽만 구독하는 셈이라 접근 자체를 바꿔야 함(예: `moveit_bridge.py`를
     2단계 plan()/execute()로 바꿔 결과를 직접 받는 방식 — 단 이건 현재의 단일 액션 실행 흐름을
     바꾸는 것이라 실기 안전 검토 필요).
   - 확인되면: `ros2 run pick_fsm planned_tcp_path_node` 실행 → RViz `Marker` 디스플레이,
     Topic `/pick/planned_tcp_path` 추가해서 라인이 뜨는지, 라인 위 지점을 물리적으로 막았을 때
     replan(`opt.replan=true`, `moveit_bridge.py:222`)이 걸려 라인이 갱신되는지까지 시연으로 확인.
   - 실행 명령 상세는 [planned_tcp_path_node.py](../src/pick_fsm/pick_fsm/planned_tcp_path_node.py)
     docstring 참고.

**상시 실행** — ⚠️ **MoveIt octomap 경로에는 더 이상 필요 없다(2026-08-02).**
`sensors_3d.yaml`이 RealSense가 직접 발행하는 `/camera/camera/depth/color/points`를 쓰므로
`depth_image_proc`를 거치지 않는다. 아래 노드는 **RViz 육안 확인·nav2용 별도 클라우드**일 때만 띄운다.
(이 노드가 죽어도 MoveIt 충돌회피는 안 멈춘다 — 예전 설명이 틀렸다.)
```bash
export ROS_DOMAIN_ID=93
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz
```
카메라를 재기동하면 이 노드도 같이 죽는다. `ros2 topic list | grep points_xyz`로 먼저 확인할 것.

## 실기 안전 경고 (상시 유효)
> 완료된 작업 경위는 [[ws/cobot2/review_moveit]] §0으로 이관했다.
- ⚠️ **로봇 명령 경로가 두 개 살아 있다**: `dsr_controller2`(서비스 movej/movel → DRFL) 와
  `dsr_moveit_controller` (JTC → `Drfl.servoj_rt`/`Drfl.amovej`, `dsr_hw_interface2.cpp:494-503`).
  **동시에 명령하지 말 것.**
- ⚠️ `realsense-viewer`가 USB를 독점해 ROS 카메라 노드를 죽인다 — 증상이 "TF 프레임 없음"으로 나와 오진 유발. 뷰어 먼저 닫을 것.

## 출근 후 D435i 세션 (순서 고정)

**rosbag → 캘리브 순서로 한다.** 캘리브가 그날 제일 잘 깨지는 단계라, 먼저 하다 실패하면 빈손이 된다.
캘리브 결과는 static TF 6개라서 bag 재생 시 `static_transform_publisher`로 나중에 얹을 수 있다 — depth 데이터 자체는 캘리브와 무관하게 유효하다.

1. 카메라를 최종 위치에 **최대한 고정** + 마스킹테이프 표시 + 사진 (재현용)
2. **rosbag 녹화** ← 여기서 개인PC 작업이 풀린다
3. **eye-to-hand 캘리브** — 1~3 사이에 카메라를 건드리지 않는다 (건드리면 bag과 짝이 안 맞음)
4. 결과 `T_cam2base.npy`는 **사본을 만들지 않는다** — 정본은 `corecode/Calibration_Tutorial/`이고
   `m0609_rg2_bringup/config/`는 symlink다. (~~`config/handeye/`에 복사해 커밋~~ 은 폐기된 지시다.)

> ⚠️ **녹화 런치·토픽 목록·재생 절차는 여기 두지 않는다. 단일 출처는 [[ws/cobot2/rosbag-d435i]] §6(재녹화 절차), 재생은 §3이다.**
> 예전 이 자리에 있던 `rs_align_depth_launch.py` 기반 명령은 **폐기됐다** — 그 런치엔 `camera_calib_tf`가
> 없어 `base_link→camera_link` 없는 bag 4.8GB가 나왔다. 현행은 `camera.launch.py`(= `reals` alias)다.
> 녹화 중 **임시 static TF를 띄우지 말 것**(bag의 `/tf_static`에 가짜 값이 박힌다)만 여기 남긴다.

> 문서 위치 규칙은 [[ws/cobot2/README]] "문서를 쓸 때"가 단일 출처다(중복 삭제).
> 실기 실측 사실(doosan-robot2 launch 기본값, D435i 토픽 네임스페이스 등)은 [[ws/cobot2/context/constraints]]로 이관했다.
