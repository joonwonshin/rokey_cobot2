<!-- meta
updated: 2026-08-09
status:  live — 런치 7개의 인자 기본값 39개는 소스와 1:1 대조 완료(cross-review 재검증).
         단 5절 cuMotion 쪽 서술 일부는 config/README.md 에서 옮겨온 것이고 이 문서를
         쓰면서 해당 yaml 을 직접 열어 확인하지는 않았다.
         **실기 동작은 전부 미검증** — 이 세션은 실기 랩탑(`rokey`)이 아닌 개인PC에서 돌았다.
         2026-08-09: §3 `run_bridge`/`seg_source` 기본값이 소스와 반대로 적혀 있던 것을
         발견·정정(ws README 재검토 중 대조에서 드러남 — README §0 "문서 간 대조" 참고).
         "소스와 1:1 대조 완료"였던 2026-08-08 검증이 이 두 값은 놓쳤다는 뜻이다.
owns:    ws 안 모든 런치파일의 인자 지도 · 인자 vs config 파일의 경계
-->

# 런치 인자 · config 지도

`src/` 에서 **이 ws가 직접 쓴 런치 7개**만 다룬다. `doosan-robot2/`, `onrobot-ros2/` 는
업스트림 vendored 라 건드리지 않는다(`dsr_bringup2/*`, `dsr_moveit_config_*` 등은 이 ws의
실행 경로가 아니다 — README "알려진 함정" 참고).

**원칙**: *실행할 때마다 바뀌는 것*만 런치 인자다. *한 번 정하면 유지되는 값*(하드웨어 IP,
보정값, 플래너 튜닝)은 config yaml 이 정본이고 런치는 경로만 안다.

> ⚠️ **yaml 을 고친 뒤 rebuild 가 필요한지는 패키지 빌드타입에 달렸다** (2026-08-08 `ls -l` 실측).
> - `ament_cmake` + `install(DIRECTORY config)` — `m0609_rg2_bringup`, `m0609_rg2_moveit`:
>   share 가 src 로의 **심볼릭 링크**다. 고치면 즉시 반영, rebuild 불필요.
> - `ament_python` — `pick_fsm`, `cumotion`: share 가 `build/<pkg>/config/` 로의 링크라
>   **`colcon build` 를 다시 돌려야** src 수정이 넘어간다.
>
> CLAUDE.md §4 의 "yaml 은 복사본이다" 규칙은 후자(ament_python)에서 나온 것이다. 앞의
> 두 패키지에 그대로 적용하면 없는 문제를 쫓게 된다.

---

## 1. `m0609_rg2_bringup` — 로봇 + 그리퍼

### `bringup.launch.py` — [터미널 1] 실기 주 경로

| 인자 | 기본값 | 설명 |
|---|---|---|
| `mode` | `virtual` | `real` \| `virtual`. virtual 은 Docker DRCF 에뮬레이터를 자동 기동 |
| `host` | `127.0.0.10` | **실기는 반드시 `192.168.1.100`.** 기본값은 에뮬레이터용 루프백 |
| `port` | `12345` | DRCF 포트 |
| `rviz` | `true` | bringup RViz(`default.rviz`). **moveit 과 함께 쓰면 `false`** — RViz 2개가 octomap 을 각각 렌더해 CPU 를 갉는다(2026-08-05 실측 21%+15%) |

**config 로 뺀 것** — `config/rg2_driver.yaml` (OnRobot RG2 드라이버, `mode:=real` 에서만 로드)

| 키 | 값 | 성격 |
|---|---|---|
| `/onrobot/ip` | `192.168.1.1` | Compute Box IP. **로봇(192.168.1.100)과 다른 장비** |
| `/onrobot/port` | `502` | Modbus/TCP |
| `/onrobot/changer_addr` | `65` | Quick Changer 슬레이브 주소 |
| `/onrobot/gripper` | `rg2` | rg2 \| rg6. 최대 개구·힘 한계가 달라진다 |
| `/onrobot/offset` | `5` | 🔴 **modbus 경로에서 아무 효과가 없다** — 선언 후 다시 안 쓰이고, 장비가 보고하는 `gfof` 로 덮인다. 손끝 길이의 정본은 `pick_fsm/rg2.py` 실측표. 여기를 튜닝하지 말 것 |

**일부러 인자로 빼지 않은 것** (이 ws 에서 바뀌지 않는 값 — 인자로 만들면 틀릴 자유만 는다)
`dsr01`(네임스페이스), `m0609`(모델), `UPDATE_RATE=100`, `world→base_link` static TF,
컨트롤러 파라미터(`dsr_controller2/config/dsr_controller2.yaml`, 업스트림 소유).

### `camera.launch.py` — [터미널 2] RealSense + 캘리브 TF

| 인자 | 기본값 | 설명 |
|---|---|---|
| `driver` | `true` | `false` 면 static TF 만. 캘리브 미세보정 반복에 쓴다 |
| `dxyz` | `0 0 0` | 평행이동 보정 `"x y z"` (m, `base_link` 축) |
| `drpy` | `0 0 0` | 회전 보정 `"roll pitch yaw"` (deg, `camera_link` 축). 테이블이 기울어 보이면 pitch 부터 |
| `depth_profile` | `424x240x15` | 낮게 잡은 값. 이유는 **MoveIt octomap updater 가 단일 스레드**라서다 — 코어·GPU 를 늘려도 콜백 하나의 처리 시간은 안 줄어든다 |
| `color_profile` | `424x240x15` | `align_depth` 가 이 해상도를 따라간다 — depth 만 낮추면 의미 없다 |

**config**: `config/T_cam2base.npy` (캘리브 결과. 재캘리브 시 npy 만 교체, rebuild 불필요).
TF 값을 런치에 하드코딩하지 않는 것이 이 런치의 설계 의도다.

### `bringup_camera.launch.py` — **현재 리그에서 쓰지 않는다**

eye-**in**-hand(카메라를 tool0 에 부착) 전용 변형. 지금 D435i 는 eye-to-hand 라
`bringup.launch.py` + `camera.launch.py` 조합이 맞다. 인자는 `mode`/`host`/`port`/`camera`.

> 🔴 이 파일은 `bringup.launch.py` 의 복사본이고 **이미 어긋나 있다**: `rviz` 인자가 없고,
> `joint_state_publisher` 에 `publish_default_velocities: True` 가 빠져 있다(cuMotion 전제조건).
> **정본은 `bringup.launch.py` 다.** eye-in-hand 로 전환할 때 이 파일을 쓰려면 먼저 저쪽과
> diff 를 떠서 그동안 밀린 수정을 옮겨오고 시작할 것 — 지금 상태 그대로 띄우면 안 된다.
> (2026-08-08 사용자 판단으로 삭제하지 않고 남긴다. `rg2_driver.yaml` 은 두 파일이 공유한다.)

---

## 2. `m0609_rg2_moveit` — `moveit.launch.py` [터미널 3]

| 인자 | 기본값 | 설명 |
|---|---|---|
| `standalone` | `true` | **bringup 위에 얹을 땐 `false` 필수.** true 면 rsp/jsp/static_tf 를 자기가 띄워 실기 관절값을 시뮬값이 덮는다(에러 없이) |
| `rviz` | `true` | MoveIt RViz(MotionPlanning 패널). bringup RViz 를 쓸 거면 false |
| `octomap` | `true` | RealSense 클라우드 → 3D 장애물. false 면 RViz 로 놓은 장애물만(알고리즘 디버깅) |
| `use_sim_time` | `false` | `ros2 bag play --clock` 과 한 짝. **실기는 false 고정** |
| `cumotion` | `false` | 두 번째 플래닝 파이프라인 등록. Isaac ROS 컨테이너 전용 + `cumotion_planner_node` 가 따로 떠 있어야 한다 |

`standalone:=false` 일 때만 `dsr_moveit_controller` spawner 가 뜬다 — Execute 에 필요하다.

**config (전부 `m0609_rg2_moveit/config/`)**

| 파일 | 내용 | 자주 만지는 값 |
|---|---|---|
| `joint_limits.yaml` | 관절 제한 + **충돌 여유** | `default_object_padding`(0.02), `default_robot_padding`(0.0), `default_*_scaling_factor` |
| `sensors_3d.yaml` | octomap updater | `octomap_frame`(**`base_link`**, `world` 아님), `octomap_resolution`, `max_update_rate`, `point_subsample` |
| `kinematics.yaml` | IK 솔버 | |
| `ompl_planning.yaml` | 플래너 | |
| `moveit_controllers.yaml` | 컨트롤러 이름 | 이름은 `dsr_controller2.yaml` 의 블록명과 **한 짝**. 한쪽만 바꾸면 Execute 만 조용히 죽는다 |

> `default_object_padding`/`default_robot_padding` 은 2026-08-08 에 `moveit.launch.py` 에서
> `joint_limits.yaml` 로 옮겼다. 런치는 이제 yaml 을 그대로 얹기만 한다.

---

## 3. `graspgenx_perception` — `graspx.launch.py`

config yaml 이 없다 — **전부 런치 인자다.** 실행마다 바뀌는 값(대상 클래스, 디바이스)이
대부분이라 정본을 따로 둘 이유가 없다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `run_yolo` / `run_bridge` | **`true`** / **`false`** | **한 머신에서 둘 다 true 금지.** yolo=컨테이너, bridge=호스트. 기본값 그대로 두면 컨테이너에서 YOLO만 뜨고, 호스트에서 bridge를 쓰려면 `run_yolo:=false run_bridge:=true`를 **둘 다** 명시해야 한다 (2026-08-09 소스 `graspx.launch.py` `ARGS`와 대조해 정정 — 이 문서가 이전엔 `true`/`true`로 잘못 적어 뒀었다) |
| `seg_source` | **`yolo`** | `geometric`(신경망 0개) \| `yolo`(2026-08-08부터 **기본값**, `target_classes`로 대상 좁히기 가능) — 이전엔 `geometric`이 기본이라고 잘못 적혀 있었다 |
| `image_topic` | `/camera/camera/color/image_raw` | |
| `device` / `conf` | `0` / `0.1` | GPU 인덱스 \| `cpu` / YOLO 신뢰도 |
| `classes` | `[]` | COCO **인덱스** 필터(탐지 대상, 넓게). banana46 apple47 cup41 bottle39 |
| `target_classes` | `''` | 클래스 **이름**(파지 대상, 좁게). 좁히면 grasp 연산이 실제로 줄어든다 |
| `min_pixels` | `300` | |
| `obj_max_h` | `0.12` | geometric 전용 self-filter. 안 걸면 그리퍼가 `obj_1` 로 잡힌다(2026-08-08 실측) |
| `publish_overlay` / `out_dir` | `true` / `''` | |

---

## 4. `pick_fsm` — `pick_fsm.launch.py` [상태머신]

**값의 정본은 `config/pick_fsm.yaml`(60여 개).** 런치 인자는 그 위에 덮는 *안전 스위치*만이다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| ~~`dry_run`~~ | — | **2026-08-09 제거.** 이 FSM 은 항상 실기를 움직인다. 🔴 옛 인자를 붙여도 **경고 없이 무시된다** — `dry_run:=true` 로 안전하다고 착각하지 말 것 |
| `require_approval` | `true` | `/pick/approve` 없이는 APPROACH 로 못 간다. **남은 유일한 소프트 안전장치** |
| `voice` / `target` | `true` / `''` | voice:=false 면 `target` 을 쓴다. **voice:=true 면 `/get_keyword` 를 제공하는 노드가 있어야 한다** → 아래 4-1 |
| `grasp_source` | `legacy_trigger` | `legacy_trigger` \| `compute_grasp` \| `manual`. ⚠️ 2026-08-09 에 기본값이 `compute_grasp` → `legacy_trigger` 로 바뀌었다(그 서버는 이 ws 에 없다). 이 표가 옛 값을 적고 있었다 |
| `planning_pipeline` | `ompl` | move_group 에 그 파이프라인이 떠 있어야 한다 |
| `gripper_backend` | `real` | `real` \| `virtual`. 숫자 명령의 **단위가 다르다** |
| `robot_ns` / `log_level` / `params_file` | `dsr01` / `info` / (share) | |

yaml 쪽에서 실기 전에 반드시 보는 값: `vel_scale`/`acc_scale`(0.05), `home_joints_deg`,
`place_joints_deg`, `grasp_standoff_m`, `force_down_steps`, `max_grip_width_m`.
UNVERIFIED 표시가 붙은 값은 지우지 말 것 — 실측 안 된 값이라는 뜻이다.

---

## 4-1. `voice_processing` — `vla_command.launch.py` [지시 입력]

외부 VLA(또는 음성)의 "이걸 집어라"(JSON)를 `pick_fsm` 의 기존 `/get_keyword` 자리로 넘긴다.
같은 채널로 rqt 패널의 **시작·중단·리셋** 버튼도 대신할 수 있다(`cmd:"start"/"abort"/"reset"`,
`voice` 값과 무관하게 항상 동작). **`승인` 버튼만은 명령어 자체가 없다** — `cmd:"approve"`는
코드 경로가 없어 무조건 거부된다.

pick 지시(`cmd:"pick"`)를 쓰려면 `pick_fsm` 을 `voice:=true`(기본)로 띄워야 한다 —
`voice:=false` 면 `LISTENING` 을 건너뛰어 `/get_keyword` 를 아무도 안 부른다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `auto_start` | `false` | `pick` 지시가 오면 이 노드가 **덧붙여서** `/pick/start` 도 부른다. `cmd:"start"` 를 명시적으로 보내는 것과 별개 |
| `pixel_policy` | `warn` | `warn` \| `reject`. 🔴 `pixel`(개체 지정)은 아직 선정에 안 쓰인다. **같은 클래스 물체가 2개 이상이면 `reject`** |
| `ttl_sec` | `10.0` | `pick` 지시 유효시간. **받은 시각** 기준(송신 `stamp_ns` 아님 — 두 PC 시계가 안 맞는다) |
| `wait_timeout_sec` | `50.0` | `/get_keyword` 를 붙잡고 기다릴 시간. ⚠️ `fsm_listening_timeout_sec - listening_margin_sec` 보다 커지면 **노드가 기동 때 죽는다**(의도된 것 — 안 죽으면 매 사이클 ABORT) |
| `fsm_listening_timeout_sec` / `listening_margin_sec` | `60.0` / `5.0` | 전자는 `task_manager.DEFAULT_TIMEOUTS[State.LISTENING]` 의 사본(정본이 바뀌면 같이 바꾼다), 후자는 서비스 탐색 여유 |
| `allowed_classes` | `config/objects.yaml` 의 `detect` | 콤마 목록. 밖의 클래스는 즉시 거부 |
| `command_topic` / `result_topic` / `keyword_service` | `/vla/pick_command` / `/vla/pick_result` / `/get_keyword` | |
| `start_service` / `abort_service` / `reset_service` | `/pick/start` / `/pick/abort` / `/pick/reset` | `cmd:"start"/"abort"/"reset"` 이 부르는 서비스 이름 |

⚠️ **마이크 노드(`voice_processing get_keyword`)와 동시에 띄우지 않는다** — 둘 다
`/get_keyword` 를 제공해서 어느 쪽이 답할지 알 수 없다.

스키마·결과 계약·검증 상태는 [`src/PACKAGES.md#voice_processing`](../src/PACKAGES.md#voice_processing).

---

## 5. `cumotion` — `dynamic_avoid.launch.py` (옵션 경로)

**⚠️ `mode:=check` 외에는 로봇이 실제로 움직인다.**

| 인자 | 기본값 | 설명 |
|---|---|---|
| `mode` | `check` | `check`(안 움직임) \| `joint` \| `pose` \| `pingpong` |
| `vel` | `0.15` | `cumotion_planner.yaml` 이 `override_moveit_scaling_factors:false` 라 이 값이 실제 속도다 |
| `replan_hz` | `3.0` | 세그멘터가 3.7 Hz 라 그 위는 새 정보가 없다 |
| `lookahead` | `0.35` | 계획시간(~0.2s)+handover 보다 커야 한다 |
| `static` | `false` | true = 재계획 끄기(대조군) |
| `pipeline` | `isaac_ros_cumotion` | \| `ompl` |
| `goal_joint_deg` | `[0.0, 0.0, 90.0, 0.0, 90.0, 0.0]` | ⚠️ **소수점을 빼고 복붙하지 말 것** — rcl 이 리스트 안 int/float 혼합을 거부한다 |
| `goal_pose` | `[0.45, 0.0, 0.35, 180.0, 0.0, 0.0]` | x y z(m) roll pitch yaw(deg). 위와 같은 함정 |

나머지는 전부 `src/cumotion/config/dynamic_avoid.yaml`.

### ws 루트 `config/` — 런치가 아니라 `--params-file` 로 직접 주는 것

`cumotion_segmenter.yaml`(T4), `nvblox_realtime.yaml`(T5), `cumotion_planner.yaml`(T6),
`moveit_sensors_3d.yaml`(T7, 심볼릭 링크). 결합 규칙은 `src/PACKAGES.md`(cumotion 절, "config
파일" 소절)와 실행 명령은 `config/testcommand.md` 가 단일 출처다. ⚠️ nvblox 파라미터는
노드 생성 시 1회만 읽는다 —
`ros2 param set` 으로 안 바뀐다.
