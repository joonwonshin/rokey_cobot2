# 2026-08-09 실기 검증 — cuMotion 재계획 트리거 + VRAM 배분

2026-08-08 세션(개인PC `kimkh-17U70N-GA70K`, GPU 없음)에서 **소스만 읽고** 세운 가설들의
검증 계획. 여기 적힌 예상은 전부 `⚠️ 미검증` 이다 — 실기에서 뒤집히면 예상 쪽을 고친다.

**전제**: `rokey` 머신(RTX 4060 8GB). T1~T7 은 `config/testcommand.md` 순서 그대로.

---

## 0. 배경 — 이 세션에서 소스로 확인한 것 (재확인 불필요)

| 사실 | 근거 (직접 읽음) |
|---|---|
| pick_fsm 은 재계획 로직을 자기가 갖고 있지 않다. `opt.replan` 을 켜서 move_group 에 맡긴다 | `moveit_bridge.py:216-227` |
| 정지 판정 주체는 move_group 의 PlanExecution 하나뿐. 감시 대상은 `planning_scene_monitor_` | `/opt/ros/humble/include/moveit/plan_execution/plan_execution.h:137-138,147-149` |
| cuMotion 플래너는 `scene.world.collision_objects` **만** 읽는다. `world.octomap` 도 `robot_state.attached_collision_objects` 도 안 읽는다 (`attached` grep 0건) | `cumotion_planner.py:594,662-665` |
| 플러그인은 현재 씬 전체를 diff 로 실어 보낸다 → `pick_target` 구는 넘어간다 | `cumotion_move_group_client.cpp:72,81` |
| GraspGenX 워커는 별도 프로세스이고, 한 번 뜨면 노드 shutdown 까지 안 죽는다 | `grasp_bridge_node.py:164-165, 352-357` |
| 워커에 넘기는 인자는 `--gripper` 하나뿐. VRAM 관련 값은 워커 기본값에 박혀 있다 | `grasp_bridge_node.py:166-167`, `graspgen_worker.py:69-76` |

## 0-1. 이미 반영한 변경

- `sensors_3d.yaml:23` `octomap_resolution: 0.1 → 0.05` (2026-08-08).
  RViz 에서 복셀이 두껍게 보이던 원인.

### 🔴 이 변경이 컨테이너 move_group 에 실제로 도달하는지부터 확인한다

**T7 move_group 은 컨테이너 안에서 뜬다** (2026-08-09 사용자 확인, `testcommand.md:305`).
컨테이너는 **install 트리가 따로다** — `install/` 이 아니라 `install_container/`
(`scripts/container_setup.sh:60`, `testcommand.md:230`).

```
호스트  /home/kimkh/cobot2_ws   →  컨테이너  /workspaces/cobot2_ws   (바인드 마운트)
호스트  install/                   컨테이너  install_container/
```

- 마운트 경로가 다르니 트리를 분리해 둔 것이고, 컨테이너 안에서 빌드했다면
  심볼릭 링크는 `/workspaces/cobot2_ws/src/...` 를 가리켜 정상 동작한다.
- ⚠️ **문제는 `install_container/` 가 `--symlink-install` 로 빌드됐느냐다.**
  아니면 `sensors_3d.yaml` 이 **복사본**이라 어제 고친 0.05 가 **안 넘어간다.**
  CLAUDE.md 4절의 "ament_cmake 는 share 가 src 링크라 즉시 반영" 은 호스트 `install/` 에서
  `ls -l` 로 확인한 사실이고, `install_container/` 에서는 **확인된 적이 없다.**
  (개인PC 엔 `install_container/` 자체가 없어서 여기서 볼 수 없었다)

**실기 첫 단계 — 컨테이너 셸에서:**
```bash
ls -l /workspaces/cobot2_ws/install_container/share/m0609_rg2_moveit/config/sensors_3d.yaml
grep octomap_resolution /workspaces/cobot2_ws/install_container/share/m0609_rg2_moveit/config/sensors_3d.yaml
```
- [ ] 심볼릭 링크이고 `0.05` 가 보이면 → 그대로 진행
- [ ] 복사본이고 `0.1` 이면 → **컨테이너 안에서** 재빌드 후 진행:
      `colcon build --symlink-install --build-base build_container --install-base install_container --packages-select m0609_rg2_moveit`
      ⚠️ 이 명령은 미검증이다. `install_container/` 를 원래 어떤 명령으로 만들었는지 기록이 repo 에 없다
      (`grep -rn "install_container"` → `container_setup.sh:60`, `testcommand.md:230` 두 곳뿐, 빌드 명령 없음).
      실제로 쓴 명령을 알아내면 `config/testcommand.md` 에 적어둘 것 — 다음 사람이 또 찾는다.

- [ ] move_group 기동 로그에 `No 3D sensor plugin(s) defined for octomap updates` 가 **없는지** 확인
      (뜨면 `moveit.launch.py:17` 이 `except EnvironmentError: return None` 으로 삼키고
       `:159` 의 `or {}` 가 빈 dict → `sensors` 키가 사라진 것. B·D 절이 통째로 헛측정이 된다)
- [ ] `ros2 topic hz /moveit/filtered_cloud` 가 나오는지 (이전 실측 2.3 Hz)

---

## A. VRAM — 이걸 먼저 한다 (A 가 안 풀리면 B~D 를 못 돌린다)

### A-1. 프로세스별 분리 측정 ⚠️ 미검증

"각각 3.5 GB" 는 총량 눈대중이다. GPU 프로세스는 2개가 아니라 **최대 5개**이고,
**호스트와 컨테이너에 흩어져 있다** (`pick_fsm/README.md` §2 기동 순서 기준):

| 어디서 | 프로세스 | 근거 |
|---|---|---|
| 컨테이너 | `nvblox_node` (T5) | `testcommand.md` §5 |
| 컨테이너 | `cumotion_planner_node` (T6) | `testcommand.md` §6 |
| 컨테이너 | `yolo_seg_node` | README §2 3.5 — `scripts/graspx_container.sh` |
| **호스트** | `grasp_bridge_node` → `graspgen_worker` (uv venv 자식) | README §2 4 — "**호스트에서** 띄운다" |

> 🔴 **`nvidia-smi` 는 호스트에서 돌린다.** 컨테이너 안에서 돌리면 호스트 PID 를 못 보거나
> 프로세스명이 안 풀려서 절반만 보인다. 위 표가 섞여 있는 게 바로 그 이유다.

기동 순서(README §2)대로 **노드 하나 띄울 때마다** 호스트에서:

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

| 띄운 뒤 | used_memory | 누적 |
|---|---|---|
| nvblox_node | | |
| cumotion_planner_node | | |
| yolo_seg_node | | |
| graspgen_worker | | |
| **합계 / 8192 MiB** | | |

> 각 프로세스는 CUDA context 만으로 300~600 MB 를 먹는다. **프로세스 수 자체가 비용**이다.

### A-2. 실패 증상 확인 ⚠️ 미검증

GraspGenX 가 안 뜰 때 **로그에 `CUDA out of memory` 가 실제로 찍히는가?**

- [ ] 찍힌다 → A-3 으로
- [ ] 안 찍힌다 → **원인이 VRAM 이 아니다.** A-3 이하 전부 무의미. stderr 전문을 보고 다시 판단
  (워커 stderr 는 `stderr=None` 이라 브리지 노드 콘솔에 그대로 나온다 — `grasp_bridge_node.py:171`)

### A-3. 손잡이 — 위에서부터 하나씩, 한 번에 하나만 ⚠️ 전부 미검증

**cuMotion (T6 재기동만, 코드 수정 0).** 현재 T6 명령엔 메모리 인자가 하나도 없다 = 전부 기본값.

**한 번에 하나만 바꾼다.** 제일 큰 손잡이 하나부터:

```bash
# 기존 T6 명령 그대로 + 이 한 줄만 추가
  -p num_trajopt_seeds:=2
```

| 순서 | 인자 | 기본 | 시도값 | 근거 | 절감(MiB) |
|---|---|---|---|---|---|
| 1 | `num_trajopt_seeds` | 6 (`cumotion_planner.py:67`) | 2 | 롤아웃 배치 = `seeds × num_trajopt_time_steps(32)`. 여기가 제일 크다 | |
| 2 | `num_graph_seeds` | 6 (68행) | 2 | 위와 같음 | |
| 3 | `voxel_size` | 0.05 (75행) | 0.08 | ESDF 격자. `grid_size_m` [2,2,2] 와 짝 | |
| 4 | `grid_size_m` | [2,2,2] (88행) | 작업영역 | 작업영역은 x 0~0.7 / y ±0.3 인데 2 m³ 를 잡고 있다 | |

> 🔴 **seeds 를 줄이면 계획 성공률·계획시간이 나빠진다.** VRAM 만 보는 손잡이가 아니다.
> 줄인 상태로 B 절을 돌리면 "cuMotion 이 우회 경로를 못 냈다"가 지도 불일치 때문인지
> seed 부족 때문인지 구분이 안 된다 → **B 절 기준선은 반드시 기본값(6)으로 먼저 잡는다.**

**표에서 뺀 것 (2026-08-08 cross-review 지적, 근거 확인함)**
- `publish_curobo_world_as_voxels:=False` — ❌ **절감이 0 에 가깝다.** `max_publish_voxels: 500000` 은
  선할당이 아니라 발행 직전 잘라내기 상한이고, 발행 블록 전체가 `get_subscription_count() > 0` 으로
  게이트된다(`cumotion_planner.py:622-623`). RViz 를 안 붙이면 애초에 아무 일도 안 한다.
  게다가 이걸 끄면 B-2 판정의 **유일한 증거인 `/curobo/voxels` 를 버리게 된다.** 켜 둔다.
- `collision_cache_cuboid/mesh 20→4/1` — ❌ 몇 개 float 짜리라 절감 사실상 0. 나중에 물체를
  하나 더 놓으면 조용히 넘치는 대가만 남는다.

**GraspGenX.** 손잡이가 ROS 파라미터로 안 나와 있다. 줄이려면 `graspgen_worker.py` 기본값을 직접 고친다:

- `--num_grasps 64 → 32` (69행. 주석이 **"8GB VRAM 기준"** = 혼자 쓸 때 기준으로 이미 튜닝된 값)
- `--max_scene_points 8192 → 4096` (75행)

**공짜 하나.** `grasp_bridge_node.py` 의 `Popen` env 에:
```
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```
8 GB 가 꽉 찬 상태의 OOM 은 총량보다 **조각화**로 나는 경우가 많다.

**그래도 안 되면 (마지막 수단).** pick_fsm 은 순차다 — ComputeGrasp(GPU) 가 끝난 **뒤에** 모션(GPU)이 시작되므로
둘이 동시에 상주할 이유가 없다. 워커는 별도 프로세스라 **종료 = VRAM 전량 반환**.
`ask_worker()` 응답 후 종료하는 파라미터 하나가 최소 diff. 대가는 pick 마다 "모델 로딩 수십 초"(168행 자기 문구).
👉 **A-3 상단 손잡이로 해결되면 이건 하지 않는다.**

---

## B. 정지가 걸리고, 그 재계획이 **현재 파이프라인**으로 전달되는가

> **검증 목표 (2026-08-08 사용자 확정)**: octomap·nvblox 는 **둘 다 켠다.** 어느 쪽이 정지를
> 유발했는지는 목표가 아니다. 확인할 것은 두 가지뿐 —
> ① 모션 중 장애물에 **멈추는가**, ② 멈춘 뒤 재계획이 **그때 켜둔 파이프라인으로 나가서 다른 경로를 내놓는가**.
>
> (octomap 을 끄는 대조 실험은 **하지 않는다.** 대신 B-0 의 로그로 트리거 주체를 비파괴로 읽는다.)

두 실험 다 **같은 물체·같은 목표**로, 모션 중에 **같은 자리에 손을 넣는다.**
`vel_scale: 0.05` 그대로. 손은 로봇 경로 위, 그리퍼 진행 방향 앞쪽.

### B-0. 무엇을 보고 "정지가 걸렸다"고 판정할지 (두 실험 공통)

멈추는 걸 눈으로 보는 것으로는 부족하다 — 계획 실패로 안 움직이는 것과 구분이 안 된다.
**move_group 콘솔**에 경로 무효화 메시지가 찍히는지를 본다. 그 메시지가
`PlanExecution` → `planning_scene_monitor_` 경로가 동작했다는 증거다.

- [ ] move_group 콘솔의 해당 줄 원문을 그대로 적을 것: `________________________`
  - ⚠️ 정확한 문구는 확인 못 했다(`plan_execution.cpp` 소스가 로컬에 없다. 헤더만 있음).
    "invalid" / "became invalid" / "Stopping execution" 근처를 찾는다
- [ ] 그 줄이 **안 나오는데** 로봇이 멈췄다면 → 정지가 아니라 **계획 실패**다. B 절 판정을 그쪽으로 돌린다

> ⚠️ **이 로그로 "무엇이" 경로를 무효화했는지까지는 알 수 없다.** planning scene 에는 octomap 말고도
> pick_fsm 이 직접 넣은 `pick_target` 구와 attach 된 물체가 같이 들어 있다
> (`task_manager.py:111,526,733`). 이 로그가 판정하는 건 **"planning scene 경유로 정지가 걸렸다"** 까지다.
> 그거면 이번 검증 목표엔 충분하다 — 어느 지도가 유발했는지는 목표가 아니다.

### B-1. OMPL + octomap + nvblox (기준선)

기동 순서는 `src/pick_fsm/README.md` §2 가 정본이다. **거기서 두 군데만 다르게 한다:**

```bash
# README §2 의 2번 — cumotion:=true 를 붙인다
#   🔴 README 133행에는 이 인자가 없고, moveit.launch.py 의 cumotion 기본값은 false 다
#      (`moveit.launch.py:51`, `testcommand.md:124`). 안 붙이면 B-2 에서
#      "그 파이프라인이 move_group 에 없다"로 goal 이 거부된다
# T7 (컨테이너). 이후 B-2 까지 이것만 띄워두고 FSM 만 재기동한다
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true

# README §2 의 5번 — 호스트. voice 를 끈다 (dry_run 은 2026-08-09 제거돼 인자가 없다)
ros2 launch pick_fsm pick_fsm.launch.py \
  planning_pipeline:=ompl \
  grasp_source:=legacy_trigger voice:=false target:=apple
```
> 1·3·3.5·4번(bringup / 카메라 / YOLO 컨테이너 / grasp_bridge 호스트)은 README 그대로.
> `require_approval` 은 기본 `true` 라 매 사이클 `/pick/approve` 가 필요하다 — 그대로 둔다.
> 🔴 ~~`dry_run:=false` 를 빠뜨리면 실험이 성립하지 않는다~~ → **2026-08-09 `dry_run` 제거.**
> 이제 FSM 은 항상 실제로 움직이므로 이 실험의 전제가 기본으로 충족된다.
> 🔴 옛 인자를 붙여도 **경고 없이 무시된다**(2026-08-09 실측) — `dry_run:=true` 를 붙여
> "안전 모드로 돌렸다"고 착각하는 게 이 변경의 유일한 새 위험이다.
> `voice:=false` 도 필수 — 기본값 `true` 인데 `voice_processing` 이 `COLCON_IGNORE` 라
> `/get_keyword` 가 없어서 FSM 이 그 앞에서 멈춘다. `config/testcommand.md:92-94` 와 같은 인자 조합이다.
>
> `cumotion:=true` 로 띄우되 FSM 은 `ompl` 을 쓴다 — 파이프라인만 바꿔 비교하려고 **T7 은 한 번만 띄운다.**
> (`pick_fsm.launch.py` 는 `task_manager` + `robot_safety_node` 만 띄우고 move_group 을 include 하지 않는다 — 확인함)
>
> 경로 A(컨테이너 T4 segmenter → T5 nvblox → T6 cumotion_planner)와 octomap 이 **동시에** 살아 있어야 한다.

- [ ] 멈추는가? **예상: 멈춘다** (기존 실기 검증대로)
- [ ] B-0 의 로그 줄이 나오는가?
- [ ] 멈춘 뒤 **다른 경로로 재개하는가?** **예상: 한다** — OMPL 과 octomap 은 같은 지도를 본다
- 멈추기까지 체감 지연: ____ 초

### B-2. cuMotion + octomap + nvblox ← 이번 검증의 본체

T7 은 그대로 두고 FSM 만 재기동:

```bash
ros2 launch pick_fsm pick_fsm.launch.py \
  planning_pipeline:=isaac_ros_cumotion \
  grasp_source:=legacy_trigger voice:=false target:=apple
```
- [ ] 멈추는가? **예상: 멈춘다** (트리거는 octomap 이 낸다)
- [ ] B-0 의 로그 줄이 나오는가?
- [ ] 멈춘 뒤 **cuMotion 이 다른 경로를 내놓는가, 같은 경로를 다시 내놓는가?** ← **여기가 이번 검증의 전부**
- [ ] FSM 로그의 실패 코드: ____ (**예상: 항상 `-1`**. `cumotion_interface.cpp` 가 진짜 코드를 `PLANNING_FAILED` 로 덮어쓴다)
  → 진짜 원인은 **T6 콘솔의 `Motion planning failed wih status:` 줄에만** 있다. 그것도 같이 적을 것

**판정** (B-1 은 "멈추고 다른 경로로 재개"가 나온다는 전제. 아니면 B-1 부터 다시)

| B-2 결과 | 결론 | 다음 |
|---|---|---|
| 멈춤 + **다른 경로** | ✅ 목표 달성. 트리거(octomap)와 재계획(cuMotion)이 실제로 맞물린다 | 그대로 쓴다 |
| 멈춤 + **같은 경로 반복** | ⚠️ 트리거는 갔는데 cuMotion 이 그 장애물을 못 본다. 두 지도가 어긋난 것 | **B-3 (타이밍)** 으로 |
| 안 멈춤 | 🔴 **원인 미확정** | 아래 배제 순서를 밟는다 |

> **"안 멈춤"에서 결론으로 직행하지 말 것.** `PlanExecution` 은 생성자가
> `(node, planning_scene_monitor, trajectory_execution)` 만 받는다(`plan_execution.h:97-99`) —
> **어느 파이프라인이 궤적을 만들었는지 알지 못한다.** 감시는 플래너가 아니라 실행에 붙는다.
> 따라서 "cuMotion 이라서 감시가 안 붙었다" 는 성립하지 않는다. 배제할 후보:
> ① B-1 에서는 멈췄나 (아니면 octomap 이 손을 애초에 못 잡은 것) →
> ② `/moveit/filtered_cloud` 에 손이 찍히나 (`padding_scale: 2.5` 가 지워버렸을 수 있다) →
> ③ 손을 넣은 시점에 로봇이 이미 그 구간을 지났나 → ④ 그래도 남으면 그때 가설을 다시 쓴다

> "같은 경로 반복" 이면 정지·재개를 되풀이하다 `replan_attempts: 3` 소진 →
> `_motion_failed` → `motion_retries: 2` → NEXT_CANDIDATE 로 떨어진다. 그 흐름이 로그에 보이는지도 확인.

### B-3. 타이밍 (B-2 가 "같은 경로"로 나왔을 때만)

두 지도의 갱신 속도가 다른 게 원인인지 본다.

```bash
ros2 topic hz /moveit/filtered_cloud            # 이전 실측 2.3 Hz — octomap 쪽
ros2 topic hz /cumotion/camera_1/world_depth    # 이전 실측 3.7 Hz, 최악 공백 3.1 s — nvblox 쪽
```
- `replan_attempts: 3` × `replan_delay: 1.0` = **3.0 s** vs nvblox 최악 공백 **3.1 s** → 아슬아슬하게 못 미친다.
- 손잡이는 attempts 가 아니라 **delay** 다. `replan_delay: 1.0 → 1.5` 를 시험. (`pick_fsm.yaml:33`)
- ⚠️ 소수점 필수 — `1` 이면 INTEGER 로 읽혀 노드가 죽는다.
- 🔴 **`pick_fsm.yaml` 은 고쳐도 그냥 안 먹는다.** `pick_fsm` 은 `ament_python` 이라
  `--symlink-install` 이어도 share 가 `build/pick_fsm/config/` 를 가리킨다 (CLAUDE.md 4절).
  `colcon build --symlink-install --packages-select pick_fsm` 을 다시 돌린다.
  (`.py` 는 반영돼서 "yaml 도 됐겠지" 하고 착각하기 쉬운 자리다)
  → 빠르게 시험만 할 거면 빌드 대신 런치 인자/`ros2 param set` 을 쓴다.

---

## C. cuMotion 은 attach 한 물체를 모른다

> **가설**: 파지 후 LIFT/PLACE 구간에서 cuMotion 은 **빈 손인 것처럼** 계획한다.
> 근거: `cumotion_planner.py` 에 `attached` 가 0건. pick_fsm 은 파지 후 물체를 world 에서 빼고 attach 한다.

`planning_pipeline:=isaac_ros_cumotion` 으로 한 사이클 완주시킨 뒤:

- [ ] LIFT/PLACE 궤적이 든 물체 부피를 무시하는가 (좁은 틈으로 물체를 밀어 넣는가)
- [ ] 반대 방향 — 든 물체가 nvblox ESDF 에 남아 `START_STATE_IN_COLLISION` / 계획 실패가 나는가
  `robot_segmenter` 는 **로봇만** 지운다. `distance_threshold:=0.15` 안에 들어와 같이 지워질지는 여기서만 알 수 있다
- [ ] 확인되면 아래 셋이 cuMotion 경로에서 **무효**임도 같이 기록 (전부 octomap 전용):
  `clear_octomap_before_descend`, `allow_gripper_octomap_collision`, `merge_acm()`

## D. octomap_resolution 0.05 확인

- [ ] RViz 복셀 두께가 줄었는가 (0.1 일 때의 10 cm 블록 → 5 cm)
- [ ] B-1 에서 정지가 **더 늦게/정확하게** 걸리는가 (0.1 은 부풀려져 헐겁게 걸렸을 것)
- [ ] 🔴 **빈 공간에 허깨비 복셀이 뜨는가** — D435i 노이즈는 거리 제곱으로 커져 `max_range: 2.0`
      끝에서 수 cm 급이다. 0.1 은 그걸 흡수했지만 0.05 는 **없는 장애물로 정지를 걸 수 있다.**
      `default_object_padding: 0.02`(`joint_limits.yaml:19`)라 패딩이 가려주지도 않는다
- [ ] move_group CPU % 가 감당되는가 (`top -o %CPU`), 로그에 `queue is full` 이 뜨는가
- 못 견디거나 허깨비가 뜨면 0.07 쯤에서 타협. 값을 바꾸면 `sensors_3d.yaml` 이력 주석에 한 줄 추가

---

## 검증 후 할 일

1. **A-2 가 "CUDA OOM 아님"으로 나오면** — 이 문서의 A-3 이하는 폐기하고 진짜 원인부터 다시 잡는다
2. **B 판정이 나오면** `md/context/constraints.md` 에 승격 (2026-08-08 세션에서 "실기 검증 때 다시 묻겠다"고 보류해 둔 항목):
   - 정지 트리거는 planning scene(=octomap) 경유다. B-0 로그 원문을 근거로 같이 적는다
   - cuMotion 은 attach 물체·octomap 을 안 본다 (C 절 결과)
3. **B-2 가 "안 멈춤" 또는 "같은 경로 반복"으로 끝나면** `pick_fsm/README.md` 의
   `planning_pipeline:=isaac_ros_cumotion` 안내에 **"동적 회피용이 아니라 계획 속도용"** 이라고 명시
4. **B-2 가 ✅ 로 나오면** `pick_fsm.yaml` 의 `planning_pipeline` 기본값을 바꿀지 결정.
   단 A 절에서 VRAM 이 빠듯하면 기본값은 `ompl` 로 두고 인자로만 쓴다

---
확신도: **추론** — 0절 표의 "근거" 열은 전부 이번에 소스를 직접 읽은 것이지만,
`cumotion_planner.py`·`cumotion_*.cpp` 는 이전 세션 scratchpad 사본이라 컨테이너 실물과 버전 일치는 미확인.
A~D 의 "예상" 은 **하나도 실행해 보지 않았다.** 이 문서의 명령 중 실기에서 돌려본 것은 0개다.
