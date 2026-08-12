<!-- meta
updated: 2026-08-09
status:  live — src/cumotion/README.md 에서 이관(2026-08-09, 3-README 통합 작업).
         날짜별 실험 로그다. 안정된 레퍼런스(노드 인터페이스·파라미터·배포 방법)는
         src/PACKAGES.md "cumotion" 절로 옮겼다 — 거기가 지금 참인 값의 단일 출처다.
         여기는 "그 값이 왜 그렇게 됐는지"의 이력이다.
owns:    cumotion 패키지의 날짜별 실기 실험 기록·발견·미해결 이슈 (원문 그대로 보존)
-->

# cumotion — 실기 실험 로그 (이력)

> 레퍼런스(인터페이스·파라미터·실행법·배포)는 `src/PACKAGES.md` "cumotion" 절이 단일 출처다.
> 여기는 그 값들이 나온 **실기 디버깅 과정**을 날짜순으로 보존한 것이다 — 원문을 그대로 옮겼다.

---

## ⭐ 2026-08-08 — 두 접근을 나란히 만들어 비교했다

**이 절이 최신이다.** 아래 0~0-5 절은 2026-08-07 이력이고 여전히 유효하다.

### 왜 파일을 두 개 만들었나

"동적 회피가 된다"를 주장하려면 **안 되는 것**을 먼저 보여야 한다. 그래서 같은 목표
(A `[45,0,90,0,90,0]` ↔ B `[-45,0,90,0,90,0]` deg)를 **두 방식으로** 돌린다.

| | `goal_setter_replan.py` (대조군) | `reactive_replan.py` (실험군) |
|---|---|---|
| 계획 요청 | `plan_only=**False**` — move_group 이 실행까지 | `plan_only=**True**` — 궤적만 받아온다 |
| 실행 경로 | MoveIt 실행관리자 → 컨트롤러 | **JTC 액션 직접** (실행관리자 우회) |
| 재계획 | 실행이 **끝나야** 다음 계획 | **3 Hz** 로 계속, 실행 중 궤적 교체 |
| 원본 | `isaac_ros_moveit_goal_setter` (NVIDIA 예제) | 없음 — 직접 설계 |
| 동적 장애물 | 🔴 **안 피한다 (실기 확인됨)** | 피하도록 설계. **아직 미검증** |

### 🔴 대조군 실증 — 예제 방식으로는 실행 중 회피가 안 된다

**2026-08-08 실기에서 확인했다.** 이동 중 장애물을 넣어도 그대로 밀고 간다.

이유는 로그에 숫자로 찍힌다:
```
✅ 목표 A 도착 (plan+execute 완료) | 회피 불가 구간 5.94초
```
`plan_only=False` 면 `send_goal` 이 **plan+execute 완료까지 블록**한다(원본
`move_group_client.py:83-84` 의 `while self._result is None`). 그 **5.9초 동안 계획 요청이
한 번도 안 나간다.** cuMotion 은 계획 요청 1건당 ESDF 를 1번만 읽으므로(1절), 그 사이에
사람이 들어와도 알 방법이 원리적으로 없다.

⚠️ 로봇이 **이미 목표에 있어도** 사이클이 5.8~6.0 초로 일정했다. cuMotion 이
`num_trajopt_time_steps: 32` × `interpolation_dt: 0.025` 의 고정 길이 궤적을 내고
`time_dilation_factor` 로 늘리기 때문으로 **보인다**(32×0.025÷0.15 ≈ 5.3 s). **추론이다** —
`vel_scale` 을 2배로 올려 절반이 되는지 보면 확정된다.

### ✅ 파이프라인이 cuMotion 이 맞다 (이전에 미확정이던 것)

move_group 로그에서 직접 확인:
```
[move_group]: Using planning pipeline 'isaac_ros_cumotion'
```
OMPL 폴백이 아니다. 계획 20회 **실패 0회**, 계획시간 ~150 ms.

### 🔴 T6 가 없으면 move_group 이 **무한 블록**된다 (조용한 고장)

`cumotion_move_group_client.cpp:82-85`:
```cpp
if (!client_->wait_for_action_server()) {   // ← 타임아웃 인자가 없다 = 영원히 기다린다
```
`cumotion_planner_node`(T6)가 안 떠 있으면 move_group 의 cuMotion 플러그인이 여기서 멈춘다.
`cumotion_interface.cpp` 의 5초 폴링 루프는 이 줄 **뒤**에 있어서 도달조차 못 한다.

> 증상: goal 은 **수락**되는데 결과가 영영 안 온다 → 클라이언트 타임아웃.
> "거부"가 아니라 "무응답"이면 T6 를 먼저 의심한다.

확인 한 줄:
```bash
ros2 action info /cumotion/move_group     # Action servers: 1 이어야 한다. 0 이면 T6 죽음
```

### 🔴 IK_FAIL — 관절 목표인데도 IK 를 푼다

`cumotion_planner.py:714-730` 이 **관절 목표를 FK 로 EE pose 로 바꾼 뒤 IK 를 푼다:**
```python
self.get_logger().info('Calculating goal pose from Joint target')
goal_pose = self.motion_gen.compute_kinematics(goal_state).ee_pose.clone()
```
그 IK 는 **ESDF 충돌 검사를 통과해야** 한다. 그래서 관절 목표인데 `MotionGenStatus.IK_FAIL`
이 뜬다 = **목표 자세의 IK 해가 전부 장애물과 충돌**.

⚠️ 2026-08-08 에도 재현됐다(query 0~6 성공 → 7~10 IK_FAIL). 목표는 고정인데 산발적으로
실패하므로 원인은 **(a) ESDF 가 바뀌어 목표가 덮였다** 또는 **(b) 시작상태가 바뀌어 IK 시드가
달라졌다** 둘 중 하나다. **아직 안 갈렸다.** 판별법은 0-4 절 1번(RViz `/curobo/voxels` 육안).

### 🔴 정정 — `allowed_start_tolerance` 는 0.01 이 아니라 **0.08** 이다

3절과 `arm.py` 상단 주석이 "`allowed_start_tolerance`(0.01 rad)" 를 moveit_py 배제 근거로
들고 있는데, **그 값은 `dsr_moveit_config_m0609`(두산 원본)의 것이고 T7 이 실제로 쓰는 파일이
아니다.** T7 이 쓰는 `m0609_rg2_moveit/config/moveit_controllers.yaml` 은:
```yaml
allowed_start_tolerance: 0.08     # ≈ 4.6°. 원본(0.01)보다 훨씬 관대하다
```
⚠️ 즉 "MoveIt 실행관리자를 타면 교체가 **반드시** 거부된다"는 주장은 과했다. 0.08 이면 통과할
여지가 있다. 실험군이 JTC 를 직접 잡는 진짜 이유는 **재계획 주기(0.33초) 동안 실행이 블록되지
않아야 하기 때문**이지, 이 tolerance 때문이 아니다.

### NVIDIA 소스에서 확인한 것 (문서만 보고 믿으면 안 되는 것들)

| 주장 | 실제 |
|---|---|
| `publish_world_collision_spheres` 파라미터 | ❌ **소스에 0건.** 진짜 이름은 `publish_curobo_world_as_voxels` |
| `plan_timer_period` 가 cuMotion 파라미터 | ❌ planner 에 **0건.** `isaac_ros_moveit_goal_setter` 소속 |
| `update_esdf` 파라미터 | ❌ 없음. `update_esdf_on_request` 와 혼동한 것 |
| `isaac_ros_manipulation_object_following` 패키지 | ❌ 이 워크스페이스에 없음 |
| `time_dilation_factor` 0.5 / `interpolation_dt` 0.025 / `read_esdf_world` False | ✅ 맞음 (`params/isaac_ros_cumotion_params.yaml`) |

🔴 **`cumotion_action_server` 에는 재계획 주기 파라미터가 없다.** 순수 액션 서버라
(`ActionServer(self, MoveGroup, 'cumotion/move_group', ...)`) 누가 물어볼 때만 계획한다.
**재계획 주기는 언제나 부르는 쪽이 정한다** — 우리 경우 `reactive_replan.py` 의 `replan_hz`.

🔴 **NVIDIA 예제에 버그가 있다.** `move_group_client.py:60-66` 의 `_get_joint_constraints()`
가 `Constraints` 를 만들어 append 만 하고 **`return` 이 없다** → `None` 을 돌려준다.
원본이 pose 목표만 써서 안 드러났을 뿐이다. `goal_setter_replan.py` 는 고쳐서 썼다.

---

## ⭐-1. 실행법 (2026-08-08 기준)

### 전제

**T1~T7 이 전부 떠 있어야 한다.** 순서와 명령은 6절 "0 에서 시작하는 전체 기동 순서" 가 주인이다.
빠뜨리기 쉬운 것만 다시 적는다:

```bash
export ROS_DOMAIN_ID=93        # 🔴 호스트·컨테이너 양쪽. 빠뜨리면 노드가 하나도 안 보인다
ros2 action info /cumotion/move_group    # Action servers: 1  (0 이면 T6 죽음 → 무한 블록)
```

### 빌드

```bash
cd ~/cobot2_ws
colcon build --symlink-install --packages-select cumotion
source install/setup.bash
```
⚠️ `--symlink-install` 이라 파이썬 파일 수정은 재빌드 없이 반영되지만, **entry point 를 추가한
뒤에는 반드시 빌드해야 한다**(`setup.py` 변경).

### ① 대조군 — "실행 중엔 안 피한다"를 먼저 확인한다

```bash
# 목표 1개만 (도착하면 자동 종료)
ros2 run cumotion goal_setter_replan --ros-args -p vel_scale:=0.15

# A ↔ B 왕복 — 이동 중에 손/상자를 작업영역에 넣어 본다 → 안 피하는 것을 확인
ros2 run cumotion goal_setter_replan --ros-args \
    -p mode:=sequential -p pingpong:=true -p vel_scale:=0.15
```

주요 파라미터:

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `mode` | `sequential` | `sequential`(원본 거동) / `preemptive`(선점 실험, **미검증**) |
| `pingpong` | `false` | `true` 면 A ↔ B 왕복 |
| `stop_when_arrived` | `true` | 목표 1개일 때 도착하면 종료. `false` 면 같은 goal 을 무한 반복한다 |
| `plan_timer_period` | 2.0 | 재계획 시도 주기(초). 원본 `goal_initializer.py` 와 같은 이름 |
| `goal_change_position_threshold` | **0.0** | 원본은 0.1(목표가 10 cm 움직여야 재계획). **목표 고정인 우리 용도엔 0.0 이어야 한다** |
| `goal_joint_deg` | `GOAL_A_JOINT_DEG` | 관절 목표 (deg) |
| `goal_pose` / `goal_pose_b` | `GOAL_A_POSE` / `GOAL_B_POSE` | xyz 목표. 7개=쿼터니언, 6개=rpy(deg) |
| `goal_type` | `GOAL_MODE` | `joint` / `pose` — 🎯 **파일 상단 블록에서 고른다 (⭐-3 절)** |
| `waypoints` | 1 | 🎯 A→B 를 N 등분해 회피 반응을 1/N 로 (⭐-3b). **N=2 권장**, `pingpong` 필요 |
| `vel_scale` `acc_scale` | 0.15 | 속도 스케일 |

### ② 실험군 — 동적 회피를 시도한다

```bash
ros2 run cumotion reactive_replan
```
🔴 **현재 정상 동작하지 않는다** (아래 "미해결" 1번). 목표 A/B 가 `main()` 에 하드코딩돼 있어
파라미터로 못 바꾼다 — 이것도 고칠 항목이다.

주요 파라미터: `replan_hz`(3.0) · `lookahead_s`(0.35) · `handover_s`(0.05) ·
`swap_threshold_rad`(0.05) · `max_start_jump`(0.25) · `vel_scale`(0.15) · `timeout_s`(60)

### ③ 기존 노드 (arm.py 기반)

```bash
ros2 launch cumotion dynamic_avoid.launch.py mode:=check          # 사전 점검, 안 움직임
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong vel:=0.2
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong static:=true vel:=0.15   # 대조군
```

### ⚠️ 안전

- **B(−45°) 는 아직 단독으로 가본 적 없는 자세다.** 왕복 걸기 전에 한 번 따로 확인한다:
  ```bash
  ros2 run cumotion goal_setter_replan --ros-args \
      -p goal_joint_deg:="[-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]" -p vel_scale:=0.15
  ```
- 첫 실행은 `vel_scale:=0.15`, 비상정지 버튼에 손을 올린 채로
- 루프가 도는 동안 `movej`/`movel` 을 부르지 않는다 (5절)

---

## ⭐-2. preemptive 실험 결과 + 🔴 그리퍼 자기충돌 발견

### move_group 은 동시 goal 을 **거부하지 않는다 — 큐에 쌓는다**

| 조건 | 시작 → A | A → B |
|---|---|---|
| `preemptive` (취소 없음) | 8.0초 (goal 4개) | 30.0초 (15개) |
| `preemptive` + `cancel_previous:=true` | 10.0초 (5개) | **118초+ 도착 실패** (60개+) |
| `sequential` (대조) | ~6초 | ~6초 |

move_group 로그가 보여주는 실제 거동 — **abort 가 한 번도 없다:**
```
593.233  Combined planning and execution request received
593.433  Starting trajectory execution ...
598.784  Controller '/dsr01/dsr_moveit_controller' successfully finished
599.033  Completed trajectory execution with status SUCCEEDED
599.043  Received request                    ← 그제서야 다음 goal 을 꺼낸다
```
**하나씩 끝까지 실행하고 다음을 꺼내는 FIFO 큐다.** 우리는 2초에 1개를 만드는데 소비는
5.9초에 1개 → **3배로 적체**된다. 8초 → 30초 → 72초로 발산한 이유가 이것이다.

직접 증거 (Ctrl+C 직후, 밀린 goal 들이 응답할 클라이언트를 못 찾는다):
```
Failed to send goal response ... (timeout): client will not receive response   ← 10회 이상
```

🔴 **큐는 노드를 죽여도 안 비워진다.** move_group 프로세스 안에 남아 계속 실행된다.
> **실험 사이에 T7(move_group)을 재시작해야 한다.** 안 하면 이전 실험의 goal 이 계속 돌아서
> "왜 목표를 바꿨는데 안 가지?" 같은 혼란이 생긴다. 2026-08-08 에 실제로 물렸다.

⚠️ **`cancel_previous:=true` 실험은 무효다.** `cancel()` 이 `_goal_handle`(가장 최근 **수락된**
goal)만 취소하는데, 실행 중인 것은 **가장 오래된** goal 이라 안 건드린다. 밀린 goal 전부를
추적해서 취소해야 유효한 실험이 된다. 표의 118초는 그래서 해석하면 안 된다.

### 🔴 MoveIt 이 우리 start_state 를 버린다 (`plan_only=False` 일 때)

```
WARN: Execution of motions should always start at the robot's current state.
      Ignoring the state supplied as start state in the motion planning request
```
→ 예제 방식에서는 `lookahead_s` 같은 **인계 예측이 원리적으로 불가능**하다.
실험군이 `plan_only=True` 를 쓰는 이유가 하나 더 확인된 셈이다.

### 🔴🔴 가장 중요한 발견 — 그리퍼가 자기 자신과 충돌 판정되어 계획이 통째로 버려진다

```
ERROR: Computed path is not valid. Invalid states at index locations:
       [ 0 1 2 ... 31 ] out of 32                      ← 웨이포인트 전부 무효
INFO:  Found a contact between 'rg2_left_inner_knuckle' (Robot link)
       and 'rg2_right_outer_knuckle' (Robot link), which constitutes a collision
INFO:  Motion plan was found but it seems to be invalid (possibly due to postprocessing).
       Not executing.
```

cuMotion 은 계획에 **성공**했는데(`Trajectory success!`) MoveIt 이 재검증에서 전부 거부하고
**실행을 건너뛴다.** 🔴 **우리 노드에는 아무 에러도 안 온다 — 조용히 버려진다.**

원인은 `m0609_rg2_moveit/config/m0609_rg2.srdf` 의 **좌↔우 교차쌍 9개 중 4개 누락**이다:

| 쌍 | SRDF |
|---|---|
| left_outer ↔ right_outer / left_inner_knuckle ↔ right_inner_knuckle | ✅ |
| left_inner_finger ↔ right_inner_finger / left_inner_finger ↔ right_outer_knuckle | ✅ |
| right_inner_finger ↔ left_outer_knuckle | ✅ |
| **left_inner_knuckle ↔ right_outer_knuckle** | 🔴 **없음** ← 로그가 지목한 쌍 |
| **right_inner_knuckle ↔ left_outer_knuckle** | 🔴 없음 (위의 대칭쌍) |
| **left_inner_knuckle ↔ right_inner_finger** | 🔴 없음 |
| **right_inner_knuckle ↔ left_inner_finger** | 🔴 없음 |

⚠️ 이 4쌍이 빠진 건 우연이 아니다. MoveIt Setup Assistant 는 무작위 자세를 샘플링해
**"한 번도 안 부딪힌 쌍"만** 자동 등록한다. 즉 이 4쌍은 **어떤 자세에서 실제로 겹쳤기 때문에**
제외된 것이다. 무작정 추가하면 그리퍼 자기충돌 검사를 끄는 셈이라 **실기 안전에 영향이 있다.**

🔴 **이것이 0-2 절 ① `INVALID_MOTION_PLAN(-2)` 의 진짜 원인일 가능성이 크다.**
그동안 용의자로 적어둔 `link_4 ↔ rg2_base_link`(XRDF/SRDF 불일치)는 **엉뚱한 곳을 보고 있었다.**

**먼저 할 것 — 공짜 판별 실험: 그리퍼를 열고 같은 명령을 돌린다.**
측정 당시 `rg2_finger_joint = 0.757 rad`(닫힘 쪽)이었다. 열었을 때 이 경고가 사라지면
"닫힌 자세의 메시 겹침"이 원인으로 확정되고, **SRDF 를 안 고치고도 우회할 수 있다.**

⚠️ 이 문제는 **두 파일 모두에 똑같이 영향**을 준다. 루프 구조와 무관한 더 아래 계층의 문제다.

---

## ⭐-3. 목표를 주는 방법 — 관절 vs xyz

### 🎯 `goal_setter_replan.py` 상단의 "목표 설정" 블록에서 고른다

파일 맨 위(`GOAL_MODE`)에 큰 박스로 표시해 뒀다. **주석 한 줄로 바꾼다:**

```python
# ① 관절값으로 목표를 준다 (deg)
GOAL_MODE = 'joint'
GOAL_A_JOINT_DEG = [ 45.0, 0.0, 90.0, 0.0, 90.0, 0.0]
GOAL_B_JOINT_DEG = [-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# ② base_link 기준 xyz 로 목표를 준다
# GOAL_MODE = 'pose'
GOAL_A_POSE = [0.2558,  0.2646, 0.4244, -0.653298, -0.270371, 0.653320, -0.270692]
GOAL_B_POSE = [0.2646, -0.2558, 0.4244, -0.653132,  0.270770, 0.653376,  0.270559]
```

xyz 는 **길이로 형식을 자동 판별**한다:

| 길이 | 형식 |
|---|---|
| **7개** | `[x, y, z, qx, qy, qz, qw]` 쿼터니언 — **권장** |
| 6개 | `[x, y, z, roll, pitch, yaw]`, 각도는 **deg** |

🔴 A/B 자세는 **pitch 가 정확히 90°(짐벌락)** 다. rpy 로 적으면 같은 자세를 나타내는
(roll, yaw) 조합이 무수히 많아 헷갈린다 → 쿼터니언을 쓴다.

실행 인자로도 덮어쓸 수 있다(인자가 파일 값을 이긴다):
```bash
ros2 run cumotion goal_setter_replan --ros-args -p goal_type:=pose -p pingpong:=true
```

### A/B 의 Cartesian 값 (URDF FK 계산 — **실측 아님**)

| | 관절 (deg) | base_link 기준 xyz (m) | quat (x,y,z,w) |
|---|---|---|---|
| A | `[45,0,90,0,90,0]` | `[0.2558, 0.2646, 0.4244]` | `[-0.6533, -0.2704, 0.6533, -0.2707]` |
| B | `[-45,0,90,0,90,0]` | `[0.2646, -0.2558, 0.4244]` | `[-0.6531, 0.2708, 0.6534, 0.2706]` |

A/B 는 `joint_1` 만 ±45° 다르므로 **y 부호만 뒤집힌 대칭**이고 높이는 같다(z=0.4244).
검산: `joint_1=0` 일 때 x=0.368 이고 이를 z축으로 ±45° 돌리면 위 xy 가 정확히 나온다.

⚠️ **계산값이다.** 실기에서는 로봇을 그 자세에 두고 아래가 단일 출처다:
```bash
ros2 run tf2_ros tf2_echo base_link tool0    # Translation + Rotation(Quaternion) 을 복사
```
⚠️ 기준 링크는 **`tool0`** 이지 RG2 손가락 끝(TCP)이 아니다 (`m0609_rg2.xrdf` 주석).

### 🔴🔴 "관절 목표"는 사실 관절 목표가 아니다

가장 중요한 개념적 발견이다. cuMotion 은 **관절 목표를 FK 로 pose 로 바꾼 뒤 IK 를 다시 푼다.**

`cumotion_planner.py:714-730`:
```python
goal_state = CuJointState.from_position(position=goal_config, ...)   # 우리가 준 관절값
goal_pose  = self.motion_gen.compute_kinematics(goal_state).ee_pose  # → FK 로 pose
```
`cumotion_planner.py:782` — **pose 목표 API 를 부른다:**
```python
motion_gen_result = self.motion_gen.plan_single(start_state, goal_pose, ...)
```

🔴 **cuRobo 에는 관절 목표 전용 API 가 있는데 ROS 래퍼가 안 쓴다:**
```
motion_gen.py:1530   def plan_single(start_state, goal_pose, ...)      ← 쓰는 것
motion_gen.py:2045   def plan_single_js(start_state, goal_state, ...)  ← 안 쓴다
```
`plan_single` 호출부가 파일 전체에 **하나뿐**이다. 즉 어떤 목표를 주든 전부 pose 로 간다.

**결과 3가지:**
1. **관절 목표인데 `IK_FAIL` 이 난다** (0-2 절 ②, ⭐절에서 재현)
2. **요청한 관절값과 다른 자세로 도착할 수 있다** — 6축은 같은 pose 를 만드는 IK 해가 최대 8개
3. **관절공간 도착 판정을 믿을 수 없다** — 2번의 직접적 결과

> 즉 `[45,0,90,0,90,0]` 은 "이 관절값으로 가라"가 아니라
> **"이 관절값이 만드는 pose 로 가라"** 다. 팔꿈치가 뒤집혀 도착해도 cuMotion 은 성공이다.

### 그래서 도착 판정 근거를 모드별로 나눴다

2026-08-08 에 실제로 물렸다: 목표 B 에서 **다른 IK 해로 도착** → 관절공간 비교가 영영 False
→ B→A 전환이 안 되고 제자리 궤적만 5.89초씩 무한 반복.

| 모드 | 도착 판정 | 이유 |
|---|---|---|
| `sequential` | **MoveGroup `SUCCESS`** | 결과를 기다리므로 이게 가장 믿을 만하다 |
| `preemptive` | `/joint_states` 0.01 rad | 결과를 안 기다려서 이것밖에 없다 |

⚠️ 따라서 **`joint_goal_tol` 은 `preemptive` 에서만 효과가 있다.**
⚠️ **`preemptive` + pose 목표는 도착 판정이 아예 안 된다** (`_arrived()` 가 joint 전용).
   `preemptive` 도 B 목표에서 같은 IK 해 문제에 걸릴 수 있다 — 미해결.

### ✅ sequential 왕복 실측 (수정 후)

```
goal #1 (A) 17.69초 → 도착 → B 전환
goal #2 (B) 13.49초 → 왕복 1회
...  4왕복 정상 동작.  goal 13회 (실패 5)
```
⚠️ **실패율 38%** — `PLANNING_FAILED(-1)` 4연속 + `INVALID_MOTION_PLAN(-2)` 1회.
실패해도 다음 틱에 다시 던져 결국 도착하지만 시간이 낭비된다(그 구간이 25.9초 걸렸다).
`-2` 는 ⭐-2 절의 **그리퍼 자기충돌**일 공산이 크다.

### 두산 API 와 섞으면 안 되는 이유

**우리 경로는 두산 Cartesian API 를 아예 안 탄다.** xyz 를 줘도 cuMotion 이 IK 로 관절값을
만들어 내려보내고, 하드웨어 인터페이스는 관절 서보만 받는다:
```
xyz + quat  →  cuMotion(IK)  →  관절 궤적  →  JTC  →  Drfl.servoj_rt(pos[6], ...)
                                                       (dsr_hw_interface2.cpp:496)
```
그래서 두산의 xyz-abc 규약이 개입할 자리가 없다. **대신 값을 베껴오면 안 된다:**

| | 두산 API | 우리 |
|---|---|---|
| 길이 단위 | **mm** | **m** (1000배) |
| 자세 표현 | **Euler ZYZ** (a,b,c) — `dsr_common2/include/DRFS.h:769` | **쿼터니언** |
| 기준 | 두산 TCP | **`tool0`** |

🔴 펜던트나 `posx` 에서 읽은 `a,b,c` 를 `goal_pose` 의 rpy 자리에 넣으면 **엉뚱한 자세로 간다.**
숫자가 6개로 똑같이 생겨서 조용히 틀린다. 변환하지 말고 `tf2_echo` 로 ROS 쪽 값을 직접 얻는다.

⚠️ 그리고 **루프가 도는 동안 `movej`/`movel` 을 부르지 않는다**(5절). xyz 로 목표를 준다고
   두산 네이티브 모션을 병행해도 된다는 뜻이 아니다 — 같은 DRFL TCP 연결 하나를 공유한다.

---

## ⭐-3b. 🎯 경유점(waypoints) — 회피 반응을 N 배 빠르게

### 발상

`plan()` 호출 하나가 곧 ESDF 재조회다(2절). A→B 를 N 등분해 **goal 을 N 번 던지면 장애물
지도도 N 번 읽는다** → 회피 반응이 1/N 로 줄어든다. 구간 안에서는 여전히 못 피한다.

```
지금:   A ────────────────────── B     계획 1회, 회피 불가 구간 8.2초
N=2:    A ──────────●─────────── B     계획 2회, 회피 불가 구간 4.4초
```

🔴 **이건 "피하는 척"이 아니라 실제 회피다.** 2026-08-08 에 A 도착 후 출발 전에 장애물을
넣었더니 다음 계획이 우회 경로를 만들었다(⭐-3c). 경유점은 그 기회를 N 배로 늘리는 것뿐이다.

### 🔴 바닥값이 N 을 제한한다 (실측 기반)

궤적 시간은 `time_dilation_factor`(= `min(vel_scale, acc_scale)`)에 반비례한다.
**2026-08-08 실측으로 확정:**

| vel/acc | A↔B 이동 | 널 궤적(제자리) |
|---|---|---|
| 0.15 | 13.50초 | 5.89초 |
| **0.25** | **8.18초** | **3.75초** |

비율 `13.50/8.18 = 1.65` ≈ 예측 `0.25/0.15 = 1.667` — **1% 오차로 일치.**

바닥값 공식도 맞는다:
```
num_trajopt_time_steps(32) × interpolation_dt(0.025) = 0.8초
0.8 ÷ 0.25 = 3.20초  +  MoveIt 오버헤드 ~0.55초  =  3.75초   ← 실측과 일치
```

⇒ **구간을 아무리 잘게 쪼개도 3.75초(vel 0.25) 밑으로는 안 내려간다.**

| N | 구간당 | 총 시간 | 회피 반응 |
|---|---|---|---|
| 1 (직행) | 8.2초 | **8.2초** | 8.2초 |
| **2** | 4.4초 | **8.7초** (+0.5) | **4.4초** ← 사실상 공짜 |
| 3 | 3.8초 (바닥) | 11.3초 (+3.1) | 3.8초 |
| 4+ | 3.8초 (바닥) | 15초+ | 3.8초 (**더 안 줄어듦**) |

**N=2 가 최적, N=3 이 실용 한계다.** 그 이상은 시간만 늘고 반응은 그대로다.

3.8초보다 빠르게 하려면:
1. `vel_scale`·`acc_scale` 을 더 올린다 (0.5면 바닥 1.6초) — 단 회피 여유가 줄어든다
2. T6 의 `num_trajopt_time_steps` 를 32 → 16 으로 (**미검증**, 궤적이 거칠어질 수 있다)

### 쪼개는 방식

`t = (k+1)/N` 지점을 k 번째 경유점으로 삼는다. **경유점 하나가 완전한 goal 하나**다 —
계획 → 실행 → 정지를 온전히 한 번씩 한다. 그래서 구간마다 오버헤드가 붙는다.

```
[A 도착] → goal(경유점1) → 계획(ESDF 새로) → 실행 → 정지
         → goal(경유점2) → 계획(ESDF 새로) → 실행 → 정지
         → goal(B)      → 계획(ESDF 새로) → 실행 → 정지  → [A 로 전환]
```

| 모드 | 보간 | N=2 경유점 | N=3 경유점 |
|---|---|---|---|
| `joint` | 관절값 선형 | `joint_1 = 0°` | `+15°`, `−15°` |
| `pose` | 위치 선형 + 자세 **slerp** | `[0.2602, 0.0044, 0.4244]` | `[0.2587, 0.0911, …]`, `[0.2617, −0.0823, …]` |

### 🔴 pose 모드는 경로가 안쪽으로 잘린다

A·B 는 반경 **0.368 m** 인데 xyz 직선보간 경유점은 반경 **0.26~0.27 m** 다.
`joint_1` 만 다른 두 점을 직선으로 이으면 호가 아니라 **현(chord)** 을 지나기 때문이다.

```
      A                    B          ← 반경 0.368 (자연스러운 호)
       \                  /
        ●────────────────●            ← 경유점, 반경 0.26~0.27 (현)
              베이스 쪽으로 ~10 cm 당겨짐
```

⚠️ **경유점을 켜면 로봇 움직임 모양이 눈에 띄게 달라진다.** 안쪽 공간에 뭔가 있으면
부딪힐 여지도 생긴다. **`joint` 모드는 이 왜곡이 없다**(관절 균등분할 = 자연스러운 호).
경유점 실험만 놓고 보면 `joint` 모드가 깔끔하다.

### 한계 — 경유점은 **고정점**이다

🔴 장애물이 경유점 위에 앉으면 그 구간 계획이 실패한다. 목표가 먼 진짜 재계획은 경로를
자유롭게 우회하지만, 경유점은 "반드시 여길 지나가라"라서 빠져나갈 구멍이 없다.
**N 을 키울수록 경유점이 촘촘해져 이 위험이 커진다.**

### 사용

```bash
ros2 run cumotion goal_setter_replan --ros-args \
  -p mode:=sequential -p pingpong:=true \
  -p vel_scale:=0.25 -p acc_scale:=0.25 -p waypoints:=2
```

로그:
```
✅ 목표 B 도착 ... | 회피 불가 구간 4.4초
  └ 경유점 1/2 통과 — 다음 경유점으로
```
**`회피 불가 구간`이 8.2 → 4.4초로 줄면 성공이다.**

⚠️ `waypoints` 는 **`pingpong` 일 때만 동작**한다(A/B 양 끝점을 알아야 사이를 나눈다).
⚠️ **첫 구간은 경유점을 안 쓴다.** 로봇이 실제로 어디서 출발하는지 모르는데 반대쪽 끝점에서
   보간하면 엉뚱한 지점이 나온다. 첫 목표에 도달한 뒤부터 적용된다.
⚠️ **`acc_scale` 도 같이 줘야 한다.** cuMotion 이 `min(vel, acc)` 를 쓰므로
   (`cumotion_planner.py`), `vel_scale` 만 올리면 아무것도 안 바뀐다.
   T6 로그의 `Planning with time_dilation_factor: 0.25` 로 확인한다.

---

## ⭐-3c. ✅ 장애물 회피 최초 성공 (2026-08-08)

**조건**: `sequential` + `pose` 목표 + A 도착 후 **출발하기 전에** 장애물 투입
→ 다음 계획이 **우회 경로를 만들어 피해서 이동**했다.

즉 **goal 경계 단위의 회피**다. 실행 중(궤적 진행 중) 회피는 여전히 안 된다(⭐절).
이것이 경유점 방식의 전제를 실증한다 — goal 을 다시 던지면 새 장애물을 본다.

### 같이 확인된 것

- **`GOAL_MODE = 'pose'` 정상 동작** — ⭐-3 절의 FK 계산 좌표가 실기에서 맞았다는 뜻이다
- **pose 모드 실패율이 낮다**: pose **0/9** vs joint **5/13(38%)**
  ⚠️ **추론이다.** 표본이 작고 장애물 조건도 달랐다. 다만 사실이라면 관절 목표의
  FK→IK 우회 단계(⭐-3)가 실패 지점을 하나 더 만든다는 설명과 맞아떨어진다
- vel 0.25 에서는 **2/15(13%)** 실패. 속도를 올리니 실패가 생겼으나 표본 부족

---

## ⭐-4. 미해결 (2026-08-08 시점, 우선순위 순)

### 1. 🔴 `reactive_replan.py` 가 못 움직인다 — `_same_path()` 시간정렬 버그

실측: 계획 20회(실패 0) / 교체 19회 / **생략 0회**. 장애물이 없으면 새 궤적은 기존과 같아야 하니
**생략이 많이 나와야 정상인데 0 이다.** 매번 교체 → 매번 v=0 재출발 → 가속을 못 해 제자리.

원인은 두 궤적을 **0.25초 어긋난 시점끼리** 빼는 것이다:

| 위치 | 기준 시점 |
|---|---|
| `reactive_replan.py:471` — 새 계획의 시작점 | 기존 궤적의 `(now−exec_t0) + lookahead − handover` = **+0.30 s** |
| `reactive_replan.py:392` — 비교 기준 `base` | 기존 궤적의 `(now−exec_t0) + handover` = **+0.05 s** |

로봇이 0.2 rad/s 로만 움직여도 0.25초면 0.05 rad — `swap_threshold_rad` 기본값과 같다.
**그래서 장애물이 없어도 항상 "달라졌다"고 판정한다.**

고칠 곳은 `base` 한 줄이다(`lookahead_s` 를 `_same_path` 에 넘겨 정렬). ⚠️ **문턱값을 올리는 건
증상만 가린다** — 정렬을 먼저 맞추고, 그다음에 0-1 절의 튜닝표(0.12)를 다시 본다.
⚠️ `arm.py:757` 의 `_same_path` 도 **같은 구조**라 같은 문제가 있을 수 있다. 미확인.

### 2. IK_FAIL 의 원인이 (a) ESDF 인지 (b) IK 시드인지 안 갈렸다

판별 실험: **로봇을 세워둔 채로 목표 A 에 대해 계획만 반복해서 던진다**(plan_only, 안 움직임).
- 정지 상태에서도 IK_FAIL → **(a)** 목표가 정적으로 막혀 있다 (그리드/테이블 문제)
- 정지 땐 성공, 움직일 때만 실패 → **(b)** 또는 로봇 자기 몸이 지도에 새는 것

### 3. ~~`mode:=preemptive` 가 미검증~~ → **해결됨.** 대신 새 숙제 2개

⭐-2 절 참고. **거부는 안 하고 큐에 쌓는다** → 예제 방식은 `sequential`이든 `preemptive`든
동적 회피에 못 쓴다(이유만 다르다). 남은 것:

- `cancel()` 이 실행 중인(가장 오래된) goal 을 못 잡는다 → 밀린 handle 전부 추적 필요
- `preemptive` 의 `실패 N` 은 **의미 없는 값**이다. 결과를 안 기다리고 리턴해서
  `stat_fail` 증가 코드에 도달하지 못한다. 계획이 다 실패해도 0 으로 찍힌다

### 4. ~~회피 자체를 아직 한 번도 성공 못 했다~~ → **부분 해결** (⭐-3c)

**goal 경계 회피는 성공했다** (A 도착 후 출발 전 장애물 투입 → 우회). 남은 것:
- **실행 중(궤적 진행 중) 회피는 여전히 안 된다.** 그게 실험군(`reactive_replan`)의 몫이고
  1번 버그 때문에 아직 못 돌린다
- 경유점(⭐-3b)으로 반응을 4.4초까지 줄였으나 **실기 검증 안 했다** — `waypoints:=2` 미실행

### 5. 환경 잔재 (오늘 겪은 것)

- **`nvblox_node` 가 2개 떠 있던 적이 있다.** 둘 다 같은 ESDF 서비스를 제공해 cuMotion 요청이
  아무 쪽에나 붙는다. 0-2b 절의 "복셀 2500 ↔ 2~5개 붕괴"의 설명일 **가능성**이 있다(미확인).
  `ps -eo pid,cmd | grep nvblox_node` 로 쌍이 하나인지 확인한다.
- `od_kimkh` 컨테이너에 `yolo_seg_node` 좀비 4개가 있다. PID 1 이 `sleep infinity` 라 자식을
  안 거둔다. **자원은 안 먹으므로 무해하다.** 없애려면 `docker restart od_kimkh`.

---

## 0. 🔴 2026-08-07 실기 첫 실행 — 루프가 로봇을 못 가게 한다

**M0609 실기 + T1~T7 전 구간에서 처음 돌렸다.** 인프라는 전부 붙었는데 **루프 설계에 결함이 드러났다.**
아래 두 줄이 이 패키지에서 가장 중요한 실측치다. 같은 목표(`[45,0,90,0,90,0]` deg), 같은 `vel:=0.15`,
장애물 없음:

| | 계획 | 결과 |
|---|---|---|
| `static:=true` (재계획 OFF) | **1회** | **7.7초 만에 도착** (최대오차 0.0098 rad) |
| `static:=false` (3 Hz 재계획) | 179회 (실패 23) | **60초 타임아웃, 도착 실패** · 궤적 교체 155회 |

### 왜 그런가 — 산수가 명확하다

궤적 하나의 길이가 **7.7초**인데 루프는 **0.33초마다** 갈아끼운다. 로봇은 매 궤적의 **앞 4%**만
실행하고 버린다. 그 앞 4%는 정지에서 출발하는 **가속 램프**라 거의 안 움직이고, cuMotion 은
시작 속도를 버리므로(4절) 다음 궤적도 **또 v=0 에서** 시작한다.

```
7.7초짜리 궤적 ─┬─ 앞 0.33초(가속 램프)만 실행 → 폐기
                ├─ 앞 0.33초(또 가속 램프)만 실행 → 폐기
                └─ … 155회 반복 = 제자리 기어감
```

직접 증거: 교체 155회 내내 `이음새`(교체 순간 실측 관절속도 최대성분)가 **0.000~0.037 rad/s**였다.
`vel 0.15`로 45° 이동이면 0.1~0.5 rad/s 는 나와야 한다 — **속도가 붙을 기회 자체가 없었다.**

### 그래서 바꾼 것: "무조건 교체" → "달라졌을 때만 교체"

장애물이 안 변했으면 새 궤적은 **기존 것과 같은 경로를 처음부터 다시 시작하는 것**뿐이다.
교체할수록 손해다. 그래서 계획은 3 Hz 로 계속 던지되(장애물 감시는 그대로 유지), 새 궤적이
현재 궤적의 남은 부분과 **유의미하게 다를 때만** 발행한다 — `swap_threshold_rad` (4절).

⚠️ `replan_hz` 를 낮추는 건 임시방편이다. 회피 반응도 같이 느려지고, 궤적이 7.7초라 1 Hz 로도
여전히 앞 13%만 실행한다.

### 같이 관측된 것

- `INVALID_MOTION_PLAN(-2)` **12.8%** (179회 중 23회). bench 로는 20~30%. 원인은 0-2 절.
- 복셀 **2500~2850** 사이에서 계속 변동 → nvblox 가 살아서 지도를 갱신 중.
  (2026-08-06 기록은 27,646개 — **10배 차이**, 원인 미확인)
- 계획시간 평균 **197 ms** — `testcommand.md` 실측 204 ms 와 일치
- `max_start_jump` 폐기 경고 **0회** → 인계 위치 예측 자체는 잘 맞았다

---

## 0-1. ✅ 수정 검증 — `swap_threshold_rad` 튜닝 (같은 날)

**루프 결함은 해결됐다.** 같은 목표(`[45,0,90,0,90,0]` deg), `vel:=0.15`:

| 설정 | 도착 | 교체 | 생략 | 이음새 max | `-2` 실패율 |
|---|---|---|---|---|---|
| 수정 전 (교체 무조건) | ❌ **60초 타임아웃** | 155 | — | 0.037 | 23/179 (13%) |
| 문턱 0.05 | ✅ 35.6초 | 50 | 36 | 0.067 | 18/105 (17%) |
| **문턱 0.12** | ✅ **17.0초** | **9** | 36 | 0.073 | **3/49 (6%)** |
| `static:=true` (기준) | ✅ 7.7초 | 0 | — | — | — |

**문턱을 올릴수록 교체가 줄고 도착이 빨라진다.** 이음새가 0.037 → 0.073 으로 커진 것도
**좋은 신호**다 — 로봇이 실제로 속도를 내고 있다는 뜻이다(수정 전엔 속도가 붙질 못했다).

🔴 아직 static 의 **2.2배**(17.0 vs 7.7초)다. 문턱을 더 올릴 여지가 있으나, 너무 올리면
장애물이 와도 교체를 안 하게 된다. **실제 장애물 투입 실험 전까지는 상한을 모른다.**

⚠️ 계획시간이 197 ms → **약 400 ms** 로 늘어난 시점이 있었다(nvblox 감쇠 파라미터를 되돌린 뒤).
그러면 `lookahead_s: 0.35` 가 부족해진다 — `mode:=check` 가 자동으로 경고해준다:
```
⚠️ lookahead_s(0.35) < 계획시간+handover(0.45) → 재계획 궤적이 매번 폐기될 수 있다.
```
계획시간이 400 ms 대면 **`lookahead:=0.5`** 로 올려서 이동한다.

## 0-2. 미해결 2건 — 계획 실패

### ① `INVALID_MOTION_PLAN(-2)` — MoveIt 이 궤적을 재검증해 거부

> 🔴 **2026-08-08: 아래의 "유력 용의자"는 틀렸을 가능성이 크다.** move_group 로그에서 실제로
> 잡힌 충돌쌍은 `link_4 ↔ rg2_base_link` 가 아니라 **`rg2_left_inner_knuckle ↔
> rg2_right_outer_knuckle`**(그리퍼 내부)였고, SRDF 에 그 쌍이 누락돼 있다. **⭐-2 절이 최신이다.**
> 아래 내용은 그 발견 이전의 추론이므로 참고용으로만 남긴다.

플래너가 낸 코드가 **아니다.** MoveIt 이 플래너가 준 궤적을 자기 planning scene 으로 다시
검증해서 거부한 것이다 (`planning_pipeline.h` 의 `check_solution_paths_`).

배제된 것 (실측):
- **옥토맵 아니다** — T7 을 `octomap:=false` 로 내려도 그대로 발생 (cuMotion 7/10 → 8/10)
- **목표 자세 아니다** — `/check_state_validity` 조회 결과 `valid=True`

남은 것: 같은 시작·목표인데 **산발적**으로 실패한다 → cuMotion 이 시드마다 다른 궤적을 내고
그중 일부의 **중간 지점**이 MoveIt 검사에 걸린다는 뜻.

🔴 유력 용의자 — **SRDF 와 XRDF 의 자기충돌 면제 목록이 다르다:**
```
XRDF (cuMotion): link_4 → [link_5, link_6, rg2_base_link]   ← 무시하고 계획
SRDF (MoveIt)  : link_4 → [link_3, link_5, link_6]          ← rg2_base_link 를 검사
```
`config/testcommand.md` 12절이 "XRDF `link_4 ↔ rg2_base_link` 자기충돌 검사를 꺼 뒀다 —
실기 모션 전 재검토 필수"로 이미 적어둔 그 항목이다. **미확정** — 실패한 궤적의 중간 자세를
실제로 검사해 봐야 한다.

### ② `IK_FAIL` — 목표 pose 의 IK 해가 전부 ESDF 와 충돌

4회 연속 실패로 `max_consecutive_failures` 에 걸려 감속 정지했다.

🔴 **여기서 코드 오류를 하나 잡았다.** T6 은 `IK_FAIL` 을 `NO_IK_SOLUTION(-31)` 로 반환하는데
우리에겐 `PLANNING_FAILED(-1)` 로 온다. **플러그인(`cumotion_interface.cpp`)이 실패 시
플래너의 진짜 error_code 를 덮어쓰고 `-1` 로 고정하기 때문이다.**

> **cuMotion 경로에서 `-1` 은 원인을 알려주지 않는다. 반드시 T6 로그의
> `Motion planning failed wih status:` 줄을 봐야 한다.** (`arm.py` 힌트에 반영해 둠)

🔴 **진짜 의심 지점 — T6·T5 가 이 ws 의 튜닝 yaml 로 안 돌고 있다:**
```
지금 (T6 로그의 ESDF req):        중심 (0,0,0),          2×2×2 m    ← 바닥 아래·로봇 뒤까지 포함
config/cumotion_planner.yaml:     중심 (0.35,0,0.325),   1.10×1.0×0.75 m
```
`testcommand.md` 의 T5·T6 명령이 `--params-file` 을 안 줘서 **이 ws 가 튜닝해 둔 yaml 두 개가
실제로는 한 번도 적용된 적이 없다.** T5 도 `workspace_bounds_type: unbounded` 로 돌고 있다.
감시 상자가 의도보다 훨씬 커서 바닥·로봇 뒤 공간까지 장애물로 들어온다.

## 0-2b. 🔴 미해결 ③ — 복셀이 한 자릿수다 (조사 진행 중, 미완)

**증상**: `config/` 의 튜닝 yaml 을 T5·T6 에 적용한 뒤 복셀이 **2500~2850개 → 2~5개**로 붕괴했다.
계획은 계속 성공하므로 **"성공처럼 보이는 실패"**다 — 이 상태로 움직이면 장애물을 통과한다.

### 소거법으로 배제한 것 (전부 실측)

| 후보 | 판정 | 근거 |
|---|---|---|
| 입력 `world_depth` | ✅ 무죄 | T4 살아있고 **3.728 Hz** 정상 발행 |
| T6 ESDF 서비스 클라이언트 | ✅ 무죄 | T5 재시작 후 T6 도 재시작 → 그대로 2개 |
| RViz 재시작 필요 여부 | ✅ 무죄 | `ros2 topic hz` 가 독립 구독자를 만든다 |
| nvblox 감쇠 4종 | ✅ 무죄 | base 값으로 되돌려도 **2개 → 5개** |
| `map_clearing_radius_m` | ✅ 무죄 | `clear_map_outside_radius_rate_hz:=-1.0` 으로 꺼도 그대로 |
| nvblox `workspace_bounds` | ✅ 무죄 | `unbounded` 로 바꿔도 **4개** |
| 감시상자 z 하한 (nvblox) | 🔧 **고침** | −0.05 → **−0.35** (아래 실측) |

### 🔴 실측: base_link 의 z=0 은 테이블 상판이 아니다

카메라 포인트클라우드를 `base_link` 로 변환해 백분위수를 뽑았다(표본 6497점):

```
축      min       1%      50%      99%      max
x   -24.854   -4.181    0.416    1.003    1.035
y   -24.296   -2.793   -0.107    3.097    6.519
z   -14.084   -1.732   -0.339    0.406    0.473
                        ↑ 테이블 상판
```

**상판은 z ≈ −0.30 m. 로봇이 받침대 위에 올라가 있다.** `nvblox_realtime.yaml` 의
`workspace_bounds_min_height_m: -0.05` 가 테이블과 그 위 물체를 통째로 잘라내고 있었다.
→ **−0.35** 로 고쳤다(상판 −0.30 − 여유 0.05). 상자 통과율 13.9% → **19.9%**.

### 🔴 다음 용의자 (미확인) — T6 의 그리드도 같은 z 가정을 쓴다

`config/cumotion_planner.yaml`:
```yaml
grid_center_m: [0.35, 0.0, 0.325]
grid_size_m:   [1.10, 1.0, 0.75]     # z 범위 = 0.325 ± 0.375 = -0.05 ~ 0.70
                                     #          ↑ nvblox 와 **똑같이 틀린 하한**
```

🔴 **`/curobo/voxels` 는 nvblox 가 아니라 T6 의 cuRobo 충돌월드에서 세는 숫자다.**
nvblox 쪽 z 만 고치고 T6 그리드는 안 고쳤으므로, **테이블이 여전히 T6 그리드 바닥 아래**에 있다.
이것이 남은 유력 원인이다 — **아직 시험 안 했다.**

nvblox 상자(x −0.20\~0.80, y −0.50\~0.50, z −0.35\~0.70)와 **같은 상자**로 맞추려면:
```yaml
grid_center_m: [0.30, 0.0, 0.175]
grid_size_m:   [1.00, 1.00, 1.05]    # 각 성분이 voxel_size(0.05)의 정수배여야 한다
```
(현재 값은 x 도 어긋나 있다: T6 −0.20\~**0.90** vs nvblox −0.20\~**0.80**)

### ⚠️ `config/nvblox_realtime.yaml` 이 지금 실험 상태로 남아 있다

이분법 실험 때문에 아래가 **원래 값이 아니다.** 조사 끝나면 되돌려야 한다(원래 값은 주석에 있음):
```
workspace_bounds_type: "unbounded"                       (원래 "bounding_box")
tsdf_decay_factor: 0.95                                  (원래 0.75)
projective_integrator_max_weight: 5.0                    (원래 2.0)
projective_tsdf_integrator_invalid_depth_decay_factor: -1.0   (원래 0.8)
decay_integrator_deallocate_decayed_blocks: false        (원래 true)
```
`workspace_bounds_min_height_m: -0.35` 만은 **실측 근거가 있는 확정 수정**이라 되돌리지 않는다.

## 0-3. 핵심 로그 발췌 (다른 PC 에서 분석용)

**A. 루프 결함 — `static:=true` (대조군)**
```
[dynamic_avoid] 🔴 static_mode=true — 재계획을 하지 않는다.
[dynamic_avoid] 목표(관절 deg): 45.0, 0.0, 90.0, 0.0, 90.0, 0.0
[dynamic_avoid] 목표 도착 (최대오차 0.0098 rad)                      ← 7.7초
[dynamic_avoid] 계획 1회 (실패 0) / 궤적 교체 0회 / 계획시간 평균 230 ms
```

**B. 루프 결함 — 3 Hz 재계획 (수정 전)**
```
[dynamic_avoid] 교체 #1   | 이음새 0.000 rad/s | 복셀 2741개
[dynamic_avoid] 교체 #2   | 이음새 0.014 rad/s | 복셀 2741개
   … (이음새가 끝까지 0.000~0.037 rad/s 를 벗어나지 않는다) …
[dynamic_avoid] 교체 #155 | 이음새 0.000 rad/s | 복셀 2697개
[dynamic_avoid] 타임아웃 60s — 정지
[dynamic_avoid] 계획 179회 (실패 23) / 궤적 교체 155회 / 계획시간 평균 197 ms, 최대 252 ms /
                이음새 최대 0.037 rad/s
```

**C. 수정 후 (`swap_threshold_rad: 0.05`) + IK_FAIL 중단**
```
[dynamic_avoid] 교체 #9 | 이음새 0.000 rad/s | 복셀 2663개
[dynamic_avoid] 계획 실패: PLANNING_FAILED(-1) …          ×4 연속
[dynamic_avoid] 계획 4회 연속 실패 — 감속 정지.
[dynamic_avoid] 계획 18회 (실패 6) / 궤적 교체 9회 (동일해서 생략 2회) /
                계획시간 평균 196 ms / 이음새 최대 0.020 rad/s
```

**D. 같은 시각 T6 (진짜 원인은 여기에만 있다)**
```
[cumotion_action_server] Planning with time_dilation_factor: 0.15
[cumotion_action_server] Calling ESDF service
[cumotion_action_server] ESDF req = Point(x=-1.0, y=-1.0, z=-1.0), Vector3(x=2.0, y=2.0, z=2.0)
                                    ↑ 🔴 튜닝 안 된 기본 그리드
[cumotion_action_server] Updated ESDF grid
[cumotion_action_server] Calculating goal pose from Joint target
[cumotion_action_server] Motion planning failed wih status: MotionGenStatus.IK_FAIL
```

**F. 문턱 0.12 — 루프 수정 검증 성공 (17.0초)**
```
[dynamic_avoid] 목표(관절 deg): 45.0, 0.0, 90.0, 0.0, 90.0, 0.0     ← 16:00:00.776
[dynamic_avoid] 교체 #1 | 이음새 0.031 rad/s | 복셀 0개 …
   … (교체가 9회뿐이다. 수정 전엔 같은 구간에 50~155회였다) …
[dynamic_avoid] 목표 도착 (최대오차 0.0073 rad)                       ← 16:00:17.755 = 17.0초
[dynamic_avoid] 계획 49회 (실패 3) / 궤적 교체 9회 (동일해서 생략 36회) /
                계획시간 평균 204 ms, 최대 398 ms / 이음새 최대 0.073 rad/s
```

**G. 복셀 붕괴 — 소거법 (0-2b)**
```
config/ yaml 적용 전 (base yaml 만):        복셀 2500~2850개
config/ yaml 적용 후:                       복셀 0~2개
  z 하한 -0.05 → -0.35 (실측 근거):          복셀 2개
  T6 재시작 (ESDF 클라이언트 의심):           복셀 2개
  감쇠 4종을 base 로 되돌림:                  복셀 5개
  workspace_bounds_type: unbounded:          복셀 4개    ← 여기까지가 오늘
```
```
# 포인트클라우드 실측 (base_link 기준, 표본 6497점)
z   min -14.084 | 1% -1.732 | 50% -0.339 | 99% 0.406 | max 0.473
현재 상자(z -0.35~0.70) 통과: 19.9% (z축만 50.2%)
```

**E. bench (`scripts/bench_planning_time.py --repeat 10`, plan_only)**
```
octomap:=true    ompl 10/10 (wall 98.7 ms)   cuMotion 7/10 (wall 199.9 ms)  실패 #2,#4,#5 = -2
octomap:=false   ompl 10/10 (wall 100.2 ms)  cuMotion 8/10 (wall 198.1 ms)  실패 #4,#5    = -2
```

## 0-4. 🔴 이어서 할 것 (우선순위 순)

**1. T6 그리드의 z 하한을 고친다** ← 지금 막힌 자리, 가장 유력

`config/cumotion_planner.yaml` 을 nvblox 상자와 같은 상자로 맞춘다:
```yaml
grid_center_m: [0.30, 0.0, 0.175]
grid_size_m:   [1.00, 1.00, 1.05]
```
T6 재기동 후 로그의 `ESDF req` 가 `Point(-0.2, -0.5, -0.35), Vector3(1.0, 1.0, 1.05)` 로
바뀌는지 확인하고, `mode:=check` 로 복셀이 수백 이상 돌아오는지 본다.
→ 돌아오면 **0-2b 조사 종료**. 안 돌아오면 `world_depth` 가 실제로 뭘 담고 있는지
(로봇 마스크가 화면을 얼마나 지우는지) 봐야 한다.

**2. `nvblox_realtime.yaml` 의 실험값을 되돌린다** (0-2b 마지막 표) — 특히
`workspace_bounds_type` 을 `bounding_box` 로. `unbounded` 로 두면 작업영역 밖까지 전부
장애물로 들어와 **IK_FAIL 이 재발한다**(오늘 실측).

**3. 복셀이 돌아온 상태에서 도착 시간을 다시 잰다.** 지금의 17.0초는 **장애물이 거의 없는
지도**에서 나온 값이다. 복셀이 수천 개로 돌아오면 계획시간·실패율이 달라져 문턱을 다시
튜닝해야 할 수 있다. 계획시간이 400 ms 대면 `lookahead:=0.5`.

**4. `-2` 확정** — 실패한 궤적의 **중간 자세**를 `/check_state_validity` 에 넣어 어느 링크쌍이
걸리는지 본다. `link_4 ↔ rg2_base_link` 가 나오면 SRDF/XRDF 정합 작업으로 넘어간다.
(빈 세계에서도 17% 나므로 **장애물과 무관**한 것은 이미 확인됐다)

**5. 그다음에야 `mode:=pingpong` + 실제 장애물 투입.**
🔴 **회피 자체는 아직 한 번도 검증 안 됐다.** 오늘 한 이동 실험은 전부 장애물 없이 돌린 것이다.

## 0-5. 🔴 조용히 물리는 함정 (오늘 실제로 당한 것)

에러도 경고도 안 난다. **틀린 채로 그냥 돈다.**

| 함정 | 증상 | 대처 |
|---|---|---|
| **`-p static_mapper.*` 가 nvblox 에서 무시된다** | 오버라이드를 줬는데 yaml 값이 그대로. "감쇠를 되돌렸는데 효과 없다"는 **잘못된 결론**을 낼 뻔했다 | `static_mapper.*` 는 **yaml 을 직접 고쳐야** 한다. 접두어 없는 최상위 파라미터(`decay_tsdf_rate_hz` 등)는 `-p` 가 먹는다. 반드시 `ros2 param get` 으로 확인 |
| **`ros2 run` 은 `static` 이 아니라 `static_mode`** | `-p static:=true` 는 선언 안 된 파라미터라 **조용히 무시**되고 재계획이 돈다. 대조군인 줄 알고 잘못된 데이터를 받는다 | launch 는 `static:=true`, `ros2 run` 은 `-p static_mode:=true`. 시작 로그에 `🔴 static_mode=true` 경고가 뜨는지로 확인 |
| **`ROS_DOMAIN_ID` 누락** | `/move_action 액션 서버 없음` → T7 이 죽은 줄 알게 된다. 실제로는 도메인이 0이었을 뿐 | 호스트 터미널마다 `export ROS_DOMAIN_ID=93` |
| **`container_setup.sh` 누락** | T4 는 `import cv2 → _ARRAY_API not found`, T6 은 `warp has no attribute 'torch'` | 컨테이너를 **새로 만들 때마다** 실행 |
| 🔴 **컨테이너에 `root` 로 들어감** | `container_setup.sh` 를 돌렸는데도 위 에러가 그대로 난다. pip 이 사용자별로 설치되기 때문이다 — `admin` 은 `~/.local` 에 numpy 1.26.4, `root` 는 시스템 numpy 2.2.6 그대로 (2026-08-07 실측) | **`docker exec -it -u admin …`** 로 들어가거나 `run_dev.sh` 를 다시 실행한다(떠 있는 컨테이너엔 `-u admin` 으로 attach 한다, `run_dev.sh:195`). `docker exec -it <c> bash` 는 기본이 **root** 라 물린다 |
| **`config/*.yaml` 미적용** | `testcommand.md` 의 T5·T6 명령이 `--params-file` 을 안 준다. 튜닝값이 **한 번도 적용된 적이 없었다** | T6 로그의 `ESDF req` 줄, `ros2 param get` 으로 확인 |
| **cuMotion 의 `PLANNING_FAILED(-1)`** | 플러그인이 플래너의 진짜 error_code 를 덮어쓴다. T6 은 `NO_IK_SOLUTION` 인데 우리에겐 `-1` | **T6 로그의 `Motion planning failed wih status:` 줄**을 봐야 한다 |

## 0-6. 아직 안 고친 것 (알고 남겨둔 것)

- **`do_check()` 가 계획을 1회만 던진다.** `-2` 실패율이 13% 라 **파이프라인이 멀쩡해도 check 가
  8번에 1번꼴로 실패**하고, 그때 `T4/T5/T6 중 하나가 문제다` 라는 엉뚱한 메시지가 찍힌다.
- **`/curobo/voxels` 구독을 첫 계획 뒤에 건다** → `mode:=check` 의 첫 계획 복셀을 못 본다
  (계획을 한 번 더 던져 우회 중). publisher 는 T6 시작 시점에 생기므로 앞당길 수 있다.

