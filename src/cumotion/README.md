<!-- meta
updated: 2026-08-11
status:  live (2026-08-11)
         ✅ `_same_path` 시간정렬 버그(⭐-4 1번) 수정 완료 — cross-review 반영, 가상환경(virtual
            DRCF+OMPL) 검증 통과(5회 목표 왕복 전부 도착)
         ✅ `max_start_jump` 폐기 버그 원인 규명 + 수정 — lookahead_s(cuMotion 기준 0.35s)가
            OMPL 실측 계획시간(~0.07s)과 5배 차이나 시드가 항상 어긋났다. plan_dt EMA 기반
            예측으로 폐기율 30~53% → ~8%. (1차 시도는 "제자리 걸음" 재발로 실패, 되돌림 — 경위는
            ⭐-4 1-2절)
         ⏸️ **pick_fsm 연결은 보류(2026-08-11, 사용자 결정)** — GraspGenX 장애물 배치가 아직
            불안정해서 pre_grasp(APPROACH) 구간엔 의도적으로 장애물을 안 둔다(IK/그립 자세 해석
            성공률을 위해). 장애물이 없는 구간에서 reactive_replan은 moveit_bridge 대비 이득이
            없다 — 장애물 배치가 안정화되거나 다른 동적 요소(사람 개입 등)가 생기는 상태가 연결의
            전제조건. 그 전엔 연결 작업 자체가 불필요.
         🔴 **그리퍼 SRDF 자기충돌 4쌍 누락**(⭐-2, 미해결) — 계획이 조용히 버려진다.
            판별 절차는 `md/plans/2026-08-10-srdf_self_collision_test.md`
         🔴 **T7 컨트롤러 스포너가 호스트 서비스를 못 부른다(2026-08-11, 신규·미해결)** —
            아래 ⭐-4 1-3절
next:    (연결 보류 상태) SRDF 자기충돌 판별 실험(실기) → T7 서비스 디스커버리 문제 원인 조사
         → GraspGenX 장애물 배치 안정화 여부 확인 → 그때 pick_fsm 연결 재검토
warn2:   실험 사이에 **T7(move_group) 재시작 필수** — goal 큐가 노드를 죽여도 안 비워진다(⭐-2)
warn3:   가상환경(virtual) 테스트 후 `ros2_control_node`/`robot_state_publisher`/
         `static_transform_publisher`/`joint_state_publisher`/`gripper_virtual_node.py`를
         launch 부모 프로세스만 죽이면 고아로 남는다 — 개별 PID로 kill 할 것.
         `md/context/constraints.md` "가상환경(virtual DRCF) 프로세스 정리" 절 참고.
warn:    config/nvblox_realtime.yaml 이 이분법 실험 상태로 남아 있다 → 0-2b 마지막 표 참고
owns:    MoveIt+cuMotion 스택의 파이썬 제어 · 실행 중 재계획 루프 설계 · 이 패키지의 배치/실행 위치
-->

# cumotion — 실행 중 재계획으로 동적 장애물을 회피한다

실행 명령·검증은 [[ws/cobot2/testcommand]], 파이프라인 파라미터는 `config/README.md`
(T4~T7 노드 yaml)가 단일 출처다. 여기는 **이 패키지 코드가 왜 이렇게 생겼는지**만 둔다.

| 파일 | 역할 |
|---|---|
| `cumotion/arm.py` | 라이브러리. 계획(MoveGroup 액션) + 실행(FollowJointTrajectory 액션) + 재계획 루프 |
| `cumotion/dynamic_avoid.py` | 실행 노드. `mode` 파라미터로 check/joint/pose/pingpong |
| **`cumotion/reactive_replan.py`** | 🆕 **실험군.** arm.py 를 안 쓰는 독립 단일 파일. 3 Hz 재계획 + JTC 직접 선점교체 |
| **`cumotion/goal_setter_replan.py`** | 🆕 **대조군.** NVIDIA 예제 방식(`plan_only=False`, move_group 이 실행까지) |
| `config/dynamic_avoid.yaml` | 파라미터 기본값 (주석이 본체다) |
| `launch/dynamic_avoid.launch.py` | 자주 바꾸는 것만 launch 인자로 노출 |

---

## 🎓 `reactive_replan.py` 알고리즘 소개 (발표·데모용 요약)

**한 줄 요약**: MoveIt의 "계획 한 번 → 실행 끝까지"라는 표준 흐름을 깨고, **실행 중에도
초당 3회씩 계속 새로 계획해서, 달라졌으면 그 자리에서 궤적을 바꿔치기**하는 루프다.

### 왜 필요한가
표준 MoveIt 실행(`plan_only=False`)은 `send_goal`이 **실행이 끝날 때까지 블록**한다. 그 동안은
계획 요청 자체가 안 나가므로, 이동 중 새 장애물이 나타나도 로봇이 알 방법이 없다(실기로 확인 —
아래 대조군 절). `reactive_replan.py`는 계획(`plan_only=True`)과 실행(FollowJointTrajectory
액션 직접 호출)을 분리해서, MoveIt의 실행관리자를 거치지 않고 **컨트롤러에 직접, 실행 중에도
새 궤적을 선점 교체**한다.

### 루프 한 바퀴 (`run_to_goal()`)
```
1. 지금이 몇 번째 재계획 주기인지 확인 (replan_hz, 기본 3Hz)
2. "새 계획이 준비될 즈음(수십~백여 ms 후) 로봇이 있을 위치"를 예측해 시드로 삼는다
   → 이 예측 정확도가 알고리즘 성패를 가른다 (아래 2026-08-11 수정 참고)
3. 그 시드에서 목표까지 새로 계획 요청 (cuMotion/OMPL 등, plan_only=True)
4. 새 궤적이 "지금 실행 중인 궤적"과 사실상 같은가? (_same_path)
   같다 → 교체 안 함(불필요한 v=0 재출발 방지)
   다르다 → 컨트롤러에 새 궤적 직접 전송, 교체
5. 목표 도달 오차 이내면 종료, 아니면 1로
```

### 오늘(2026-08-11) 고친 것 — 발표에 쓸 수 있는 before/after
이 알고리즘은 "예측이 얼마나 정확한가"에 성패가 갈린다는 걸 실측으로 보여준 사례다.

| | 원인 | 증상 | 결과 |
|---|---|---|---|
| 버그 1 (`_same_path`) | 같은 궤적 비교 기준 시점이 서로 어긋남(0.25s) | 장애물이 없어도 "달라졌다"고 항상 오판 → 매번 교체 | 시드 채취 시점을 그대로 넘기도록 수정 |
| 버그 2 (`max_start_jump`) | 예측 시야(`lookahead_s`=0.35s)가 실제 계획시간(~0.07s)의 5배 | 시드가 미래를 과하게 앞서 예측 → 새 궤적이 매번 버려짐(폐기 30~53%) | 실측 계획시간의 이동평균(EMA)으로 예측폭을 스스로 맞춤 |

가상환경(virtual DRCF + OMPL) A↔B 왕복 검증:

| | 폐기 비율 | 목표 도착 |
|---|---|---|
| 수정 전 | 30~53% | 종종 실패 |
| 수정 후 | ~8% | 항상 성공 (A 13초, B 24초) |

⚠️ 발표 시 정확히 짚을 것 (cross-review 지적 반영, 2026-08-11):
- 이 검증은 **OMPL + 가상 로봇**(cuMotion 컨테이너·실기 없이) 기준이다. 실제 배치 대상인
  **cuMotion 파이프라인(계획시간 ~0.2s+, GPU/ESDF 재조회 스파이크 가능 — OMPL의 ~0.07s와
  자릿수부터 다르다)에서 EMA가 똑같이 잘 수렴하는지는 미검증**이다. 재검증 전엔 프로덕션
  취급 금지.
- "폐기율이 낮을수록 좋다"는 절반만 맞다. 폐기는 관절 점프 관점에선 안전(위험한 궤적을
  거른 것)이지만, EMA가 부풀어 폐기가 잦아지면 로봇이 **낡은 궤적을 계속 따라가는** 것이라
  회피 반응성 관점에선 나쁜 쪽으로 작용할 수 있다 — 단순 안전 편향이 아니라 트레이드오프다.
- pick_fsm 연결도 현재는 보류 상태다(장애물 배치 안정화 전이라 실익 없음 — 아래 meta 참고).

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

#### 🆕 2026-08-10 가상환경(virtual DRCF + OMPL) 1차 재현 — 로그 오귀인 정정

pick_fsm 연결 전 사전 점검으로 `bringup.launch.py mode:=virtual` + `moveit.launch.py cumotion:=false`
+ `reactive_replan.py --ros-args -p pipeline_id:=ompl -p vel_scale:=0.15` 를 돌렸다. **당시
"`_same_path` 로그가 재현된다"고 적었던 것은 오귀인이었다** — `새 궤적 시작점이 실측과 X rad
어긋남 — 폐기` 경고는 `_same_path()`(생략 카운터, `stat_skips`)가 아니라 **`run_to_goal`의
`max_start_jump` 체크**(487-490행, `_same_path` 호출보다 먼저 일어남)에서 나오는 완전히 다른
경로다. `_same_path`는 실제로 그날도 8회(A) 스킵을 냈다(요약 로그 `생략 8회`) — "생략 0"과는
다른 양상이었다. 아래 2026-08-11 절에서 두 버그를 분리해 정리한다.

#### 🆕 2026-08-11 — `_same_path` 정렬 수정 + cross-review + 재검증

**수정 1차** (`base = (time.time() + lookahead_s - handover_s) - cur_t0`)는 cross-review에서
"부분 수정"으로 지적됨: `_same_path` 안에서 `time.time()`을 새로 읽으면 `plan()` 왕복 지연
(~lookahead_s 자릿수, 블로킹) 만큼 시드 채취 시점(`run_to_goal`의 `now`)보다 늦은 시각이 섞여
들어가, 정적 오차(~0.25s)를 가변 오차(~0.2s)로 바꾼 것뿐이었다.

**수정 2차(최종)**: `_same_path`가 `time.time()`을 아예 안 읽도록 바꿨다. 시드를 뽑을 때
(`run_to_goal`) 쓴 `seed_base`(= `t_on_traj - handover_s`, cur_traj 위에서 new_traj 시작점에
대응하는 시각) 값을 그대로 인자로 넘긴다. 시드가 cur_traj에서 채취되지 않은 경우
(`traj_finished`나 궤적 추월 시엔 현재 실측 위치에서 시작 — `seed_base=None`)는 애초에
비교 대상이 없으므로 `_same_path` 호출 자체를 건너뛴다.

**가상환경 재검증** (동일 구성, 완전히 정리된 ROS_DOMAIN_ID=93에서 재실행):

| 시점 | 목표 | 계획 | 교체 | 생략 | 결과 |
|---|---|---|---|---|---|
| 수정 전(2026-08-10) | A | 33 | 14 | 8 | ✅ 도착 |
| 수정 전(2026-08-10) | B | 114 | 45 | 24 | 🔴 이동 실패 |
| 수정 2차 후(2026-08-11) | A | 32 | 14 | 7 | ✅ 도착 |
| 수정 2차 후(2026-08-11) | B | 59 | 25 | 11 | ✅ 도착 |
| 수정 2차 후(2026-08-11) | A(재왕복) | 87 | 34 | 17 | ✅ 도착 |

45초 창 안에서 A→B→A 왕복이 전부 성공했다(수정 전엔 B가 실패했었다). ⚠️ **이걸로 "완전히
고쳤다"고 말할 수는 없다** — 표본 1회뿐이고, `max_start_jump` 폐기(아래 미해결 1-2번)가 여전히
계획의 30~50%를 잡아먹고 있어 결과가 타이밍에 따라 갈릴 수 있다. 또한 **이 OMPL+virtual 조합은
원래 "생략 0회" 증상(2026-08-08, cuMotion 파이프라인 기준)을 애초에 재현한 적이 없다** — cuMotion
경로(T4~T7 컨테이너)에서의 재검증은 아직 안 했다.

#### 🆕 2026-08-11 미해결 1-2. `max_start_jump` 폐기 — 원인 규명됨, 1차 수정은 되돌림

위 표에서 "생략"과 별도로, 계획의 30~53%가 `run_to_goal:487-490`(`jump > max_start_jump`)에서
통째로 버려진다(`새 궤적 시작점이 실측과 0.25~0.67 rad 어긋남 — 폐기`). `_same_path` 수정과
무관하게 그대로 남아 있었다.

**진단**: `plan()` 호출 앞뒤로 `plan_dt`(왕복 소요시간)를 재는 로그를 추가해 가설 3개를 검증했다.

| | 폐기 시 `plan_dt` | 성공 시 `plan_dt` |
|---|---|---|
| n | 31 | 50 |
| 평균 | 0.073s | 0.071s |
| lookahead_s(0.35s) 초과 횟수 | **0/31** | — |

→ **가설 1 완전히 배제.** `plan()`이 느려서 시드가 낡는 게 아니다 — 폐기 시와 성공 시의
`plan_dt`가 통계적으로 구분이 안 되고, `lookahead_s`를 초과한 적이 단 한 번도 없다.
**가설 3이 맞다**: `lookahead_s=0.35s`는 cuMotion 계획시간(~0.2s) 기준으로 튜닝된 값인데,
OMPL 실측 계획시간은 평균 0.07s — **5배** 짧다. 시드는 "0.35초 뒤"를 예측하는데 검증 시점엔
0.07초밖에 안 지나 있어, 그 차이(~0.28초치 관절 이동량)만큼 항상 어긋난다.

**1차 수정 시도(EMA) — 실패, 되돌림**: `lookahead_s`를 실측 `plan_dt`의 이동평균으로 대체했다.
결과: `max_start_jump` 폐기는 **0회**로 사라졌지만(계획 129/교체 123/생략 4), **45초+ 동안
목표에 한 번도 도착하지 못하고 "이동 실패"로 끝났다.** 원인: `seed_base = effective_lookahead
- handover_s`인데, `effective_lookahead`가 `plan_dt`(~0.07~0.13s) 수준으로 작아지면
`handover_s`(0.05s)를 뺀 나머지가 거의 0에 가까워져 — 매 재계획이 사실상 "지금 이 순간에서
v=0 재출발"이 된다. cuMotion/OMPL이 매번 v=0 완결 궤적을 주는 구조(근거1)라, 이러면 로봇이
가속 램프를 벗어나기 전에 계속 교체당해 **원래 ⭐-4 1번 버그("매번 교체 → 매번 v=0 재출발 →
제자리")와 같은 증상이 다른 경로로 재발한다.** "폐기 0회"만 보고 개선이라 판단하면 안 되는
사례 — 되돌리고 고정 `lookahead_s`로 복귀해 정상 도착(계획34/교체15/생략8, ✅ 도착) 확인했다.

**2차 수정 시도 — 성공, 유지**: `seed_base = t_on_traj - handover_s`인 구조는 그대로 두고,
`effective_lookahead = max(plan_dt_ema, 0.02) + handover_s`로 바꿨다. 대수적으로
`seed_base = (now-exec_t0) + plan_dt_ema`가 되어 — "새 궤적이 준비될 즈음(plan_dt_ema 초 후)
로봇이 있을 위치"를 그대로 예측하면서도 1차 시도처럼 `handover_s`를 이중으로 빼는 문제가 없다.

**가상환경 재검증** (완전히 정리된 ROS_DOMAIN_ID=93):

| 목표 | 계획 | 교체 | 생략 | 폐기 | 결과 |
|---|---|---|---|---|---|
| A | 40 | 27 | 3 | 3(~8%) | ✅ 도착(13초) |
| B | 111(누적) | 92(누적) | 8(누적) | 9(누적, ~8%) | ✅ 도착(24초) |

**폐기 비율이 수정 전 30~53%에서 ~8%로 떨어졌고, 두 목표 모두 정상 도착했다**(1차 EMA
시도처럼 제자리에 머무는 증상 없음). ⚠️ 0%는 아니다 — 잔여 폐기(가설 2, virtual 실행 타이밍이
`exec_t0` 기준 wall-clock 가정과 다를 가능성)는 미해결로 남아 있지만, 현재로선 pick_fsm 연결을
막을 정도는 아니라고 판단한다. `plan_dt` 진단 로그는 코드에 남겨뒀다.

### 1-3. 🔴 T7 컨트롤러 스포너가 호스트 `controller_manager` 서비스를 못 부른다 (2026-08-11, 신규·미해결)

cuMotion 파이프라인(T4~T7) 전체로 reactive_replan을 재검증하려다 여기서 막혔다. 시도 경위:

1. T4(robot_segmenter) 기동 실패 — `numpy 2.2.6`이 `cv2`를 깨서(2026-08-07에 이미 한 번 겪은
   것과 동일 증상) `AttributeError: _ARRAY_API not found`. `scripts/container_setup.sh`(문서화된
   최초 셋업 절차, 안 돼 있었다)로 해결 — numpy 1.26.4로 재설치.
2. T4·T5·T6 정상 기동, T6는 `cuMotion is ready for planning queries!`까지 확인.
3. T7(`moveit.launch.py cumotion:=true`, 컨테이너 안에서 `docker exec -d`로 기동)에서
   `dsr_moveit_controller` 스포너가 `/dsr01/controller_manager/list_controllers`(호스트,
   `dsr_controller2`/`joint_state_broadcaster`와 같은 컨트롤러 매니저) 호출에 실패하고
   10초×3회 재시도 후 죽는다. **2회 재현, 둘 다 동일 실패.**

**격리 결과**:
- 컨테이너에서 호스트의 `/dsr01/joint_states` 등 **토픽·노드 디스커버리는 정상**(`ros2 node
  list`/`ros2 topic list`로 확인).
- **서비스만 안 된다** — 컨테이너 안에서 직접 `ros2 service call
  /dsr01/controller_manager/list_controllers`를 호출해도 무한 대기(`waiting for service to
  become available...`에서 멈춤). 같은 서비스를 호스트에서 호출하면 즉시 응답한다.
- `RMW_IMPLEMENTATION`은 양쪽 다 `rmw_fastrtps_cpp`로 동일 — warn 절이 경고하는
  "cyclonedds 교차 벤더" 케이스는 아니다.

**미확인 가설(우선순위 미정, 다음에 조사할 것)**:
1. FastDDS 서비스 디스커버리가 request/response 토픽 쌍 QoS 불일치로 실패 — 토픽 pub/sub은
   되는데 서비스만 안 되는 비대칭이 이 가설과 맞는다.
2. `docker exec -d`로 띄운 프로세스가 `run_dev.sh`의 원래 인터랙티브 세션(`-it`, 전체 컨테이너
   entrypoint)과 네트워크·환경 설정이 미묘하게 다를 가능성 — 이번 세션은 이미 떠 있던 컨테이너에
   `docker exec -d`로 얹은 것이라 `run_dev.sh`가 설정했을 어떤 조건을 안 물려받았을 수 있다.
3. `graspx_container.sh`가 다른 컨테이너(`od_kimkh`)에 `FASTRTPS_DEFAULT_PROFILES_FILE`을
   명시적으로 넘기는 걸 보면, 이 ws에 컨테이너-호스트 DDS 설정 이슈가 이미 알려진 카테고리다 —
   `isaac_ros_dev-x86_64-container`에도 비슷한 프로파일이 필요할 수 있다.

**영향 범위**: reactive_replan.py 자체 결함이 아니다 — T7(move_group)이 실행 경로
(`FollowJointTrajectory` 액션)를 열지 못하면 이 노드 말고 **어떤 방식으로 cuMotion 파이프라인을
실기/에뮬레이터에 실행시키려 해도 똑같이 막힌다.** 2026-08-08 성공 기록(⭐절, goal 경계 회피)은
`run_dev.sh` 인터랙티브 세션으로 띄웠을 때이므로 — 그 방식과 오늘의 `docker exec -d` 방식의
차이가 유력한 단서다.

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

---

## 1. 🔴 왜 "루프"인가 — 이 패키지의 존재 이유

`cumotion_planner_node` 는 **계획 요청 1건당 ESDF 를 딱 1번** 읽는다
(`cumotion_planner.yaml` 의 `update_esdf_on_request` 주석, `cumotion_planner.py:621`).

> 궤적이 한 번 만들어지고 나면, 실행 중에 사람이 걸어 들어와도 cuMotion 은 모른다.

즉 **nvblox 지도를 실시간으로 만든 것만으로는 실시간 회피가 안 된다.** 지도는 재료일 뿐이고,
회피는 *이 노드가 계획을 계속 다시 시키고 실행 중인 궤적을 갈아끼울 때* 비로소 생긴다.
`config/README.md` 의 "아직 안 된 것" 마지막 항목("실행 중 동적 회피는 여전히 안 된다 …
그 다음 단계(실행 중 재계획 루프)의 전제 조건일 뿐")이 가리키는 게 정확히 이 패키지다.

```
                        ┌──────── 3 Hz 로 반복 ────────┐
현재/예측 상태 ──▶ plan() ──▶ nvblox ESDF pull ──▶ 새 궤적 ──▶ JTC 로 교체 발행 ──┘
                                                              (기존 goal 은 JTC 가 선점)
```

## 2. 🔴 이 노드는 nvblox 를 구독하지 않는다

가장 헷갈리는 지점이다. **장애물 데이터는 이 노드를 안 거친다.**

```
nvblox_node ──서비스── /nvblox_node/get_esdf_and_gradient
                           ▲  cumotion_planner_node 가 pull 한다
                           │  (cumotion_planner.yaml: read_esdf_world: true,
                           │   esdf_service_name, update_esdf_on_request: true)
                    cumotion_planner_node ── cuRobo 충돌월드
                           ▲ /cumotion/move_group
                    move_group (cuMotion 플러그인)
                           ▲ /move_action  ← 이 노드는 여기만 잡는다
                    dynamic_avoid
```

우리가 ESDF 를 받아서 플래너에 넘겨주는 구조가 **아니다.** `cumotion_planner_node` 가
자기 요청을 처리하는 도중에 nvblox 에 직접 서비스 콜을 날린다. 그래서:

> **`plan()` 호출 그 자체가 nvblox 에 ESDF 를 물어보는 트리거다.**
> "장애물을 다시 본다" = "`plan()` 을 다시 부른다" — 1절의 루프가 회피를 만드는 이유가 이것이다.

(직접 구독해서 MoveIt collision object 로 넘기는 방식은 `motion_planning/nvblox_bbox_bridge.py`
가 하는 **별개 접근**이다. OMPL/octomap 경로용이고 cuMotion 경로와 섞으면 안 된다.)

### 그래서 감시가 따로 필요하다

🔴 **nvblox 가 죽어도 계획은 성공한다.** 장애물이 없는 세상에서 계획할 뿐이다.
계획 성공/실패로는 절대 드러나지 않고, 로봇이 장애물을 통과한 뒤에야 안다.
`testcommand.md` 가 "성공처럼 보이는 실패"라 부르는 그것이다. 그래서 두 겹을 넣었다:

| 무엇 | 어떻게 | 걸리면 |
|---|---|---|
| ESDF 서비스 존재 (`check_obstacle_pipeline()`) | `esdf_service_name` 이 실제로 떠 있는지 | `require_obstacle_pipeline: true` 면 **이동을 거부**한다 |
| cuMotion 이 실제로 본 복셀 (`/curobo/voxels`) | 궤적 교체마다 복셀 수를 로그에 남긴다 | 0개면 "nvblox 는 살아 있어도 지도가 비었다" 경고 |

⚠️ 서비스 존재 확인은 nvblox 가 *떠 있다*는 것만 본다. `esdf_mode` 가 `2d` 면 nvblox 는
cuMotion 첫 요청에 FATAL 로 죽는데, 그건 첫 계획을 실제로 던져 봐야 드러난다 —
`mode:=check` 가 계획을 1회 던지는 이유다.

⚠️ `/curobo/voxels` 는 **계획 요청을 처리하는 중에만** 발행된다(`testcommand.md` 8절).
대기 중에 `topic hz` 로 확인하려 들면 안 나온다.

`pipeline_id:=ompl` 로 쓸 땐 nvblox 가 필요 없으므로 `require_obstacle_pipeline:=false` 로 내린다.

## 3. 전부 표준 ROS 2 인터페이스다

| 하는 일 | 인터페이스 |
|---|---|
| 계획 | `/move_action` — 액션 `moveit_msgs/action/MoveGroup` (`pipeline_id: isaac_ros_cumotion`, `plan_only: true`) |
| 실행 | `/dsr01/dsr_moveit_controller/follow_joint_trajectory` — 액션 `control_msgs/action/FollowJointTrajectory` |
| 상태 | `/joint_states` — 토픽 `sensor_msgs/msg/JointState` |
| 정지 | `/dsr01/motion/move_stop` — 서비스 `dsr_msgs2/srv/MoveStop` |

RViz MotionPlanning 패널이 쓰는 것과 같은 진입점이고, GUI 대신 이 노드가 클라이언트다.

### 🔴 왜 `moveit_py` 가 아니라 액션 클라이언트인가

`ARCHITECTURE.md` 2절이 권하는 `moveit_py` 는 **이 루프에는 못 쓴다.** 셋 다 치명적이다:

1. **`moveit_py.execute()` 가 MoveIt 실행 관리자를 탄다** → `allowed_start_tolerance` 검사에 걸려
   움직이는 중의 궤적 교체가 거부될 수 있다.
   🔴 **2026-08-08 정정: 여기 적혀 있던 "0.01 rad" 는 틀린 값이다.** 그건
   `dsr_moveit_config_m0609`(두산 원본)의 값이고, **T7 이 실제로 쓰는
   `m0609_rg2_moveit/config/moveit_controllers.yaml` 은 `0.08`**(≈4.6°)로 훨씬 관대하다.
   따라서 "**매번** 거부된다"는 과장이었다. 액션 클라이언트를 쓰는 진짜 이유는 ②다 — ⭐절 참고.
2. **실행 중 궤적을 선점 교체하는 API 가 없다.** plan→execute 순차 모델이라 표현 자체가 안 된다.
3. **프로세스 안에 RobotModel/PlanningScene 을 또 띄운다** → `move_group` 의 파라미터 일습
   (robot_description·SRDF·kinematics·planning_pipelines)을 이 노드에도 똑같이 먹여야 한다.
   액션 클라이언트는 이미 떠 있는 `move_group` 에 붙기만 하면 된다.

JTC 액션을 직접 부르면 ① 의 검사가 없고, 새 goal 이 오면 JTC 가 기존 goal 을 스스로 선점한다.
`plan_only: true` 로 궤적만 받아오는 이유가 이것이다.

⚠️ 컨트롤러 이름 앞의 `/dsr01/` 은 오타가 아니다. bringup 의 `controller_manager` 가
`dsr01` 네임스페이스에 있어서 액션도 그 밑에 뜬다.

## 4. 인계(handover) 타이밍 — 세 파라미터가 전부다

계획 1회에 wall **204 ms**(`testcommand.md` 9절 실측). 그동안 로봇은 계속 움직인다.
그래서 "지금 상태"로 계획하면 결과가 나올 땐 이미 그 지점을 지나쳐 있다 → 인계 시 점프.

| 파라미터 | 기본 | 의미 | 어긋나면 |
|---|---|---|---|
| `lookahead_s` | 0.35 s | **미래 시점**의 궤적 위 상태에서 계획을 시작 | 작으면 "새 궤적 시작점이 실측과 어긋남" 경고 후 폐기 |
| `handover_s` | 0.05 s | 새 궤적을 뒤로 밀어 JTC 가 보간해 올라타게 함 | 0 이면 즉시 점프, 크면 반응이 느려짐 |
| `replan_hz` | 3.0 Hz | 재계획 주파수 | 위로 올려도 **새 정보가 없다** (아래) |

🔴 **`lookahead_s > 계획시간 + handover_s`** 가 성립해야 루프가 돈다. 0.35 는 204 ms + 여유다.
`vel_scale` 을 올리면 같은 시간에 더 멀리 가므로 `lookahead_s` 도 같이 올려야 한다.
`mode:=check` 가 실측 계획시간과 비교해서 이 조건을 자동으로 경고해준다.

### 🔴 다만 lookahead 로 고쳐지는 건 **위치뿐**이다

**cuMotion 은 우리가 보낸 시작 velocity 를 버린다.** `cumotion_planner.py:675` 가
`CuJointState.from_position(position=, joint_names=)` 로만 시작상태를 만들어 velocity 가 0 으로
채워지고, `is_diff=False` 라 라이브 `/joint_states` 를 읽는 686~698 분기도 타지 않는다.

> 새 궤적의 **첫 점 velocity 는 언제나 0** 이다. 로봇이 달리는 중에 "정지 상태에서 출발하는"
> 궤적을 인계받으므로, 교체마다 속도 불연속이 남는다.

이건 튜닝 실패가 아니라 플래너의 성질이라 `lookahead_s` 를 아무리 키워도 안 없어진다.
할 수 있는 건 셋뿐이다 — **`handover_s` ↑ / `replan_hz` ↓ / `vel_scale` ↓.**
크기는 눈으로 재지 말고 교체 로그와 `summary()` 의 **`이음새 N rad/s`**(교체 순간의 실측
관절속도 최대성분 = 불연속의 크기) 로 본다. 0 에 가까울수록 매끄럽다.

⚠️ `start_pos` 를 아예 안 주면(`is_diff=True`) cuMotion 이 `/joint_states` 의 실제 velocity 를
읽는다(`:694-698`). 대신 lookahead 가 사라져 204 ms 뒤처진 상태로 계획하게 되므로,
이 루프에서는 그쪽 손해가 더 크다고 보고 현재 구조를 유지한다.

🔴 **3 Hz 위로 올리는 건 의미가 없다.** `robot_segmenter_node` 가 3.7 Hz 라
nvblox 지도 자체가 그 속도로만 갱신된다(`config/README.md` 병목 항목). GPU 부하만 늘어난다.

## 5. 안전 — 코드가 하는 것과 사람이 해야 하는 것

코드가 하는 것:
- **장애물 경로 gate** (`require_obstacle_pipeline`, 기본 true): ESDF 서비스가 없으면
  **이동을 거부**한다. nvblox 없이도 계획은 성공하므로 이게 없으면 통과한 뒤에야 안다 (2절)
- **시작점 점프 검사** (`max_start_jump`, 0.25 rad): 예측이 빗나간 궤적은 발행하지 않고 버린다
- **연속 실패 차단** (`max_consecutive_failures`, 4회): 감속 정지 후 종료
- **감속 정지** (`brake()`): goal 을 cancel 하면 JTC 가 그 자리를 홀드해 급정지가 된다.
  정상 종료 경로에서는 현재 속도에서 0 까지 등감속하는 짧은 궤적을 대신 쏜다
- **비상정지** (`emergency_stop()`): `/dsr01/motion/move_stop`, 기본 Soft stop

사람이 해야 하는 것:
- **`mode:=check` 를 먼저** 돌린다. 여기서 걸리는 게 실기에서 걸리는 것보다 싸다
- **첫 실행은 `vel:=0.15`**, 비상정지 버튼에 손을 올린 채로
- `pingpong_a_deg`/`pingpong_b_deg` 를 **`mode:=joint` 로 각각 따로 한 번씩** 가보고 눈으로 확인
- ⚠️ **루프가 도는 동안 `movej`/`movel` 을 부르지 말 것.** MoveIt 경로와 두산 네이티브 모션
  서비스가 **같은 DRFL TCP 연결 하나**를 공유한다(`ARCHITECTURE.md` 3절). 모션 모드가 충돌한다

## 6. 이 패키지를 어디에 두고 어디서 돌리나

### 결론

**GPU PC 호스트의 `~/cobot2_ws/src/cumotion/`.** 컨테이너 안에서 빌드·실행한다.

`testcommand.md` 3절의 기동 명령이 이미 그 디렉토리를 마운트하고 있어서 추가 설정이 없다:

```bash
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
#                   └─ 호스트 ~/cobot2_ws  →  컨테이너 /workspaces/cobot2_ws
```

🔴 **컨테이너에서 돌릴 거면 마운트된 경로 안에 있어야 한다.** 도커는 마운트 안 된 호스트
디렉토리를 아예 못 본다. 다른 곳에 두려면 `-v` 를 하나 더 붙인다:

```bash
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws -v $HOME/내경로:/workspaces/mypkg"
```

⚠️ `run_dev.sh` 는 컨테이너를 재사용하지 않고 **매번 새로 만든다**(`testcommand.md` 3절).
마운트는 띄울 때마다 붙여야 하고, 그래서 마운트를 늘릴수록 기동 명령이 길어진다.

### 왜 `~/cobot2_ws` 인가 — 기능이 아니라 관리 때문이다

이 패키지의 `esdf_service_name` · `base_frame` · `voxel_topic` 은 `config/` 의
`cumotion_planner.yaml` / `nvblox_realtime.yaml` 과 **짝을 맞춰야 하는 값**들이다
(어긋나면 에러 없이 조용히 장애물을 놓친다 — 2절). 같은 트리 안에 있어야 같이 고친다.

### 🔴 호스트에서 돌려도 된다

이 패키지는 **GPU 를 안 쓴다.** CUDA·curobo·moveit 코어 라이브러리 전부 무관하고,
필요한 건 메시지 패키지뿐이다(순수 액션 클라이언트라서 — 3절).

```bash
# 호스트에서 이 4개가 다 나오면 호스트 실행 가능
ros2 pkg list | grep -E "^(moveit_msgs|control_msgs|visualization_msgs|dsr_msgs2)$"
# moveit_msgs 가 없으면:  sudo apt install ros-humble-moveit-msgs
```

호스트 실행의 이점: 컨테이너를 새로 띄울 때마다 재빌드할 필요가 없고, 비상정지용
`dsr_msgs2` 가 호스트엔 확실히 있다(bringup 이 쓴다).

**어디서 돌리든 통신은 된다.** 이 노드는 `/move_action`(컨테이너)과 `/dsr01/...`(호스트)을
동시에 잡아야 하는데, T7 move_group 이 컨테이너에서 호스트 `controller_manager` 를 이미
그렇게 쓰고 있으니 검증된 경로다. 단 둘은 지킨다:

- `export ROS_DOMAIN_ID=93` — 호스트·컨테이너 양쪽 다
- ⚠️ **`RMW_IMPLEMENTATION` 을 건드리지 말 것.** cycloneddds 로 바꾸면 컨테이너↔호스트
  **서비스**가 안 붙는다(토픽만 됨). `check_obstacle_pipeline()` 도 서비스 조회라 같이 깨지고,
  그러면 "nvblox 가 없다"고 오판해서 이동을 거부한다.

### 빌드

```bash
# 컨테이너 T8
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=93

cd /workspaces/cobot2_ws
colcon build --symlink-install --build-base build_container \
             --install-base install_container --packages-select cumotion
source install_container/setup.bash
```

⚠️ **`install_container` 를 따로 쓰는 이유가 있다.** 호스트와 컨테이너가 같은 `install/` 에
빌드하면 파이썬 경로·ABI 가 섞여 한쪽이 깨진다. 호스트에서도 빌드할 거면 호스트는
기본 `build/`·`install/` 를, 컨테이너는 `build_container/`·`install_container/` 를 쓴다.

`--symlink-install` 이면 파이썬 파일과 yaml 을 고쳐도 **재빌드 없이** 반영된다(노드 재시작만).
호스트에서 편집하면 마운트를 통해 컨테이너에 즉시 보인다.

### 🔴 0 에서 시작하는 전체 기동 순서 (2026-08-07 실기 관통 확인)

> **T1~T7 은 `config/testcommand.md` 의 발췌다.** 그쪽이 단일 출처이고, 어긋나면 그쪽이 이긴다.
> 여기 두는 이유는 T8(이 패키지)만 따로 보면 못 돌리기 때문이다. **T8 절은 여기가 주인이다.**

터미널 8개. **T2(실기 로봇)는 사람이 직접 띄운다.**

#### 호스트 터미널 — 매 터미널 첫 줄

```bash
cd ~/cobot2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=93
```
🔴 **`ROS_DOMAIN_ID` 를 빠뜨리면 노드가 하나도 안 보인다.** 2026-08-07 에 T8 에서 실제로 겪었다 —
`/move_action 액션 서버 없음` 으로 나와서 T7 이 죽은 줄 알았는데 도메인이 0 이었던 것뿐이다.

```bash
# T1 카메라
ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=848x480x15 color_profile:=848x480x15
#   확인: ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw   → 10~15 Hz
#   확인: ros2 node list | grep -c "camera/camera"                        → 1 (2면 depth 반토막)

# T2 실기 로봇  ← 사람이 띄운다
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false
#   확인: ros2 topic echo /joint_states --once   → name/position/velocity 각 12개
#   🔴 velocity 가 비어 있으면 cuMotion 계획이 전부 실패한다
```

#### T3 — 컨테이너

```bash
export ROS_DOMAIN_ID=93          # ⚠️ run_dev.sh 가 -e 로 넘긴다. 먼저 해야 한다
cd ~/cobot2_ws/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

🔴 **`run_dev.sh` 는 `docker run -it --rm` 이다 — 그 터미널을 닫으면 컨테이너가 통째로 삭제된다.**
이미 떠 있으면 `docker exec -it isaac_ros_dev-x86_64-container bash` 로 들어간다(새로 안 만든다).

**새 컨테이너면 맨 처음 한 번:**
```bash
bash /workspaces/cobot2_ws/scripts/container_setup.sh    # warp 1.5.0 / numpy 1.26.4 / cv2 OK
```
🔴 **이걸 빠뜨리면 T4 는 `import cv2 → _ARRAY_API not found`, T6 은 `module 'warp' has no
attribute 'torch'` 로 죽는다.** 2026-08-07 에 둘 다 겪었다. 컨테이너를 새로 만들 때마다 매번이다.
(출력의 `🔴 패치가 없다` 줄은 git `dubious ownership` 오탐이니 무시 — curobo 패치 2개는 살아 있다)

#### 컨테이너 셸 — T4~T7 매 셸 첫 줄

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93
```
⚠️ `RMW_IMPLEMENTATION` 은 건드리지 않는다 (6절).

T4~T7 명령은 `config/testcommand.md` 4~7절 그대로. 각 단계 확인:

| | 노드 | 확인 |
|---|---|---|
| T4 | `robot_segmenter_node` | `ros2 topic hz /cumotion/camera_1/world_depth` → 3~4 Hz **(T5 가 떠야 나온다 — 구독자가 있을 때만 발행한다)** |
| T5 | `nvblox_node` (`esdf_mode:=3d`) | `ros2 service list \| grep esdf` · `pgrep -f nvblox_node` |
| T6 | `cumotion_planner_node` | 로그에 `cuMotion is ready for planning queries!` (5~10초) |
| T7 | `moveit.launch.py standalone:=false octomap:=true cumotion:=true` | 로그 3줄: `ompl` / `isaac_ros_cumotion` 파이프라인 + `Configured and activated dsr_moveit_controller` |

#### RViz (T7 창, 재시작할 때마다)

- `Add → rviz_default_plugins/Marker` → Topic **`/curobo/voxels`** ← **MarkerArray 아님**
- `Trajectory` 디스플레이 → `Interrupt Display: **true**` (기본 false 면 궤적이 실제보다 뒤처져 보인다)
- 🚨 **MotionPlanning 패널의 Plan 버튼을 누르지 말 것** — `planner_busy` 로 T8 이 `FAILURE(99999)` 로 실패한다
- ⚠️ 보이는 궤적(`/display_planned_path`)은 **계획된 것**이지 실행 중인 것이 아니다.
  `max_start_jump` 로 폐기된 궤적도 거기 그려진다. 실행 실체는
  `ros2 topic echo /dsr01/dsr_moveit_controller/controller_state` (desired/actual/error)

#### T8 — 호스트 (이 패키지)

🔴 **T8 은 컨테이너가 아니라 호스트에서 돌린다.** GPU 를 안 쓰고, 비상정지용 `dsr_msgs2` 가
호스트에 확실히 있다(6절). 빌드도 호스트다:
```bash
colcon build --symlink-install --packages-select cumotion
```

### 실행

```bash
# ① 사전 점검 — 로봇 안 움직임 (plan_only 로 계획만 1회)
ros2 launch cumotion dynamic_avoid.launch.py mode:=check
#   → "cuMotion 이 장 본 장애물 복셀 N개" 가 나와야 한다. 0 개면 장애물을 못 보는 상태다
#   ⚠️ check 는 **제자리 계획**이라 RViz 에 볼 궤적이 없다. 궤적을 보려면:
#      python3 scripts/bench_planning_time.py --repeat 10   (plan_only 고정, 로봇 안 움직임)

# ② 관절 목표 1회 이동 (deg)
ros2 launch cumotion dynamic_avoid.launch.py mode:=joint \
    goal_joint_deg:="[0.0, 0.0, 90.0, 0.0, 90.0, 0.0]" vel:=0.15

# ③ 🔴 동적 회피 시연 — 왕복 중에 작업영역에 손/상자를 넣는다
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong vel:=0.2

# ④ 대조군 — 재계획을 끈 같은 왕복. ③ 과의 차이가 유일한 증거다
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong static:=true vel:=0.15

# ⑤ TCP 목표 (m, deg)
ros2 launch cumotion dynamic_avoid.launch.py mode:=pose \
    goal_pose:="[0.45, 0.0, 0.35, 180.0, 0.0, 0.0]" vel:=0.15

# ⑥ OMPL(octomap)로 같은 루프 — 플래너 비교용
ros2 launch cumotion dynamic_avoid.launch.py mode:=joint pipeline:=ompl vel:=0.15

# launch 없이 직접
ros2 run cumotion dynamic_avoid --ros-args \
    --params-file $(ros2 pkg prefix cumotion)/share/cumotion/config/dynamic_avoid.yaml \
    -p mode:=check
```

launch 인자에 없는 파라미터는 `config/dynamic_avoid.yaml` 을 고친다(주석이 본체다).
launch 인자가 yaml 을 덮어쓴다.

### 종료 — 올린 순서의 반대로

T7 → T6 → T5 → T4 → T2 → T1 각각 `Ctrl+C`. 컨테이너 셸은 `exit`.

```bash
ps -eo pid,user,cmd | grep -E "move_group|nvblox|cumotion|segmenter|realsense2_camera_node" | grep -v grep
nvidia-smi --query-gpu=memory.used --format=csv,noheader     # ~15 MiB 면 반납 완료
```

🔴 **`pkill -f` 를 쓰지 말 것.** 자기 명령줄에도 매칭돼 자기 셸을 먼저 죽이고, 공유 랩탑이라
남의 프로세스까지 걸린다. PID 로 죽인다 (`testcommand.md` 10절).

⚠️ `run_dev.sh` 로 띄운 컨테이너는 그 셸에서 나가면 **삭제된다**(`--rm`). 다음에 다시 띄우면
`container_setup.sh` 를 또 돌려야 한다.

## 7. 라이브러리로 쓰기

pick-and-place 같은 걸 짤 땐 노드를 쓰지 말고 `arm.py` 를 직접 import 한다.

```python
import rclpy
from cumotion.arm import ArmConfig, CumotionArm

rclpy.init()
cfg = ArmConfig()          # cfg 를 주면 ROS 파라미터를 선언하지 않는다
cfg.vel_scale = 0.2
arm = CumotionArm(cfg); arm.start_spin(); arm.wait_until_ready()

target = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]           # rad
arm.run_to_goal(arm.joint_goal(target), goal_positions=target, replan_hz=3.0)

print(arm.summary())       # 계획 N회 / 궤적 교체 M회 / 계획시간 평균·최대
```

그리퍼(RG2)는 MoveIt 밖이다 — `/onrobot/sendCommand`(`onrobot_rg_msgs/srv/SetCommand`)
서비스로 따로 부른다. 이 패키지에는 넣지 않았다.

## 8. 증상 → 어디를 볼 것인가

| 증상 | 원인 | 조치 |
|---|---|---|
| check 에서 "액션 서버 없음" | T7 move_group 미기동 | `ros2 action list \| grep move_action` |
| check 에서 "dsr_moveit_controller 없음" | 컨트롤러 spawn 실패 | T7 로그의 `Configured and activated dsr_moveit_controller` |
| `/joint_states 에 velocity 가 없다` 경고 | bringup 설정 | `publish_default_velocities: True` |
| `START_STATE_IN_COLLISION` 반복 | 로봇 몸이 nvblox 지도에 찍힘 | T4 `distance_threshold` ↑, T5 재시작 |
| `GOAL_IN_COLLISION` | 목표가 장애물 안 | 치워질 때까지 못 간다. 정상 동작이다 |
| "새 궤적 시작점이 실측과 어긋남" 반복 | 인계 예측 실패 | `lookahead` ↑ 또는 `vel` ↓ |
| 궤적 교체 순간 덜컹거림 | 인계 불연속 | `handover_s` 를 0.05 → 0.1 |
| `ESDF 서비스 없음` → 이동 거부 | T5 nvblox 미기동/사망 | `pgrep -f nvblox_node`. 죽었으면 `esdf_mode:=3d` 로 재기동 |
| `복셀 0개` 경고 | nvblox 는 살아 있는데 지도가 빔 | 카메라 FOV / `workspace_bounds_*` / T4 `world_depth` 발행 |
| `복셀 미수신` | `/curobo/voxels` 안 옴 | `publish_curobo_world_as_voxels: true` 확인. 대기 중엔 원래 안 온다 |
| 계획은 되는데 장애물을 통과 | T4/T5 누락 또는 `read_esdf_world:=False` | `testcommand.md` 4·5절. **2절의 감시 두 겹이 이걸 잡으라고 있다** |
| 궤적 교체가 0회 | `static_mode` 가 켜졌거나 목표가 너무 가까움 | 종료 시 찍히는 `summary()` 확인 |
| launch 가 `FileNotFoundError` | share 에 config 미설치 | `setup.py` 의 `data_files` 확인 후 재빌드 |
| 컨테이너에서 `Package 'cumotion' not found` | 마운트 밖에 뒀거나 `install_container` 미소스 | 6절 — `-v` 마운트 확인 후 재빌드 |
| 토픽은 보이는데 **서비스만** 안 붙는다 | `RMW_IMPLEMENTATION` 을 cyclonedds 로 바꿈 | 6절 — 지우고 기본값(fastrtps)으로 |
| 아무 노드도 안 보인다 | `ROS_DOMAIN_ID` 불일치 | 호스트·컨테이너 양쪽 `export ROS_DOMAIN_ID=93` |

## 9. 아직 안 된 것 / 검증 안 한 것

- ~~🔴 **GPU PC 실기에서 아직 한 번도 안 돌렸다.**~~
  **2026-08-07 실기 실행 완료.** `mode:=check` / `mode:=joint`(static 양쪽) 확인. 결과는 0절.
  아직 안 돌린 것: **`mode:=pingpong`, `mode:=pose`, `pipeline:=ompl`, 그리고 실제 장애물 투입.**
  0절의 두 실험은 **장애물 없이** 돌린 것이라 회피 자체는 여전히 미검증이다.
- ~~**재계획 시 시작 속도(velocity)를 cuMotion 이 실제로 반영하는지 미확인.**~~
  **확인됨 — 반영하지 않는다** (`cumotion_planner.py:675` 소스 + 실기 양쪽).
  ⚠️ **결과 예측이 틀렸었다.** "교체마다 덜컹인다"고 적어놨는데, 실측된 증상은 정반대다 —
  덜컹이지 않는 대신 **속도가 아예 안 붙어서 목표에 못 간다**(0절). `이음새` 최대 0.037 rad/s.
- ~~**JTC 가 새 goal 로 기존 goal 을 선점하는 동작에 의존한다.**~~
  **확인됨 — 선점 교체가 동작한다.** 2026-08-07 실기에서 궤적 교체 155회가 `cancel_execution()`
  없이 끊김 없이 이뤄졌다. `send_trajectory()` 앞에 취소를 넣을 필요가 없다.
- **`pingpong_a_deg`/`pingpong_b_deg` 는 안전 검증된 자세가 아니다.** 임의로 잡은 값이다.
  (A = `[45,0,90,0,90,0]` 만 `mode:=joint` 로 도달 확인됨. B 는 아직 안 가봤다)
- 🔴 **`swap_threshold_rad`(0절의 수정)를 실기에서 아직 튜닝 안 했다.** 기본값은 추정치다.
  너무 크면 장애물이 와도 교체를 안 하고(=회피 실패), 너무 작으면 0절의 기어감이 재현된다.
  `mode:=joint` 로 **도착 시간이 static 의 7.7초에 근접하는지**부터 확인하고 올린다.
- **최악 반응시간 미측정.** 파이프라인 지연 ~0.6 s + 재계획 주기 0.33 s + 인계 0.35 s
  ⇒ 장애물이 나타나고 궤적이 바뀌기까지 **1.3 s 내외**로 추정된다. 사람 손 속도에는 부족할 수
  있다. 실제로 재봐야 하고, 부족하면 `vel` 을 낮추는 것 말고는 이 코드가 할 수 있는 게 없다
  (진짜 해법은 세그멘터 3.7 Hz 병목을 푸는 것).
- **패키지 이름 `cumotion` 은 `isaac_ros_cumotion` 과 다른 것이다.** 파이썬 모듈명도
  `cumotion` 이라 헷갈릴 수 있다 — `from cumotion.arm import ...` 는 **이 패키지**다.
