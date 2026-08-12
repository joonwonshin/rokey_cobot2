<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    MoveIt2 OMPL+Octomap 수행 기록 · 채택 설정값 스냅샷 · cuRobo 비교 설계
-->

# MoveIt2 OMPL + Octomap 충돌회피 — 과정 리뷰 & cuRobo 비교 설계

**작성:** 2026-08-03 | **대상 스택:** `m0609_rg2_moveit` (MoveIt2 OMPL + `occupancy_map_monitor/PointCloudOctomapUpdater`)
**목적:** 오늘 수행한 3D octomap 생성→경로계획→회피 과정을 기록해 두고, GPU PC 확보 후 cuRobo와 같은 조건으로 비교할 수 있는 기준선(baseline)을 남긴다.

> 📁 문서 지도: [[ws/cobot2/README]] · 계획서: [[ws/cobot2/plans/2026-08-03-octomap-integration]](이 문서가 그 결과다)
> · 실측 사실: [[ws/cobot2/context/constraints]] · **이 문서에서 발견된 오류: [[ws/cobot2/errors-log]] §1·2·7·8**

> ⚠️ 이 문서의 "수행 과정"은 사용자 구두 보고 + 이 세션에서 `git diff`로 확인한 설정 변경을 근거로 재구성했다.
> `ros2 topic hz` / `tf2_echo` / RViz 스크린샷 등을 **이 턴에서 직접 실행해 확인한 것은 아니므로** 3절에 검증 상태를 분리해 표시한다.

---

## 0. 여기까지 온 경위 (2026-08-02, `state.md`에서 이관)

- ✅ 캘리브 결과 → static TF 연결 성공: `base_link → camera_link`, `tf2_echo base_link camera_depth_optical_frame` 정상.
  (이 줄에 처음 적혀 있던 993.4 mm는 좌표 규약 버그 수정 **전**의 값이라 폐기.)
  ⚠️ **여기 적혀 있던 `[1.148, 0.640, 0.678]` (약 1.48 m)는 2026-08-03 재캘리브로 폐기.**
  값의 출처는 `T_cam2base.npy` 하나뿐이다 — **거리 수치를 문서에 베껴 적지 말 것.**
  읽는 법과 사고 이력은 [[ws/cobot2/context/constraints]].
- ✅ **`base_0`는 TF에 존재하지 않는 프레임임을 확인** — `base_link`가 맞다. 계획서 전체를 `base_0`→`base_link`, `link6`→`link_6`으로 수정 완료. 근거·표는 [[ws/cobot2/context/constraints]].
- ✅ **좌표 규약 버그 수정 후 육안 검증 통과** — 클라우드 속 로봇 팔이 모델에 정확히 포개짐. 캘리브 잠정값 사용 가능.
- ✅ **`octomap_server`는 이 파이프라인에서 불필요하다고 결론.** MoveIt은 `/octomap_binary`를 구독하지 않고
  `move_group` 내부에서 octree를 직접 만든다. 둘 다 돌리면 CPU 이중 소모 — 정식 경로는 `sensors_3d.yaml`이다.
- ✅ **`m0609_rg2_moveit/config/sensors_3d.yaml` 작성 완료** + `moveit.launch.py`에서 주입(`octomap:=true` 기본).
  실제 채택값: **`octomap_frame: base_link`**, **`octomap_resolution: 0.02`**(계획은 `0.03`),
  토픽 `/camera/camera/depth/color/points`(계획은 `/depth/points_xyz`), 센서명 `realsense_pointcloud`.
  `ros2 param get /move_group`으로 주입까지 확인. ✅ `moveit-ros-perception` 설치·플러그인 로드 확인(2026-08-03).
  ※ 한때 `world`로 적혀 있었으나 **틀렸다**(2026-08-02 실측): SRDF에 `virtual_joint(fixed, parent_frame="world")`가
  있어도 MoveIt은 fixed 타입으로는 모델 프레임을 만들지 않아 플래닝 프레임이 루트 링크(`base_link`)로 남는다.
  `frame_id='world'`로 CollisionObject를 발행하면 `Unknown frame: world` 에러와 함께 **조용히 무시**된다.
  RViz Scene Objects도 같은 규칙. 경위는 [[ws/cobot2/context/constraints]].
- ✅ **캘리브 결과를 launch가 npy에서 직접 계산** — `m0609_rg2_bringup/config/T_cam2base.npy` →
  `camera.launch.py`가 매 실행 `calib_npy_to_tf.py`로 static TF 생성. **하드코딩된 `static_transform_publisher`
  명령을 다시 만들지 말 것** (낡은 값으로 340 mm 어긋난 이력 있음).
- ✅ **런치 3분할 확정**: `bringup`(로봇 전용) / `camera`(RealSense + 캘리브 TF) / `moveit`(move_group + JTC spawner + RViz).
  `bringup_camera.launch.py`는 **eye-in-hand 전용**(URDF가 camera_link를 tool0에 붙임) — 현재 리그와 섞으면 TF가 깨진다.
- ✅ **MoveIt 실기 Plan·Execute 성공** — Execute ABORTED의 원인은 두 개였다: bringup이 `dsr_moveit_controller`를
  안 띄움 + 네임스페이스 불일치. `moveit.launch.py`에 spawner 추가 + `moveit_controllers.yaml`의 컨트롤러 이름을
  절대경로 `/dsr01/dsr_moveit_controller`로. **`dsr_controller2`와 동시 active 가능**(인터페이스를 claim하지 않는 서비스 래퍼).
- ✅ **팀원용 통합 `README.md` 작성**(ws 루트) — 3터미널 실행 절차·인자표·기능확인 체크1~3·알려진 함정.

---

## 1. 수행한 파이프라인 (3터미널 구성)

`md/state.md`·`constraints.md`에 이미 확정된 런치 3분할을 그대로 따른다.

```bash
# T1 — 로봇
ros2 launch m0609_rg2_bringup bringup.launch.py model:=m0609 mode:=<real|virtual>

# T2 — 카메라 + 캘리브 static TF (npy 기반 자동 발행 + 이번 세션에 추가된 미세보정)
ros2 launch m0609_rg2_bringup camera.launch.py \
  depth_profile:=424x240x15 color_profile:=424x240x15 \
  drpy:="0 1.5 0"     # 예시 — 실제 보정값은 팀원이 맞춘 값을 씀

# T3 — MoveIt (move_group + JTC spawner + RViz, octomap:=true 기본)
ros2 launch m0609_rg2_moveit moveit.launch.py
```

경로:
```
RealSense (/camera/camera/depth/color/points, pointcloud.enable=true)
  → move_group 내부 occupancy_map_monitor::PointCloudOctomapUpdater (self-filter → raycast)
  → PlanningScene.world.octomap
  → OMPL(FCL 충돌검사: 로봇 mesh vs octree cell) → 궤적
```
**`octomap_server` 노드는 쓰지 않는다** — MoveIt은 `/octomap_binary`를 구독하지 않고 자체 octree를 만든다(constraints.md에 실측 근거 있음). 둘 다 띄우면 같은 클라우드로 octree를 이중으로 만들어 CPU를 낭비한다.

### 확인 순서 (관문 — 위가 안 되면 아래로 가지 않는다)
1. `/moveit/filtered_cloud`를 RViz에서 육안 확인 — 로봇 팔이 지워졌는가 (self-filter 실패 시 자기 몸을 장애물로 보고 못 움직임)
2. RViz Scene Objects / `/monitored_planning_scene`에 실제 장애물 클라우드가 반영되는가
3. 목표 pose로 드래그 → Plan(fake execution) → 궤적이 장애물을 피하는가
4. 필요 시에만 실기 Execute (속도 스케일 20~30% 제한)

---

## 2. 실제 채택 설정값 스냅샷

> 값의 단일 출처는 파일이다(`sensors_3d.yaml`, `camera.launch.py`, `ompl_planning.yaml`). 여기는 2026-08-03 시점 스냅샷일 뿐, 값이 바뀌면 이 표가 아니라 파일이 맞다.

### `sensors_3d.yaml`
| 파라미터 | 값 | 근거 |
|---|---|---|
| `point_cloud_topic` | `/camera/camera/depth/color/points` | RealSense가 직접 발행, `depth_image_proc` 불필요 |
| `max_range` | **2.0 m** | 2.5 → 1.5(CPU 절감 목적) → **2.0(되돌림)**. 1.5는 카메라~base 실측 **1.684 m**보다 작아 베이스 부근을 잘라내고 있었고, CPU 이득도 체감되지 않았다(사용자 확인). **CPU는 `point_subsample`·카메라 프로파일에서 줄인다** — 비용은 거리가 아니라 점 개수에 비례한다. 경위: [[ws/cobot2/errors-log]] §7 |
| `point_subsample` | **3** | GPU 없는 i7-10510U에서 CPU 부하 축소 (1→3) |
| `padding_offset` | 0.1 m | self-filter 잔여점 방지 (0.03→0.1 — 자기 팔이 장애물로 잡히는 문제 대응) |
| `padding_scale` | **2.0** | 위와 같은 목적의 배율 손잡이 (1.0→2.0) |
| `max_update_rate` | 1.0 Hz | `ros2_control_node` 상시 204% 점유와 경합 방지 (변경 없음) |
| `octomap_frame` | `base_link` | `world`는 planning scene이 모르는 프레임 (constraints.md 실측) |
| `octomap_resolution` | 0.02 m | 변경 없음. 현재는 캘리브 잔차(40.1 mm)가 이보다 커서 정밀도를 캘리브가 지배한다 |

`moveit.launch.py`에 **오늘 신설**: `default_object_padding: 0.02` / `default_robot_padding: 0.0`
(`robot_description_planning`에 주입. scene object 충돌 판정에 더할 여유 거리 — self-filter의 `padding_*`과 별개다).

### `camera.launch.py`
| 항목 | 값 |
|---|---|
| `depth_profile` / `color_profile` 기본값 | `424x240x15` (848x480x30은 이 랩탑에서 12.2 M point/s로 과부하) |
| 캘리브 소스 | `config/T_cam2base.npy` (정본은 `corecode/Calibration_Tutorial/`) |
| 미세보정 인자 (오늘 신규 추가) | `dxyz`(m, base_link 축) / `drpy`(deg, camera_link 축) — 드라이버는 그대로 두고 TF만 재발행해 맞출 수 있게 함 |

### OMPL (`ompl_planning.yaml`)
- 플래너 후보: `AnytimePathShortening`, `SBL`, `EST`, `LBKPIECE`, `BKPIECE`, `KPIECE`, `RRT` 계열 등 — 기본 파라미터, 이번 스프린트에서 개별 튜닝은 아직 안 함(스프린트 계획 Day3 P1 항목).
- `joint_limits.yaml`: `default_velocity/acceleration_scaling_factor = 1.0` (실기 실행 시 별도로 `/move_group` 파라미터를 0.2~0.3으로 낮춤 — 코드 기본값이 아니라 런타임 안전 조치).

---

## 3. 검증 상태 (이 턴 기준)

| 항목 | 상태 | 근거 |
|---|---|---|
| `sensors_3d.yaml`/`camera.launch.py` 튜닝값 변경 | ✅ 검증됨 | 이 턴에 `git diff` 실행해 확인 |
| self-filter 통과(팔이 클라우드에서 지워짐) | 🟡 추론 (구두 보고) | 이 턴에 RViz 실행/스크린샷 확인 안 함 |
| 장애물 놓았을 때 궤적이 실제로 회피 경로로 바뀜 | ✅ 실기 Execute로 확인 (구두 보고) | fake execution이 아니라 **실기 Execute까지 진행**. 다만 아래 캘리브 오차 항목 참고 |
| 캘리브 오차로 장애물 영역 경계가 모호함 | ⚠️ 확인됨 (구두 보고) — **무시되지는 않았음** | 방해 영역이 정확한 경계가 아니라 다소 뭉개진 형태로 잡혔으나, 회피 자체는 수행됨. 정량 오차(cm)는 미측정 — 다음 세션에 알려진 좌표 물체로 측정 필요 |
| planner 성공률/평균 계획시간 수치 | ❌ 미기록 | 스프린트 계획 Day3 P1(`ros2 bag record .../monitored_planning_scene /joint_states`)이 아직 실행 안 됨 |

**캘리브 오차가 미친 영향 (해석):** self-filter의 `padding_offset`(현재 0.1m)이 실제 캘리브 오차를 흡수할 만큼 넉넉했던 것으로 보인다 — 그래서 장애물이 완전히 사라지진 않고 "모호하게"(경계가 부정확하게) 잡혔을 가능성이 높다. `padding_offset`이 이보다 작았다면 오차가 장애물 자체를 지워버렸을 수도 있다. **이건 추론이며, 실측 오차값(cm) 없이는 확정할 수 없다.** state.md의 열린 이슈("카메라 마운트 강성 미확보 → 캘리브는 잠정값")와 정합적이다.

**다음 세션에서 채워야 할 것**: ① 캘리브 오차를 알려진 좌표 물체로 정량 측정(cm) ② planner 성공률/계획시간 로그. 아래 4절의 cuRobo 비교는 이 정량값이 있어야 "같은 조건"이라고 주장할 수 있다.

---

## 4. cuRobo 비교 설계

### 4.1 전제 조건 (아직 미충족)
- GPU PC의 `nvidia-docker` 런타임 확인 (`docker info | grep -i runtime`) — state.md 열린 이슈, 미확인
- cuRobo는 두 경로 중 택1: ① `isaac_ros_cumotion`(Jazzy 재편으로 Humble 지원 불확실, 스프린트 계획 리스크표 참고) ② cuRobo Python 패키지를 커스텀 rclpy 노드로 감싸 Humble 유지 (스프린트 계획 6절 "다음 스프린트 후보" 참고)
- **cuRobo는 기본적으로 nvblox ESDF를 충돌 표현으로 쓴다** — MoveIt은 occupancy octree(FCL mesh-vs-cell)를 쓴다. 두 표현이 다르므로 "같은 클라우드에서 같은 voxel 해상도로 만든 collision world"를 맞추지 않으면 비교가 아니라 서로 다른 문제를 푸는 꼴이 된다. **비교 전 반드시 두 쪽의 `resolution`/`max_range`를 동일하게 맞출 것.**

### 4.2 비교 축 (metrics)
| 축 | 측정 방법 | 왜 보는가 |
|---|---|---|
| 평균/최대 계획 시간 (ms) | `ros2 bag`로 planning request~response 타임스탬프 차 | cuRobo의 GPU 병렬 배치 최적화가 주장하는 핵심 이득 |
| 성공률 | 동일 시작/목표 joint state로 N회(≥30) 반복, 실패=계획 실패 또는 충돌 궤적 | narrow-passage 등 어려운 씬에서 특히 차이 날 것으로 예상 |
| 경로 품질 | 궤적 길이(joint space), 최대 jerk/가속도 | OMPL은 후처리 스무딩 의존, cuRobo는 최적화 기반이라 다를 수 있음 |
| 장애물 갱신 반영 지연 | 장애물 이동 후 재계획까지 걸리는 시간 | octomap은 raycast 기반 프레임 누적, cuRobo/nvblox는 실시간 ESDF 갱신 — 구조적 차이 |
| 자원 사용 | CPU%(OMPL), GPU%/VRAM(cuRobo) | 이 랩탑 CPU-only 제약과 직결 — 스프린트 계획 6-2절의 보류 사유 근거 데이터가 됨 |
| 통합 난이도 | 설정 파일 수, 신규 의존성 수, Humble 호환 여부 | cuRobo 채택 여부의 실질적 결정 요인 |

### 4.3 실험 프로토콜 (초안)
1. **고정 장면 3종**을 정의: 빈 작업공간 / 단순 장애물 1개 / narrow-passage(장애물 2개, 그 사이만 통과 가능한 목표 pose) — 스프린트 계획 Day3 P1 시나리오 재사용
2. 각 장면에서 시작 joint state·목표 pose를 고정하고, OMPL 쪽 먼저 30회 이상 반복 → `ros2 bag record -o review_moveit_<scene> /move_group/monitored_planning_scene /joint_states /planning_scene`
3. cuRobo 확보 후 **동일 장면·동일 시작/목표**로 동일 횟수 반복, 같은 토픽 구조로 기록
4. 두 bag을 같은 스크립트로 파싱해 4.2 표를 채운다 — 스크립트는 이번 스프린트 범위 밖, 필요 시 별도 착수
5. 결과는 이 문서 5절에 추가(새 스냅샷으로, 기존 값은 지우지 않고 날짜별로 남긴다)

### 4.4 알려진 비교 함정 (미리 적어둠)
- **CPU-only 랩탑에서 cuRobo 자체를 못 돌린다** — 비교는 GPU PC에서만 가능. 개인PC 결과(OMPL)와 GPU PC 결과(cuRobo)를 그대로 나란히 놓으면 "다른 머신" 변수가 섞인다. 가능하면 OMPL도 GPU PC에서 한 번 더 돌려 머신 변수를 통제한다.
- `point_subsample:4`, `max_update_rate:1.0` 같은 이 랩탑 전용 완화값을 GPU PC에서도 그대로 쓰면 OMPL 쪽이 불리하게 나온다 — GPU PC에서는 원래 계획값(`point_subsample:1` 등)으로 되돌려 비교할 것.

---

## 5. 결과 로그 (비교 실행 후 채움)

_아직 없음. cuRobo 착수 후 이 절에 날짜별 표로 추가._

---

**확신도:** 추론(1·2절 설정값은 git diff로 확인됐으나 3절의 "실기 Execute로 회피 성공" 및 캘리브 오차 영향 해석은 구두 보고 기반, 이 턴에 재실행 안 함)
**내가 채워넣은 가정:** ① `padding_offset`(0.1m)이 캘리브 오차를 흡수해 장애물이 완전히 사라지지 않고 모호하게만 잡혔을 것이라는 해석은 제 추론이며 실측 오차값 없이는 확정 불가 ② cuRobo 비교는 GPU PC 확보 후로 전제함 ③ narrow-passage 시나리오를 스프린트 계획 Day3 P1과 동일한 것으로 재사용 가정
**확인 요청:** 캘리브 오차를 대략 몇 cm 정도로 체감했나요? (다음 세션 정량 측정의 기준점으로 쓰겠습니다)