<!-- meta
updated: 2026-08-06 12:00
status:  frozen
owns:    없음 — 완료된 계획 원본. 결과·채택값은 review_moveit.md 소유
-->

# 2026-08-03 계획 — octomap 결합 + self-filter 검증

> ✅ **완료(2026-08-03).** 이 문서는 계획 이력으로만 보존한다 — **결과·채택값·검증 상태는
> [[ws/cobot2/review_moveit]]가 소유한다.** 여기 적힌 값(`padding_offset` 0.03 등)은 실행 전 목표치라
> 현재 파일과 다르다. 문서 지도: [[ws/cobot2/README]]

**작성:** 2026-08-02 | **관련:** [[ws/cobot2/state]] · [[ws/cobot2/context/constraints]] · [[ws/cobot2/M0609_perception_motion_sprint_plan]]

---

## 0. 어제(08-02) 한 일 — 이어받는 지점

| 항목 | 결과 |
|---|---|
| 캘리브 (eye-to-hand, `corecode/Calibration_Tutorial`) | ✅ 확정. `T_cam2base.npy` → `m0609_rg2_bringup/config/`에 사본 |
| 좌표 규약 버그 (OpenCV optical → ROS body) | ✅ 수정. `calib_npy_to_tf.py`가 기본 보정. 육안 검증 통과 |
| `base_link → camera_link` static TF | ✅ `camera.launch.py`가 npy에서 **매 실행 계산**. 하드코딩 제거 |
| RealSense 분리 런치 | ✅ `camera.launch.py` 신설. bringup은 로봇 전용으로 원복 |
| MoveIt 구동 | ✅ move_group + RViz + `dsr_moveit_controller` spawner. **실기 Plan·Execute 성공 확인** |
| `sensors_3d.yaml` + `octomap:=true` 주입 | ✅ 작성·주입 확인. ⛔ **플러그인 미설치라 아직 동작 안 함** |
| `octomap_frame` | ✅ `base_link` 확정 (`world`는 planning scene이 모름 — 실측) |
| 통합 README | ✅ ws 루트 `README.md` (팀원용 3터미널 실행 절차) |

**남은 한 줄:** octomap이 아직 한 번도 생성된 적 없다. 플러그인이 없어서다.

---

## 1. 오늘의 순서 (위가 막히면 아래로 안 내려간다)

### ① 플러그인 설치 — 5분, 사용자가 직접 (sudo)
```bash
sudo apt install ros-humble-moveit-ros-perception
```
없으면 `move_group`이 `Failed to load sensor: realsense_pointcloud ... PointCloudOctomapUpdater ... does not exist`를 뱉고 **조용히** octomap 없이 계속 돈다.

### ② 카메라 프로파일 낮추고 켜기 — CPU가 먼저 문제다
근거: 이 랩탑은 **i7-10510U 15W / GPU 없음**, `ros2_control_node`가 상시 204%. 기본 848×480×30 = **12.2 M point/s**는 안 돌아간다는 게 어제 `octomap_server`에서 이미 확인됐다.

`camera.launch.py`에 depth 프로파일 인자 추가 후:
```bash
ros2 launch m0609_rg2_bringup camera.launch.py   # depth 424x240x15 목표
```
- ✅ 판정: `ros2 topic hz /camera/camera/depth/color/points` ≈ 15, `top`에서 `move_group` < 100%
- ❌ 이면 `sensors_3d.yaml`의 `point_subsample`을 2~3으로

### ③ **self-filter 검증 ← 오늘의 진짜 관문**
```bash
# RViz에 PointCloud2 → /moveit/filtered_cloud 추가
```
- ✅ 판정: **로봇 팔이 클라우드w서 지워져 있다**
- ❌ 팔이 남아 있으면 로봇이 자기 몸을 장애물로 보고 한 발짝도 못 움직인다 → `sensors_3d.yaml`의 `padding_offset`을 0.03 → 0.05 → 0.08로 올린다

여기서 막히면 ④ 이후는 의미가 없다. **오늘 여기까지만 되어도 성공.**

### ④ 장애물 → 궤적 변화 확인
1. RViz Scene Objects로 상자 배치 (**프레임 반드시 `base_link`** — `world`면 조용히 무시됨)
2. Plan → 경로가 상자를 우회하는지
3. 실물 장애물을 카메라 앞에 놓고 복셀이 뜨는지 → Plan이 바뀌는지
4. 실기 Execute는 **③ 통과 후에만**

### ⑤ README 4절 체크1~3 실측 (⚠️ 미검증 표기 제거)
실기 켠 김에: `tf2_echo base_link camera_link` / `topic hz .../points` / `topic echo /dsr01/joint_states`

### ⑥ D435i depth rosbag 녹화 — 시간 남으면
개인PC에서 실기 없이 개발하려면 필수. 절차·명령은 [[ws/cobot2/state]] "출근 후 D435i 세션".
**녹화 중 임시 static TF 띄우지 말 것.**

---

## 2. 미룬 것 (오늘 하지 않는다)

- **`AttachedCollisionObject`** — 그리퍼가 물건을 집었을 때 planning scene에 붙이기. 픽앤플레이스에는 필수지만 octomap이 먼저 돌아야 의미가 있다.
- **경유지 정리** — MoveIt+octomap이 자유공간 대이동을 맡으면 기존 `movel` 경유지 대부분이 불필요해진다. 단 **pre-grasp/grasp 구간의 경유지는 유지한다**(접근 방향은 충돌 제약이 아니라 작업 요구, Cartesian 경로는 충돌회피를 안 함, OMPL은 확률적이라 비재현적).
- **카메라 마운트 강성** — 캘리브는 계속 잠정(provisional).
- **npy 사본 정리** — 정본은 `corecode/Calibration_Tutorial/T_cam2base.npy`(사용자 결정 2026-08-02).
  `m0609_rg2_bringup/config/` 사본은 삭제 대상이지만, **`camera.launch.py`가 corecode를 직접 읽게
  바꾼 다음**이다. 순서 반대로 하면 static TF가 사라진다.

---

## 3. ⚠️ 오늘 지킬 안전 규칙

**로봇에 도달하는 명령 경로가 두 개 살아 있다** (소스 확인, [dsr_hw_interface2.cpp:494-503](../../src/cobot_rg2/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp#L494-L503)):

| 경로 | 방식 |
|---|---|
| `dsr_controller2` | 서비스 `movej`/`movel` → DRFL 직접 |
| `dsr_moveit_controller` (JTC) | `Drfl.servoj_rt()` / `Drfl.amovej()` |

둘 다 active다. **기존 `movel` 노드와 MoveIt Execute를 동시에 돌리지 않는다.** 한 시점에 한 경로만.

그 외:
- 실기 Execute 전 fake execution / virtual 모드로 먼저
- `realsense-viewer`를 먼저 닫는다 (USB 독점 → "TF 프레임 없음"으로 오진됨)
- MoveIt 2.5.9는 Ctrl-C에 segfault를 뱉는다 — 알려진 종료 순서 버그, 무시

---
확신도: 어제 결과는 검증됨(빌드·실행·실측), 오늘 계획의 소요시간·성공 여부는 추측
