<!-- meta
updated: 2026-08-06 12:00
status:  frozen
owns:    없음 — NotebookLM 소스용 스냅샷(08-03 시점, 값 갱신 안 함)
-->

# 2026-08-03 세션 다이제스트 — eye-to-hand 캘리브 · TF · MoveIt2 Octomap 충돌회피

> **이 문서의 용도**: NotebookLM 소스로 넣어 로보틱스 전공지식(핸드아이 캘리브레이션, 좌표 규약,
> TF, MoveIt2 perception 파이프라인, 샘플링 기반 모션플래닝)을 질문하기 위한 자료.
> 실제 현장에서 하루 동안 겪은 문제와 그때 내린 판단을 **원인-증상-근거** 형태로 남겼다.
> 저장소 코드를 모르는 독자도 읽을 수 있도록 배경을 각 절 앞에 붙였다.

**시스템 구성**
- 로봇: 두산 M0609 6축 협동로봇 (ROS 2 네임스페이스 `/dsr01`), OnRobot RG2 2지 그리퍼
- 카메라: Intel RealSense D435i, **eye-to-hand**(로봇에 붙지 않고 작업대 옆 고정) + Logitech C270(미착수)
- 소프트웨어: ROS 2 Humble / Ubuntu 22.04 / MoveIt 2.5.9 / OMPL
- 연산 자원: Intel i7-10510U (4C8T, 15W 노트북 U-시리즈), **NVIDIA GPU 없음**, RAM 15GB
  - 상시 부하: `ros2_control_node`가 CPU 204%(=2코어) 점유. 이 제약이 오늘 튜닝값 대부분의 이유다.

---

## 1. 핸드아이 캘리브레이션 (eye-to-hand)

### 1.1 배경 개념

카메라가 본 좌표를 로봇이 쓰려면 **카메라 좌표계 ↔ 로봇 베이스 좌표계**의 강체변환을 구해야 한다.
이것이 핸드아이 캘리브레이션이고, 두 가지 구성이 있다.

| 구성 | 카메라 위치 | 구하는 변환 | 보드 위치 |
|---|---|---|---|
| **eye-in-hand** | 로봇 팔(플랜지)에 부착 | `T_gripper←camera` | 고정 (작업대) |
| **eye-to-hand** | 작업공간에 고정 | `T_base←camera` | 로봇 팔(그리퍼)에 부착 |

두 경우 모두 **AX = XB** 형태로 귀결된다. 로봇을 자세 i, j 두 개로 움직였을 때
- `A` = 두 자세 사이의 **로봇 쪽** 상대 운동 (순기구학으로 정확히 안다)
- `B` = 두 자세 사이의 **카메라가 본 보드의** 상대 운동 (체커보드 PnP로 구한다)
- `X` = 구하려는 미지의 고정 변환

핵심 성질: **보드↔그리퍼 부착 변환 G는 AX=XB 유도 과정에서 소거된다.**
→ 보드를 그리퍼 어디에 붙였는지는 **정밀할 필요가 없다**. 대신 **수집 20분 내내 움직이지 않는
강성**과 **완전한 평면도**는 타협 불가다. 종이를 테이프로 붙이면 휘고, 휘면 알고리즘의
"모든 코너가 한 평면 위" 가정이 깨진다 → 알루미늄/아크릴/두꺼운 MDF에 전면 접착.

오늘 실제로 확인한 따름정리: eye-to-hand에서 로봇 쪽 기준 프레임을 **flange로 잡든 TCP로 잡든
`T_base←camera` 결과는 같다**. G가 소거되는 것과 같은 이유로, X는 base·camera 쪽 변환이라
팔 쪽 기준 프레임 선택과 무관하다. (합성 데이터 `--selfcheck`로 검증)

### 1.2 오늘의 수치와 판정

`eye2hand_calibration.py`(Park-Martin 자체 구현)에 **자체 진단 리포트**를 오늘 추가했다.
AX=XB를 푼 뒤 각 자세쌍에 대해 잔차를 되짚는다.

| 수집 | 자세 수 | 병진잔차 중앙값 | 30 mm 초과 쌍 | 판정 |
|---|---|---|---|---|
| 직전 | 26장 | **23.5 mm** | — | 경계 |
| 오늘 | 34장 | **40.1 mm** | 31쌍 중 **21쌍** | ⚠️ **불합격** |

**장수를 늘렸는데 나빠졌다** → 데이터 양이 아니라 **자세 품질**의 문제다.
판정 기준선은 octomap voxel 크기 **20 mm**로 잡았다. 캘리브 오차가 voxel보다 크면
지도 정밀도를 캘리브가 지배해버려 해상도를 올리는 의미가 없다.

### 1.3 오늘 배운 함정 3가지

**(a) 내부파라미터(intrinsics)를 캘리브 이미지셋으로 재추정하면 안 되는 경우가 있다**
`cv2.calibrateCamera`로 추정한 `fx`가 D435i 공장값보다 **7.7% 낮게** 나왔다(839.78 vs 909.53).
원인: 카메라가 **고정**이라 보드까지의 **거리 다양성이 부족**하다. 초점거리와 물체 거리는
투영식에서 곱으로 얽혀 있어(`u = fx·X/Z + cx`), 거리 범위가 좁으면 fx와 Z가 서로를 흡수한다
(scale ambiguity). fx가 7.7% 틀리면 거리 추정이 통째로 7.7% 틀어진다.
→ **개체 고유 공장 내부파라미터를 쓴다.** 코드에 `USE_FACTORY_INTRINSICS = True`로 고정.
→ 따름: **RMS 재투영오차는 공장값을 쓰는 동안 결과 품질 지표가 아니다** (참고값일 뿐).

**(b) LOO(leave-one-out)를 재현성 지표로 읽으면 안 된다**
"자세쌍 하나를 빼면 결과가 얼마나 움직이나"를 재현성으로 해석했다가 반증당했다.
- LOO는 **리스트 순서만 섞어도** 같은 크기로 움직인다 → 무작위 요동을 재고 있었다.
- **계통오차(systematic error)에는 완전히 눈이 멀다**: 잔차 큰 쌍을 한꺼번에 빼면 결과가
  **80 mm** 옮겨간 반면 LOO는 3.6 mm였다.
→ 일반 교훈: **민감도 분석은 "무엇에 대한 민감도인지"를 명시해야 한다.** 단일 샘플 제거는
  랜덤 노이즈에만 민감하고, 모든 샘플에 공통으로 실린 편향은 절대 드러내지 못한다.

**(c) 인쇄물의 실제 칸 크기를 캘리퍼스로 재라**
코드에 `square_size`가 24 mm 대신 **25로 하드코딩**돼 있던 버그가 있었다. 이 상태로 24 mm 보드를
쓰면 모든 거리가 25/24 = **4.2% 부풀어** 나온다 (500 mm → 521 mm). 에러는 나지 않는다.
비슷하게 `cv2.findChessboardCorners`가 받는 건 **칸 수가 아니라 내부 코너 수**다
(칸 11×8 → 내부 코너 10×7). 이건 헷갈리면 코너를 한 장도 못 찾고 요란하게 실패하므로 오히려 안전하다.

> **패턴**: 캘리브 버그는 두 종류다 — ① 요란하게 실패하는 것(코너 개수) ② **조용히 스케일만 틀어지는 것**
> (칸 크기, 내부파라미터). ②는 실기에서 "왜 조금씩 빗나가지"로만 나타나므로 수치 검증이 유일한 방어다.

**(d) 수집 시 회전을 섞어야 한다**
자세를 **평행이동만** 시키면 `A`의 회전 성분이 0이 되어 Park-Martin의 `logR()`이
0으로 나눠 **NaN**이 된다. 자세마다 회전을 30° 이상, 여러 축으로 섞는다. (합성 데이터로 재현 확인)

---

## 2. 좌표 규약 — 오늘 세션의 최대 교훈

### 2.1 배경

같은 "카메라 좌표계"라도 분야마다 축 정의가 다르다.

| 규약 | 전방 | 우 | 상/하 | 쓰는 곳 |
|---|---|---|---|---|
| **OpenCV optical** | **+z** | +x | +y (아래) | `cv2` 출력, `*_optical_frame` |
| **ROS body (REP-103)** | **+x** | −y | +z (위) | `camera_link`, 로봇 전반 |

두 규약은 90° 회전 짝만큼 다르다.

### 2.2 증상과 오진

`cv2`가 뱉은 `T_cam2base.npy`를 그대로 ROS `camera_link`로 발행했더니
**포인트클라우드가 로봇 옆으로 통째로 떨어져 나갔다.** TF 트리는 멀쩡히 연결돼 있어서
"캘리브 값이 나쁜가", "`np.linalg.inv(T)`인가"로 오진하기 쉽다 — **둘 다 아니었다.**

**지문**: 발행 중인 TF의 RPY에서 **roll ≈ ±90°**. 실제 값이 `[-95.7°, 12.9°, 110.9°]`였다.

### 2.3 가설을 숫자로 채점하는 법 (일반화 가치 있음)

"눈으로 보니 이게 맞는 것 같다"를 피하려고, **물리적으로 참이어야 하는 값**을 하나 정해
4가지 가설을 전부 채점했다. 기준: 카메라 시선 벡터가 로봇 base를 향하는 각도(작을수록 옳다).

| 가설 | 시선각 |
|---|---|
| T 그대로, body 규약 (틀린 상태) | 81.0° |
| `inv(T)`, body 규약 | 42.0° |
| `inv(T)`, optical 규약 | 52.0° |
| **T 그대로, optical 규약** ✅ | **32.5°** |

남은 32.5°는 카메라가 base 원점이 아니라 그 앞 작업대를 겨냥하기 때문 — 물리적으로 말이 된다.
보정 후 RPY는 `[12.9°, 5.6°, -157.8°]`, "1 m 옆에서 로봇 쪽을 되돌아보는 자세"로 해석 가능하다.

**육안 판정 기준도 엄격하게 정의했다**: depth 이미지에 로봇 팔이 찍혀 있으면 포인트클라우드에도
팔이 있어야 하고, 그게 **로봇 모델 위에 정확히 포개져야** 한다.
"로봇 근처에 있다"는 통과가 아니다.

**수정 위치**: 변환은 **한 곳(`calib_npy_to_tf.py`)에서만** 한다. 호출부마다 고치면 재발한다.
roll이 여전히 ±90° 근처면 경고를 찍게 했다.

---

## 3. TF 트리 — "TF에 있다"와 "그 프레임을 안다"는 별개다

### 3.1 URDF가 두 개면 프레임 이름 체계도 두 개다

이 로봇은 URDF를 **두 개** 쓴다.

| URDF | 소비자 | 프레임 이름 | `/tf`에 발행? |
|---|---|---|---|
| `dsr_description2/…/m0609.urdf.xacro` | `ros2_control_node` 하드웨어 인터페이스 | `base_0`, `link6` | ❌ **안 한다** |
| `m0609_rg2_bringup/…/m0609_with_rg2.urdf.xacro` | `robot_state_publisher` | `base_link`, `link_1`…`link_6`, `tool0`, `rg2_*` | ✅ |

→ 벤더 예제/문서에 나오는 `base_0`은 **TF 트리에 절대 나타나지 않는다.**
그대로 따라 쓰면 전부 `Invalid frame ID "base_0" … frame does not exist`로 죽는다.

실제 트리:
```
world → base_link → link_1 … link_6 → tool0 → rg2_base_link → rg2_*
camera_link → camera_depth_frame / camera_color_frame → *_optical_frame
        ↑ 캘리브 static TF가 없으면 이쪽은 별개의 섬(disconnected tree)
```
eye-to-hand라서 `base_link → camera_link`가 URDF에 없다 — 캘리브 결과로만 이어진다.

### 3.2 planning frame ≠ TF frame (오늘 값을 두 번 뒤집은 항목)

MoveIt SRDF에 `virtual_joint(type=fixed, parent_frame="world", child_link="base_link")`가 있으면
플래닝 프레임이 `world`일 것 같다. **아니다.**
MoveIt은 **fixed 타입 virtual joint로는 모델 프레임을 만들지 않아** 플래닝 프레임이
루트 링크(`base_link`)로 남는다.

실측으로 가른 방법:
- `frame_id='world'`로 `CollisionObject` 발행 → `[ERROR] Unknown frame: world`, **장애물이 조용히 무시됨**
- `frame_id='base_link'` → `/monitored_planning_scene`에 정상 등록

> **교훈: `world`는 TF에는 있지만 planning scene은 그 프레임을 모른다.**
> 따라서 `octomap_frame`도, RViz Scene Objects의 프레임도 `base_link`.

이 항목엔 **판단 번복 이력**이 남아 있고, 그 자체가 교훈이다:
① 근거 없이 `base_link`를 넣음(값은 맞았지만 근거가 없었음) → ② 다음 세션에 SRDF만 보고 추론해
"`world`가 맞다"로 바꾸고 **사실이 아닌 이유로 문서화** → ③ 실측으로 `base_link` 확정, 되돌림.
필요했던 건 **CollisionObject 발행 한 번**이었다.

### 3.3 `octomap_frame`은 고정 프레임이어야 한다

움직이는 프레임(`camera_link` 등)을 주면 **로봇이 움직일 때마다 지도가 통째로 흔들린다.**
octomap은 누적 지도이므로 기준계가 움직이면 과거 관측이 전부 어긋난다.

### 3.4 캘리브 산출물은 사본을 만들지 않는다 (사고 2건)

같은 구조의 사고가 이틀 연속 일어났다.

| 날짜 | 형태 | 어긋난 양 |
|---|---|---|
| 08-02 | `static_transform_publisher` 명령에 숫자를 하드코딩 | **340 mm** |
| 08-03 | npy를 `cp`로 복사해 두고 재캘리브 후 복사를 잊음 | **480 mm** (`[-184.31, 425.18, -118.45]`) |

**둘 다 에러가 나지 않는다. 틀린 TF로 정상 동작한다.**
→ 규칙: **"생성 위치가 정해진 산출물"은 사본을 만들지 않는다. symlink 아니면 경로 참조다.**
(상대경로 symlink는 git에 mode `120000`으로 커밋되어 clone한 다른 PC에서도 동작한다.
단 colcon `--merge-install`에서는 상대 깊이가 달라져 깨진다.)

또 하나: **읽기 의도의 실행에 쓰기 부작용을 두지 않는다.** 진단 목적으로 캘리브 스크립트를
돌렸다가 실기 TF의 소스인 `T_cam2base.npy`를 실제로 덮어썼다(계산이 결정적이라 값은 같았지만
운이 좋았을 뿐). → `--no-save` 플래그 신설.

**그리고 "현재 카메라 위치"를 문서에 베껴 적지 않기로 했다** — 오늘 하루에 세 번 바뀌었다
(1.48 → 1.542 → 1.684 m). 적는 순간 낡는다. 필요하면 npy를 읽는다.
파생 규칙: **거리에 의존하는 임계값을 다른 파일에 하드코딩하지 않는다** —
`sensors_3d.yaml`의 `max_range`가 낡은 1.48 m를 근거로 잡혀 있었고, 그 사이 실제 거리가 그걸 넘어섰다.

---

## 4. MoveIt2 Octomap 충돌회피 파이프라인

### 4.1 `octomap_server`와 MoveIt의 octomap은 별개다

이게 오늘 정리된 큰 오해다.

```
                        ┌─→ octomap_server ─→ /octomap_binary, /projected_map
depth → pointcloud ─────┤   (독립 지도. RViz·nav2용. MoveIt과 무관)
                        └─→ move_group 내부 occupancy_map_monitor
                            → PlanningScene.world.octomap   ← MoveIt은 이쪽만 본다
```

**`/octomap_binary`를 구독하는 MoveIt 기능은 없다.** MoveIt은 `move_group` 프로세스 안에서
`occupancy_map_monitor` + `PointCloudOctomapUpdater`로 **자기 octree를 직접 만든다.**
→ 충돌회피가 목적이면 `octomap_server`를 켜지 않는다. 둘 다 돌리면 같은 클라우드로
octree를 두 번 만들어 CPU를 이중으로 먹는다(이 랩탑에선 치명적).

### 4.2 프레임당 실제로 일어나는 일

```
클라우드 수신
  → TF 조회 (클라우드의 stamp 시각 기준)
  → self-filter: ShapeMask가 URDF collision 형상으로 "로봇 자신"을 제거
  → raycast: 센서~끝점 경로는 free, 끝점은 occupied로 갱신
  → PlanningScene.world.octomap 반영 → /monitored_planning_scene 발행
플래닝 시:
  → FCL이 로봇 링크 mesh vs octree cell로 충돌검사 → 충돌 샘플 폐기 → OMPL이 궤적 생성
```

**MoveIt은 "점유된 공간 덩어리"만 안다.** 물체의 종류도, 어디를 잡아야 하는지도 전혀 모른다.
그건 별도 인식 스택(6D pose 추정 + grasp 생성)이 `CollisionObject`/grasp pose로 넣어주는 몫이다.

### 4.3 self-filter가 최우선 관문인 이유

`padding_offset`(로봇 링크를 얼마나 부풀려 지울지)이 **캘리브 오차보다 작으면**
자기 팔의 잔여 점이 클라우드에 남는다 → **로봇이 자기 몸을 장애물로 보고 한 발짝도 못 움직인다.**
검증 방법은 `/moveit/filtered_cloud`를 RViz에 띄워 **팔이 지워졌는지 눈으로 보는 것**뿐이다.

여기에 오늘 배운 트레이드오프가 있다:
- `padding_offset`을 **키우면** 자기 팔은 확실히 지워지지만, **진짜 장애물도 같이 깎인다.**
- **작으면** 잔여 점이 남아 계획 자체가 실패한다.
- 즉 **캘리브 정확도가 이 여유값의 하한을 정한다.** 캘리브를 고치지 않고 padding으로 때우면
  장애물 경계가 뭉개진다.

오늘 실기에서 관측된 것과 정확히 일치한다: 장애물이 **무시되지는 않았고 회피는 수행됐지만,
경계가 모호하게 잡혔다.** (정량 오차 cm는 아직 미측정 — 다음 세션 과제)

> ⚠️ **경쟁 가설을 하나 빠뜨렸다** (세션 리뷰에서 뒤늦게 발견): `max_range`가 **1.5 m**인데
> 카메라~base 실측 거리는 **1.684 m**다. `max_range`는 센서 원점 기준 거리로 자르므로
> **로봇 베이스 부근과 그보다 먼 작업면이 클라우드에 아예 안 들어왔을 수 있다.**
> "경계가 모호"의 원인이 padding이 아니라 range일 가능성 — padding을 만지기 전에
> `max_range`를 되돌려 재현하는 실험으로 먼저 가른다.
> **일반 교훈: 원인 해석을 하나로 적기 전에 경쟁 가설을 한 줄이라도 나열한다.**

### 4.4 잔상 (persistence)

**octomap은 시간이 지난다고 지워지지 않는다. free 공간을 다시 관측해야 지워진다.**
팔에 가려진 뒤쪽은 계속 장애물로 남는다. 강제 초기화는 `/clear_octomap` 서비스.
(nvblox 계열의 실시간 ESDF 갱신과 구조적으로 다른 지점이다.)

### 4.5 오늘 실제로 조정한 값과 근거

| 파라미터 | 변경 | 근거 |
|---|---|---|
| `max_range` | 2.5 → **1.5 m** | 뒷벽 오탐 방지. ⚠️ 단 카메라~base 실측 거리가 이 값을 넘어서서 **보류 상태** — 재캘리브로 거리가 확정된 뒤 다시 정한다 |
| `point_subsample` | 1 → **3** | GPU 없는 15W CPU 부하 축소 (N개마다 1개만 사용) |
| `padding_offset` | 0.03 → **0.1 m** | self-filter 잔여점 제거 (4.3의 트레이드오프) |
| `padding_scale` | 1.0 → **2.0** | 위와 같은 목적의 배율 쪽 손잡이 |
| `max_update_rate` | 1.0 Hz (변경 없음) | `ros2_control_node` 상시 204%와의 CPU 경합 방어선 |
| `octomap_resolution` | 0.02 m (변경 없음) | 캘리브 잔차(40 mm)가 이보다 커서 현재는 캘리브가 정밀도를 지배 |
| `default_object_padding` (신규) | **0.02 m** | scene object 충돌 판정 여유. `default_robot_padding`은 0.0 |
| depth/color 프로파일 | **424×240×15** | 848×480×30 = **12.2 M point/s**는 이 랩탑에서 안 돌아감(실측) |

**대역폭 산술이 설계를 지배한 예**: 848×480 = 407k point/frame × 30 Hz = 12.2 M point/s.
`octomap_server`의 `sensor_model.max_range` 기본값이 **−1(무제한)**이라 ray 하나가 수백 voxel을
free로 갱신한다. 게다가 단일 스레드다. → 해상도·프레임레이트를 **센서 드라이버 단에서** 줄이는 게 정답
(`topic_tools` throttle은 이미 만들어진 메시지를 버리는 것이라 생성 비용을 못 줄인다).

### 4.6 캘리브 미세보정을 TF 쪽에 둔 이유 (오늘 신설)

`camera.launch.py`에 `dxyz`(m, base_link 축) / `drpy`(deg, camera_link 축) 인자를 추가했다.
드라이버는 그대로 두고 **TF만 다시 발행하며** 맞출 수 있다 (`driver:=false`로 카메라 재기동 없이).
재캘리브가 20분 걸리는 반면 이건 수 초다 — **빠른 반복 루프를 만드는 것 자체가 목적**이다.

---

## 5. ROS 2 운영에서 오늘 다시 걸린 함정들

### 5.1 침묵의 3원인 — `topic hz`가 아무것도 안 뱉을 때

`hz`만으로는 구분이 안 된다. 위에서부터 확인한다.
1. `ros2 topic list | grep <이름>` — **토픽 자체가 없다**(발행 노드가 죽었다)
2. `ros2 topic info -v <이름>` — **QoS 불일치**
3. 그제서야 진짜 처리량/TF 문제

### 5.2 QoS: 센서 토픽은 BEST_EFFORT다

`depth_image_proc`가 발행하는 포인트클라우드는 **BEST_EFFORT**인데, RViz와 `ros2 topic hz`는
기본이 **RELIABLE**이라 그냥 붙이면 **한 개도 못 받는다**.
```
[WARN] New subscription discovered on topic '...', requesting incompatible QoS.
       No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```
- RViz: PointCloud2 display → Topic → Reliability Policy = **Best Effort**
- CLI: `ros2 topic hz <topic> --qos-reliability best_effort`
- 발행자 쪽에서 바꿀 수 없는 경우가 많다 → **구독자가 맞춘다.**

### 5.3 `ROS_DOMAIN_ID`

`ros2 node list`가 비어 보이면 노드가 죽은 게 아니라 **도메인이 다른 것**부터 의심한다.
카메라 런치가 도메인을 지정하지 않아 기본값(0)에서 뜨는데 작업 셸은 93이면, 토픽이 통째로 안 보인다.

### 5.4 `realsense-viewer`가 USB를 독점한다

뷰어를 켜두면 `realsense2_camera` 노드가 프레임을 못 받고 `/camera/*` 토픽이 사라진다.
증상이 **"TF 프레임 없음"**으로 나타나 캘리브 문제로 오진하기 쉽다. 뷰어를 먼저 닫는다.

### 5.5 octomap 로그 읽는 법

- `Message Filter dropping message … queue is full`
  → **CPU가 밀린 게 아니라 TF를 못 구한 것.** message_filter 큐(기본 5)가 넘친 것이다.
  (단 TF를 고치면 리소스 문제로 같은 메시지가 다시 뜬다 — **원인이 두 개다**)
- `Nothing to publish, octree is empty` → 포인트가 하나도 안 들어옴. 위와 같은 원인.

### 5.6 명령 경로가 두 개 살아 있다 (안전)

| 경로 | 방식 |
|---|---|
| `dsr_controller2` | ROS 서비스 `movej`/`movel` → 벤더 SDK(DRFL) 직접 |
| `dsr_moveit_controller` (JointTrajectoryController) | `servoj_rt()` / `amovej()` |

둘 다 active하다(벤더 컨트롤러가 command/state 인터페이스를 하나도 claim하지 않는
**순수 서비스 래퍼**라서 공존 가능). **한 시점에 한 경로만 명령한다.**

관련해서 오늘 이전에 잡은 버그: MoveIt SimpleControllerManager는 액션 이름을
`<controller_name>/<action_ns>`로 **문자열 조립**한다. controller_manager가 네임스페이스 안에 있으면
설정에 **절대경로**(`/dsr01/dsr_moveit_controller`)를 적어야 한다.
어긋나면 **Plan은 되고 Execute만 ABORTED** — 에러 메시지가 원인을 안 가리킨다.

### 5.7 조용히 죽는 플러그인

`ros-humble-moveit-ros-perception`이 **기본 설치되어 있지 않다**(Humble).
없으면 `PointCloudOctomapUpdater` 클래스가 없어 **3D 장애물 감지만 조용히 죽는다** —
계획·실행은 정상이라 놓치기 쉽다.
비슷하게 `planning_scene_monitor`의 발행 파라미터(`publish_geometry_updates` 등)를 기본값으로 두면
RViz Scene Objects에 놓은 장애물이 **화면에는 보이는데 계획에는 안 먹는다.**

> **패턴**: 이 스택의 실패는 대부분 **크래시가 아니라 무음(silent no-op)**이다.
> "설정했다"와 "동작한다" 사이에 항상 육안/토픽 확인을 하나 끼워야 한다.

---

## 6. 지금 열려 있는 질문 (NotebookLM에 물어볼 것)

### 캘리브
1. eye-to-hand에서 **잔차 40 mm를 20 mm 이하로 줄이려면 자세를 어떻게 설계해야 하는가?**
   (자세 개수보다 자세 분포가 중요하다는 게 오늘의 관측 — 회전축 다양성, 보드 거리 다양성,
   카메라 FOV 내 보드 위치 분포 중 무엇이 지배적인가?)
2. Park-Martin vs Tsai-Lenz vs Daniilidis(dual quaternion) vs Andreff — **어떤 오차 구조에서
   어느 것이 유리한가?** OpenCV `calibrateHandEye`의 method 선택 기준.
3. AX=XB 대신 **AX=YB**(로봇↔월드까지 동시 추정)를 쓰면 이 상황에서 이득이 있는가?
4. 잔차가 큰 자세쌍을 **제거하는 것이 정당한가**, 아니면 robust estimation(RANSAC/M-estimator)을
   써야 하는가? 제거의 통계적 위험은?
5. 고정 카메라에서 내부파라미터를 신뢰성 있게 재추정할 방법이 있는가
   (거리 다양성 부족 = fx/Z scale ambiguity 문제의 표준 해법)?
6. 캘리브 정확도를 **결과가 아닌 독립적인 방법으로 검증**하는 표준 절차는?
   (알려진 좌표의 물체를 놓고 재는 것 외에)

### 좌표계 / TF
7. REP-103 / REP-105의 프레임 규약 전문과, `camera_link`↔`camera_*_optical_frame` 관계의 정확한 정의.
8. 규약 오류를 **자동으로 검출**하는 방법 — 오늘은 "시선각 채점"이라는 임시 지표를 만들어 썼다.
   더 일반적인 sanity check가 있는가?

### MoveIt / 모션플래닝
9. `PointCloudOctomapUpdater`의 self-filter(ShapeMask) 내부 동작과, `padding_offset`/`padding_scale`이
   각각 어디에 곱해지는지. 둘을 동시에 올리면 효과가 곱해지는가?
10. **캘리브 오차가 있을 때 octomap 해상도를 어떻게 정하는 것이 이론적으로 맞는가?**
    (voxel < 오차면 낭비, voxel > 오차면 정보 손실 — 최적점의 근거는?)
11. OMPL 플래너 선택 기준: RRTConnect / RRT* / BKPIECE / LBKPIECE / EST / SBL /
    AnytimePathShortening — **narrow passage** 시나리오에서 무엇이 유리한가?
    narrow-passage 전용 샘플러(bridge test, obstacle-based sampling)의 실제 효과.
12. 샘플링 기반 플래너의 **비재현성**을 실무에서 어떻게 다루는가? (pre-grasp 구간처럼
    재현성이 필요한 곳은 Cartesian/경유지로 남기는 게 맞는 판단인가?)
13. `AttachedCollisionObject` — 그리퍼가 물체를 집은 뒤 planning scene을 갱신하는 표준 패턴.
14. MoveIt octomap(occupancy octree + FCL mesh-vs-cell) vs cuRobo/nvblox(**ESDF** 기반)
    — 충돌 표현의 차이가 계획 품질/속도에 미치는 영향. **공정한 비교를 위한 통제 변수**는?
15. octomap의 "잔상" 문제 — 관측되지 않은 영역을 다루는 표준 전략
    (unknown space를 free로 볼 것인가 occupied로 볼 것인가, 그 트레이드오프).

### 시스템
16. 15W CPU-only 환경에서 3D perception + 모션플래닝을 돌릴 때 **어디를 먼저 줄여야 하는지**의
    일반 원칙 (해상도 / 프레임레이트 / subsample / update rate 중 정보 손실 대비 이득이 큰 순서).
17. ROS 2 QoS 프로파일 설계 — 센서/명령/상태 각각의 표준 조합과 그 근거.

---

## 7. 이 세션에서 뽑은 방법론 (도메인 무관)

1. **가설은 숫자로 채점한다.** "이게 맞는 것 같다" 대신 물리적으로 참이어야 하는 양을 정하고
   모든 가설을 그 위에서 비교한다 (§2.3).
2. **정당화 문장을 쓰기 전에 명령을 한 번 돌린다.** SRDF만 읽고 추론해서 문서화한 `world` 결론이
   틀렸고, 필요했던 건 발행 한 번이었다 (§3.2).
3. **민감도 지표는 무엇에 민감한지 명시한다.** LOO는 랜덤 노이즈만 재고 계통오차엔 눈이 멀다 (§1.3b).
4. **값이 두 곳에 있으면 반드시 갈라진다.** 사본 금지, symlink 아니면 경로 참조 (§3.4).
   같은 구조의 사고가 340 mm, 480 mm로 이틀 연속 났다.
5. **읽기 의도의 실행에 쓰기 부작용을 두지 않는다** (§3.4).
6. **자주 바뀌는 값은 문서에 적지 않고 읽는 명령을 적는다.** 카메라 거리가 하루에 세 번 바뀌었다 (§3.4).
7. **조용한 실패를 가정하고 설계한다.** 이 스택의 오류는 대부분 크래시가 아니라 무음이다 (§5.7).
8. **물리값이 개입한 버그는 코드보다 하드웨어(강성·평면도·실측 치수·조명·케이블)를 먼저 의심한다.**
   부호나 threshold를 만지는 건 증상만 가린다.

---

## 8. 다음 세션 과제 (측정되지 않은 것)

- 캘리브 오차의 **정량 측정**: 알려진 좌표의 물체를 놓고 cm 단위로 잰다. §4.3의 padding 해석은
  이 값 없이는 **추론**이다.
- OMPL 플래너별 **성공률 / 평균 계획시간** 로그 — cuRobo 비교의 기준선(baseline)이 된다.
- 카메라 마운트 **강성 확보** 후 자세 재수집 (현재 캘리브는 모두 잠정값)
- `max_range` 재결정 (재캘리브로 카메라~base 거리가 확정된 뒤)
- D435i depth **rosbag 녹화** — 실기 없이 개발하기 위한 전제.
  주의: 녹화 중 임시 static TF를 띄우면 bag의 `/tf_static`에 가짜 값이 박혀 나중에 진짜 캘리브 값과 충돌한다.
