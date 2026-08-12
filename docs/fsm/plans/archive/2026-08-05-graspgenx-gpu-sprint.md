<!-- meta
updated: 2026-08-06 12:00
status:  archived
owns:    없음 — 폐기된 GPU 세션 계획(전제였던 팀 공유 RTX 4070 좌석 폐기), 이력 보존용
-->

# Plan: GraspGenX GPU 세션 — 분 단위 실행 계획

> ⚠️ 이 문서의 4070/12GB 언급은 낡았다. 실제 로컬 GPU는 RTX 4060 Laptop 8GB — 단일 출처는 [[ws/cobot2/state]].

**작성:** 2026-08-05
**전제:** 팀 공유 **RTX 4070** 좌석을 곧 확보. 시간이 제한적이므로 GPU에서만 되는 일만 GPU에서 한다.
**본 설계:** [[ws/cobot2/detect_graspx]] · 상태 단일출처: [[ws/cobot2/state]]

> ⛔ **이 계획의 한 문장**: GraspGenX 서버는 **ZMQ**로 뜨고 **클라이언트는 CUDA가 필요 없다.**
> 따라서 파이프라인의 90%(세그멘테이션·프레임 변환·폭 계산·grasp 선택·MoveIt)는 **지금 이 랩탑에서 CPU로 완성**할 수 있다.
> GPU 세션에 남는 건 **모델 추론 하나뿐**이다.

---

## 0. 왜 이렇게 짜는가 — 2026-08-04 nvblox 세션의 교훈

태운 시간의 내역을 보면 **GPU가 필요했던 건 거의 없었다**:

| 잡은 문제 | GPU 필요? |
|---|---|
| `global_frame` 기본값이 `odom` (지뢰 3) | ❌ TF 문제 |
| Fast DDS가 848×480 depth를 못 흘림 (지뢰 2) | ❌ 미들웨어 |
| `ros2 bag play -l`이 TF 버퍼를 날림 (지뢰 5) | ❌ bag 재생 |
| `$(ros2 pkg prefix)` 빈 문자열 (지뢰 1) | ❌ 셸 |
| Foxglove 서브프로토콜 (지뢰 6) | ❌ 시각화 |
| `--params-file` 접두사 (지뢰 4) | ❌ 파라미터 |

**6건 중 6건이 GPU 무관이다.** 같은 부류의 문제를 이번엔 GPU 좌석에 앉기 **전에** 털어낸다.

---

## 1. 컷 목록 — GPU 세션에 넣지 않는다

| 항목 | 왜 자르나 |
|---|---|
| **nvblox** | 충돌회피는 Octomap이 이미 실기 검증됨. nvblox는 시각화 전용 = **DoD에 기여 0**. [[ws/cobot2/plans/2026-08-03-gpu-dependent-candidates]] §1-3에서 이미 "우선순위 낮음"이었다 |
| **FoundationPose (B안)** | CAD 메시가 물체마다 필요. A안(GraspGenX 직접)이 임의 물체에 되는데 CAD를 만들 이유가 없다. `detect_graspx.md` §4-3 결론 그대로 |
| **TensorRT 엔진 빌드** | 4070에서 만들어도 다른 PC로 못 옮긴다. 지금은 PyTorch 경로로 충분 — **속도가 병목으로 실측된 뒤에** 한다 |
| **cuRobo/cuMotion** | OMPL 병목이 아직 실측되지 않았다(state.md 다음할일 4번 미완). 착수 조건 미충족 |
| **SAM / FastSAM** | §2-A가 신경망 없이 되는지 먼저 본다. 안 되면 그때 |

컷 근거가 흔들리는 순간(예: 물체가 서로 겹쳐 클러스터링 실패) 이 표를 다시 연다. **지금은 열지 않는다.**

---

## 2. Phase 0 — 지금, GPU 없이 (이 랩탑, CPU)

**전부 `d435i_0803_2149_apple` bag으로 검증한다.** bag 실측 확인(2026-08-05 `ros2 bag info`):

```
/camera/camera/aligned_depth_to_color/image_raw    768개  ← depth (raw, 압축 아님)
/camera/camera/aligned_depth_to_color/camera_info  769개  ← intrinsics
/tf_static  4개   ← base_link → camera_link 체인 포함
```
컬러는 compressed만 존재 — **A/B/C 어디에도 컬러가 필요 없다.** republish 안 띄운다(지뢰 9 회피).

### A. 세그멘테이션 노드 — 신경망 0개 (예상 2h)

`src/webcam_perception/` 아래 또는 신규 `graspgen_bridge` 패키지.

```python
# depth(H,W,uint16 mm) + K → 역투영 → 평면제거 → DBSCAN → instance_mask(H,W,int32)
# 0 = 배경. 서버 모드 2/3 규약 그대로.
```

- 입력: `aligned_depth_to_color/image_raw` + 같은 이름의 `camera_info`
- 출력: `/perception/instance_mask` (`sensor_msgs/Image`, 32SC1) + 디버그용 색칠 PointCloud2
- 라이브러리: **`open3d 0.19.0` 이미 설치돼 있음**(2026-08-05 확인). 새 의존성 0개

```python
# ponytail: 전부 실측 튜닝값. 도면값 아님 — 사과 bag에서 잡고, 실기에서 다시 잡는다
VOXEL_M        = 0.005
PLANE_THRESH_M = 0.010
CLUSTER_EPS_M  = 0.015
MIN_PTS        = 50
```

**DoD:** apple bag 재생 시 클러스터 개수가 **눈에 보이는 물체 개수와 일치**. RViz에서 색칠 확인.
**셀프체크:** `test_segment.py` 하나 — 합성 점군(평면 + 구 2개)에서 `labels`의 고유값이 정확히 3개(배경+2)인지 `assert`.

> ⚠️ **실패 지점 예측**: 물체가 테이블에 그림자처럼 닿는 부분에서 평면 inlier가 물체 밑동을 먹는다.
> `PLANE_THRESH_M`을 낮추면 테이블이 안 지워지고, 높이면 물체가 깎인다. **1cm 근방에서 타협**하고 넘어간다 — 이걸 완벽하게 만들려다 시간을 태우지 않는다.

### B. GraspGenX 클라이언트 + **가짜 모드** (예상 1.5h)

```python
from graspgenx.serving.zmq_client import GraspGenXClient   # 이 import는 CUDA 불필요
# zmq_client.py:246  infer_scene_depth(depth, intrinsics, instance_mask, sweep_volume_params)
#   → {instance_id: (grasps, scores)},  **카메라 프레임**
```

RG2의 sweep-volume 12개 숫자는 **로컬 config.json에서 이미 읽었다** (2026-08-05 확인,
`ext/gripper_descriptions/.../x_grippers/onrobot_RG2/config.json`) — 서버에 assets 없어도 된다:

```python
RG2_SWEEP = dict(
    extents_open=[0.102, 0.02, 0.028],   offset_open=[0.0, 0.0, 0.175],
    extents_mid =[0.065, 0.02, 0.028],   offset_mid =[0.0, 0.0, 0.207],
    gripper_type=1,          # config.json "type": "revolute_2f"  (기본 체크포인트는 무시함)
    fingertip_depth=0.18,    # config.json "fingertip"[2]
)
```

**`--fake` 플래그를 반드시 같이 만든다.** 서버 대신 물체 점군 위에 임의 grasp K개를 얹어 돌려준다(≈8줄).
이걸로 **C·D를 GPU 없이 완주한다.** GPU가 오면 엔드포인트만 바꾼다.

**DoD:** `--fake`로 `{1: (K,4,4), (K,)}` 형태가 나오고 D까지 흐른다.

### C. 프레임 변환 검증 (예상 0.5h) — **여기가 제일 조용히 틀리는 곳**

```python
T_base_cam = np.load('T_cam2base.npy').copy()
T_base_cam[:3, 3] /= 1000.0     # mm → m. 빠뜨리면 로봇이 1.6km 밖을 향한다
```
서버 모드 2의 출력은 **카메라 프레임**이므로 `T_base_cam @ grasp`가 필요하다.
그리고 npy는 **OpenCV optical 규약**이다 — 2026-08-02에 이걸로 90° 틀어진 이력이 있다.
`calib_npy_to_tf.py`가 이미 보정하므로, **TF를 신뢰하고 `tf2` lookup을 쓰는 쪽이 안전하다:**

```
lookup_transform('base_link', 'camera_color_optical_frame')   # ← 이걸 쓴다. npy 직접 로드 금지
```
(마스크가 **컬러 정렬** depth 기준이므로 부모는 `camera_color_optical_frame`이다. depth optical 아님.)

**DoD:** `--fake` grasp의 원점들이 RViz `base_link`에서 **사과 위치에 뜬다**. 로봇 옆·1km 밖 아님.

### D. grasp 선택 정책 + RG2 폭 (예상 1h)

```python
ok  = conf > CONF_MIN
ok &= width_from_sweep(g, obj_pc) < RG2_MAX_OPENING_M - CLEARANCE_M   # 0.102 기준 (§5-2)
ok &= norm(g[:3,3]) < 0.900                    # M0609 도달, 검증됨
ok &= g[2,2] < -0.3                            # +Z 접근축이 아래를 향하는 것만
ok &= collision_free(g, scene_pc)              # cdist. T4 K=64 → 8.85ms 실측
best = g[ok][argmax(conf[ok])]
rgwd = int(round(width_m * 10000))             # m → 1/10 mm. §5-4
```

**DoD:** `--fake` 후보 중 최소 1개가 살아남고, `rgwd`가 0~1100 범위. **실기 명령은 보내지 않는다.**

### E. 4070 PC 사전 스테이징 — **GPU 좌석에 앉기 전에** (예상 0.5h, 원격/부탁 가능)

GPU 세션에서 다운로드를 기다리는 건 순수 낭비다. 좌석 잡기 전에 끝내둔다:

```bash
git clone https://github.com/gwanhuiGIM/0730_cobo2_personal.git cobot2_ws && cd cobot2_ws
git checkout init_sett
uv python install 3.12 && uv sync --python 3.12 --extra serve   # §1-3 + ZMQ 확장
python -c "import graspgenx"        # 체크포인트 자동 다운로드 (HF). 로컬 ext/는 224K = LFS 포인터다
python isaac_ros-dev/src/GraspGenX/scripts/list_grippers.py | grep onrobot_RG2
```
⚠️ 미검증 — 위 4줄은 4070 PC에서 아직 안 돌렸다. Lightning T4에서 `uv sync --python 3.12`와
`list_grippers.py`는 통과한 이력이 있다(`detect_graspx.md` §7).

---

## 3. Phase 1 — GPU 좌석 (총 90분 예산)

> **좌석에 앉는 순간 이 블록만 본다. 위로 스크롤하지 않는다.**
> Phase 0이 안 끝났으면 **앉지 않는다** — 앉아서 클라이언트를 짜기 시작하면 오늘과 같아진다.

### T+0 ~ 10분 — 서버 기동

```bash
cd <repo>/isaac_ros-dev/src/GraspGenX
python client-server/graspgenx_server.py \
  --config ext/graspgenx_checkpoints/release \
  --assets_dir assets \
  --port 5556
```
- `--config`는 **체크포인트 루트**(`gen/`·`dis/`를 담은 디렉토리)다. 안쪽 `config.yaml` 아님
- `--default_gripper` 안 준다 — 클라이언트가 sweep-volume 12개 숫자를 매 요청에 실어 보내므로 불필요
- **게이트**: 여기서 10분 넘게 막히면 → §4 분기 A

### T+10 ~ 25분 — 왕복 1회 (`--fake` 해제)

랩탑에서 apple bag 재생 + Phase 0 노드 → 원격 5556.
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 지뢰 2. 848×480 depth는 Fast DDS로 못 흐른다
ros2 bag play rosbag/bag_0803calibed/d435i_0803_2149_apple --clock -r 0.25   # -l 금지 (지뢰 5)
```
- **게이트**: grasp가 하나라도 오면 통과. 개수·품질은 아직 안 본다

### T+25 ~ 45분 — ★ 이번 세션의 진짜 목적: **추론 시간 측정**

`detect_graspx.md` §7-5가 "미측정"으로 남긴 단 하나의 숫자다. **이것만 얻어도 세션은 성공이다.**

| 재는 것 | 어떻게 | 왜 |
|---|---|---|
| `infer_scene_depth` 왕복 (물체 1개) | 20회 중앙값 + p95 | 실시간 가능 여부의 유일한 판정 기준 |
| 물체 3개 동시 | 같은 방식 | 객체 단위 독립 추론이라 **선형 증가하는지** 확인 |
| VRAM peak | `nvidia-smi --query-gpu=memory.used -l 1` | 4070 12GB. T4 15GB보다 **좁다** |
| 네트워크 왕복분 | 같은 PC에서 서버+클라 돌려 차이 | LAN이 병목인지 분리 |

**판정 기준 (여기서 A안의 운명이 갈린다):**

| 중앙값 | 결론 |
|---|---|
| < 200 ms | 준실시간. 매 프레임 재계산 가능 |
| 200 ms ~ 1 s | **트리거 방식으로 간다** — 로봇이 멈춘 상태에서 1회 계산 → 실행. 실용상 이걸로 충분 |
| > 1 s | TensorRT 검토 or B안(FoundationPose) 재개 |

> **선입견 방지**: 200ms를 못 넘겨도 실패가 아니다. 픽앤플레이스는 물체가 안 움직인다 —
> "잡기 직전 1회 계산"이 정상 설계다. 실시간이 필요하다고 **가정하지 말 것.**

### T+45 ~ 70분 — bag 4종 회귀

`obstacle1` / `hand` / `robot_moving` / `apple`. 각 5분.
- **`hand`, `robot_moving`은 실패가 예상되는 케이스다** — 사람 손·움직이는 팔이 클러스터로 잡힌다.
  실패해도 고치지 않는다. **"어떤 장면에서 깨지는가"를 기록하는 게 목적**이다
- 로그에 남길 것: 클러스터 개수 / grasp 개수 / conf 최대값 / 추론 ms

### T+70 ~ 90분 — 회수 및 기록

- [ ] 측정값을 **`detect_graspx.md` §7-5와 부록 표에 즉시 반영** (다음 세션의 나는 이 대화를 기억 못 한다)
- [ ] 깨진 장면·에러 메시지를 `md/errors-log.md`에 지문과 함께
- [ ] 서버 프로세스 종료 확인 · 좌석 반납

---

## 4. 실패 분기 — 막혔을 때 어디로 가나

| 분기 | 증상 | 조치 |
|---|---|---|
| **A** | 서버가 10분 안에 안 뜸 | 서버를 포기하고 `run_planner_on_object(obj_pc, ...)`를 **4070 PC 로컬에서 직접** 호출. numpy 배열 하나만 넘기면 되므로 ROS 없이 `.npy` 파일 주고받기로 대체 |
| **B** | 클러스터가 물체를 못 나눔 | 컷 목록 열고 **FastSAM/MobileSAM** 투입. 단 **이번 세션엔 하지 않는다** — 별도 계획으로 |
| **C** | grasp가 0개 | `grasp_threshold`를 먼저 낮춘다(기본 0.7 → 0.3). 그래도 0이면 **좌표 프레임을 의심**한다 — 물체 점군이 카메라 프레임에 있는지 base 프레임에 있는지 |
| **D** | grasp가 물체를 뚫음 | `fingertip 0.18m` 부호. **RViz에서만 확인, 실기 금지** (§3-2) |
| **E** | VRAM OOM | 4070은 12GB. `num_grasps` 200 → 64. cdist 필터의 K도 같이 |

---

## 5. 좌석용 체크리스트 (복사해서 쓴다)

```
[ ] Phase 0 A~E 전부 DoD 통과했는가?   ← 아니면 앉지 않는다
[ ] 4070 PC에 체크포인트 다운로드 완료?
[ ] export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   (모든 셸)
[ ] ros2 bag play 에 -l 붙이지 않았는가
[ ] 서버 --config 가 체크포인트 "루트"인가 (config.yaml 아님)
[ ] 측정 끝나면 detect_graspx.md 갱신했는가
[ ] 서버 프로세스 죽였는가
```

---

## 6. 이 계획이 하지 않는 것

- 실기 로봇 모션 — **이 세션에 로봇을 켜지 않는다.** grasp를 RViz까지만 띄운다
- MoveIt IK 연결 — Phase 0 D의 출력이 나온 뒤 별도 세션
- 재캘리브 — 마운트 강성 확보 후로 이미 미뤄져 있음(state.md)
- nvblox 재시도

---
확신도: **추론** — Phase 0의 사실 근거(open3d 0.19.0 설치, apple bag 토픽 구성, RG2 config.json 12개 값, `zmq_client.infer_scene_depth` 시그니처, 체크포인트 디렉토리 224K)는 2026-08-05에 직접 확인. **Phase 1의 시간 배분과 명령어는 아직 한 줄도 실행하지 않았다(미검증).**
내가 채워넣은 가정: (1) "GPU를 뺏는다" = **팀 공유 RTX 4070 좌석 확보**이고 Lightning T4 복귀가 아니다 (2) 좌석 시간이 90분 내외 (3) 4070 PC와 이 랩탑이 같은 LAN에 있어 ZMQ 왕복이 가능하다
확인 요청: **4070 PC와 이 랩탑이 같은 네트워크에 있나요? (O/X)** — X면 Phase 1이 전부 "4070 PC 안에서 bag까지 재생" 형태로 바뀝니다.
