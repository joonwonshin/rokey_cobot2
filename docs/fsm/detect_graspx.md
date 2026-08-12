<!-- meta
updated: 2026-08-07 10:40
status:  live
owns:    GraspGenX 출력 규약(§3) · 그리퍼 폭 계산·1/10mm 함수(§5) · 상류 버그(§6) · grasp_selector.py 연결 상태(§7-10)
-->

# GraspGenX 통합 설계 — Detect → Grasp → Execute

> 작성 2026-08-04 · 대상 하드웨어 M0609(`dsr01`) + OnRobot RG2 + RealSense
> 실행 환경: Lightning AI 원격 인스턴스 (Tesla T4 15GB, Driver 580.173.02, CUDA 13.0)
> **상태: 설계 문서. 로봇 실기 검증 안 됨.**
> 원격에서 `demo_object_pc.py --gripper_name onrobot_RG2`는 **정상 동작 확인**(2026-08-04).
> §1의 "차단 이슈"는 §1-1이 로컬 체크아웃 한정으로 격하돼 실질 2건이다.

---

## 0. 요약 (먼저 읽을 것)

GraspGenX는 **세그멘테이션된 객체 포인트클라우드 → 6-DOF grasp 자세 후보 K개 + 신뢰도**를 내놓는 모델이다.
그 이상도 이하도 아니다. 특히:

- **그리퍼 개구 폭을 출력하지 않는다.** 폭은 우리가 따로 계산해야 한다 (§5).
- **어떤 물체를 잡을지 안 정해준다.** 세그멘테이션은 우리 몫 (SAM 등).
- **충돌 회피 궤적을 안 만든다.** MoveIt 몫.

우리 프로젝트에서 GraspGenX가 맡는 구간은 딱 여기다:

```
[RealSense] → [세그멘테이션] → ★GraspGenX★ → [grasp 선택] → [MoveIt IK] → [M0609 실행]
                                    ↑
                          이 문서가 다루는 범위
```

---

## 1. 차단 이슈 — 이것부터 고쳐야 함

### 1-1. 【로컬 한정】 gripper_descriptions가 git-lfs 포인터 상태

> **범위 정정 (2026-08-04)**: 이 문제는 **로컬 체크아웃(`kimkh@rokey`) 한정**이다.
> Lightning AI 원격에서는 RG2 메시가 정상 로드돼 데모가 돌았다(사용자 확인).
> 로컬에서 시각화·메시 로드를 할 때만 필요하다. **원격 작업에는 차단 요인이 아니다.**

로컬 `ext/gripper_descriptions/`에 LFS 포인터 텍스트가 실제 데이터 대신 들어 있다.
(전체 개수 `781개 중 570개`는 서브에이전트 집계이고 내가 직접 세지 않았다 — 참고치.
직접 확인한 것은 `coll_mesh.obj` / `tsdf.npy` 두 파일이다.)

```bash
$ head -c 100 ext/gripper_descriptions/.../onrobot_RG2/tsdf.npy
version https://git-lfs.github.com/spec/v1
oid sha256:c71df636358e17d5d5058eb50da65ba3efb1ed5446af2c556f74773fad3c2212
size 524736          # ← 실제 524KB인데 파일은 131바이트

$ git lfs version
git: 'lfs' is not a git command.      # ← 설치조차 안 됨
```

**왜 조용히 안 넘어가는가**: `x_grippers.py:133-154`의 폴백이 `os.path.exists()`로만 검사한다.
포인터 파일도 "존재"하므로 더미 폴백을 안 타고, 131바이트 텍스트를 `np.load()` 하다가 **예외로 죽는다.**
성능 저하가 아니라 로드 단계 크래시다.

**해결**:
```bash
sudo apt install git-lfs        # 또는 원격이면 해당 환경 패키지 매니저
cd ext/gripper_descriptions && git lfs install && git lfs pull
```
> ⚠️ 미검증 — 아직 실행 안 했다. 실행 후 `ls -la ../onrobot_RG2/tsdf.npy`가 ~524KB인지 확인할 것.

### 1-2. 【치명】 T_cam2base.npy는 mm 단위다

```python
>>> np.load('corecode/Calibration_Tutorial/T_cam2base.npy')[:3,3]
array([1063.22, 1166.01, 586.97])      # norm = 1683.6
```

- **방향은 맞다**: `eye2hand_calibration.py:696`에 `# 이름은 cam2base지만 내용은 T_base<-cam` 이라고 본인이 적어놨다. 즉 실제로 `T_base_cam`이고, §4의 변환 체인에 그대로 쓸 수 있다.
- **단위가 틀리다**: 병진이 **밀리미터**다 (`:697` 주석 `# 병진 (mm, base 기준 카메라 위치)`).
  GraspGenX와 FoundationPose는 **미터**를 쓴다.

**변환 없이 체인에 넣으면 1000배 어긋난다.** 반드시:
```python
T_base_cam = np.load('T_cam2base.npy').copy()
T_base_cam[:3, 3] /= 1000.0        # mm → m. 이 줄을 빠뜨리면 로봇이 1.6km 밖을 향한다
```

### 1-3. 【높음】 파이썬 3.14로는 설치 불가

`uv sync`가 기본으로 CPython 3.14를 잡으면 torch가 없다 (cp310~cp313 휠만 존재).
```bash
rm -rf .venv && uv python install 3.12 && uv sync --python 3.12
```

---

## 2. 데이터 흐름 — GraspGenX가 실제로 먹는 것

`demo_object_pc.py`가 `real_world` 샘플을 읽는 경로가 우리 파이프라인과 정확히 같은 모양이다.

| 입력 파일 | 우리 쪽 대응 |
|---|---|
| `rgb.png` | RealSense 컬러 |
| `depth.npy` (float32, m) | RealSense depth (정렬 필요) |
| `seg.png` (int32 라벨맵) | **SAM 마스크** |
| `meta_data.json > intrinsics` | `/camera/color/camera_info` |
| `meta_data.json > camera_pose` | **`T_base_cam` (§1-2 단위 주의)** |

내부 처리 ([scene_loaders.py:79-90](../isaac_ros-dev/src/GraspGenX/graspgenx/utils/scene_loaders.py#L79-L90)):
```
depth + intrinsics ──역투영──> xyz_cam
xyz_cam ──camera_pose 적용──> xyz_world
seg 라벨별로 분리 ──────────> 객체별 포인트클라우드
```

**핵심 활용 포인트**: `camera_pose`에 `T_base_cam`을 넣으면 GraspGenX가 말하는 "world"가
곧 **로봇 베이스 프레임**이 된다. 출력 grasp를 추가 변환 없이 IK에 던질 수 있다.

`collect_object_items`는 객체마다 독립 아이템을 만들고, 추론도 **객체 단위로 독립 실행**된다
(씬 전체를 한 번에 처리하는 게 아니다).

---

## 3. 출력 규약 — 여기서 틀리면 로봇이 물체를 뚫는다

### 3-1. 반환값

[planner.py:39-45](../isaac_ros-dev/src/GraspGenX/graspgenx/samplers/planner.py#L39-L45):
```python
grasps_world: (K, 4, 4) float32   # 동차변환 행렬
grasp_conf:   (K,)      float32   # discriminator 신뢰도 [0,1]
branch_tags:  ["diff" | "obb", ...]   # 어느 샘플러에서 나왔는지
obb_dict:     {"center", "half_extent", "R"}
```

`grasps_world`는 **입력 포인트클라우드와 같은 프레임**이다.
내부에서 객체 중심으로 평균 이동(normalize)했다가 `grasp_server.py:447`의
`grasps[:, :3, 3] += obj_pcd_center`로 정확히 되돌린다. 회전은 건드리지 않으므로 프레임 불변.

### 3-2. Grasp 프레임 규약

[robot.py:59](../isaac_ros-dev/src/GraspGenX/graspgenx/robot.py#L59)에 명시:

```
        +Z  ← 접근 축 (approach). 손가락이 뻗은 방향
         ↑
      ┌──┴──┐
   ←──┤     ├──→   ±X ← 닫히는 방향 (closing)
      └─────┘
         ●  ← 원점 = 그리퍼 base_link
```

RG2 config의 `fingertip: [0, 0, 0.18]` → **원점에서 Z축 +18cm가 손가락 끝**.
`x_grippers.py:156`이 `tool_tcp_transform = translation([0, 0, 0.18])`로 이걸 그대로 쓴다.

> ⚠️ **주의**: `graspmoe.py:289`의 주석이 `gripper Z (closing direction)`이라고 **반대로 적혀 있다.**
> 코드가 맞고 주석이 틀렸다. 이 주석 보고 X/Z를 바꾸면 그리퍼가 90° 틀어져 물체를 옆에서 친다.

**M0609 TCP 오프셋**: 우리 TCP를 flange 기준으로 잡았다면 이 0.18m를 반영해야 한다.
부호를 틀리면 로봇이 물체를 18cm 뚫고 들어간다. **RViz에서 먼저 확인할 것.**

---

## 4. FoundationPose 결합

### 4-1. 먼저 — CAD 메시가 반드시 필요한가? → 그렇다

`isaac_ros_foundationpose`는 **model-based 전용**이다. model-free(참조 이미지) 모드가 없다:
```
foundationpose_node.cpp:156    mesh_file_path_(declare_parameter<std::string>("mesh_file_path", ...))
config/nitros_foundationpose_node.yaml:1137    mesh_file_path: textured_simple.obj
```
`grep -i "model.free\|reference.image"` → **0건**.

**포인트클라우드는 CAD가 아니다.** 데모 화면의 빨간 점 뭉치는 카메라 한 시점에서 보이는
표면만 담긴 **부분(partial) 관측**이다. FoundationPose가 요구하는 건 뒷면까지 닫힌
**완전한 텍스처 메시**(`.obj` + 텍스처)다. 서로 다른 물건이다.

### 4-2. 두 가지 경로

#### A안 — CAD 없이 (권장 시작점)

```
RealSense ─┬─ rgb ──→ SAM ──→ mask ─┐
           └─ depth ───────────────┤
                                   ↓  역투영 + T_base_cam(단위 m!)
                        객체별 pointcloud (base frame)
                                   ↓  GraspGenX 추론 (T4, 매 프레임)
                        grasps (K,4,4) + conf
                                   ↓  충돌 필터 + 최고점 선택
                        MoveIt IK → M0609
```

- 장점: **임의의 물체**에 동작. CAD 준비 불필요. 지금 당장 가능.
- 단점: 매 프레임 GPU 추론이라 트래킹 불가. 정적인 장면 전용.

#### B안 — FoundationPose 결합 (CAD 있을 때)

```
[오프라인 · 1회 · GPU]
  객체 CAD mesh
      ↓ demo_object_mesh.py --no-visualization --output_file grasps.yml
  T_obj_grasp × K개   ← 객체 좌표계 기준 grasp DB

[온라인 · 매 프레임 · GPU 추론 없음]
  RealSense rgb+depth
      ↓ FoundationPose (mesh 필요)
  T_cam_obj
      ↓ T_base_cam @ T_cam_obj          (단위 m로 맞춘 것!)
  T_base_obj
      ↓ 행렬곱만
  T_base_grasp = T_base_obj @ T_obj_grasp
      ↓ MoveIt IK
  M0609
```

- 장점: GPU 추론 1회로 끝. 이후엔 행렬곱이라 **실시간 트래킹** 가능.
- 단점: 물체마다 CAD 필요.

> **B안 전제조건**: `demo_object_mesh.py`의 `--output_file`은 world가 아니라
> **원본 메시 프레임**에 저장한다 (`grasps_original_frame = inv(T_subtract_pc_mean) @ g`).
> 단 `obj.apply_scale(scale)`이 평균 빼기 **전에** 적용되므로, 정확히는
> "스케일 적용된 메시 프레임"이다.
> **`--mesh_scale`을 1이 아닌 값으로 쓰면, FoundationPose가 로드하는 메시와
> 스케일이 정확히 같아야 `T_obj_grasp`가 유효하다.**

### 4-3. CAD를 이미지로 대충 만들 수 있나?

가능하지만 B안의 이득을 대부분 까먹는다:

| 방법 | 비용 | 판정 |
|---|---|---|
| 실측 후 CAD 직접 모델링 | 물체당 수십분 | 박스·원통 등 단순 형상이면 이게 제일 빠르다 |
| 포토그래메트리 (COLMAP 등) | 물체당 수십장 촬영 + 재구성 | 텍스처 없는/반사 물체에서 실패 |
| 스마트폰 3D 스캔 앱 | 물체당 수분 | 정확도 낮음. FoundationPose 정합 품질 저하 |

**권고: A안부터 한다.** CAD가 확보된 특정 물체(파지 대상이 고정된 시나리오)에 한해
B안을 나중에 얹는다. CAD를 급하게 만들어서 B안을 억지로 가는 건, A안보다 나은 게 없다.

---

## 5. 그리퍼 폭 계산 — 모델이 안 주는 부분

### 5-1. config의 joint 값은 시각화 전용이다

`onrobot_RG2/config.json`:
```json
"open":  { "finger_joint": -0.5585 },
"close": { "finger_joint":  0.7853 }
```

우리 URDF의 값과 소수점까지 일치한다 (같은 상류 출처 확정):
```
onrobot_rg2.urdf:226   <limit lower="-0.558505" upper="0.785398" .../>
```

**하지만 이 값은 모델 입력이 아니다.** 이 관절값을 읽는 곳은
`vis_gripper_desc.py:207`, `demo_object_pc.py:284,660` — 전부 **URDF 애니메이션**이다.
모델이 실제로 conditioning으로 먹는 건 `points.json` / `tsdf.npy` /
`proc_gripper_only_pointnet_vae_repr.json` / `sweep_volume`이다.

### 5-2. 【함정】 "gripper width"가 코드에 두 개 있고 값이 다르다

| 심볼 | 값 | 정체 |
|---|---|---|
| `XGripperInfo.width` (`x_grippers.py:158,205`) | **0.152 m** | `bbox[1][0]-bbox[0][0]` = 그리퍼 **몸통 전체 X 폭** |
| `gripper_width_m` (`graspmoe.py:587`) | **0.102 m** | `sweep_volume.extents[0]` = **실제 개구 폭** |

**개구 폭으로 써야 하는 건 `sweep_volume.extents[0]` = 0.102 m 쪽이다.**
`.width`(0.152)를 개구 폭으로 오해하면 **50% 과대평가** → RG2가 물리적으로 못 벌리는
폭을 명령하거나 충돌 판정을 놓친다.

### 5-3. 폭 산출 절차

```
1. grasp pose T의 닫히는 축 추출:   x_axis = T[:3, 0]
2. sweep volume 박스 안에 들어오는 객체 점만 남김
   (config.json의 extents/offset으로 필터)
3. 남은 점들을 x_axis에 투영 → 폭 w = max - min
4. 목표 폭 = w + CLEARANCE      ← 실측 튜닝값
5. RG2에 명령
```

```python
# ponytail: sweep_volume 기준값. .width(0.152)는 몸통 폭이라 여기 쓰면 안 된다.
RG2_MAX_OPENING_M = 0.102     # sweep_volume.extents[0] 기준
CLEARANCE_M       = 0.008     # UNVERIFIED: 실측 튜닝 필요. 도면값 아님
```

### 5-4. 【치명】 드라이버 단위는 mm가 아니라 **1/10 mm**다

RG2 드라이버는 각도가 아니라 **목표 폭 + 힘**을 받는 게 맞다. 하지만 단위가 다르다.

`src/cobot_rg2/onrobot-ros2/onrobot_rg_msgs/msg/OnRobotRGOutput.msg`:
```
rgwd : ... must be provided in 1/10th millimeters.
       The valid range is 0 to 1100 for the RG2      # = 110.0 mm
rgfr : ... must be provided in 1/10th Newtons.
       The valid range is 0 to 400 for the RG2       # = 40.0 N
```

**미터 → mm로만 바꿔 넣으면 10배 좁게 명령해 물체를 으깬다.**
```python
rgwd = int(round(width_m * 10000))   # m → 1/10 mm.  0.048 m → 480
```

> 이 문서의 이전 판은 "mm + N"이라고 적었다. §1-2에서 지적한 mm/m 사고와
> **정확히 같은 부류의 오류**를 문서가 새로 만든 것이다. 확인에 필요했던 건 grep 한 번이었다.

**개구 폭 한계가 출처마다 다르다 — 둘 다 사실이고 용도가 다르다:**

| 값 | 출처 | 용도 |
|---|---|---|
| **0.102 m** | `config.json` `sweep_volume.extents[0]` | **grasp 선별 기준**. 모델이 이 볼륨으로 conditioning했다 |
| **0.110 m** | 드라이버 `max_width=1100` | 하드웨어 물리 한계 |

모델이 0.102를 가정하고 뽑은 grasp이므로 **선별에는 좁은 쪽**을 쓴다.

**폭↔각도 변환은 이미 있다 — 새로 짜지 말 것:**
`onrobot_rg_control/_OnRobotRGIsaacSimController.py:131` `widthToJointValue()`
(RViz/Isaac 시각화용. 실기 명령에는 `rgwd`를 그대로 쓴다.)

> ✅ **위 절차 자체는 이미 구현·테스트돼 있다** — `corecode/GraspSelection/grasp_selector.py`
> (442줄, 2026-08-06 코드 감사로 확인). `compute_grasp_width()`가 §5-3 그대로이고, `GraspCandidate.rgwd`
> 프로퍼티가 §5-4 1/10mm 변환이며, 신뢰도→도달범위→접근축→폭→재충돌(GPU cdist) 5단계 필터에
> `assert c.rgwd == 500` 같은 자체 테스트도 있다. **여기 남은 문제는 알고리즘이 아니라 배선이다 — §7 참고.**

---

## 6. 알려진 상류 버그

### `grasp_server.py:~466-470` — open 포인트클라우드를 close로 덮어씀

```python
gripper_open_ptc = self.gripper.open_pointcloud.copy()     # 만들어놓고
...
gripper_open_ptc = torch.from_numpy(gripper_close_ptc[mask]...)   # 덮어쓴다
```

모델 conditioning 입력이 조용히 틀린다. 추론 결과 품질이 이상하면 여기를 의심할 것.
**우리가 고칠 수 있는 부분이지만, 상류와 diverge하므로 먼저 이슈 확인 권장.**

### `data_recording.py:77` — 자기모순 주석

`:77`은 "결과 부모 프레임도 flange"라고 하고, `:44-46`은 "TCP 플래그와 무관하게 결과 동일"이라고 한다.
eye-to-hand AX=XB에서 TCP 오프셋은 `A_i` 계산에서 소거되므로 **`:44`가 맞고 `:77`이 틀렸다.**
부모 프레임은 항상 base다.

---

## 7. 진행 순서

- [x] ~~**1.** git-lfs~~ — 원격에선 불필요. 로컬 시각화 시에만 (§1-1)
- [x] **2.** `uv sync --python 3.12` 성공 (§1-3)
- [x] **3.** `list_grippers.py`에 `onrobot_RG2` 확인
- [x] **4.** `demo_object_pc.py --gripper_name onrobot_RG2` → RG2 그리퍼 렌더 확인 (2026-08-04)
- [~] **5.** T4 측정 — **충돌필터만 실측 완료, GraspGenX 추론 자체는 아직 미측정**
      `python3 corecode/bench.py` (원격 T4, 2026-08-04):
      cdist 충돌필터 K=64 / 씬 20k점 → **8.85ms** (p95 9.14), reserved 336MB / 총 14912MB.
      OOM 경계는 K=4096, 안전마진 0.85 적용 시 **K≤1740**.
      → 4단계는 병목이 아니다. A안 실시간성은 **GraspGenX 추론 시간**이 정한다 (미측정).
      주의: 8.85ms는 K=64 값이다. cdist는 O(K)이므로 K=1740이면 대략 240ms — 실제 쓸 K에서 다시 잰다.
- [ ] **6.** `camera_pose`에 `T_base_cam`(m 단위 변환!) 넣고 출력이 베이스 프레임으로 나오는지 검증 (§1-2)
- [ ] **7.** TCP 0.18m 오프셋 부호 검증 — **RViz에서. 실기 아님** (§3-2)
- [ ] **8.** SAM 마스크 → `seg.png` 포맷 어댑터 작성
- [ ] **9.** (A안 동작 후) CAD 확보된 물체에 한해 B안 검토
- [ ] **10.** `graspgenx_perception/grasp_bridge_node.py`의 자체 `select()`(L93~124, 폭 계산·`rgwd`·재충돌 필터
      없음)를 `corecode/GraspSelection/grasp_selector.py`의 `select_grasps()`로 교체
      (2026-08-06 코드 감사로 발견). **알고리즘은 이미 있다(§5-3 위 박스) — 새로 짤 게 아니라
      import 배선만 하면 된다.** 남은 세 조각:
      ① `from grasp_selector import select_grasps` (지금 없는 import)
      ② `/grasp/best` 응답에 `width_m` 필드 추가 (지금은 `PoseStamped`뿐이라 폭이 어디에도 안 남는다)
      ③ `OnRobotRGOutput` 퍼블리시 — 지금 `grasp_bridge_node.py`에 그리퍼 관련 코드가 0줄이다
      (드라이버 쪽 `rgfr`/`rgwd` 송신·상태 피드백은 `OnRobotRGControllerServer.py`에 이미 있다 — §5-4).
      [[ws/cobot2/state]] "0-c"가 이 항목을 가리킨다.

---

## 8. 시각화 읽는 법 (viser GUI)

| 요소 | 의미 |
|---|---|
| 포인트클라우드 색 | **실제 RGB**. 점수가 아니다 |
| grasp 선 색 | `get_color_from_score`: **빨강=0, 초록=1**. 초록일수록 고신뢰 |
| 주황 와이어프레임 `[255,130,0]` | GraspMoE의 OBB (`demo_object_pc.py:528`) |
| 파란 그리퍼 메시 `[0,100,255]` | `--plot_top_mesh`의 1등 grasp (`:608-613`) |
| Next Object 버튼 | 씬 내 다음 객체로 (객체당 독립 추론) |

파란색은 1등 grasp **선**(`:565`)과 애니메이션 sweep volume bbox(`:649`)에도 쓰이니 혼동 주의.

---

## 부록: 검증 상태

| 항목 | 상태 |
|---|---|
| LFS 포인터 (로컬 2개 파일) · git-lfs 미설치 | **검증됨** (파일 내용·`git lfs version` 직접 확인) |
| "781 중 570개" 집계 | **미검증** (서브에이전트 보고. 직접 세지 않았다) |
| 원격에서 RG2 데모 정상 동작 | **검증됨** (사용자 실행 확인, 2026-08-04) |
| 드라이버 단위 1/10 mm · 1/10 N | **검증됨** (`OnRobotRGOutput.msg` 직접 확인) |
| M0609 도달 0.900 m (shoulder 기준) | **검증됨** (`macro.m0609.white.xacro` 링크 길이 합산) |
| `T_cam2base.npy` mm 단위 | **검증됨** (`np.load` 실행, norm 1683.6) |
| width 0.152 vs 0.102 두 값 | **검증됨** (코드 확인) |
| `grasp_server.py` 덮어쓰기 버그 | **검증됨** (코드 확인) |
| config joint 값이 시각화 전용 | **검증됨** (grep으로 사용처 전수 확인) |
| FoundationPose가 mesh 필수 | **검증됨** (`mesh_file_path` 파라미터 + model-free grep 0건) |
| 프레임 규약 (+Z 접근, +X 닫힘) | **검증됨** (`robot.py:59` + control points 생성 코드) |
| LFS 크래시 **시점**이 로드 단계 | 추론 (`os.path.exists` 가드 읽고 판단, 실행 안 함) |
| RG2 드라이버가 폭+힘을 받음 | **검증됨** — 단 단위는 mm/N이 아니라 **1/10 mm · 1/10 N** (§5-4) |
| `--mesh_scale`의 B안 영향 | 추론 |
| T4에서의 GraspGenX 추론 속도 | **미측정** |
| T4에서의 충돌필터(cdist) 속도·VRAM | **검증됨** — 원격 실행, §7-5 |
