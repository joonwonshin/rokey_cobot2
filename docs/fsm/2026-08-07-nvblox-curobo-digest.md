<!-- meta
updated: 2026-08-09 (§6-1, §7 신규 문항, §9-3 추가 — src/PACKAGES.md에서 domain knowledge 발췌. voice_processing은 미포함)
status:  live
owns:    nvblox·cuRobo·cuMotion·GraspGenX 알고리즘 설명 · MoveIt vs cuRobo 비교(개념·연산구조)
         · 파지 전처리(테이블 높이 필터링·클래스 사전지식) · 그리퍼 접근축 규약·IK 시드 연결
         (실행 명령은 소유하지 않는다 → config/testcommand.md, plans/2026-08-05-cumotion-bringup.md)
-->

# nvblox · cuMotion · cuRobo · GraspGenX 알고리즘 정리 + MoveIt vs cuRobo 비교

> **이 문서의 용도**: NotebookLM 소스로 넣어 **GPU 기반 로보틱스 스택의 원리**를 질문하기 위한 자료.
> 08-03 다이제스트가 "CPU·샘플링 기반(MoveIt/OMPL/octomap)"을 다뤘다면, 이 문서는 그 대척점인
> **"GPU·최적화 기반(nvblox/cuRobo)"** 을 다루고, 마지막에 둘을 정면 비교한다.
> 저장소를 모르는 독자도 읽히도록 각 절 앞에 배경을 붙였다.
>
> **근거 원칙**: 모든 알고리즘 서술은 **실제 소스 파일의 줄**에서 왔다. 어느 파일인지 본문에 적었다.
> 소스를 못 본 것은 `추론`이라고 명시했다. 검증 상태 요약은 §8.
>
> 읽은 소스:
> - nvblox `v3.2-14` — `isaac_ros-dev/src/isaac_ros_nvblox/` (이 저장소에 있음)
> - cuRobo 커밋 `36ea382` — 우리가 쓰는 **바로 그 커밋**을 새로 받아 읽었다(§8)
> - `isaac_ros_cumotion` `v3.2-14`
> - GraspGenX — `isaac_ros-dev/src/GraspGenX/` (이 저장소에 있음)

**시스템 구성** (이 문서의 수치가 나온 환경)
- 로봇: 두산 M0609 6축 (ROS 2 네임스페이스 `/dsr01`) + OnRobot RG2 2지 그리퍼
- 카메라: Intel RealSense D435i, **eye-to-hand** 고정
- GPU PC: RTX 4060 Laptop **8 GB**, Isaac ROS 3.2 컨테이너(Humble)
- 개인 PC: GPU 없음 → OMPL/octomap 경로만 가능

---

## 0. 한 장 요약 — 네 덩어리가 무엇을 하는가

```
      "세계가 어디에 있나"                  "어디로 어떻게 갈까"        "무엇을 어떻게 잡나"
 ┌──────────────────────────┐       ┌──────────────────────┐    ┌──────────────────┐
 │ octomap (CPU)            │       │ OMPL/MoveIt (CPU)    │    │ GraspGenX (GPU)  │
 │  점유 octree             │──────▶│  샘플링 탐색          │    │  확산모델 + 판별기 │
 └──────────────────────────┘       └──────────────────────┘    └──────────────────┘
 ┌──────────────────────────┐       ┌──────────────────────┐
 │ nvblox (GPU)             │       │ cuRobo (GPU)         │
 │  TSDF → **ESDF**         │──────▶│  배치 궤적 **최적화** │
 └──────────────────────────┘       └──────────────────────┘
        ▲                                    ▲
        └─ cuMotion = 이 둘을 ROS 2 노드로 감싸고 move_group에 꽂는 접착제
```

한 문장씩:

| 이름 | 정체 | 한 줄 |
|---|---|---|
| **nvblox** | GPU 3D 재구성 **라이브러리**(+ROS 래퍼) | depth → TSDF → **ESDF**(모든 복셀에 "가장 가까운 장애물까지 거리") |
| **cuRobo** | GPU 모션 생성 **파이썬/CUDA 라이브러리** (ROS 무관) | 로봇을 **구 집합**으로, 세계를 **거리장**으로 놓고 궤적을 **미분 최적화** |
| **cuMotion** | `isaac_ros_cumotion` — cuRobo의 **ROS 2 포장** | MoveIt 액션 ↔ cuRobo, nvblox ESDF를 서비스로 당겨옴 |
| **GraspGenX** | 파지 자세 **생성 모델** | 물체 포인트클라우드 → 6-DoF grasp 후보 K개 + 신뢰도 |

**세 가지가 서로 다른 문제다.** 지도(nvblox/octomap)는 "어디가 막혔나"만, 플래너(cuRobo/OMPL)는
"어떻게 지나가나"만, GraspGenX는 "손을 어디에 놓나"만 안다. 이 경계가 흐려지면
[[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §5-1의 `task_manager` 설계 규칙이 깨진다.

---

## 1. nvblox — depth를 GPU 위의 거리장으로

### 1.1 배경: 왜 octree가 아니라 "거리장"인가

MoveIt/octomap이 플래너에게 주는 정보는 **불리언**이다: "이 복셀은 점유/비점유".
플래너는 어떤 자세가 충돌인지 아닌지만 알 수 있고, **"얼마나 아슬아슬한지"는 모른다.**

최적화 기반 플래너는 그걸로는 못 움직인다. 최적화는 **기울기(gradient)**를 먹고 산다 —
"이 관절을 조금 돌리면 장애물에서 조금 멀어진다"는 방향을 알아야 하강할 수 있다.
불리언 점유 격자는 경계에서 불연속이라 미분이 없다.

그래서 필요한 것이 **ESDF(Euclidean Signed Distance Field)**다.
모든 복셀에 **가장 가까운 장애물 표면까지의 유클리드 거리**를 담고, 장애물 내부는 음수로 둔다.
이러면 거리 자체가 연속 함수가 되고 **공간 미분이 곧 회피 방향**이 된다.

> 이것이 "octomap ↔ nvblox" 차이의 뿌리다. **자료구조 취향의 문제가 아니라, 뒤에 붙는 플래너가
> 요구하는 정보의 종류가 다르다.** cuRobo가 nvblox를 요구하는 이유가 이것이다.

nvblox 헤더 자신의 설명 (`nvblox/include/nvblox/integrators/esdf_integrator.h:41-44`):

> "The Euclidean Signed Distance Function (ESDF) is a distance function where obstacle distances are
> **true** (in the sense that they are not distances along the observation ray as they are in the TSDF)."

### 1.2 3층 구조 — TSDF → ESDF (→ Mesh)

nvblox는 depth를 한 번에 ESDF로 만들지 않는다. 중간에 **TSDF**를 둔다.

| 층(Layer) | 각 복셀이 담는 것 | 만드는 사람 |
|---|---|---|
| **TSDF** | *광선 방향* 부호거리 + 관측 가중치. 절단거리(truncation) 밖은 안 쓴다 | `ProjectiveTsdfIntegrator` |
| **Occupancy** | log-odds 확률 (TSDF 대신 쓸 수 있는 대안 경로) | `ProjectiveOccupancyIntegrator` |
| **ESDF** | *유클리드* 부호거리 + `parent_direction` | `EsdfIntegrator` (TSDF 또는 Occupancy에서 파생) |
| Mesh | 시각화용 삼각형 | marching cubes |

TSDF는 여러 프레임을 가중평균해 센서 노이즈를 눌러주는 층이고, ESDF는 그 결과에서 "표면"을
뽑아 거리 전파를 하는 층이다. **거리 전파를 매 프레임 원시 depth에서 하면 노이즈가 그대로 튀므로,
융합층(TSDF)을 한 번 거치는 것이 핵심 설계다.**

TSDF 융합식은 교과서 그대로 (`src/integrators/projective_tsdf_integrator.cu:70-82`):

```
fused = (d_new · w_new + d_old · w_old) / (w_new + w_old)      # 가중평균
fused = clamp(fused, −truncation, +truncation)                  # 절단
```

가중치 `w_new`는 센서 모델에서 온다 — `WeightingFunctionType`에
`kConstantWeight` / `kInverseSquareWeight` / `kInverseSquareDropoffWeight` /
`kInverseSquareTsdfDistancePenalty`가 있다 (`include/nvblox/integrators/weighting_function.h:11-16`).
거리 제곱에 반비례하는 가중치를 쓰면 **먼 관측은 자동으로 덜 믿는다** —
octomap의 `max_range`처럼 딱 잘라 버리는 대신 부드럽게 감쇠시키는 방식이다.

### 1.3 갱신 연산 — **광선당**이 아니라 **복셀당**이다 (octomap과의 결정적 차이)

octomap의 갱신은 **beam 기반**이다. 점 하나마다 센서 원점 → 끝점까지 광선을 긋고,
그 경로의 복셀을 전부 free로 갱신한다. 비용이 `점 개수 × 광선 길이의 복셀 수`로 늘어난다.
08-03 다이제스트 §4.5의 "12.2 M point/s는 이 랩탑에서 안 돌아감"이 정확히 이 비용이다.

nvblox는 **두 단계로 쪼개고, 둘 다 GPU에서 돈다.**

```
① 어느 블록이 시야에 들어오나  ─ ViewCalculator
    - 절두체(frustum) 코너 검사, 또는
    - depth 이미지를 **서브샘플링해** raycast (raycast_subsampling_factor)
    → 갱신 대상 **블록** 목록                 ← 여기서만 광선이 쓰인다. 블록 단위다
② 그 블록 안의 **모든 복셀**을 한 번에 갱신 ─ ProjectiveIntegrator
    - 복셀 중심을 카메라로 **투영**해 픽셀을 읽는다 (역방향)
    - depth − 복셀깊이 = 부호거리
    → 복셀 하나 = CUDA 스레드 하나
```

근거: `include/nvblox/integrators/view_calculator.h:46-64` (두 방식이 각각 문서화돼 있다),
`src/integrators/projective_tsdf_integrator.cu:29-33` (`operator()(surface_depth_measured,
voxel_depth_m, ...)` — 복셀이 자기 픽셀을 읽는 시그니처다).

> **일반 교훈: 같은 문제도 "무엇을 반복문의 바깥으로 두느냐"에 따라 병렬화 가능성이 갈린다.**
> 광선 기반은 광선끼리 같은 복셀을 건드려 쓰기 충돌이 나므로 병렬화가 어렵다.
> 복셀 기반은 **복셀 하나를 정확히 한 스레드가 소유**하므로 락이 필요 없다.
> "GPU라서 빠르다"가 아니라 **GPU에 올릴 수 있도록 문제를 뒤집었기 때문에** 빠른 것이다.

자료구조도 이에 맞춰져 있다: octree가 아니라 **`VoxelBlock` 8×8×8 + GPU 해시**
(`include/nvblox/gpu_hash/`). 트리 순회는 분기가 많아 GPU에서 느리고, 평평한 해시는
블록 포인터를 한 번에 뽑을 수 있다 — 실제로 `esdf_integrator.cu:1093-1100`에
"이 커널이 해시 조회를 전부 앞으로 끌어내(hoist) 뒤따르는 커널 여섯 개의 성능을 개선한다"고 적혀 있다.

### 1.4 ESDF 전파 알고리즘 — 3축 밴드 스윕 + 블록 간 전파

거리장 계산은 고전적으로 **brushfire/wavefront**(BFS로 파문 퍼뜨리기)인데, 이건 본질적으로 순차적이다.
nvblox는 이걸 **블록 안 스윕 + 블록 간 이웃 전파의 반복**으로 바꿨다
(`src/integrators/esdf_integrator.cu`).

```
markAllSites          : TSDF/Occupancy를 보고 "표면 복셀(site)"을 표시. 사라진 site는 clear 큐로
   ↓
computeEsdf (:1465-1495):
   sweepBlockBandAsync         ─ 블록 내부에서 x·y·z **3축을 각각 앞뒤로 스윕**
   while (블록 목록이 안 빔):    (:1390-1428, sweepSingleBand)
       updateNeighborBands     ─ 블록 경계를 넘어 이웃 블록으로 거리 밀어넣기 (:1135)
       sweepBlockBandAsync     ─ 갱신된 이웃 블록만 다시 스윕
       swap(현재, 갱신됨)
```

두 가지가 이 알고리즘의 핵심이다.

**(a) `parent_direction` — 거리가 아니라 "누구까지의 거리인지"를 들고 다닌다.**
각 ESDF 복셀은 자기 값 외에 **가장 가까운 site로의 정수 오프셋 벡터**를 저장한다
(`esdf_integrator.cu:172`, `:580-595`). 축별 스윕은 원래 **맨해튼 거리**밖에 못 주는데,
부모 벡터를 전파하면 각 복셀이 실제 site 좌표를 알게 되어 **진짜 유클리드 거리**가 나온다.
(Felzenszwalb 계열 거리변환과 같은 착상 — 축 분리 가능성을 이용한다.)

**(b) 증분(incremental) 갱신.** 매 프레임 전체 지도를 다시 계산하지 않는다.
TSDF가 바뀐 블록만 입력으로 받고, 거리가 실제로 바뀐 이웃으로만 전파가 번진다.
`clearAllInvalid`(`:1522-1587`)는 **부모 site가 사라진 복셀만 골라 무효화**한다 —
장애물을 치웠을 때 그 그림자 영역만 다시 계산하기 위한 장치다.

### 1.5 "잔상" — octomap과 구조적으로 다른 지점

08-03 다이제스트 §4.4에 적어 둔 octomap의 성질:
**"시간이 지난다고 지워지지 않는다. free 공간을 다시 관측해야 지워진다."**
팔에 가려진 뒤쪽은 영원히 장애물로 남고, 강제 초기화는 `/clear_octomap` 서비스뿐이다.

nvblox에는 세 가지 다른 장치가 있다.

| 장치 | 소스 | 하는 일 |
|---|---|---|
| **decay integrator** | `TsdfDecayIntegrator`, `OccupancyDecayIntegrator` | **시간 기반**으로 확신도를 깎는다. 안 보이는 곳은 서서히 "모름"으로 되돌아간다 |
| `decay_tsdf_rate_hz` | `nvblox_ros/include/nvblox_ros/node_params.hpp:224` (기본 **5 Hz**) | 감쇠를 초당 몇 번 돌릴지 |
| **map clearing** | `map_clearing_radius_m` 기본 **5 m**, `map_clearing_frame_id` 기본 `base_link` (`:94-99`) | 로봇에서 반경 밖 블록을 통째로 해제(메모리) |
| **shape clearing** | `ShapeClearer`, ESDF 서비스의 `aabbs_to_clear` / `spheres_to_clear` | **요청자가 지정한 상자·구 안의 TSDF를 지운다** |
| `invalid_depth_decay_factor` | `projective_tsdf_integrator.cu:35-38` | depth가 무효(음수)인 픽셀은 가중치를 곱해 **공격적으로 감쇠** |

감쇠 확률 기본값은 free 영역 `.55`, occupied 영역 `.4`
(`occupancy_decay_integrator_params.h:23-31`) — **점유 쪽을 더 빨리 잊는다.**
"있다고 잘못 아는 것"과 "없다고 잘못 아는 것" 중 전자를 덜 오래 유지하겠다는 선택이고,
이건 안전 쪽으로 기울인 설정이 **아니다.** 회피 성능(경로가 막히지 않게)을 위한 선택이다.

> 🔴 **이건 우리 프로젝트에서 실측으로 확인해야 할 항목이다.** 사람 팔이 지나간 자리가
> 얼마나 오래 장애물로 남는지가 `decay` 파라미터에 달려 있는데, 우리는
> `nvblox_base.yaml` 기본값을 그대로 쓰고 있고 이 값을 실기에서 재본 적이 없다.
> ([[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §7-2 "octomap vs nvblox 갱신 지연"이 이 측정이다.)

### 1.6 ESDF 서비스 — 🔴 이름이 거짓말을 한다

cuMotion이 nvblox에서 세계를 받아오는 통로는 토픽이 아니라 **서비스** 하나다:
`/nvblox_node/get_esdf_and_gradient` (`nvblox_msgs/srv/EsdfAndGradients.srv`).

요청에 담기는 것:
- `use_aabb` + `aabb_min_m` / `aabb_size_m` — **관심 상자만 잘라 받는다**(전체 지도 전송 회피)
- `update_esdf` — 받기 전에 ESDF를 새로 계산할지
- `aabbs_to_clear` / `spheres_to_clear` — 요청과 동시에 지울 영역

응답: `origin_m`, `voxel_size_m`, `esdf_and_gradients`(`Float32MultiArray`), `success`.

> 🔴 **필드 이름이 `esdf_and_gradients`인데, 실제로 들어오는 건 거리 하나뿐이다 — 기울기는 없다.**
> 변환 함수가 `SignedDistanceFunctor` 단 하나이고, 복셀당 float **1개**만 쓴다
> (`nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu:28-48, 88-110`,
> 배열 차원도 x·y·z **3개**뿐이다).
> **그럼 cuRobo는 기울기를 어디서 얻는가? 자기가 계산한다** (§2.3).
> 이름만 보고 "기울기가 온다"고 가정하면 디버깅이 엉뚱한 데로 간다.

또 하나: `frame_id`가 nvblox의 `global_frame`과 다르면 **빈 격자를 돌려주고 경고만 찍는다**
(`nvblox_node.cpp:1696-1704`). 조용한 실패다 — 우리 구성에서 둘 다 `base_link`여야 하는 이유.

### 1.7 `esdf_mode` 2d/3d

`nvblox_base.yaml`의 기본값은 **`2d`**다. 2d는 `z_min~z_max` 사이의 장애물을 **높이 한 장으로 눌러**
슬라이스 하나만 만든다 — 지상 주행 로봇(nav2)용이다. 매니퓰레이터는 3차원으로 팔을 뻗으므로
반드시 `3d`여야 하고, **2d 상태로 ESDF 서비스를 부르면 nvblox가 FATAL로 죽는다**
(실기에서 겪었다 — [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6-1a).

> **패턴**: nvblox의 기본값은 **주행 로봇(nav2) 기준**이다. 매니퓰레이터에 쓰려면
> `esdf_mode` 외에도 `global_frame`, `map_clearing_frame_id` 등을 전부 다시 봐야 한다.

---

## 2. cuRobo — 궤적을 "탐색"하지 않고 "최적화"한다

### 2.1 배경: 두 가지 문제 정의

같은 "A에서 B로 가는 경로"라도, 문제를 세우는 방법이 두 가지다.

**① 탐색(search) — OMPL/RRT 계열.**
관절공간에서 자세를 무작위로 찍고, 충돌 없는 것끼리 이어 그래프를 키운다.
시작과 목표가 연결되면 끝. **필요한 건 불리언 판정 하나뿐**이다("이 자세 충돌인가?").
확률적 완결성(probabilistic completeness) — 해가 있으면 시간이 무한하면 언젠가 찾는다.
대신 **나온 경로의 품질은 보장이 없고**, 실행 전에 스무딩·시간 파라미터화를 따로 해야 한다.

**② 최적화(optimization) — cuRobo/CHOMP/TrajOpt 계열.**
궤적 전체를 변수로 놓고 **비용함수를 정의한 뒤 경사하강**한다.
비용 = 목표 도달 + 충돌 + 관절한계 + 부드러움(가속·저크). **미분 가능한 거리장이 필요**하다.
해가 나오면 이미 부드럽고 시간까지 잡혀 있다. 대신 **국소 최적(local minimum)에 갇힐 수 있다** —
이게 최적화 기반의 고전적 약점이고, cuRobo의 설계 전체가 이 약점을 GPU로 때우는 이야기다.

### 2.2 국소 최적을 어떻게 이기는가 — **병렬 시드**

cuRobo의 답은 단순하다: **여러 출발점에서 동시에 최적화한다.** 하나가 갇혀도 다른 게 산다.
그리고 그 "동시에"가 GPU 배치 차원이라 개수를 늘려도 벽시계 시간이 거의 안 는다.

기본값 (`curobo/src/curobo/wrap/reacher/motion_gen.py:171-173`):

| 시드 | 기본 개수 | 우리 설정(`config/cumotion_planner.yaml`) |
|---|---|---|
| `num_ik_seeds` | **32** | (미지정 → 32) |
| `num_graph_seeds` | 4 | **6** |
| `num_trajopt_seeds` | 4 | **6** |

파이프라인 (`motion_gen.py`, 클래스 `MotionGenConfig`):

```
목표 포즈
  ↓ ① IK — 32개 시드를 배치로 동시에 풀어 목표 관절자세 후보를 만든다
  ↓ ② 시드 생성 — 시작↔목표 선형보간, retract 자세 경유(include_trajopt_retract_seed),
  ↓              그리고 필요하면 ③
  ↓ ③ Graph planner = **PRM*** (motion_gen.py:688  graph_planner = PRMStar(graph_cfg))
  ↓              ← 최적화가 못 빠져나오는 좁은 통로에서 "지형을 넘겨주는" 역할.
  ↓                즉 cuRobo는 샘플링 플래너를 **버린 게 아니라 부품으로 안에 넣었다**
  ↓ ④ Trajectory optimization — MPPI(파티클) → L-BFGS(경사)
  ↓ ⑤ Finetune trajopt — dt를 줄여가며 시간최적에 가깝게 다시 푼다
궤적 (위치·속도·가속도·저크 + dt)
```

⑤가 있는 이유: ④는 고정 시간격자에서 푼다. 실행 시간을 줄이려면 dt 자체를 줄여야 하는데
그러면 속도·가속 한계에 걸리므로, **dt를 스케일(`finetune_dt_scale`, 기본 0.9)해 다시 수렴**시킨다.
우리 설정은 `trajopt_finetune_iters: 400`, `num_trajopt_time_steps: 32`.

### 2.3 최적화 안쪽 — MPPI와 L-BFGS를 왜 둘 다 쓰나

| 단계 | 알고리즘 | 소스 | 성질 |
|---|---|---|---|
| 전반 | **MPPI**(model predictive path integral) | `opt/particle/parallel_mppi.py`, 설정 `task/particle_trajopt.yml` (`num_particles: 25`, `n_iters: 2`) | 미분 안 쓰고 **표본으로 지형을 훑는다**. 국소 최적 탈출에 강하지만 정밀하지 않다 |
| 후반 | **L-BFGS** | `opt/newton/lbfgs.py`, 설정 `task/gradient_trajopt.yml` | 준뉴턴 경사법. `n_iters: 100`, `history: 15`, `line_search_type: approx_wolfe`, `use_cuda_graph: True` |

**거친 탐색 → 정밀 수렴**의 고전적 조합이다. 특이한 건 둘 다 **배치**로 돈다는 것 —
`n_problems`가 명시적 파라미터이고, `use_cuda_graph: True`는 CUDA Graph로 커널 실행 오버헤드까지
없앤다(같은 형태의 문제를 반복해서 푸는 전제가 깔려 있다. 그래서 시드 개수를 바꾸면
CUDA Graph가 무효화된다고 소스 주석이 경고한다 — `motion_gen.py:937-939`).

**충돌 비용의 실제 계산** (`curobo/src/curobo/curobolib/cpp/sphere_obb_kernel.cu`):

```
로봇 링크  →  구 N개로 근사 (우리 M0609+RG2는 XRDF 기준 75개)
각 구 중심 →  ESDF 격자 좌표로 변환 → 이웃 복셀 값을 **가중 보간** (:716-770)
비용       →  d(구 중심) − 반지름 을 activation_distance(η)로 부드럽게 힌지 (scale_eta_metric, :590-701)
기울기     →  같은 커널에서 해석적으로 같이 뽑는다 (loc_grad)
```

여기서 §1.6의 답이 나온다: **기울기는 nvblox가 주는 게 아니라 cuRobo가 보간하면서 직접 만든다.**

`activation_distance: 0.025`(`gradient_trajopt.yml`)는 "충돌 25 mm 전부터 비용이 켜진다"는 뜻이다.
0에서 갑자기 켜지면 미분이 불연속이라 최적화가 진동한다 — **여유값이 아니라 미분가능성을 위한 장치**다.
(단, 결과적으로 안전 여유로도 작동한다.)

**연속 충돌 검사(swept collision).** `gradient_trajopt.yml`:
```yaml
primitive_collision_cfg:
  use_sweep: True
  sweep_steps: 4
  use_speed_metric: True
```
시간격자 사이를 4단계로 훑으면서, 거리장 값만큼 **건너뛰며(sphere marching)** 검사한다
(`sphere_obb_kernel.cu:926-952`, `curr_jump_distance += max(fabsf(distance), sphere.w)`).
거리장이 있으니 "다음 장애물까지 최소 이만큼은 안전하다"를 알아서 **큰 보폭으로 건너뛸 수 있다** —
거리장을 쓰는 두 번째 이득이다.

> 대비: MoveIt/OMPL은 두 자세 사이를 **고정 비율로 잘게 쪼개** 하나씩 불리언 검사한다.
> 우리 설정 `ompl_planning.yaml:166`의 `longest_valid_segment_fraction: 0.005` (= 관절범위의 0.5%)가
> 그 분해능이다. **비율을 잘못 잡으면 얇은 장애물을 그냥 통과한다**(터널링).
> 거리장 기반 스윕은 원리적으로 이 문제가 없다.

### 2.4 로봇 표현 — 왜 메시가 아니라 구인가

MoveIt/FCL은 URDF **collision mesh** 대 octree cell로 충돌을 검사한다. 정확하지만 GPU에 안 맞는다
(메시-메시 검사는 분기와 메모리 접근이 불규칙하다).

cuRobo는 로봇을 **구 집합**으로 근사한다. 구 대 거리장 검사는
`d = grid(center) − radius` — **분기 없는 산술 세 줄**이고, 구 하나가 스레드 하나다.
`(시드 × 시간스텝 × 구)` 전체가 하나의 배치 차원이 된다: 6 × 32 × 75 ≈ **14,400개 검사가 한 커널**.

**대가는 정확도다.** 구는 링크를 정확히 못 덮으므로 **반드시 실제보다 뚱뚱하거나 홀쭉하다.**
우리가 실기에서 겪은 것이 바로 이것이다:

> `INVALID_START_STATE_SELF_COLLISION` — XRDF 구가 실제 링크보다 뚱뚱해 6쌍이 겹쳤다.
> **같은 자세를 OMPL은 통과한다.** 6쌍을 `self_collision.ignore`에 넣어 우회했고,
> 그중 2쌍(특히 `link_4 ↔ rg2_base_link`)은 **보호를 포기한 상태로 남아 있다.**
> ([[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §5-2)

> 🔴 **이건 설정 실수가 아니라 표현 방식의 구조적 대가다.** 구 근사를 쓰는 한
> "덜 덮으면 부딪히고, 더 덮으면 못 움직인다"의 균형을 사람이 잡아줘야 한다.
> 08-03 다이제스트 §4.3의 `padding_offset` 트레이드오프와 **정확히 같은 형태의 문제**가
> 다른 층에서 반복된다.

---

## 3. cuMotion — cuRobo를 ROS 2에 꽂는 접착제

**cuRobo는 ROS를 모른다.** 파이썬/CUDA 라이브러리일 뿐이다. `isaac_ros_cumotion`이 그것을 감싼다.

| 노드 | 역할 |
|---|---|
| `cumotion_planner_node` | `/cumotion/move_group` 액션(`moveit_msgs/action/MoveGroup`)을 받아 cuRobo로 계획 |
| `robot_segmenter_node` | **로봇 몸을 depth에서 지운다** |
| `isaac_ros_cumotion_object_attachment` | 잡은 물체를 로봇 구 집합에 붙인다 |
| `isaac_ros_cumotion_moveit` | move_group에 꽂는 **얇은 플래너 플러그인**(C++ 액션 클라이언트, CUDA 의존 없음) |

### 3.1 `robot_segmenter_node` — nvblox에는 self-filter가 없다

MoveIt octomap 경로에는 `ShapeMask` self-filter가 있어 로봇 자신을 클라우드에서 지운다
(08-03 다이제스트 §4.3). **nvblox에는 그게 없다. 원본 depth를 그대로 먹는다.**
없이 돌리면 로봇이 자기 몸을 장애물로 보고 계획이 전부
`INVALID_START_STATE_WORLD_COLLISION`으로 실패한다 (실기 확인).

segmenter가 하는 일: **cuRobo가 충돌검사에 쓰는 바로 그 구 집합**을 현재 관절각으로 정기구학 전개해
depth 이미지에 투영하고, 구 안쪽 픽셀을 지운다 (`isaac_ros_cumotion/robot_segmenter.py` →
cuRobo `wrap/model/robot_segmenter.py:113 get_robot_mask`). 파라미터 `distance_threshold` 기본 **0.1 m**.

> 🔴 **따라서 segmenter와 planner는 반드시 같은 XRDF를 써야 한다** — 지우는 몸과 검사하는 몸이
> 같아야 한다. (우리 `config/cumotion_segmenter.yaml`·`cumotion_planner.yaml`에 이 경고가 적혀 있다.)
> 어긋나면 "안 지워진 팔"이나 "지워진 진짜 장애물"이 생기는데, **둘 다 조용히 일어난다.**

### 3.2 nvblox → cuRobo 규약 변환 — 부호가 반대다

`cumotion_planner.py:408-472`가 하는 일. 짧지만 전부 함정이다.

| 단계 | 코드 | 이유 |
|---|---|---|
| voxel_size 대조 | 다르면 **FATAL** (`:410`) | nvblox가 정한 해상도와 cuRobo 격자가 어긋나면 무의미 |
| shape 대조 | 다르면 **FATAL** (`:432`) | `grid_size_m`이 voxel_size의 정수배가 아니면 여기서 죽는다 |
| 미관측 처리 | `array_data[array_data < -999.9] = 1000.0` | nvblox는 미관측을 **−1000**으로 준다 → **+1000(=아주 안전)** 으로 뒤집는다 |
| **부호 반전** | `array_data = -1.0 * array_data` | 🔴 **nvblox는 장애물 내부가 음수, cuRobo는 반대다** |
| 표면 오프셋 | `array_data += 0.5 * voxel_size` | nvblox는 표면을 0으로, cuRobo는 0을 "충돌 아님"으로 본다 |

> 🔴 **미관측 = 자유공간으로 취급된다.** `-1000 → +1000`은 "모르는 곳은 뻥 뚫려 있다고 본다"는
> 뜻이다. 카메라가 못 본 곳으로 팔이 지나가는 궤적이 **정상적으로 계획된다.**
> 08-03 다이제스트 §6 질문 15("unknown space를 free로 볼 것인가 occupied로 볼 것인가")가
> 여기서 **이미 free로 결정돼 있다** — 우리가 고른 게 아니라 상류가 골라 놓은 것이다.

### 3.3 감시상자(AABB) 밖은 존재하지 않는다

cuMotion은 `use_aabb_on_request: true`로 **상자 하나만 잘라서** 받는다.
우리 설정은 `grid_center_m: [0.35, 0, 0.325]`, `grid_size_m: [1.10, 1.0, 0.75]`
→ 복셀 22×20×15 = **6,600개** (기본 2×2×2 m 상자의 10%).

**상자 밖은 자유공간이다.** 작업영역을 좁게 잡을수록 빠르지만, 상자를 벗어난 장애물은 **통과한다.**

### 3.4 🔴 세계 갱신은 **계획 요청당 1회**다 — 실행 중 회피는 안 된다

`update_esdf_on_request: true`는 "계속 갱신"이 아니다. `cumotion_planner.py`의 갱신 호출은
계획 요청 처리 경로 안에 있다. 즉:

```
계획 요청 도착 → ESDF 1회 pull → 궤적 생성 → 반환 → [실행 중에는 지도가 안 바뀐다]
```

**궤적 실행 중에 사람이 팔을 뻗으면 cuMotion은 모른다.**
동적 회피를 하려면 상위(`task_manager`)가 주기적으로 stop → 재계획을 하거나,
cuRobo MPC(`wrap/reacher/mpc.py`)를 별도로 써야 한다.
이것이 [[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]]의 **(a) 계획시점 우회 /
(b) 실행중 stop→재계획** 선택지의 실체다.

### 3.5 그리고 cuMotion은 **octomap을 아예 안 본다**

이미 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §4-3에서 소스로 확인한 사실이라 여기서 반복하지 않는다.
요약만: `planning_scene_diff.world.collision_objects`만 읽고 `world.octomap` 필드는 버린다.
→ **`read_esdf_world:=False`면 사람 팔이 안 보인다. 그런데 계획은 성공한다.**

---

## 4. GraspGenX — 파지 자세 생성

> 출력 규약·폭 계산·1/10 mm 단위·상류 버그는 [[ws/cobot2/detect_graspx]]가 단일 출처다.
> 여기서는 **알고리즘이 무엇이냐**만 다룬다.

### 4.1 확산 생성기 + 판별기 (생성과 채점이 분리돼 있다)

`graspgenx/models/generator.py` — **DDPM 확산모델**이다.

| 항목 | 값 | 소스 |
|---|---|---|
| 확산 스텝 | 학습/추론 **100** | `generator.py:76-77` |
| 위치(position) 노이즈 스케줄 | `scaled_linear` | `:283-288` |
| 회전(rotation) 노이즈 스케줄 | `squaredcos_cap_v2` | `:290-296` |
| grasp 표현 | `r3_6d` (3D 위치 + **6D 회전표현**) | `:57, 84, 125` |
| 물체 인코더 | PointNet++ (또는 PTv3, ViT) | `:78, 134-141` |

**위치와 회전에 서로 다른 노이즈 스케줄을 쓴다**는 게 눈에 띈다. 회전은 유계(compact) 다양체라
선형 스케줄이 맞지 않는다. 회전을 쿼터니언이 아니라 **6D 표현**(회전행렬 두 열)으로 두는 것도
같은 이유다 — 쿼터니언/오일러는 불연속이라 신경망 회귀가 나쁘다는 게 알려진 결과다.

생성기는 grasp를 **뽑기만** 하고, 좋은지 나쁜지는 **별도 판별기**가 매긴다:
`discriminator.py:490` `outputs["grasp_confidence"] = outputs["logits"].sigmoid()`.
우리 `/grasp/best`가 쓰는 신뢰도가 이 값이다.

> **설계 관점**: 생성과 채점을 분리하면 **다른 방법으로 만든 후보도 같은 자로 잴 수 있다.**
> 그게 다음 절이다.

### 4.2 GraspMoE — 학습된 후보 ∪ 기하학적 후보

`graspgenx/samplers/graspmoe.py`. 기본 플래너다(`planner.py:24 planner="graspmoe"`).

```
물체 포인트클라우드
  ├─ ① 확산 분기 : 생성기가 K개 뽑음                      → "diff" 태그
  └─ ② OBB 분기  : 점군의 방향성 경계상자(OBB)를 계산하고,
                   그 면 위에 top-down grasp를 격자로 깔아 만든다   → "obb" 태그
                   (yaw 36가지 × z 오프셋 6가지 × 위치 격자 1 cm)
              ↓
        둘을 합쳐 **같은 판별기**로 전부 재채점 → 정렬
```

②는 학습이 아니라 **순수 기하**다(`_compute_obb`, `_build_face_candidates`).
왜 있는가 — 확산모델은 학습 분포 밖 물체에서 후보를 못 내놓을 수 있는데,
"위에서 수직으로 집는다"는 안전빵은 기하학적으로 언제나 만들 수 있기 때문이다.
`skip_obb_rule="auto"`는 물체 세 변이 전부 그리퍼 개구보다 크면 OBB 분기를 건너뛴다(`:395`) —
**어차피 못 잡을 물체에 후보를 만들지 않는다.**

> 이 구조는 우리 실기 사례와 직접 이어진다: 2026-08-05에 **558 px 노이즈 덩어리를 고르고
> 사과는 후보 0개**였던 사건([[ws/cobot2/state]] "0-b"). GraspGenX는
> **"어느 물체를 잡을지" 문제를 전혀 안 푼다** — 세그멘테이션이 준 덩어리를 그냥 잡는다.
> 판별기 점수는 "이 덩어리를 잡을 수 있나"이지 "이게 사과인가"가 아니다.

---

## 5. ⭐ MoveIt(move_group) vs cuRobo — 정면 비교

> 사용자 질문의 핵심 절. **"둘 다 move_group으로 묶이는데 무엇이 다른가"**를 세 층으로 나눠 답한다:
> ① 문제를 세우는 방식 ② CPU/GPU 연산 구조 ③ 실무에서 언제 무엇을 쓰나.

### 5.1 먼저 — 무엇이 "묶여" 있는가

둘은 대등한 경쟁자가 아니다. **cuMotion은 move_group 안의 플래닝 파이프라인 한 개**로 들어간다.

```
                   ┌─────────────────── move_group (MoveIt 2, CPU 프로세스) ───────────────────┐
 목표(RViz/액션) ─▶│ PlanningSceneMonitor ─ 로봇상태·충돌객체·octomap 관리                      │
                   │ planning_pipelines:                                                         │
                   │   ├ 'ompl'                → OMPL 라이브러리 (같은 프로세스, CPU)            │
                   │   └ 'isaac_ros_cumotion'  → 얇은 액션 클라이언트 ──┐                        │
                   │ 궤적 실행: JointTrajectoryController ─▶ 로봇        │                       │
                   └───────────────────────────────────────────────────┼───────────────────────┘
                                                                        │ /cumotion/move_group 액션
                                                                        ▼
                                              cumotion_planner_node (별도 프로세스·컨테이너·GPU)
                                                        │ ESDF 서비스
                                                        ▼   nvblox_node (GPU)
```

**따라서 바뀌는 것은 "궤적을 만드는 사람"뿐이다.** planning scene 관리, 컨트롤러 실행,
액션 인터페이스, RViz는 전부 그대로 MoveIt이다. RViz MotionPlanning 패널의 드롭다운으로
`ompl` ↔ `isaac_ros_cumotion`을 바꾸면 같은 목표를 두 플래너에 던질 수 있다
(우리는 이걸로 계획시간을 비교했다).

**하지만 "그대로"가 아닌 게 하나 있다 — 세계.** OMPL은 move_group 안의 octomap을 보고,
cuMotion은 **그걸 버리고 nvblox ESDF를 자기가 따로 당겨온다**(§3.5). 같은 화면, 다른 세계다.

### 5.2 연산 구조 — CPU와 GPU가 실제로 무엇을 하나

| | **MoveIt + OMPL** | **cuRobo(cuMotion)** |
|---|---|---|
| **패러다임** | 샘플링 기반 **탐색** | **미분 최적화**(+PRM* 부품) |
| **연산 장치** | CPU. 계획 요청 하나 = 대체로 **단일 스레드** | GPU. 계획 요청 하나가 수천~수만 개 병렬 커널 |
| **로봇 표현** | URDF **collision mesh** | **구 집합** (우리: 75개) |
| **세계 표현** | occupancy **octree** (+CollisionObject 원시도형) | **ESDF 격자** (+원시도형) |
| **충돌 판정** | FCL: mesh vs octree cell → **불리언** | 커널: 구 중심의 ESDF 보간 − 반지름 → **실수 + 기울기** |
| **경로 위 검사** | 두 자세 사이를 고정 비율로 **이산 분할** (`longest_valid_segment_fraction: 0.005`) | **swept sphere**, 거리만큼 건너뛰며(`sweep_steps: 4`) |
| **반복 단위** | "샘플 하나 찍고 충돌검사" — 순차적, 이전 결과에 의존 | "시드 6개 × 시간 32스텝 × 구 75개를 한 번에" — 데이터 병렬 |
| **출력** | **기하 경로**(waypoint 나열). 속도·시간은 후처리(TOTG 등)로 붙인다 | **완성된 궤적** — 위치·속도·가속도·저크·dt 포함 |
| **부드러움** | 후처리 스무딩에 의존 | **비용함수 안에 저크까지 들어 있다**(`smooth_weight`) |
| **실패 모드** | 시간 초과(해를 못 찾음). 요란하다 | 국소 최적 수렴 / 시드 전멸. `INVALID_MOTION_PLAN` 등 코드로 |
| **재현성** | ❌ 난수 시드마다 다른 경로 | 상대적으로 ✅ (같은 입력 → 같은 최적화 궤적. 단 `parallel_finetune` 등에 따라 완전 결정적은 아니다 — **추론**) |
| **VRAM** | 0 | 실측 `cumotion_planner_node` **1,508 MiB** (+nvblox 334 + segmenter 660) |
| **워밍업** | 없음 | **필요**. CUDA 커널·CUDA Graph 컴파일에 첫 기동 수 초~수십 초 |

### 5.3 "GPU라서 빠르다"가 왜 우리 실측에서 틀렸나

우리가 잰 숫자 ([[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §7-1, 로봇·카메라 없이, 빈 세계, 관절공간 목표):

| | server 중앙값 | 성공 |
|---|---|---|
| OMPL | **16.1 ms** | 20/20 |
| cuMotion | **94.7 ms** | 18/20 |

실기(로봇+카메라+nvblox 살아 있는 상태, 각 10회): OMPL **42.4 ms** / cuMotion **110.6 ms**, 둘 다 10/10.

**두 조건 모두 cuMotion이 느리다. 이건 버그가 아니라 두 알고리즘의 성질 그대로다.**

- **RRTConnect는 쉬운 문제에서 거의 공짜다.** 빈 세계에서 시작·목표를 직선으로 이으면
  충돌검사 몇 번에 끝난다. 계획 시간이 **문제 난이도에 비례**한다.
- **cuRobo는 장애물이 0개여도 최적화 파이프라인 전체를 돈다.** IK 32시드 → trajopt →
  finetune 400 iters가 그대로 실행된다. 시간이 **문제 난이도와 거의 무관**하다.
  실측 분산이 이걸 그대로 보여준다: cuMotion 91.7~96.5 ms(±3%) vs OMPL 5.3~28.4 ms(**5배 요동**).

> 🔴 **따라서 이 숫자로 cuMotion을 판정하면 안 된다.** 빈 세계·관절공간 목표는
> **RRTConnect에 가장 유리한 조건**이다. 의미 있는 비교는 **장애물이 궤적을 실제로 막는 씬**에서
> "OMPL이 얼마나 느려지는가 vs cuMotion이 얼마나 느려지는가"를 재는 것이다.
> cuRobo가 주장하는 이득은 **평균이 아니라 최악(tail)** 쪽에 있다 — 어려운 씬에서 OMPL은 초 단위로
> 튀거나 실패하고, cuRobo는 거의 그대로다(**추론** — 우리는 아직 안 쟀다).

**교훈 (도메인 무관)**: 벤치마크는 **알고리즘에 유리한 조건을 고르면 원하는 결론이 나온다.**
"어느 쪽이 빠른가"를 묻기 전에 **"어느 조건에서"**를 먼저 고정해야 한다.
우리 08-03 다이제스트 §7-1의 방법론("가설은 숫자로 채점한다")이 여기서도 그대로 적용된다.

### 5.4 활용법 — 언제 무엇을 쓰나

| 상황 | 권장 | 이유 |
|---|---|---|
| GPU 없음 | **OMPL 뿐** | cuRobo는 CUDA 필수. 개인PC는 선택지가 없다 |
| 빈 작업공간, 단순 point-to-point | **OMPL** | 더 빠르고 워밍업·VRAM 0 |
| 어수선한 씬 / 좁은 통로 | **cuRobo 후보** (검증 필요) | 시드 병렬 + 거리장 |
| **부드러운 궤적이 요구됨** (저크 제한, 물컵 운반) | **cuRobo** | 저크가 비용함수 안에 있다. OMPL은 후처리 |
| **미모델링 장애물(사람 팔) 회피** | **cuRobo + nvblox 필수** | cuMotion은 octomap을 안 본다(§3.5) |
| **재현성이 중요한 구간**(pre-grasp 접근) | 둘 다 부적합 → **Cartesian 경로/경유지** | 샘플링은 매번 다르고, 최적화도 시드에 흔들린다 |
| 실행 중 동적 회피 | **둘 다 안 됨** | OMPL: 재계획 필요 / cuMotion: 요청당 1회 갱신(§3.4). MPC를 따로 써야 한다 |
| 그리퍼 개폐 같은 단순 관절 이동 | **OMPL** | GPU 왕복이 순손해 |

**우리 프로젝트의 결론**: 두 파이프라인을 **공존**시켜 두고 목적별로 고르는 게 맞다.
`moveit.launch.py cumotion:=true`가 이미 그 구성이다(`planning_pipelines: ['ompl', 'isaac_ros_cumotion']`).
cuMotion을 켠다고 OMPL이 사라지지 않는다.

### 5.5 통합 비용 — 표에 잘 안 적히는 진짜 차이

| | OMPL | cuRobo/cuMotion |
|---|---|---|
| 설치 | apt 한 줄 (MoveIt에 포함) | Docker 컨테이너 + CUDA + 서브모듈 + **소스 패치 2건** |
| 로봇 모델 | URDF/SRDF (이미 있음) | **XRDF를 새로 만들어야 한다**(구 피팅 포함) |
| 새 실패 모드 | — | 구 과대추정 자기충돌 · `/joint_states` velocity 필수 · warp 버전 드리프트 · `esdf_mode` 기본값 · segmenter 누락 |
| 디버깅 | 로그가 대체로 원인을 가리킴 | **cuMotion 로그를 봐도 nvblox가 죽은 건지 모른다**(실기에서 겪음) |

우리가 cuMotion을 띄우기까지 하루에 함정 **6개**를 밟았고, 그 **전부가 "OMPL은 멀쩡한데 cuMotion만
죽는다"** 형태였다([[ws/cobot2/state]]). 이건 우연이 아니다 — cuMotion 경로는 스택이 더 깊고
(컨테이너·CUDA·서브모듈·별도 프로세스·서비스 의존), 깊은 스택은 실패 지점이 곱해진다.

> **판단**: cuRobo를 "더 나은 플래너"로 도입하는 게 아니라, **"nvblox ESDF를 쓰기 위한 유일한 경로"**로
> 도입하는 것이라고 보는 편이 정확하다. 우리 목적(사람 팔 회피)에서 nvblox가 필요하고,
> nvblox를 소비하는 플래너가 cuRobo뿐이다. 속도는 부수적이다.

---

## 6. 두 파이프라인 나란히 보기

```
【CPU 경로 — 지금 동작 중, 개인PC 가능】
 D435i ─ depth ─▶ move_group 내부 PointCloudOctomapUpdater
                    ├ self-filter (ShapeMask, URDF collision 형상)
                    ├ beam raycast → occupancy octree
                    └ PlanningScene.world.octomap
                         ▼
                    OMPL RRTConnect (CPU, FCL 불리언 충돌) ─▶ 후처리 스무딩·시간 ─▶ JTC

【GPU 경로 — 2026-08-06 관통, 로봇은 아직 안 움직임】
 D435i ─ depth ─▶ robot_segmenter_node (cuRobo 구로 로봇 픽셀 제거) ← 없으면 전부 실패
                    ▼ /cumotion/camera_1/world_depth
                 nvblox_node (esdf_mode:=3d)
                    ├ ViewCalculator → ProjectiveTsdfIntegrator (복셀당 GPU)
                    ├ EsdfIntegrator (3축 밴드 스윕 + parent_direction)
                    └ decay integrator (시간 기반 망각)
                    ▼ get_esdf_and_gradient (서비스, AABB로 잘라서)
                 cumotion_planner_node ── 부호반전·미관측→+1000·+0.5voxel
                    └ cuRobo: IK 32 → PRM* → MPPI → L-BFGS → finetune
                    ▼ /cumotion/move_group
                 move_group (플러그인) ─▶ JTC
```

**두 경로에서 "로봇 자신 지우기"가 서로 다른 곳에 있다**는 점이 눈에 띈다.
CPU 경로는 move_group **안**(ShapeMask), GPU 경로는 nvblox **앞**(별도 노드).
이것이 §3.1의 "segmenter 없으면 전부 실패"의 이유다.

---

## 6-1. 파지 전처리·프레임 규약 — `src/PACKAGES.md`에서 새로 뽑은 도메인 지식 (2026-08-09)

> `graspgenx_perception`·`pick_fsm`(`src/PACKAGES.md`)에 실무 함정으로만 적혀 있던 것 중,
> 일반화되는 로보틱스 원리를 뽑아 여기로 옮겼다. §1~§6과 마찬가지로 **소스 파일·줄 번호가 근거**다.
> `voice_processing`(지시 입력 층)은 알고리즘이 아니라 배선이라 대상에서 뺐다.

### 6-1.1 테이블 높이 필터링 — 전역 스칼라 대 국소 추정

**배경**: depth 기반 물체 세그멘테이션은 보통 "테이블 평면보다 몇 cm 이상 튀어나온 픽셀"로
전경을 가른다. 문제는 **그 "테이블 높이"를 어디서 재는가**다.

`capture_graspgenx_scene.py`의 `segment_from_labels()`(`graspgenx_perception`)는 전역 스칼라
`table_z` 하나 대신, **물체마다 자기 주변 링(ring) 안의 배경 픽셀 중앙값**을 그 물체의
로컬 테이블 기준으로 쓴다:

```
obj_radius_m(0.05m) 밖 ~ +yolo_table_ring_m(0.03m) 안의 배경(비물체) 픽셀
  → 그 중앙값 = "이 물체 주변"의 테이블 높이
```

**왜 전역 하나로는 부족한가**: 카메라 캘리브 잔차나 테이블 자체의 기울기가 있으면, 전역
중앙값은 물체 위치에 따라 오차가 다르게 실린다 — **위치에 따라 편향(bias)이 달라지는 문제라서,
평균을 하나 잡는 방식 자체가 구조적으로 틀린다.** 합성 데이터 검증(`test_segment_from_labels.py`,
5 cm 기울어진 테이블 위 실제 5 cm 돌출 물체 2개)에서 **국소 기준은 5.0/6.0 cm로 잡고, 전역
폴백은 2.6/8.6 cm로 잡았다** — 오차가 2~4배 커진다.

> **일반화**: 이건 §1.4의 nvblox 증분 갱신·§4.2 GraspMoE 판별기와 같은 계열의 패턴이다 —
> **"측정해야 할 기준값 자체가 공간적으로 변한다면, 전역 상수 하나가 아니라 국소 추정으로
> 바꿔야 한다."** 대가는 표본 부족(배경 픽셀이 링 안에 `yolo_min_ring_px`=20개 미만이면
> 국소값을 못 믿고 전역으로 폴백한다) — **국소화는 공짜가 아니라 표본과 편향의 트레이드오프다.**

이 필터의 상한(`obj_max_h`, 기하 경로 전용 0.12 m)은 **YOLO 경로에는 걸지 않는다** — 서 있는
콜라병(20 cm)이 클래스로 이미 걸러진 마당에 "로봇 팔 self-filter" 목적의 상한을 물려주면
멀쩡한 물체가 잘린다. **필터 파라미터는 그 필터가 원래 막으려던 실패 모드가 이 경로에도
적용되는지부터 따져야 한다** — 다른 경로에서 값이 좋았다고 그대로 물려주면 안 된다.

### 6-1.2 클래스 사전지식 기반 크롭 — 형태가 고정된 물체 대 자연물

`dimensions`(`config/objects.yaml`)는 병·컵처럼 **개체마다 형태가 거의 안 변하는 공산품**에
한해, 전역 반경/무제한 상한 대신 **그 클래스의 실측 반경·높이(+margin)**를 크롭 기준으로 쓴다.
사과·바나나 같은 **자연물은 명시적으로 제외**한다 — 개체별 편차가 커서 클래스 대표값이
의미가 없기 때문이다.

> **일반화**: "물체 클래스를 안다"가 곧 "그 물체의 치수를 안다"는 아니다. **클래스 내 형상
> 분산이 작은 경우에만 사전지식이 파라미터를 대체할 수 있고, 분산이 큰 클래스에 같은 사전지식을
> 적용하면 오차가 아니라 편향이 생긴다.** 합성테스트(`test_known_class_uses_measured_radius_and_trims_contamination`)로
> 확인한 효과: 실제 20 cm 병 라벨에 40 cm짜리 오염(이웃 물체·팔 그림자)이 붙어 있어도
> 오염 줄만 정확히 잘린다 — 실측 상한이 "안전 마진 없음"과 "무조건 큰 여유" 사이의 중간을 만든다.

⚠️ 이 필터는 **누락된 depth 자체(반사면이라 점이 안 찍힌 경우)는 못 고친다.** 있는 마스크를
다듬을 뿐 없는 점을 채우지 않는다 — 그 문제의 정공법(알려진 형상을 ICP로 정합해 빈 곳을 메움)은
설계 단계일 뿐 구현되지 않았다.

### 6-1.3 grasp 프레임은 손끝 좌표가 아니다 — 접근축 규약의 함정

**배경**: 그리퍼가 달린 로봇에서 "목표 자세"를 넘길 때 프레임을 하나로 통일해야 한다.
그런데 **접근축이 어느 성분인지는 프레임마다 다르다** — 여기서 실기 IK 전패가 났다.

- **GraspGenX 원시 grasp 프레임**: `+Z = 접근축`, `+X = 손가락 닫힘 방향`, 원점 = 그리퍼 base.
- **`tool0`**: 접근축이 +Z가 아니라 **+X**다 (`onrobot_rg2.xacro:40`, `rpy="1.5708 0 1.5708"`).

이 두 프레임 사이엔 **로컬 요(yaw) +90°** 회전 하나가 있을 뿐인데, grasp pose를 그대로
`ik_link=tool0`으로 넘기면 그리퍼가 **90° 누운 채로** IK가 풀려 실기에서 후보 전부가
`NO_IK_SOLUTION(-31)`로 실패했다(`md/context/constraints.md` "정본: grasp 프레임 = `rg2_base_link`").
고친 위치는 `_accept_grasp()`(FSM) 한 곳 — 생산자(GraspGenX/브리지)는 변환하지 않고 원시
프레임을 그대로 준다.

> **일반화**: **"같은 물리적 자세"도 어떤 프레임을 골랐느냐에 따라 접근축이 다른 축으로 매핑된다.**
> "회전 없음"과 "IK 목표로 쓸 수 있음"은 별개이고, 둘 사이의 변환은 **생산자와 소비자 중
> 한쪽에서만, 한 곳에서만** 해야 한다 — 여러 호출부가 각자 변환하면 부호·각도가 갈라진다
> (§3.4 캘리브 npy 사본 금지와 같은 형태의 교훈).

**손끝(fingertip)까지의 거리도 두 종류가 헷갈린다**: `rg2_base` 기준 `fingertip_from_rg2_base_m(width_m)`
(닫힘 0.218 m, grasp pose에서 잰다)과 플랜지 기준 `fingertip_length_m()`(0.240 m) — 차이 22 mm는
브라켓 두께다. 어느 기준점에서 잰 값인지 이름에 안 들어 있으면(`grasp_tcp` → `grasp_pose`로
개명한 이유) 18 cm 오차를 부른다.

**병렬죠 그리퍼 운동학**: RG2의 두 손가락은 독립 관절이 아니라 `rg2_left/right_outer_knuckle`이
**`rg2_finger_joint` 하나를 mimic**한다(`m0609_rg2_bringup/urdf`, §TF 구조 참고). 대칭 개폐를
mimic joint로 표현하면 컨트롤러가 관절 하나만 명령해도 되고, URDF 차원에서 "양쪽이 항상 대칭"이
강제된다 — 소프트웨어로 대칭을 유지할 필요가 없다.

### 6-1.4 IK 시드 연결 — 다점 궤적은 독립적으로 풀면 안 된다

`pre_grasp → grasp → lift` 3점을 IK로 풀 때, **직전 점의 해를 다음 점의 시드로 넘긴다.**
안 넘기면 매 점이 (다관절 로봇엔 여러 개 있는) 서로 다른 IK 분기에 독립적으로 앉을 수 있고,
그러면 10 cm 수직 하강 하나가 팔 전체를 뒤집는 궤적으로 나올 수 있다. 계획도 포즈가 아니라
**관절 목표**로 준다 — 포즈로 주면 move_group이 IK를 다시 풀어, 우리가 도달 가능하다고
이미 확인한 그 해로 간다는 보장이 없어진다.

> **일반화**: **IK는 다대일(pose → 여러 관절해) 함수다.** 궤적의 연속된 점들을 각각 독립적으로
> 풀면 "각 점은 맞는데 점들을 이은 경로가 안 맞는" 결과가 나온다 — §2.2 cuRobo가 병렬 시드로
> IK 다중해 문제를 다루는 것과 반대 극단(여기는 시드를 좁혀 **일관성**을 얻는 경우)이다.

---

## 7. 열려 있는 질문 (NotebookLM에 물어볼 것)

### 지도 표현 (nvblox / octomap)
1. **TSDF vs Occupancy(log-odds)** — nvblox는 둘 다 지원한다. 매니퓰레이터 근거리 작업(0.3~1.5 m,
   고정 카메라)에서 어느 쪽이 유리한가? 판단 기준은?
2. **ESDF 해상도의 이론적 최적점** — 캘리브 오차(현재 40 mm)와 voxel(0.05 m)의 관계.
   voxel < 오차면 낭비, voxel > 오차면 정보 손실이라는 게 우리 가설인데 근거가 있는가?
3. **거리 전파 알고리즘 계보** — nvblox의 3축 밴드 스윕 + parent_direction은
   Felzenszwalb 거리변환 / Parallel Banding Algorithm / voxblox의 brushfire와 어떤 관계인가?
   증분 갱신에서 정확도를 잃는 지점은 어디인가?
4. **미관측 공간을 free로 두는 것의 위험** — cuMotion은 −1000을 +1000(안전)으로 뒤집는다.
   매니퓰레이터 안전 관점에서 이게 정당한가? 표준적인 대안(unknown을 occupied로, 또는 별도 비용)은?
5. **decay 파라미터 설계** — occupied(.4)를 free(.55)보다 빨리 잊는 기본값의 근거는?
   사람이 드나드는 작업셀에서는 어떻게 잡아야 하는가?
6. **TSDF 가중치 함수 선택** — `kInverseSquareDropoffWeight` 등 4종의 실제 차이와 선택 기준.

### 모션 플래닝 (cuRobo / OMPL)
7. **최적화 기반 vs 샘플링 기반의 공정한 비교 설계** — 어떤 씬·지표를 통제해야 "같은 문제를 풀었다"고
   말할 수 있는가? 문헌의 표준 벤치마크(MotionBenchMaker 등)는 무엇을 통제하는가?
8. **cuRobo가 PRM*를 내부에 두는 이유** — 최적화가 실패하는 지형(좁은 통로)의 특징을 형식적으로
   기술할 수 있는가? graph seed가 실제로 얼마나 자주 필요해지는가?
9. **MPPI → L-BFGS 하이브리드** — 이 조합이 표준인가? 다른 조합(CMA-ES, iLQR, SQP)과의 트레이드오프.
10. **구 근사(sphere approximation)의 이론** — 링크를 구로 덮을 때 과대/과소 추정을 정량화하는 표준
    지표가 있는가? 우리는 XRDF 구 6쌍이 겹쳐 자기충돌 검사를 껐다 — 정공법은 무엇인가?
11. **swept collision vs 이산 분할** — `longest_valid_segment_fraction`을 얼마로 잡아야 터널링이
    없다고 보장할 수 있는가? 거리장 기반 sphere marching은 정말 보장을 주는가?
12. **`activation_distance`(η)의 역할** — 미분가능성을 위한 장치인가 안전여유인가?
    둘을 분리해서 설계하는 방법이 있는가?
13. **최적화 플래너의 재현성** — 같은 입력에 같은 궤적이 나오는가? `parallel_finetune`처럼
    비결정적 요소가 있는 구현에서 재현성을 확보하는 표준 방법.
14. **실행 중 동적 회피** — 계획시점 회피 / stop→재계획 / MPC(cuRobo MPC) 세 가지의
    지연·안전·구현비용 비교. 협동로봇 안전 규격(ISO/TS 15066)과의 관계는?

### 파지 (GraspGenX)
15. **확산모델 grasp 생성** — 위치는 `scaled_linear`, 회전은 `squaredcos_cap_v2`로 스케줄을 나누는
    이유의 이론적 근거는? SO(3) 위의 확산을 다루는 표준 방법(회전 6D 표현 vs SO(3) diffusion)은?
16. **생성기 + 판별기 분리 구조** — 이 구조가 GAN·강화학습 기반 grasp와 비교해 갖는 이점은?
17. **학습 기반 후보 ∪ 기하학적 후보(GraspMoE)** — 이런 하이브리드가 grasp 분야의 표준인가?
    판별기가 두 분포를 공정하게 채점한다는 보장이 있는가?

### 파지 전처리 / 프레임 규약 (§6-1 신규)
18-a. **국소 vs 전역 기준값 추정** — 테이블 높이처럼 "공간적으로 변하는 기준값"을 국소 추정할 때,
   표본 부족(링 안 배경 픽셀 부족)과 국소 편향 사이의 최적 반경·최소 표본수를 정하는 표준 이론은?
   (커널 밀도 추정의 대역폭 선택 문제와 동형인가?)
18-b. **클래스 사전지식이 유효한 조건** — "클래스 내 형상 분산이 작으면 대표 치수를 써도 된다"를
   정량 기준(분산 임계값 등)으로 세우는 표준 방법이 있는가? 6-DoF grasp 파이프라인에서
   이런 형태 사전지식(shape prior)을 쓰는 사례가 더 있는가?
18-c. **다중 프레임 좌표 규약 관리** — 접근축이 프레임마다 다른(GraspGenX +Z, tool0 +X 등)
   상황에서, 변환을 "생산자/소비자 중 한 곳"으로 강제하는 것 외에 표준적인 관리 패턴
   (프레임 네이밍 컨벤션, 타입 시스템으로 프레임 실수를 컴파일 타임에 잡는 방법 등)이 있는가?
18-d. **다점 IK 시드 연결의 이론적 근거** — 연속된 웨이포인트를 시드로 연결하는 방식이 IK
   분기(branch) 전환을 억제한다는 것을 형식적으로 보장할 수 있는가, 아니면 휴리스틱인가?
   redundancy resolution(여유자유도 로봇의 널스페이스 최적화)과 이 문제의 관계는?

### 시스템
18. **GPU 자원 경합** — 한 GPU(8 GB)에 nvblox + cuMotion + segmenter + (장차) GraspGenX를 올릴 때
    커널 스케줄링·VRAM 경합을 다루는 표준 전략(MPS, stream priority, 프로세스 분리)은?
19. **ROS 2에서 GPU 노드를 컨테이너로 분리했을 때의 대가** — 지연·디버깅 가시성 측면.
20. **"조용한 실패"가 많은 스택의 검증 설계** — cuMotion 경로는 실패가 대부분 무음이다
    (octomap 무시·미관측 free·segmenter 누락·AABB 밖). 이런 시스템의 표준 검증/모니터링 패턴은?

---

## 8. 검증 상태

| 항목 | 상태 |
|---|---|
| nvblox TSDF 융합식·가중치 함수 4종 | **검증됨** — `projective_tsdf_integrator.cu:70-82`, `weighting_function.h:11-16` 직접 읽음 |
| nvblox 갱신이 블록 raycast + 복셀 투영 2단계 | **검증됨** — `view_calculator.h:46-64`, integrator 시그니처 |
| ESDF 3축 밴드 스윕 + `parent_direction` + 증분 | **검증됨** — `esdf_integrator.cu:542-595, 1465-1495, 1522-1587` |
| decay integrator 존재·기본 확률(.55/.4)·`decay_tsdf_rate_hz` 5 Hz | **검증됨** — 파라미터 헤더 직접 읽음 |
| 🔴 ESDF 서비스가 **거리만** 주고 기울기는 안 준다 | **검증됨** — `esdf_and_gradients_conversions.cu`에 `SignedDistanceFunctor` 하나뿐, 배열 차원 3개 |
| cuRobo 파이프라인(IK 32 / PRM* / MPPI / L-BFGS / finetune) | **검증됨** — 우리가 쓰는 커밋 `36ea382`을 새로 받아 `motion_gen.py:171-173, 688`, `task/*.yml` 직접 읽음 |
| MPPI `num_particles: 25`, L-BFGS `history:15 / approx_wolfe` | **검증됨** — `particle_trajopt.yml`, `gradient_trajopt.yml` |
| 구-ESDF 보간 + 해석적 기울기 + sphere marching | **검증됨** — `sphere_obb_kernel.cu:716-770, 926-952, 590-701` |
| cuMotion의 부호반전·미관측 +1000·+0.5voxel | **검증됨** — `cumotion_planner.py:408-472` |
| segmenter가 cuRobo 구로 depth를 마스킹 | **검증됨** — `robot_segmenter.py` + cuRobo `wrap/model/robot_segmenter.py:113` |
| GraspGenX DDPM 100 step·스케줄 2종·`r3_6d`·PointNet++ | **검증됨** — `models/generator.py` 직접 읽음 |
| GraspMoE = 확산 ∪ OBB, 판별기 공통 채점 | **검증됨** — `samplers/graspmoe.py`, `samplers/planner.py` |
| 6D→SO(3)가 그람-슈미트이고 출처가 **Zhou et al. CVPR 2019 (arXiv:1812.07035)** | **검증됨** — `utils/transformations.py:474-489` docstring이 논문을 명시 |
| 판별기가 **데이터셋 음성 + on-policy 음성**으로도 학습된다 | **검증됨** — `dataset/dataset.py:865-870, 991-1029` |
| OBB 격자 후보가 판별기 학습 분포 안에 있는지 | 🔴 **미확인.** §9-2(E)의 반론이 여기 걸려 있다 |
| OMPL `longest_valid_segment_fraction: 0.005` | **검증됨** — 우리 `ompl_planning.yaml:166` |
| 계획시간 실측 수치 (16.1 / 94.7 / 42.4 / 110.6 ms) | **검증됨(2026-08-06 실측)** — 출처는 cumotion-bringup §7-1 |
| VRAM 1,508 / 660 / 334 MiB | **검증됨(2026-08-06 실측)** |
| "cuRobo가 어려운 씬에서 유리하다" | 🔴 **추론.** 우리는 장애물 씬에서 안 쟀다 |
| cuRobo 궤적의 완전 결정성 | 🔴 **추론.** `parallel_finetune` 등 확인 안 함 |
| decay가 실제로 사람 팔 잔상을 얼마나 빨리 지우는지 | 🔴 **미측정** |
| MoveIt이 계획 요청 하나를 단일 스레드로 처리 | 🔴 **추론.** move_group 내부 스레딩을 소스로 확인하지 않았다 |

---

## 9-1. NotebookLM 질의 결과 (2026-08-07, §7 중 5개)

> 이 문서를 소스로 넣고(노트북 「cobot2_ws — 강의자료 ↔ 내 코드」) 질의했다.
> **⚠️ NotebookLM 답변은 LLM 출력이다. 아래는 "물어본 결과"이지 "검증된 사실"이 아니다.**
> 인용이 붙은 것도 실제 출처와 어긋난 사례가 있었다(아래 Q4 주의).

**Q7 공정한 비교 설계** → 통제해야 할 것 3가지로 정리해 줬다.
① 씬 난이도는 LaValle 5장의 **ε-goodness**(자유공간 중 로컬플래너로 직접 도달 가능한 최소 부피 비율)와
narrow passage 유무로 정의된다 — ε가 작으면 균등샘플링 플래너 성공률이 **지수적으로** 떨어진다.
② **관절공간 목표는 IK를 안 풀어도 되므로 OMPL에 과도하게 유리하다 → 작업공간(Cartesian) 목표로 던져야 한다.**
(우리 벤치는 관절목표였다 — 이건 우리가 놓친 통제변수다.)
③ 평균 대신 **P50/P90/P95/P99**로 보고하고 로그축 박스플롯으로 꼬리를 본다. 계획시간 외에
**궤적 총 실행시간(Σdt)**과 **저크 제곱합**을 같이 재야 "출력물 품질"이 비교된다.

**Q4 미관측 = free의 위험** → 대안 2가지: (A) unknown을 occupied로 — 안전하지만 센서 섀도잉으로
계획이 상시 실패, (B) unknown 통과에 **위험비용**을 매김 — 유연하지만 비용지형이 복잡해져 국소최적이 는다.
우리 구성(고정 카메라 1대)에 대한 실무 권고: **로봇 뒤편·테이블 하부처럼 영원히 안 보이는 영역을
`CollisionObject`로 강제 등록**하라 — cuMotion은 octomap은 버려도 `collision_objects`는 읽으므로
이게 유효한 안전장치다(§3.5와 정합).
> ⚠️ **이 답변은 "OctoMap 논문 근거"라며 인용을 달았지만, 반환된 인용 매핑은 우리 08-03 다이제스트를
> 가리켰다(octomap 논문은 그 답변의 `sources_used`에 없었다).** 인용 문구를 그대로 인용하지 말 것.

**Q10 구 근사** → 정량 지표는 "소스에 없음"이라 밝힌 뒤 일반론으로 **Hausdorff 거리**(메시→구 =
과대근사 두께, 구→메시 = 삐져나온 두께)와 **부피비 V(S)/V(M)** 을 제시. Modern Robotics 10장은
"근사는 항상 **보수적**(메시를 완전히 덮어야)이어야 한다"고 못박는다.
정공법은 **medial axis 기반 자동 피팅 + 구 개수 제약 하 과대부피 최소화**.
실무 권고: **문제되는 링크쌍 주변만 큰 구 1개를 작은 구 3~4개로 쪼개 재피팅** — cuRobo는 구 하나가
스레드 하나라 개수를 늘려도 지연이 거의 안 는다(§2.4와 정합). `ignore`를 늘리는 것보다 이쪽이 정공법.
> ⚠️ 답변이 제안한 "링크쌍별 activation_distance 개별 튜닝"은 **미확인**이다.
> cuRobo self-collision 설정에 쌍별 여유값이 있는지 소스로 확인하지 않았다.

**Q14 동적 회피** → 세 방식 비교표를 주고, SSM(speed and separation monitoring) 최소 이격거리
`S_p = v_H(T_R+T_C) + v_R·T_R + D_S + C + Z_D + Z_I` 를 제시.
핵심 함의: **시스템 반응시간 T_R에 계획시간이 그대로 들어간다.** cuMotion 110 ms면 그만큼
요구 이격거리가 커져 "사람이 조금만 다가와도 정지"가 된다. → 규격을 지키며 협동하려면
MPC(수 ms)로 가거나, 두산 `DR_QSTOP`(Stop Category 2)를 인지 루프와 타이트하게 동기화해야 한다.
> ⚠️ ISO/TS 15066 수식과 `v_H = 1.6 m/s`는 **소스에 없는 일반론**이라고 답변 자신이 밝혔다.
> 실제 적용 전에 규격 원문을 봐야 한다. 답변이 두 PC(15W CPU / GPU PC)를 섞어 서술한 대목도 있다.

**Q2 해상도 · 오차예산** → 🔴 **이번 질의에서 가장 값어치 있는 결과.**
- 계통오차(bias)는 베이즈 갱신·TSDF 가중평균으로 **원리적으로 제거되지 않는다.** 무작위 노이즈만
  프레임 누적으로 상쇄된다. → "voxel < 캘리브 오차 = 낭비"는 **오차가 계통일 때 특히 참**이다.
  (잘못된 위치에 "아주 정밀하고 단단한 가상의 벽"을 그리게 된다.)
- 합성 규칙: **무작위 성분은 RSS**(√(σ_depth² + σ_calib²)), **계통 성분과 양자화 오차는 선형 합**.
- 그 예산을 우리 값으로 대입하면:

  ```
  필요 마진 ≳ bias(캘리브 40 mm) + voxel/2(50/2 = 25 mm) + 2σ(무작위, 미측정)
           ≳ 65 mm
  현재 cuRobo activation_distance = 25 mm
  ```

> 🔴 **따라서 현행 설정은 오차예산상 마진이 부족하다** — 계통오차 40 mm가 그대로 노출된다.
> 단 이건 **계산상의 결론이지 실측이 아니다.** 전제 두 개가 미검증이다:
> ① 캘리브 40 mm가 정말 bias인지(무작위 성분과 분리 안 됐다) ② `activation_distance`가 유일한
> 마진인지(구 반지름 자체가 이미 부풀려져 있어 실효 마진은 더 클 수 있다 — §2.4의 "6쌍 겹침"이 그 증거다).
> **→ 다음 실기 과제: 캘리브 오차 정량 측정([[ws/cobot2/state]] "다음 할 일" 3번)이 이 계산의 입력이다.**

---

## 9-2. NotebookLM 질의 결과 — GraspGenX 도입 근거 (§7 Q15~17, 발표용)

> 목적: **"왜 GraspGenX인가"를 발표에서 방어할 수 있게** 만드는 것.
> 아래는 질의 결과 + **내가 소스로 직접 확인해 보정한 것**이다. ⚠️ 표시가 보정 지점이다.

### (A) 왜 "생성기 + 판별기 분리"인가 — 세 경쟁 구조 대비

| 대안 | 약점 | GraspGenX가 피하는 방식 |
|---|---|---|
| **단일 회귀** | 정답이 여러 개인 문제에서 **평균으로 수렴** → 물체 양끝 파지의 중간(=허공)을 찌른다 | 확산모델이 분포를 모사 |
| **GAN** | 6-DoF 고차원에서 학습 불안정, **모드 붕괴**(후보 하나로 고착) | 생성기는 후보만, 채점은 별도 |
| **강화학습** | 샘플 효율 낮음, 보상 설계 난이도 | 지도학습 |

**발표에서 쓸 한 줄**: *"생성과 채점을 분리했기 때문에 **다른 방법으로 만든 후보도 같은 자로 잴 수 있다**"* —
이게 GraspMoE(확산 ∪ 기하 OBB)가 가능한 구조적 이유이고, 학습 분포 밖 물체에서
**기하 후보가 백업으로 남는다**는 실용적 이점으로 직결된다.
> ⚠️ NotebookLM이 여기에 **"공장 환경에서 99.9% 이상 무고장 구동 보장"**이라고 붙였는데
> **근거 없는 수치다. 발표에 절대 쓰지 말 것.**

### (B) 왜 "후보 K개 + 점수"가 "최적 1개"보다 나은가 — 하류 제약과 연결

이게 발표에서 가장 설득력 있는 논리다. **파지 자세는 비전 문제가 아니라 시스템 문제다.**

| 하류 제약 | 단일 후보면 | 후보 K개면 |
|---|---|---|
| **IK 도달성** | 관절한계·특이점에 걸리면 **작업 중단** | 1순위 실패 시 2·3순위로 즉시 대체 |
| **충돌 회피** | 파지 자세 자체는 무충돌이어도 **진입 경로**가 막히면 끝 | 플래너가 "경로가 있는 후보"를 고를 수 있다 |
| **그리퍼 개구폭** | 폭 초과면 실패 | 채점 단계에서 미리 배제 (`skip_obb_rule`) |

우리 코드가 이미 이 구조다 — `grasp_selector.py`의 5단계 필터
(신뢰도 → 도달범위 → 접근축 → 폭 → 재충돌)가 정확히 "K개를 하류 제약으로 거르는" 설계다
([[ws/cobot2/detect_graspx]] §5-3).

### (C) 왜 확산모델인가 — multi-modal

같은 물체에 **물리적으로 무관한 정답이 여러 개**다(위에서 집기 / 옆에서 집기 / 손잡이).
L2 회귀는 이 봉우리들의 **평균**으로 수렴해 무너지지만, 확산모델은 역확산으로 분포 자체를 따라가므로
**여러 봉우리를 각각 샘플링**할 수 있다. → "정답이 여럿인 문제"에 구조적으로 맞는 모델이다.

### (D) 왜 쿼터니언이 아니라 6D 표현인가 ⭐ 발표 예상질문 1순위

✅ **소스로 확인했다** — `graspgenx/utils/transformations.py:474-489`가 그람-슈미트
(`b1=normalize(a1)`, `b2=normalize(a2−(b1·a2)b1)`, `b3=b1×b2`)이고, **docstring이 원논문을 명시한다**:

> **Zhou et al., "On the Continuity of Rotation Representations in Neural Networks", CVPR 2019
> ([arXiv:1812.07035](http://arxiv.org/abs/1812.07035))**

**이게 발표에서 인용할 정확한 출처다.** 논지:
- **오일러각**: 짐벌락 — 3-파라미터 표현은 특이점을 피할 수 없다(Modern Robotics 3장 각주).
- **쿼터니언**: `S³`가 `SO(3)`를 **이중 피복**한다 — 하나의 물리적 회전에 `±q` 두 점이 대응한다
  (Modern Robotics 부록 B.12에 명시). 자세가 미세하게 변할 뿐인데 신경망 출력이 `+q ↔ −q`로
  **점프해야 하는 경계**가 생기고, 그 지점에서 손실·기울기가 요동친다.
- **6D**: 위상적으로 연속인 매핑을 만들려면 유클리드 차원이 4보다 커야 한다는 것이 Zhou et al.의 결과다.
  회전행렬 두 열은 이 조건을 만족한다.

> 부가: 코드는 `r3_so3`(축각, `so3_log_map`)와 `r3_euler`도 지원한다
> (`transformations.py:519-523, 544-548`). **표현법이 선택 가능한 설계이고, 기본값이 `r3_6d`다.**

**한 문단 답변안**: *"단위 쿼터니언은 SO(3)를 이중 피복해 하나의 회전에 ±q가 대응합니다.
그래서 연속적인 점군 입력에 대해 신경망 출력이 불연속으로 점프해야 하는 경계가 생기고 수렴을 방해합니다.
Zhou et al.(CVPR 2019)이 연속인 회전 표현에는 5차원 이상이 필요함을 보였고, 회전행렬 두 열을 쓰는
6D 표현이 그 조건을 만족합니다. GraspGenX는 6D로 확산을 돌린 뒤 그람-슈미트로 SO(3)에 사영합니다."*

**"그럼 SO(3) 위에서 직접 확산하면 되지 않나"에 대한 답**: 그 방식(Riemannian score-based,
IsotropicGaussianSO3)이 **기하학적으로는 더 정확하다** — Haar measure를 보존한다
(LaValle 5.2.2: 오일러각 균등샘플링은 SO(3) 균등이 아니다). 대신 매 스텝 지수/로그 사상과
열커널 급수 근사가 필요해 **연산이 무겁다.** GraspGenX는 정확도보다 **표준 DDPM 스택을 그대로
쓸 수 있는 실용성**을 택한 것이다. (뒷부분은 소스 미확인 — **추론**)

### (E) ⚠️ 예상 반론과 우리 쪽 정직한 답 — 판별기의 분포 이동

NotebookLM은 *"판별기는 확산 생성기 출력으로 학습됐으니 기하(OBB) 후보는 OOD이고,
과대신뢰(0.99)를 뱉을 수 있다"*고 답했다. **전제가 부분적으로 틀렸다 — 내 질문이 그렇게 유도했다.**

✅ **소스 확인 결과**: 판별기 학습 데이터에는 **데이터셋이 제공하는 `negative_grasps`** 와
**`negative_grasps_onpolicy`**(생성기 출력을 라벨링한 on-policy 음성) **둘 다** 들어간다
(`dataset/dataset.py:865-870, 991-1029`, `dataset/xgrasp_dataset.py:929-1012`).
→ **판별기는 생성기 출력만 본 게 아니다.** 실패 파지도 보고 배웠다.

그래도 **우려가 사라지진 않는다**: OBB 분기가 만드는 건 yaw 36 × z 6 × 1 cm 격자라는
**규칙적 인공 격자**이고, 데이터셋 음성이 그런 분포인지는 확인 못 했다. 발표에서는 이렇게 답하는 게 정확하다:

> *"판별기는 생성기 출력뿐 아니라 데이터셋 음성·on-policy 음성으로도 학습됩니다(코드 확인).
> 다만 OBB 격자 후보가 그 음성 분포에 포함된다는 보장은 없어, 분기별 점수 캘리브레이션은
> 아직 검증하지 않았습니다."*

**검증 방법(제안받음, 우리 상황에 맞게 축약)**:
- 분기별로 **신뢰도 다이어그램 / ECE**를 따로 계산한다. `diff`는 맞는데 `obb`만 과대신뢰면 캘리브레이션 붕괴다.
- 고치는 법: **분기별 Platt scaling** — 로짓은 공유하되 `tag`별로 `σ(a_tag·logit + b_tag)`를 따로 피팅.
- 🔴 **최소 실험(우리가 실제로 할 만한 것)**: 순위 1·2위가 `obb`이고 3·4위가 `diff`인
  **순위 역전 프레임**을 골라, 상위 `obb` 후보를 `grasp_selector.py`의 **재충돌 필터에 통과시켜 본다.**
  판별기 점수 0.9+인데 충돌 필터에서 탈락하면 → **그 분기에 대해 점수를 못 믿는다**는 증거가 된다.
  **로봇을 안 움직이고 데이터만으로 할 수 있다** — `branch_tags`가 이미 출력에 있다(`planner.py`).

---

## 9-3. NotebookLM 질의 결과 — §6-1(파지 전처리·프레임 규약) 관련 (2026-08-09, §7 신규 18-a~d)

> 소스는 `src/PACKAGES.md`에서 뽑은 발췌 하나만 새로 추가하고(다른 소스는 이미 있던 것),
> 같은 대화(conversation_id)로 4개를 이어 물었다. **⚠️ LLM 출력이며 검증된 사실이 아니다.**

**18-a 국소 vs 전역 기준값 추정** → "완전한 동형은 아니고, KDE보다 **비모수 회귀(지역 선형
회귀)**나 **크리깅(Kriging)**이 더 정확한 이론"이라는 답. 핵심 통찰: 크리깅은 예측값과 함께
**예측 분산**을 내놓고, 표본이 부족하면 그 분산이 커져 **전역 평균으로 자동 수렴(shrinkage)**하게
설계된다 — 우리 `yolo_min_ring_px=20` 폴백 규칙이 "이 베이지안 수렴 필터를 실무적으로 단순화한
것"이라는 해석. 더 정밀하게 하려면 국소 평균 대신 **국소 평면(z=ax+by+c) 최소자승 피팅**을
제안 — 테이블 기울기가 있는 우리 상황에 바로 적용 가능한 제안이라 **다음에 시도해볼 만하다.**

**18-b 클래스 형상 분산의 정량 기준** → **변동계수(CV=σ/μ)**를 표준 지표로 제시하며
"공산품 CV≤0.05, 자연물 CV>0.20~0.30"이라는 **구체적 수치를 붙였다.**
> ⚠️ **이 수치(0.05, 0.20~0.30)는 출처가 없다 — notebook_query 응답의 인용 매핑에 이 수치를
> 뒷받침하는 소스가 없었다.** 우리 문서의 정성적 서술(공산품=저분산, 자연물=고분산)을 그대로
> 되돌려주며 임의의 숫자를 붙인 것으로 보인다. **발표·문서에 이 수치를 인용하지 말 것.**
> 유효한 부분은 방향(6-DoF grasp에서 category-level pose(NOCS류)·기하 프리미티브 피팅·
> shape completion이 같은 계열의 shape prior 활용 사례라는 것)이다 — 이건 일반론으로 타당하다.

**18-c 프레임 관리 패턴** → 우리 방식("한 곳에서만 변환")을 넘어서는 표준 패턴 셋을 제시:
① **URDF에 목적별 가상 프레임을 새로 박아**(`rg2_grasp_frame`), IK를 그 프레임으로 바로
요청 — 지금 `_accept_grasp()`가 코드로 하는 회전을 **URDF의 fixed joint로 옮기는 안**.
② **네이밍 컨벤션**: `T_Source2Target` 표기, `*_optical_frame` 접미사 — 이미 이 저장소가
쓰는 관행과 일치. ③ **타입 시스템 방어**(⚠️ 명시적으로 "소스에 없음 — 일반 원칙"이라고
밝힘): C++ 템플릿으로 프레임 불일치를 컴파일 타임에 잡거나, Python은 `frame_id` 문자열을
런타임에 assert.
> **우리 상황에 대한 판단**: ①(URDF에 `rg2_grasp_frame` 추가)이 가장 싸고 효과 큰 제안이다 —
> 지금은 FSM 코드 한 줄이 이 변환을 쥐고 있는데, URDF fixed joint로 옮기면 **어떤 소비자든
> `ik_link=rg2_grasp_frame`만 쓰면 되고 코드 변환 자체가 사라진다.** 단 이건 검토만 된 제안이고
> 실기 적용은 안 했다.

**18-d IK 시드 연결의 이론적 지위** → **"형식적 보장이 아니라 휴리스틱"** — Modern Robotics
6.5절 인용: Newton-Raphson류는 초기값의 "수렴 영역(basin of attraction)" 안에서만 같은 해로
수렴한다는 보장이고, 특이점 근처·큰 스텝·관절한계에서는 깨질 수 있다. **형식적 보장을 원하면**
두산 `ikin`의 `sol_space`(0~7, Lefty/Righty×Above/Below×Flip/NoFlip 8분기)를 **명시적으로 고정**해
풀라는 게 매뉴얼(`Doosan_..._Programming_Manual`)의 정공법이라고 인용. 여유자유도(7축) 로봇과의
관계: 6축은 널스페이스가 없어 분기 전환이 **불연속 점프**(특이점을 뚫고 지나가야 함)인 반면,
7축은 널스페이스가 있어 분기 사이를 **연속 곡선(self-motion)**으로 이동할 수 있다 — 우리
M0609(6축)는 후자의 옵션이 원천적으로 없다.
> **우리 코드에 대한 함의**: 지금의 "직전 해를 시드로" 방식은 **정공법이 아니라 값싼 근사**임이
> 확인됐다. `sol_space`를 현재 관절각 기준으로 고정하는 안(두산 API `ikin_norm` 또는 동등 기능)이
> 이론적으로 더 안전하지만, **MoveIt의 `compute_ik`가 두산 `sol_space` 개념을 그대로 노출하는지는
> 미확인** — 이 저장소는 MoveIt IK(KDL/pick_ik 등)를 쓰지 `ikin_norm`을 직접 호출하지 않는다.
> 다음에 확인할 것: MoveIt IK 플러그인이 시드 기반 수렴 외에 분기를 명시적으로 고정하는 옵션을
> 제공하는지.

---

## 9. 이 문서에서 뽑은 방법론 (도메인 무관)

1. **자료구조 선택은 취향이 아니라 소비자의 요구다.** octomap→불리언, ESDF→실수+기울기.
   뒤에 붙는 알고리즘이 미분을 요구하면 앞단을 바꿔야 한다 (§1.1).
2. **GPU화는 "같은 알고리즘을 빠른 칩에서"가 아니라 "반복문의 바깥을 뒤집는 것"이다.**
   광선당 → 복셀당으로 뒤집으니 락이 사라졌다 (§1.3).
3. **근사는 반드시 어느 방향으로든 틀린다.** 구 근사는 뚱뚱하거나 홀쭉하고, 그 대가는 사람이
   손으로 조정해야 한다. `padding_offset`과 XRDF 구는 **같은 문제의 다른 층**이다 (§2.4).
4. **벤치마크 조건이 결론을 정한다.** 빈 세계·관절목표는 RRTConnect의 홈그라운드다 (§5.3).
5. **이름을 믿지 말고 필드를 읽어라.** `esdf_and_gradients`에는 기울기가 없다 (§1.6).
6. **스택이 깊어지면 실패 지점은 곱해지고, 대부분 무음이다.** cuMotion 경로의 함정 6개가
   전부 "OMPL은 멀쩡한데 cuMotion만 죽는다" 형태였다 (§5.5).
