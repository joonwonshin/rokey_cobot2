<!-- meta
updated: 2026-08-11
status:  live
owns:    RMPflow 원리(RMP=가속도정책+리만계량 · pushforward/pullback · 컴퓨테이션 그래프) 개념 설명
         · 반응형 모션생성 패러다임 비교(OMPL / cuRobo MotionGen / cuRobo MPC(MPPI) / RMPflow)
         · "이 ws의 실시간 솔버는 RMPflow가 아니라 MPPI-MPC"라는 소스 근거
         (실행 명령·재계획 루프 구현 상세는 소유하지 않는다 → src/cumotion/README.md,
          movegroup 실행모델 한계는 md/context/movegroup_rmpflow_review.md)
-->

# RMPflow와 반응형 모션 생성 — 원리 + 이 스택의 실제 솔버(MPPI-MPC) 정리

> **이 문서의 용도**: NotebookLM 소스로 넣어 **"실행 중에 로봇이 장애물을 피하려면 무엇이 필요한가"**
> 라는 문제(반응형/reactive 모션 생성)를 전공 수준으로 질문하기 위한 자료다.
> 08-03 다이제스트가 CPU·샘플링 전역 플래닝(OMPL), 08-07 다이제스트가 GPU·전역 궤적 최적화(cuRobo
> MotionGen)를 다뤘다면, 이 문서는 그 **다음 축 — "한 번 계획하고 끝"이 아니라 "제어 주기마다 다시
> 정하는" 반응형 계열**을 다룬다. 대표 이론이 **RMPflow**이고, 우리 스택에 실제로 설치돼 있는 것은
> **cuRobo의 MPC(MPPI)** 다. 둘은 같은 문제를 풀지만 원리가 다르다 — 그 차이가 이 문서의 핵심이다.
>
> **근거 원칙**: 우리 저장소 안의 사실(무엇이 설치돼 있나, 어느 파일 몇 줄)은 **실제 소스 줄**에서
> 왔고 본문에 파일 경로를 적었다. RMPflow 자체의 알고리즘 서술은 **이 저장소에 코드가 없어서**
> (grep 0건, §3-2) 공개 논문에 기댄 것이라 `추론`으로 표시했다. 검증 상태 요약은 §7.
>
> 읽은 소스 (우리 저장소):
> - cuRobo — `isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo/src/curobo/` (이 저장소에 있음)
>   - `wrap/reacher/mpc.py`, `wrap/reacher/motion_gen.py`, `rollout/cost/*`
> - `isaac_ros_cumotion` launch 8개 — `isaac_ros-dev/src/isaac_ros_cumotion/**/launch/*.launch.py`
> - `md/context/movegroup_rmpflow_review.md` (move_group 실행모델 한계 · SRDF 자기충돌 실기 기록)
>
> RMPflow 이론 출처 (저장소 밖, `추론`):
> - Cheng et al., "RMPflow: A Computational Graph for Automatic Motion Policy Generation",
>   WAFR 2018 (arXiv:1811.07049) — §2 수식은 이 논문과 직접 대조함(2026-08-11)
> - Ratliff et al., "Riemannian Motion Policies", 2018 (arXiv:1801.02854)

**시스템 구성** (이 문서의 수치가 나온 환경)
- 로봇: 두산 M0609 6축 (ROS 2 네임스페이스 `/dsr01`) + OnRobot RG2 2지 그리퍼
- 카메라: Intel RealSense D435i, **eye-to-hand** 고정 (팔에 붙어 있지 않다)
- GPU PC(`rokey`): RTX 4060 Laptop **8 GB**, Isaac ROS 3.2 컨테이너(Humble). 실측 GPU 2.5/8 GB 사용
- 개인 PC(`kimkh-...`): GPU 없음 → OMPL 경로만, cuRobo/MPC 불가

---

## 0. 한 장 요약 — 모션 생성의 네 가지 방식, "언제 계획하나"로 갈린다

로봇이 A에서 B로 갈 경로/궤적을 만드는 방법은 **"언제, 몇 번 계획하는가"** 로 갈린다.
이 축을 모르면 "RMPflow를 도입하자"는 제안이 무슨 층위의 이야기인지 안 잡힌다.

```
계획 시점 →   [출발 전 한 번]                    [제어 주기마다 계속]
            ┌───────────────────────┐        ┌──────────────────────────┐
 전역/샘플링 │ OMPL / MoveIt (CPU)   │        │  (해당 없음 — 샘플링 전역  │
            │  샘플링 탐색, 1회 계획 │        │   플래너는 실시간이 아니다)│
            └───────────────────────┘        └──────────────────────────┘
            ┌───────────────────────┐        ┌──────────────────────────┐
 최적화     │ cuRobo MotionGen(GPU) │        │ cuRobo MPC = MPPI (GPU)   │
            │  배치 궤적 최적화,1회  │        │  짧은 지평 반복 최적화     │
            └───────────────────────┘        └──────────────────────────┘
                                             ┌──────────────────────────┐
 반응정책   │  (해당 없음)          │        │ RMPflow                   │
            │                       │        │  지평 없음, 가속도장 즉시  │
            └───────────────────────┘        └──────────────────────────┘
```

| 방식 | 정체 | 계획 횟수 | 동적 장애물 반응 | 이 ws에 있나 |
|---|---|---|---|---|
| **OMPL/MoveIt** | CPU 샘플링 전역 플래너 | 출발 전 1회 | ❌ (계획 후엔 못 바꿈) | ✅ (기본 경로) |
| **cuRobo MotionGen** | GPU 배치 궤적 최적화 | 출발 전 1회(빠름) | ❌ (한 번의 스냅샷 기준) | ✅ (cuMotion) |
| **cuRobo MPC (MPPI)** | GPU 샘플링 receding-horizon | 제어 주기마다 | ✅ (짧은 지평) | ✅ (`mpc.py`, **단 ROS 노드 없음**) |
| **RMPflow** | 반응형 가속도 정책 (미분기하) | 제어 주기마다 | ✅ (지평 없이 즉시) | 🔴 **없음** (grep 0건) |

**핵심 한 줄**: 원문 보고서가 "RMPflow를 쓰면 실행 중 회피가 된다"고 했는데, 우리 cuRobo 설치엔
RMPflow가 없다. 같은 목적(실행 중 회피)의 도구로 실제 있는 것은 **MPC(MPPI)** 이고, 그마저도
**ROS 2 노드로 감싸는 코드는 우리가 직접 써야 한다**(§5).

---

## 1. 배경 — "실행 중 재계획"이 왜 별도의 문제인가

### 1.1 전역 플래너의 근본 한계

OMPL이든 cuRobo MotionGen이든 **전역(global) 플래너**는 이렇게 동작한다:

1. 현재 순간의 세계(장애물 지도)를 스냅샷으로 찍는다.
2. 그 스냅샷 위에서 출발→목표의 **완결된 궤적**을 만든다.
3. 컨트롤러에 넘기고 **끝까지 재생**한다.

문제는 2→3 사이에 세계가 변하면(사람 손이 들어오면) 로봇은 **이미 지나간 계획을 그대로 재생**한다는
것이다. `md/context/movegroup_rmpflow_review.md` §2가 실기로 확인한 바:

- `move_group`은 실행 중(`plan_only=False`) **5.9초 동안 블록**되고, 그 사이 재계획 요청은 큐에
  쌓일 뿐 반영되지 않는다.
- 심지어 `plan_only=False`일 땐 MoveIt이 우리가 준 start_state를 버린다
  ("Ignoring the state supplied as start state") → 실행 중 예측·인계가 원리적으로 불가능.

그래서 "실행 중 회피"는 **더 빠른 전역 플래너로 해결되는 문제가 아니다.** 계획-실행-재계획의
**루프 구조 자체**를 바꿔야 한다. 여기서 두 갈래가 나온다.

### 1.2 두 갈래 — 짧은 지평 반복(MPC) vs 지평 없는 반응정책(RMPflow)

| | 접근 | 비유 |
|---|---|---|
| **MPC** | 제어 주기마다 **짧은 미래(예: 30스텝)** 를 최적화, 첫 스텝만 실행하고 버림 | "몇 초 앞만 보고 운전, 매 순간 다시 봄" |
| **RMPflow** | 미래를 보지 않는다. 현재 상태에서 **바로 지금 낼 가속도**를 정책으로 계산 | "손이 뜨거우면 즉시 뗀다 — 미래 계산 없음" |

둘 다 "제어 주기마다 계속 계산"이라 동적 장애물에 반응한다. 차이는 **미래를 시뮬레이션하느냐** 다.

---

## 2. RMPflow란 무엇인가 (원리)  `추론` — 저장소에 코드 없음, 논문 기반

> ⚠️ 이 절은 전부 공개 논문에 근거한 개념 설명이다. **우리 저장소엔 RMPflow 구현이 없다**(§3-2에서
> grep 0건 확인). 따라서 "우리 코드가 이렇게 한다"가 아니라 "RMPflow 이론이 이렇다"로 읽어야 한다.

### 2.1 RMP = (가속도 정책 a, 리만 계량 M) 한 쌍

**RMP(Riemannian Motion Policy)** 는 어떤 태스크 공간 위에서 정의된 **두 개의 물체 쌍**이다.
논문(arXiv:1811.07049)은 두 표현을 구분한다:

- **정준형(canonical form) `(a, M)`** — **a** 는 위치·속도 `(x, ẋ)`를 받아 **원하는 가속도 ẍ_d** 를
  내는 함수(예: "목표로 당기는 가속도", "장애물에서 밀어내는 가속도"), **M** 은 그 정책이
  **얼마나·어느 방향으로 중요한지**를 나타내는 (상태 의존) 양의준정부호 행렬. 스칼라 가중치가
  아니라 **방향별 가중치**다. 밑바탕은 리만 계량이지만, 아래 pullback 대수에서 논문은 이 M을
  **"inertia matrix(관성행렬)"** 라 부른다(계량에서 유도된 것).
- **자연형(natural form) `(f, M)`** — 여기서 **f = M a** 를 "desired force map(원하는 힘)"이라 한다.
  변환이 사소해 보이지만 **RMP-대수(pullback)가 실제로 누적하는 것은 가속도 a가 아니라 이 힘 f**
  라서 이 구분이 §2.3에서 결정적이다.

왜 계량이 핵심인가: 장애물 회피 정책은 "장애물 **쪽 방향**"으로만 강하게 작동해야 하고 접선 방향은
자유로워야 한다. 목표 도달 정책은 모든 방향에서 고르게 작동한다. 이 "방향별 중요도"를 M이 담아서,
여러 정책을 합칠 때 **각자 중요한 방향에서만 우선권**을 갖게 한다. (스칼라 가중합이면 회피가 목표
당김에 방향까지 섞여 뭉개진다.)

### 2.2 태스크 공간과 태스크 맵 — 문제를 여러 leaf로 쪼갠다

RMPflow는 하나의 큰 정책을 만들지 않는다. 대신 **여러 개의 작은 태스크 공간**에 각각 RMP를 둔다:

| leaf 태스크 공간 | 거기서의 RMP |
|---|---|
| 엔드이펙터 위치 (3D) | 목표로 당기는 attractor RMP |
| 로봇 각 링크 ↔ 각 장애물까지의 거리 (1D) | 거리가 줄면 급격히 미는 repulsor RMP |
| 각 관절값 (1D) | 관절 한계에 다가가면 밀어내는 damper RMP |
| 자세/방향 | 방향 정렬 RMP |

각 태스크 공간은 **관절공간 q 로부터의 매핑(task map) φ**로 연결된다: `x = φ(q)`.
예를 들어 엔드이펙터 위치는 순기구학 `x = FK(q)`, 관절 한계는 항등사상.

### 2.3 pushforward / pullback — 컴퓨테이션 그래프로 합친다

여러 leaf RMP를 하나의 관절공간 명령 `q̈` 로 모으는 것이 RMPflow의 이름값("flow")이다. 트리 구조의
**컴퓨테이션 그래프**에서 두 연산이 오간다:

- **pushforward** (뿌리→잎): 현재 관절 상태 `(q, q̇)`를 각 태스크 맵으로 밀어 보내
  각 leaf의 `(x, ẋ)`를 구한다. `ẋ = J q̇` (J는 태스크 맵의 야코비안).
- **pullback** (잎→뿌리): 각 leaf가 낸 **자연형 `(f, M)`** (f = Ma)을 야코비안으로 **부모 공간으로
  끌어내려** 합친다. 논문의 정확한 식(child i, 맵 y=ψ(x), J=∂ψ/∂x):
  - 관성행렬: `M_parent = Σᵢ Jᵢᵀ Mᵢ Jᵢ`
  - 힘: `f_parent = Σᵢ Jᵢᵀ (fᵢ − Mᵢ J̇ᵢ ẋ)` — 여기 **`J̇ẋ` 보정항이 핵심**이다(논문이 선행연구와
    갈리는 지점으로 강조: "including J̇ẋ is critical to implement consistent policy behaviors").
- **resolve** (뿌리에서만): 관절공간에 다 모이면 최종 가속도를 **Moore-Penrose 유사역행렬**로 푼다:
  `q̈* = M_r⁺ f_r` (= `(ΣJᵀMJ)⁺ · Σ Jᵀ(f − M J̇ẋ)`). 일반 역행렬이 아니라 유사역행렬이고,
  수치 안정성 때문에 **뿌리 노드에서 단 한 번만** 역행렬을 취한다.

직관: 각 태스크가 "나는 이 방향으로 이만큼 급하게 움직이고 싶다(a), 그리고 내 방향이 이만큼
중요하다(M)"고 말하면, 뿌리에서 **각자의 중요 방향을 존중한 타협해**를 관절 가속도로 낸다.

### 2.4 왜 "리만"인가 — 기하적 일관성과 안정성

RMPflow의 이론적 매력은 이 pullback 규칙이 **기하 동역학계(GDS, Geometric Dynamical System)** 구조를
보존한다는 점이다. 각 leaf가 특정 조건(계량이 상태의 곡률과 정합)을 만족하면, 합쳐진 전체 시스템도
**Lyapunov 안정성**(목표로 수렴, 발산 안 함)을 이론적으로 보장받는다. 이게 "그냥 힘을 벡터합하는
포텐셜장 방법(APF)"과 갈리는 지점이다 — APF는 안정성·지역최소 보장이 없다.

### 2.5 RMPflow의 성질 요약 (장단점)

| | |
|---|---|
| ✅ **지평 없음** | 미래 시뮬레이션 안 함 → 계산이 가볍고 **수백 Hz** 반응 가능 |
| ✅ **모듈성** | leaf 하나 추가/제거로 새 제약(새 장애물)을 붙임 |
| ✅ **안정성 이론** | GDS 조건 하 Lyapunov 수렴 보장 |
| 🔴 **지역최소** | 전역 계획이 없다 → U자 장애물 등에서 갇힐 수 있음 (attractor 설계·전역 가이드 필요) |
| 🔴 **튜닝 난이도** | 계량·정책 함수 설계가 손이 많이 감 |
| 🔴 **정확한 목표 도달 X** | 실시간 반응이 목적이라 "정밀 배치"는 전역 플래너에 위임하는 게 보통 |

---

## 3. 우리 스택의 실시간 솔버는 RMPflow가 아니라 MPC(MPPI)다

### 3.1 MPPI란 — 샘플링 기반 receding-horizon 최적화

cuRobo의 실시간 반응 솔버는 **MPC**이고, 그 내부 최적화기는 **MPPI(Model Predictive Path
Integral)** 다. 동작 원리:

1. 현재 상태에서 **짧은 지평(예: 30스텝)** 의 제어열을 **수백~수천 개 무작위로 샘플링**(입자).
2. 각 샘플 궤적을 롤아웃해 **비용**을 매긴다(목표 거리 + 충돌 + 관절한계 + 부드러움…).
3. 비용에 지수가중(softmax 유사)해 샘플들을 **가중평균 → 갱신된 제어열**.
4. **첫 스텝만 실행**하고 나머지는 버린 뒤, 다음 주기에 1로 되돌아간다(receding horizon).

RMPflow와의 근본 차이: MPPI는 **미래를 실제로 시뮬레이션**(롤아웃)하고 **무작위 표본**으로 푼다.
RMPflow는 미래를 안 보고 **닫힌 형태의 가속도 정책**을 낸다. 전자는 지역최소에 강하지만(표본이
넓게 퍼짐) 계산이 무겁고, 후자는 가볍지만 지역최소에 약하다.

### 3.2 소스 근거 — RMPflow 0건, MpcSolver 실재

우리가 쓰는 cuRobo 커밋(`isaac_ros_cumotion/curobo_core`)을 직접 grep 한 결과:

```
$ grep -ril "rmpflow" curobo_core/curobo/src        → 0 hits          # RMPflow 없음
$ ls curobo_core/curobo/src/curobo/wrap/reacher/mpc.py → 존재          # MPC 있음
mpc.py:14  "The solver uses Model Predictive Path Integral (MPPI) optimization as the ..."
mpc.py:52  from curobo.opt.particle.parallel_mppi import ParallelMPPI  # 입자 기반 MPPI
```

MPPI가 최소화하는 **비용항**들도 소스에 그대로 있다 —
`curobo/rollout/cost/` :
`pose_cost.py`(목표 자세), `primitive_collision_cost.py`(장애물 충돌),
`self_collision_cost.py`(자기충돌), `bound_cost.py`(관절 한계), `stop_cost.py`(정지),
`manipulability_cost.py`(조작성). 이것이 §3-1의 "비용을 매긴다" 단계의 실체다.

### 3.3 RMPflow vs MPPI-MPC 개념 비교

| | RMPflow (없음) | cuRobo MPC = MPPI (있음) |
|---|---|---|
| 미래 시뮬레이션 | ❌ 없음 (즉시 가속도) | ✅ 짧은 지평 롤아웃 |
| 최적화 방식 | 닫힌 형태 (계량 가중 최소자승) | 무작위 표본 + 지수가중 평균 |
| 계산 부하 | 매우 가벼움 (수백 Hz) | 무거움, GPU 필요 (수십 Hz) |
| 지역최소 | 약함 (전역 가이드 필요) | 상대적으로 강함 (표본 확산) |
| 제약 추가 방식 | leaf RMP 추가 | 비용항(cost) 추가 |
| 안정성 이론 | GDS/Lyapunov 보장 | 확률적, 형식 보장 없음 |
| 우리 저장소 | 🔴 grep 0건 | ✅ `wrap/reacher/mpc.py` |

---

## 4. 전체 스펙트럼 한눈에 — 네 솔버 정면 비교

| | OMPL/MoveIt | cuRobo MotionGen | cuRobo MPC(MPPI) | RMPflow |
|---|---|---|---|---|
| 계열 | 샘플링 전역 | 최적화 전역 | 샘플링 반응 | 정책 반응 |
| 계획 시점 | 출발 전 1회 | 출발 전 1회 | 매 주기 | 매 주기 |
| 동적 장애물 | ❌ | ❌ | ✅(지평 내) | ✅(즉시) |
| 전역 최적/탈출 | ✅ | ✅ | △ | ❌(지역) |
| 정밀 목표 도달 | ✅ | ✅ | △ | ❌ |
| 연산 | CPU | GPU 배치 | GPU 실시간 | 경량(수백Hz) |
| 이 ws | ✅ 기본 | ✅ cuMotion | ⚠️ 라이브러리만 | 🔴 없음 |

**읽는 법**: 전역(왼쪽 둘)은 "완벽한 계획을 한 번", 반응(오른쪽 둘)은 "그럭저럭한 결정을 계속".
실전 파이프라인은 보통 **전역으로 큰 경로를 잡고 반응형으로 실시간 수정**하는 하이브리드다.
우리가 원하는 "실행 중 회피"는 오른쪽 두 칸의 능력인데, RMPflow 칸은 비어 있고 MPPI 칸은
**라이브러리는 있으나 ROS 노드가 없다**(§5).

---

## 5. 이 ws에 대한 함의 — 원문 보고서 vs 실제

`md/context/movegroup_rmpflow_review.md` §4가 정리한 격차를, 이제 원리까지 붙여 다시 본다:

| | 원문 보고서 전제 | 이 ws 실제 | 이 문서로 보강된 이유 |
|---|---|---|---|
| 실시간 솔버 | RMPflow | 🔴 없음 → 대신 cuRobo `MpcSolver`(MPPI) | §2 vs §3 — 계열 자체가 다르다 |
| ROS 2 노드 | 즉시 사용 가능 | 🔴 MPC 래핑 ROS 노드 없음(launch 8개 중 없음) → **신규 작성** | §3-2, launch 목록 |
| 실행 중 재계획 | RMPflow 기본 제공 | `reactive_replan.py` 직접 설계 중(미완성, `_same_path` 버그) | §1-2 루프 구조 문제 |
| GPU 여유 | 우려 대상 | 실측 2.5/8 GB — 문제 아님 | — |

**결론(이 문서가 더하는 판단)**:

1. **"RMPflow 도입"은 표현이 부정확하다.** 우리가 실제로 할 수 있는 선택지는
   (a) cuRobo **MPC(MPPI)** 를 ROS 노드로 감싸기, 또는 (b) RMPflow를 **처음부터 구현**(저장소에
   없으므로)이다. (b)는 사실상 새 연구과제라 우선순위가 아니다.
2. 지금 진행 중인 `reactive_replan.py`(3Hz plan_only=True + JTC 직접 선점)는 **MPC도 RMPflow도
   아닌 제3의 방식** — 전역 플래너를 빠르게 반복 호출하는 수제 루프다. 개념상 MPC에 가깝지만
   최적화가 아니라 "매번 새로 전역 계획"이라 지평 개념이 없다.
3. **어느 솔버를 고르든 먼저 풀어야 하는 하위 계층 문제**가 하나 있다 → §6.

---

## 6. 솔버와 무관하게 먼저 풀어야 할 것 — 그리퍼 SRDF 자기충돌

> 상세·해결절차는 `md/context/movegroup_rmpflow_review.md` §3이 단일 출처. 여기선 "왜 이게
> RMPflow/MPC 논의보다 아래 계층인가"만 요약한다.

cuMotion/MoveIt이 계획엔 **성공**하는데(`Trajectory success!`) 재검증에서 32개 웨이포인트가
전부 무효 처리되고 **조용히 실행이 건너뛰어진다**. 원인은 `m0609_rg2.srdf`의 좌↔우 그리퍼
교차쌍 9개 중 **4개가 자기충돌 면제 목록에서 누락**된 것(닫힌 자세에서 실제로 메시가 겹쳐
Setup Assistant가 제외한 쌍들).

이건 **재계획 루프 구조와 무관한 더 아래 계층 문제**다. RMPflow를 쓰든 MPPI를 쓰든 수제 루프를
쓰든, 그리퍼가 자기 자신과 충돌 판정되면 어떤 계획도 실행 단계에서 버려진다. 그래서 우선순위가
가장 높다(그리퍼 **열고** 같은 궤적 재실행 → 실기 무해한 1차 판별부터).

---

## 7. 검증 상태 요약

| 주장 | 상태 | 근거 |
|---|---|---|
| RMPflow가 이 cuRobo 설치에 없다 | ✅ 검증됨 | `grep -ril rmpflow` = 0 (2026-08-11) |
| 실시간 솔버는 MPC(MPPI)다 | ✅ 검증됨 | `wrap/reacher/mpc.py:14`, `parallel_mppi` import |
| MPPI 비용항 구성 | ✅ 검증됨 | `rollout/cost/*.py` 파일 목록 |
| MPC 래핑 ROS 노드가 없다 | ✅ 검증됨 | cuMotion launch 8개에 MPC 없음(seg/attach/example뿐) |
| RMPflow 알고리즘 서술(§2) | 🟢 논문 대조됨 | arXiv:1811.07049 원문과 pullback·resolve 식 직접 대조(08-11). 단 저장소 코드는 없음 |
| MPPI가 지역최소에 더 강하다(§3-3) | 🟡 추론 | 일반론, 우리 로봇에서 실측 안 함 |
| GPU 2.5/8 GB 사용 | ✅ 검증됨 | 실기 측정(08-08, review §4) |
| SRDF 4쌍 누락 | ✅ 검증됨 | `m0609_rg2.srdf` 직접 확인(08-10) |

---

## 상호참조

- **move_group 실행모델 한계 · SRDF 자기충돌 · 원문 vs 실제 비교**(실기 기록):
  [[ws/cobot2/context/movegroup_rmpflow_review]]
- **재계획 루프 구현·실행법·`_same_path` 버그**: `src/cumotion/README.md` (⭐절)
- **nvblox/cuRobo MotionGen 알고리즘·MoveIt vs cuRobo 전역 비교**:
  [[ws/cobot2/2026-08-07-nvblox-curobo-digest]] — 이 문서의 §4 "전역" 두 칸이 거기 상세히 있다
- **핸드아이 캘리브·TF·OMPL octomap**: [[ws/cobot2/2026-08-03-notebooklm-digest]]
