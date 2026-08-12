# VLM(A4) × pick_fsm 통합 — 진행 상태

> **권한 경계 한 줄**: VLM 은 *무엇을 · 몇 개를 · 어디에 · 언제 바꿔서* 하는 **높은 수준의 명령권**을
> 갖고, 실제 동작 중의 상태·안전(모션 취소, 그리퍼 개폐, 승인, 물체 보유, 충돌 씬)은
> **deterministic FSM(pick_fsm)** 이 소유한다.

작업 시작 2026-08-12 · 계획 원본은 사용자가 준 `plan.md`(디스크에 없음, 대화 첨부)

관련: [[ws/cobot2/vla-bridge-contract]] · [[ws/cobot2/state]] · [[ws/cobot2/context/constraints]]

---

## 0. 워크스페이스 / 브랜치

| ws | 경로 | 브랜치 | 기준 커밋 |
|---|---|---|---|
| FSM | `~/cobot2_ws_new` | `feat/fsm-pause-hold` | `2f5585b` (semi_Final `7270293` 위) |
| VLM | `~/M0609_VLA_system_new` | `feat/mission-supervisor` | `7069265` (`vla_integed` `a0ed3b3` 위) |

두 ws 의 origin 은 **같다** — `gwanhuiGIM/0730_cobo2_personal`. 브랜치로만 갈린다.
**한 Phase = 양쪽 각각 커밋 1개 이상.** 아티팩트리 변경이 소실된 사고 이력이 있다.

### 🔴 계획서 §1 이 실제와 달랐던 점 (2026-08-12 실측으로 정정)

계획서는 `M0609_VLA_system_new` 을 `wodud4143/M0609_VLA_system @ a4-integ` 로
checkout/clone 하라고 적었다. **그대로 하면 작업이 사라진다.**

- 이 저장소에 `a4-integ` 브랜치는 **없다**(로컬·원격 통틀어 0개). 그건 *다른 클론*
  (`ddoo0922/0730_cobo2_personal @ a4-integ`)의 브랜치였다.
- 머지 커밋 `a0ed3b3` 이 그 a4 계보(`1eaa155`)를 `vla_integ` 실행부(`7ec114f`) 위에 이미 합쳤다.
- `git log a0ed3b3..1eaa155` → **비어 있음**. a4 에만 있고 현재에 없는 커밋은 0개다.
- 반대로 현재는 a4 tip 보다 `7ec114f semifinal_commit` · `64aaac0 integ_raw` 를 더 갖는다.

**즉 `vla_integed` ⊃ `a4-integ`.** 계획서 §1-2 는 폐기하고 `vla_integed` 위에서 작업한다.
계획의 *의도*("a4 의 규칙 계층을 확보한다")는 이미 달성됐다.

---

## 1. Phase 진행

| Phase | 상태 | 내용 |
|---|---|---|
| P0 기준점 | ✅ | 브랜치·클린빌드·테스트·도메인·클래스 대조표 |
| P1 계약 결선 | — | WAIT_PLACE_TARGET 상태표 / set_place / allow_unverified_place |
| P2 MissionSupervisor | — | `mission.py`, R1~R5, MissionState msg |
| P3 Mission Revision | — | 인터럽트 보존, modify/cancel 툴, destination 슬롯 |
| P4 Hold/Release | — | `WAIT_PLACE_TARGET→RELEASE`, `/pick/release_now` |
| P5 PAUSED 분리 | — | `State.PAUSED`, `/pick/pause·resume`, `cmd:"stow"` |
| P6 손목 Seam | — | `committed_grasp`, `AcquireTarget.srv` (구현 아님, 자리만) |

### P0 결과 (2026-08-12)

- `build/ install/ log/` **전량 재생성**. 구 `install/` 에 `/home/kimkh/cobot2_ws/`(구 경로)가
  박혀 있었다 — colcon 산출물엔 절대경로가 박히므로 ws 디렉토리 이름만 바꿔도 깨진다.
- `colcon build --symlink-install` → **34 packages PASS** (경고만: Doosan deprecated API)
- `pytest src/pick_fsm/test src/voice_processing/test` → **67 passed**
- `pytest src/vla_system/test` (VLA ws) → **188 passed**
- `ROS_DOMAIN_ID=93` 양쪽 명시 — `scripts/container_setup.sh:61`(FSM), `scripts/env.sh`(VLA, 이번에 추가).
  기본값 0 이면 `perception_node` 가 "no frames processed yet" 만 찍으며 매달린다(다른 증상 없음).
- `pixel_policy` 는 계획대로 **`warn` 유지**. `select` 전환은 P2 통과 후 같은 클래스 2개 실기로 검증.

#### 🐛 P0 가 잡은 값 불일치 — `fsm_listening_timeout_sec`

`vla_command.launch.py` 의 기본값이 **`60.0`** 인데 정본
`task_manager.DEFAULT_TIMEOUTS[State.LISTENING]` 은 **`110.0`** 이었다. 노드
`declare_parameter` 는 110.0 으로 맞아 있어 **launch 로 띄울 때만** 어긋났다
(= 실기 경로에서만 재현되고 단독 노드 실행으로는 안 잡힌다).

`110.0` 으로 맞췄다. 이 값은 **세 곳에 복제**돼 있다 — 하나를 고치면 나머지 둘도 고친다:

```
src/pick_fsm/pick_fsm/task_manager.py:56          ← 정본
src/voice_processing/voice_processing/vla_command_node.py:376
src/voice_processing/launch/vla_command.launch.py:94
```

`test_vla_command.py` 의 `test_{node,launch}_default_matches_fsm_listening_timeout`
두 건이 이 복제를 감시한다. **지우지 말 것.**

---

## 2. ⏳ 사용자 결정 대기 — 클래스 허용목록 (계획 G4)

> ⚠️ **임의로 통일하지 않는다** (계획 §8 Do-not). 아래는 실측 대조표일 뿐 조치는 안 했다.

| 구분 | 클래스 |
|---|---|
| FSM `config/objects.yaml:detect` (7) | `bottle, cup, spoon, banana, apple, orange, mouse` |
| VLA `config/system.yaml:target_classes` (15) | `apple, banana, orange, cup, bottle, wine glass, book, mouse, cell phone, remote, sports ball, teddy bear, clock, scissors, knife` |
| **교집합 = 지금 실제로 통과하는 것 (6)** | `apple, banana, orange, cup, bottle, mouse` |
| FSM 에만 (1) | `spoon` — VLA 가 지시할 수단이 없다 |
| VLA 에만 (9) — **즉시 거부됨** | `wine glass, book, cell phone, remote, sports ball, teddy bear, clock, scissors, knife` |

🔴 **가장 나쁜 UX 케이스**: `scissors`/`knife` 는 VLA 의 `HAZARD_CLASSES` 라
*"가위는 위험한 물건인데 가져올까요?"* → 사람이 *"응"* → **그리고 브리지가 거부**한다.
사용자에게 확인까지 받아놓고 못 하는 흐름이다.

**선택지 (둘 중 하나, 사용자 결정)**
- **(a)** FSM `objects.yaml:detect` 에 클래스 추가 — COCO 80종이라 기술적으론 가능.
  대가: 작업대 노이즈를 물체로 오인할 여지가 늘어난다(`objects.yaml` 주석의 경고).
- **(b)** VLA `target_classes` 를 교집합 6개로 좁힘 — 즉시 안전, 표현력 감소.

---

## 3. 이번 통합에서 하지 않는 것

- **손목(eye-in-hand) RealSense 구현** — 계획 §7. 자리(seam)만 판다.
  이유가 "시간이 없어서"가 아니다: 완벽히 구현해도 **그 코드에 도달하지 못한다.**
  `_st_plan` 이 `pre_grasp`/`grasp`/`lift` **3점 IK 전부 성공**해야 다음으로 가고
  (`task_manager.py:988,1003-1005`), `PERCEIVE` 에서 grasp 못 얻으면 `SPEAK_FAIL` 이다.
  병목은 `REGRASP` 가 아니라 `PERCEIVE→PLAN` **상단 의존성**이다.
- `vla_interfaces` 커스텀 msg 를 FSM ws 로 가져오는 것 — 경계는 JSON 3채널뿐(I1).
- `require_approval` 을 코드로 되돌리는 것 — 2026-08-11 사용자 결정(false 아님, 유지).
