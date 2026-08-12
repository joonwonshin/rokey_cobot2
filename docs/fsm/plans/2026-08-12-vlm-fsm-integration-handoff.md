# VLM(A4) × pick_fsm 통합 — 인계 문서

> **권한 경계 한 줄**: VLM 은 *무엇을 · 몇 개를 · 어디에 · 언제 바꿔서* 하는 **높은 수준의
> 명령권**을 갖고, 실제 동작 중의 상태·안전(모션 취소, 그리퍼 개폐, 승인, 물체 보유,
> 충돌 씬)은 **deterministic FSM(pick_fsm)** 이 소유한다.

작성 2026-08-12 · 작성자 Claude Opus 5 · 계획 원본은 사용자가 대화로 준 `plan.md`(디스크에 없음)

**현재 상태: P0~P6 구현 완료, 빌드·단위테스트 전부 PASS, 🔴 실기 전량 미검증.**
**P4·P5·P6 는 커밋되지 않았다 (사용자 지시).** 남은 것은 P7(선택) 과 **실기 검증**뿐이다.

---

## 0. 인계받고 30초 안에 할 일

```bash
# 1) 지금 상태가 정말 green 인지 직접 확인한다 (믿지 말고 돌려본다)
cd ~/cobot2_ws_new && source /opt/ros/humble/setup.bash \
  && colcon build --symlink-install --packages-select pick_fsm pick_fsm_msgs voice_processing graspgenx_perception \
  && source install/setup.bash && python3 -m pytest src/pick_fsm/test src/voice_processing/test -q
# 기대: 4 packages finished / 77 passed

cd ~/M0609_VLA_system_new && bash scripts/build.sh \
  && source scripts/env.sh && python3 -m pytest src/vla_system/test -q
# 기대: 2 packages finished / 230 passed

# 2) 미커밋 변경이 그대로 있는지 확인 (P4·P5·P6 가 여기 들어 있다)
git -C ~/cobot2_ws_new status --short          # 8개 파일 + 신규 2개
git -C ~/M0609_VLA_system_new status --short   # 8개 파일
```

🔴 **`~/M0609_VLA_system_new` 는 `colcon build` 를 맨손으로 돌리지 마라. `bash scripts/build.sh` 를 써라.**
이유는 §6-C 에 있다. 두 번 데였다.

---

## 1. 워크스페이스 · 브랜치 · 커밋

| ws | 경로 | 브랜치 | 기준 커밋 |
|---|---|---|---|
| FSM | `~/cobot2_ws_new` | `feat/fsm-pause-hold` | `7270293` (semi_Final) |
| VLM | `~/M0609_VLA_system_new` | `feat/mission-supervisor` | `a0ed3b3` (vla_integed) |

두 ws 의 origin 은 **같다** — `gwanhuiGIM/0730_cobo2_personal`. 브랜치로만 갈린다.
**한 Phase = 양쪽 각각 커밋 1개 이상.** 한쪽만 커밋하면 다음 세션이 반쪽만 본다.

### 이번 세션이 만든 커밋

```
~/cobot2_ws_new  (feat/fsm-pause-hold)
  2f5585b  chore(P0): 통합 기준점 — semi_Final 이후 실기 작업 스냅샷
  7db2f7b  fix(P0): launch 의 fsm_listening_timeout_sec 기본값이 정본과 어긋나 있었다

~/M0609_VLA_system_new  (feat/mission-supervisor)
  7069265  chore(P0): 통합 기준점 — ROS_DOMAIN_ID=93 + cobot2_ws_new 오버레이
  b03ef51  fix(P1): waiting_place 가 GUI 에 영문 그대로 새던 것
  4157a15  feat(P2): MissionSupervisor — mission 상태를 두 계층이 공유하는 명시적 상태기계로
  1be7a4a  feat(P3): 작업 중 정정 — mission 을 지우지 않고 고친다
```

### 🔴 미커밋 = P4 + P5 + P6 전체 (사용자가 "커밋은 하지마" 지시)

**이 파일들을 지우거나 되돌리면 P4·P5·P6 가 통째로 사라진다.** 빌드·테스트는 이 상태에서 PASS다.

```
~/cobot2_ws_new
  M md/vla-bridge-contract.md              §14(release_now) · §15(PAUSED/stow) 신설
  M src/PACKAGES.md                        stateDiagram: PAUSED 상태 + 탈출 6줄 + note
  M src/pick_fsm/pick_fsm/states.py        State.PAUSED · PAUSE_EXEMPT · HOLDING_STATES
  M src/pick_fsm/pick_fsm/task_manager.py  _pause/_stow/_st_paused + 서비스 4개 + 종료훅
  M src/pick_fsm/config/pick_fsm.yaml      wait_place 0.0 + 손목 파라미터 예약(주석)
  M src/pick_fsm_msgs/CMakeLists.txt       AcquireTarget.srv 등록
  ?? src/pick_fsm_msgs/srv/AcquireTarget.srv   정의만 (서버 없음)
  M src/pick_fsm/test/test_pick_fsm.py     검정 10건
  M src/voice_processing/.../vla_command_node.py  release_now·pause·resume·stow 라우팅
  ?? md/plans/2026-08-12-vlm-fsm-integration-handoff.md   (이 문서)

~/M0609_VLA_system_new
  M .../bridge/pick_bridge.py              build_{release_now,pause,resume,stow}_command
                                           + FSM_HOLDING_STATES/_FSM_STATE_INFO 에 PAUSED
  M .../agent/tools.py                     pick_and_hold · release_held · MISSION_TOOLS 4개
  M .../agent/prompt.py                    집기/놓기 분기 · mission 절(11~15)
  M .../nodes/vla_pick_bridge_node.py      handle_{release_held,resume} · pause/stow 콜백
  M .../nodes/agent_node.py                pause_callback · dispatch_mission_tool · MissionHost
  M .../vla_gui.py                         멈춤/비상정지 2버튼 분리 · _stow_and_wait
  M .../test/test_tools_schema.py          검정 5건
  M .../test/test_pick_bridge.py           검정 9건
```

커밋할 때 반드시 넣을 것: `계약 §14·§15 갱신 포함` · `빌드 PASS / pytest 77·230 PASS` ·
`🔴 실기 미검증`. **Phase 하나 = 양쪽 각각 커밋 1개 이상**이므로 P4·P5·P6 를 나눠 커밋한다.

---

## 2. 🔴 계획서(`plan.md`)가 실제와 달랐던 곳 — **그대로 따르면 사고 난다**

### 2-A. §1-2 `a4-integ` 체크아웃 → **폐기**

계획서는 `~/M0609_VLA_system_new` 를 `wodud4143/M0609_VLA_system @ a4-integ` 로 checkout/clone 하라고 적었다.

- 이 저장소에 `a4-integ` 브랜치는 **없다** (로컬·원격 통틀어 0개). 그건 *다른 클론*
  (`ddoo0922/0730_cobo2_personal @ a4-integ`)의 브랜치였다.
- 머지 커밋 `a0ed3b3` 이 그 a4 계보(`1eaa155`)를 `vla_integ` 실행부(`7ec114f`) 위에 **이미 합쳤다.**
- `git log a0ed3b3..1eaa155` → **비어 있음.** a4 에만 있고 현재에 없는 커밋은 0개다.
- 반대로 현재는 a4 tip 보다 `7ec114f semifinal_commit` · `64aaac0 integ_raw` 를 더 갖는다.

**즉 `vla_integed` ⊃ `a4-integ`.** 계획의 *의도*("a4 의 규칙 계층을 확보한다")는 이미 달성됐다.
체크아웃하면 실행부 2커밋과 머지 결과를 버리고 과거로 돌아간다. **얻는 건 0이다.**

### 2-B. §4 Gap 표가 낡았다 — G1/G2/G3 은 이미 닫혀 있었다

| Gap | 계획서 주장 | 2026-08-12 실측 |
|---|---|---|
| G1 `FSM_HOLDING_STATES` 에 `WAIT_PLACE_TARGET` 누락 | P1 에서 고침 | **이미 있었다** (`pick_bridge.py:258`) |
| G2 `set_place` 빌더 없음 | P1 에서 신설 | **양쪽 다 있었다** (빌더 + `vla_command_node._handle_set_place`) |
| G3 `allow_unverified_place=false` | P1 에서 true 로 | **이미 `true`** (`system.yaml:134`) |

→ P1 에서 실제로 할 일은 **GUI 라벨 하나**뿐이었다(§3-P1). 다음에 Gap 표를 보면 **먼저 실측하고 믿어라.**

---

## 3. Phase 별 완료 내역

### P0 — 기준점 고정 ✅

- 두 ws 에 작업 브랜치 생성, 기준점 커밋
- `build/ install/ log/` **양쪽 전량 재생성**. 구 `install/` 에 구 워크스페이스 절대경로가 박혀 있었다
  (colcon 산출물엔 절대경로가 박히므로 ws 디렉토리 이름만 바꿔도 깨진다)
- `ROS_DOMAIN_ID=93` 양쪽 명시 — `container_setup.sh:61`(FSM) · `scripts/env.sh`(VLA, 이번에 추가).
  기본값 0 이면 `perception_node` 가 "no frames processed yet" 만 찍으며 매달린다(다른 증상 없음)
- `pixel_policy` 는 P0 에서 계획대로 `warn` 유지 → **P6 완료 후 `select` 로 올렸다**(§5-B)

### P1 — 계약 결선 ✅

브리지·계약은 이미 구현·검정돼 있었고(§2-B), 실제 구멍은 **GUI 라벨 하나**였다.

`fsm_state_view()` 가 `WAIT_PLACE_TARGET` 에 `waiting_place` 라는 별도 status 를 주는데
`vla_gui` 라벨 맵에 그 키가 없어 `.get(status, status)` 폴백이 걸렸다 — 나머지가 전부 한글인데
**한 줄만 영문**으로 뜨고, 그게 하필 사람이 목적지를 말해줘야 하는 순간이었다. 예외도 경고도 없다.

재발 방지: `test_every_bridge_status_has_a_korean_label` 이 `_FSM_STATE_INFO` 의 모든 status 를 훑는다.

### P2 — MissionSupervisor ✅ (이 계획의 핵심)

**신규 `src/vla_system/vla_system/agent/mission.py`** — ROS 도 RuleStore 도 모른다(`skill_tier` 와 같은 host-seam).

지금까지 mission 은 `SkillTier` 안의 사설 dict 3개(`_mission`/`_acted`/`picked`)로만 존재했다.
그래서 Tier 2 는 Tier 1 이 이미 무엇을 했는지 볼 수 없었고, 표현 가능한 정정이 "통째로 버리고
다시" 하나뿐이었다 — `handle()` 이 실제로 그렇게 했다. **"나머지는 테이블로" 가 원천적으로
불가능했던 이유가 이것이다.**

구현된 R1~R5 (전부 `test_mission.py` 23건이 지킨다):

| | 규칙 | 안 지키면 |
|---|---|---|
| R1 | in-flight 중 dispatch 금지. `accepted` 는 terminal 이 **아니다** | 팔 하나에 물체 둘 |
| R2 | Command Barrier — result 콜백에서 곧바로 다음을 안 보낸다 | 정정이 한 물체 늦게 걸린다 |
| R3 | 좌표 재사용 금지. id 는 살아도 위치는 안 산다 | 없는 자리로 내려간다 |
| R4 | SNAPSHOT 기본 — 명령 시점 집합을 고정 | 새 물건이 몰래 작업에 낀다 |
| R5 | 실패해도 mission 은 안 죽는다. retry → `failed_ids` → 끝에 정직하게 보고 | 조용한 부분 완료 |

- `SkillTier` 는 파싱·규칙 필터·되묻기만 남았다. `_candidates()` 필터는 보존하되 `_blocked()` 로
  뽑아 supervisor 가 **dispatch 시점에 다시** 본다 (작업 중에 생긴 금지가 아직 안 나간 물체에 물어야 한다)
- `SceneItem` 이 `skill_tier` → `mission` 으로 이사. `skill_tier` 가 재수출하므로 기존 import 는 그대로 산다
- `vla_interfaces`: `MissionState.msg` / `MissionCommand.msg` 신설. **VLA ws 내부 전용** — cobot2_ws
  경계(JSON 3채널)는 안 건드린다(I1)
- `agent_node` 가 `MissionHost` 구현: `dispatch_pick` / `pending_user_commands` / `on_mission_state`
  → `/vla/mission/state` publish

> ⚠️ `MissionHost.dispatch_pick` 의 이름에 `_pick` 이 붙은 이유: `AgentNode.dispatch()` 가 **이미
> LLM 툴 디스패처**다. `dispatch` 로 두면 `getattr(host,"dispatch")` 가 엉뚱한 걸 집는다. 실제로 밟았다.

**내 테스트가 잡은 진짜 버그 3개** (전부 수정 완료):
1. `_publish()` 의 `dataclasses.replace()` 가 얕은 복사 → GUI 가 supervisor 와 **같은 리스트**를 봤다
   (pending_ids 가 스스로 비어가는 화면)
2. R3 "한 번 다시 보기" 가 루프 **밖에서** 뜬 스냅샷을 재사용해 무의미했다 → 루프 안으로 옮김
3. `_after_mission` → 상시규칙 → 후보 없음 → `_after_mission` **무한재귀**
   (원본 `_end_mission` 에도 잠재해 있던 것. `_standing_active` 로 차단)

### P3 — 작업 중 정정 ✅

- Tier 2 툴 4개 (`MISSION_TOOLS`, **`MOTION_TOOLS` 와 분리**):
  `modify_mission` / `cancel_mission` / `pause_mission` / `resume_mission`
- 🔴 **분리한 이유**: 판단 중 정지가 걸리면 `MOTION_TOOLS` 는 보류되는데, **정지 뒤에 계획을
  고치는 것은 오히려 허용돼야 한다.** 같은 집합에 넣으면 "멈춰" 다음의 "나머지는 테이블로" 가
  조용히 버려진다
- `pause_mission` 은 정지 버튼이 **아니다**. "멈춰"는 GUI → 로봇 직행이고 LLM 을 안 거친다(I4).
  설명문에 못박고, 그 문구가 사라지면 깨지는 검정을 걸어뒀다(`test_pause_mission_is_not_the_stop_button`)
- `RULE_SCHEMA` 에 `destination` 슬롯 **하나만** 추가 (§6-C 경고: 필드를 늘리면 파서가 짐작으로
  채우고, 잘못 채워진 슬롯은 자신 있게 실행된다). `required` 집합을 통째로 고정하는 검정을 걸어
  슬롯이 표류로 늘지 않게 했다
- **목적지 없음 ≠ basket.** 빈 문자열이라야 FSM 이 `WAIT_PLACE_TARGET` 에 서서 사람에게 물어볼
  기회가 생긴다. basket 으로 조용히 채우면 그 기회가 사라진다
- `prompt.py` 에 mission 절(11~15)

### P4 — Hold / Release ✅ **(미커밋)**

`WAIT_PLACE_TARGET` 에서 **목적지를 정하지 않고** 끝내는 길. "집어서 들고만 있어줘" → "됐어, 거기 놔".

FSM 쪽:
- `states.py`: `WAIT_PLACE_TARGET: {PLACE, RELEASE, ABORT}`
- `task_manager._srv_release_now` + `/pick/release_now`(Trigger). **`WAIT_PLACE_TARGET` 에서만** 수락
- `vla_command_node`: `CONTROL_CMDS` 에 `release_now`, `release_now_service` 파라미터, 클라이언트
- 계약 `md/vla-bridge-contract.md` **§14 신설**, `src/PACKAGES.md` 다이어그램 엣지

VLA 쪽:
- `build_release_now_command()` (request_id 는 **원래 pick 의 것**을 재사용 — 새 사이클이 아니다)
- 툴 `pick_and_hold(object_id, say)` · `release_held(say)`, 둘 다 `MOTION_TOOLS`
- `vla_pick_bridge_node.handle_release_held()` + `pick_and_hold` 분기(`place` 를 강제로 비운다)

> ⚠️ **`place_held` 툴은 일부러 안 만들었다.** 계획서 §Phase4 는 3개를 요구했지만 `set_place`(§13)가
> 이미 그 툴이다. 같은 명령을 내는 툴이 둘이면 모델이 매 턴 동전을 던져야 한다 — 계획서 자신이
> §6-C 에서 경고하는 실패 모드다. 검정으로 못박아 뒀다(`test_there_is_exactly_one_way_to_name_a_destination`).

🔴 **`abort` 와 정반대라는 점을 잊지 마라**: abort 는 `HOLDING_STATES` 에서 그리퍼를 **일부러 안
연다**(떨어뜨리는 게 멈추는 것보다 위험). `release_now` 는 사람이 **그 자리를 보고** 놔도 된다고
판단해 부르는 것이라 반대 방향이다. 그래서 팔이 **멈춰 서 있는 상태에서만** 허용된다.

---

### P5 — USER_STOP / E_STOP 분리 ✅ **(미커밋)**

"멈춰"가 `abort → ABORT → SAFE_STOP` 하나뿐이라 **파괴적**이었다 — 복구에 `/pick/reset` +
HOME 왕복이 필요하고 "계속해"가 불가능했다. 사람이 잠깐 세우려는 것에까지 그걸 쓰니 매번
사이클을 버리게 됐고, 결국 사람이 "멈춰"를 안 쓰게 된다.

**정지의 정의 (2026-08-12 사용자 결정 — 이 두 줄이 사양이다)**
> ① "멈춰" = 즉시 멈추고, **다음 명령이 올 때까지 대기**한다.
> ② 다음 명령이 오면 **그대로 한다.** 재개를 위해 두 번 말하게 하지 않는다.

FSM 쪽:
- `states.py`: `State.PAUSED`. **진입 전이를 손으로 안 적는다** — `PAUSE_EXEMPT`
  (`IDLE`·`SPEAK_FAIL`·`ABORT`·`SAFE_STOP`)의 여집합에 자동 주입한다. 17줄을 표에 흩뿌리면
  표가 안 읽히고, 더 나쁘게는 **새 상태를 만든 사람이 pause 를 빠뜨려 거기서만 안 멈추는**
  구멍이 생긴다. `* → ABORT` 를 다이어그램에서 note 로 묶은 것과 같은 취급이다
- `HOLDING_STATES` 에 `PAUSED` 포함 — 물체를 든 채로도 멈출 수 있고, 그때 그리퍼를 놓으면
  일시정지가 아니라 낙하다. 이 집합의 뜻은 "확실히 물고 있다"가 아니라 **"물고 있을 수
  있으니 건드리지 마라"**다
- `DEFAULT_TIMEOUTS` 에 **넣지 않았다.** 넣는 순간 ①이 죽는다(`test_PAUSED_에는_제한시간이_없다`)
- `task_manager`: `_pause()`(goal 취소 + `_paused_from` 기록 + 전이) · `_st_paused()`(**아무것도
  안 한다**) · `/pick/pause`·`/pick/resume`·`/pick/stow` 서비스
- `_tick` 우선순위: **abort → pause → 제한시간**. 순서가 바뀌면 "제한시간이 거의 다 된
  상태에서 멈춰"가 pause 대신 abort 로 떨어진다
- `resume` 복귀: 보유+목적지 O → `PLACE` 재계획 · 보유+목적지 X → `WAIT_PLACE_TARGET` ·
  비보유 → `PERCEIVE`. **보유 중엔 재인식하지 않는다**(그리퍼가 자기 물체를 오인식한다).
  어느 쪽이든 `solutions` 를 비워 **옛 trajectory 를 재사용하지 않는다**
- `release_now` 가 `PAUSED` 에서도 먹는다 — 두 출발지의 공통점이 안전의 근거다(팔이 멈춰 서 있다)

🔴 **`wait_place_timeout_sec` 자동 내려놓기 제거** (2026-08-11 결정을 뒤집는다):
`PARAM_DEFAULTS` `60.0` → **`0.0`**, `pick_fsm.yaml` `100.0` → **`0.0`**.
코드는 `timeout > 0.0` 게이트로 남겨 **yaml 로 되돌릴 수 있게** 했다
(`grip_narrow_retries` 관례). 이게 "시간 경과만으로 팔이 움직이는" 마지막 경로였다(I12).

🔴 **`/pick/stow` — 종료 정리. 순서가 요청과 반대인 것이 요점이다.**
`보유 → (먼저 멈춤) → PLACE → RELEASE → HOME → IDLE` / `비보유 → 그리퍼 open → HOME → IDLE`.
"그리퍼를 열고 홈 복귀"를 글자대로 하면 물체를 **지금 있는 자리**에 떨어뜨린다.
`main()` 의 종료 훅은 **즉시 끝나는 것만** 한다: goal 취소 + 경고 로그. **그리퍼는 안 건드린다** —
문 채 죽으면 RG2 는 전원이 있는 한 계속 물고 있고, 그게 떨어뜨리는 것보다 안전하다.

VLA 쪽:
- `vla_gui`: **버튼 2개로 분리.** `⏸ 멈춰`(주황, `/vla/robot/pause`) vs `🔴 비상정지 (ESC)`
  (빨강, `/vla/estop`). **정지 키워드는 이제 pause 로 간다** — 넓게 잡아도 되는 이유가
  거기 있다("계속해"로 되돌아온다)
- `vla_gui.stop_pipeline()` → `_stow_and_wait()` 를 **먼저** 부르고 IDLE 확인 후 프로세스 종료.
  "닫으면 정리된다"는 종료 훅이 아니라 **버튼**이 지킨다
- `pick_bridge`: `build_{pause,resume,stow}_command`. `FSM_HOLDING_STATES`/`_FSM_STATE_INFO` 에
  `PAUSED` 추가 — status 는 `"paused"`이고 **`"error"` 가 아니다**(빨간 상태로 보이면
  사용자가 리셋을 누르고, 그게 파괴적 경로다)
- `agent_node.pause_callback`: `stop_epoch += 1`(정지 이전에 결정된 action 차단) +
  **`supervisor.pause()`** — 안 하면 FSM 은 섰는데 supervisor 가 다음 물체를 내보낸다
- `resume_mission` 툴이 **FSM 까지 푼다**(`publish_action("resume", …)` → `/pick/resume`).
  supervisor 만 재개하면 얼어 있는 팔에 새 pick 을 보내게 된다

검정 12건 추가 (FSM 7 · VLA 5). 특히:
`test_PAUSED_에는_제한시간이_없다` · `test_시간_경과만으로_팔이_움직이는_경로는_기본값에_없다` ·
`test_멈출_게_있는_모든_상태에서_멈출_수_있다` · `test_paused_is_its_own_status_and_not_an_error`

**🔴 P5 는 실기로만 확인되는 것이 많다** (계획서 §6 T7~T9):
정지 후 **5분 방치** → 팔 그대로 · `WAIT_PLACE_TARGET` 5분 방치 → 자동 basket 미발동 ·
보유 중 강제 `Ctrl-C` → 그리퍼 유지 · `APPROACH` 중 "멈춰" → **LLM 호출 로그 0건**.

---

### P6 — 손목 RealSense **Seam** ✅ **(미커밋, 구현은 안 함)**

**구현하지 않는 이유가 "시간이 없어서"가 아니다** — 완벽히 구현해도 **그 코드에 도달하지
못한다.** 실측:

- `_st_plan` docstring: `"""pre-grasp → grasp → lift 3점 IK. 하나라도 실패하면 다음 후보로 간다."""`
- 3점 **전부** 성공해야 `WAIT_APPROVAL` → … → `APPROACH` → `REGRASP`
- `PERCEIVE` 에서 grasp 못 얻으면 `SPEAK_FAIL`

병목은 `REGRASP` 가 아니라 **`PERCEIVE→PLAN` 상단 의존성**이다. 그래서 자리만 팠다.

**넣은 것 넷 (전부 동작 불변, 지금 아무것도 바꾸지 않는다):**

1. **grasp 3층** — `global_grasp`(Top 원본) · `committed_grasp`(**모션이 실제로 쓰는 것**) ·
   `grasp_revision`. 지금은 셋이 항상 같다. 모든 하강·계획이 `committed_grasp` 만 읽는다.
   바꾸는 문은 `_commit_grasp(pose, source=…)` **하나뿐**이고, 검정이 대입 개수를 2개로
   고정한다(`test_하강_자세를_바꾸는_문은_하나다`).
   → 손목이 붙으면 `source='WRIST'` 로 부르는 것이 변경의 전부다.
2. **`_invalidate_final_solutions()`** — `grasp`/`lift` IK 해만 버린다(`pre_grasp` 는 남긴다).
   🔴 **지금은 호출처가 없다.** 현재 `_commit_grasp` 호출 두 곳은 IK 를 풀기 전이라 지울 게
   없다. `APPROACH` **뒤에** grasp 가 바뀌기 시작하면 그때 필요하다 —
   **빠뜨리면 조용히 틀린다**(새 자세는 로그에만, 로봇은 옛 IK 해로 옛 자리로).
3. **`pick_fsm.yaml` 손목 파라미터 예약** — `wrist_camera_ns` · `wrist_grasp_service` ·
   `observation_offset_m` · `max_active_views`. **전부 주석**이고, 주석에서 풀리면 검정이
   깨진다(`test_손목_파라미터는_예약만_되어_있다`). `regrasp_enabled` 기본 `false` 유지(I9).
4. **`pick_fsm_msgs/srv/AcquireTarget.srv`** — 정의 + `CMakeLists` 등록만. **서버 없음.**
   목적: `ComputeGrasp` 가 *"물체를 찾았나"* 와 *"어떻게 잡을지 알아냈나"* 를 한 `success` 에
   묶어 놔서 **"물체는 보이는데 grasp 를 못 찾았다"를 표현할 수단이 없고**, 그래서 fallback
   자체가 불가능하다. 손목 카메라의 존재 이유가 바로 그 경우인데 거기까지 갈 길이 없다.

`_st_regrasp` 의 HOOK 주석에 **채울 세 줄을 그대로 적어 뒀다**(commit → invalidate → PLAN).
손목이 붙을 때 고칠 곳 7군데는 계획서 §7-3 표.

---

## 4. 남은 일


### P7 — PLACE_REDIRECT (선택, 맨 마지막)

물체를 든 채 `PLACE` 이동 중에 목적지 변경. P1~P5 가 **실기로 안정된 다음**에만.
지금은 `modify_mission(CURRENT_AND_REMAINING)` 이 `WAIT_PLACE_TARGET` 일 때만 `set_place` 로
따라잡고, 이동 중이면 "이미 나간 물체는 그대로 갑니다"라고 **정직하게 말한다**.

---

## 5. ⏳ 사용자 결정 대기 — 손대지 말 것

### 5-A. 클래스 허용목록 (계획 G4) — **가장 먼저 물어볼 것**

> ⚠️ 계획서 §8 Do-not: **임의로 통일하지 않는다.** 아래는 실측 대조표일 뿐 조치는 안 했다.

| 구분 | 클래스 |
|---|---|
| FSM `config/objects.yaml:detect` (7) | `bottle, cup, spoon, banana, apple, orange, mouse` |
| VLA `config/system.yaml:target_classes` (15) | `apple, banana, orange, cup, bottle, wine glass, book, mouse, cell phone, remote, sports ball, teddy bear, clock, scissors, knife` |
| **교집합 = 지금 실제로 통과하는 것 (6)** | `apple, banana, orange, cup, bottle, mouse` |
| FSM 에만 (1) | `spoon` — VLA 가 지시할 수단이 없다 |
| VLA 에만 (9) — **즉시 거부됨** | `wine glass, book, cell phone, remote, sports ball, teddy bear, clock, scissors, knife` |

🔴 **가장 나쁜 UX**: `scissors`/`knife` 는 VLA 의 `HAZARD_CLASSES` 라
*"가위는 위험한 물건인데 가져올까요?"* → *"응"* → **그리고 브리지가 거부**한다.
사용자에게 확인까지 받아놓고 못 하는 흐름이다.

선택지: **(a)** FSM `detect` 에 추가 (COCO 80종이라 가능하나 오인식 여지 증가) / **(b)** VLA
`target_classes` 를 교집합 6개로 축소.

### 5-B. `pixel_policy` → ✅ **`select` 로 올렸다 (2026-08-12)**

계획 §0-4 가 `warn` 을 **Phase 0 한정 임시값**으로 뒀고("Phase 2 통과 후 별도 검증"),
통합 테스트 T3·T11 이 `select` 를 전제한다. P0~P6 이 끝났으므로 올렸다.

`warn` 이면 VLA 가 "이거"라고 지목해 pixel 을 실어 보내도 **FSM 이 그 값을 버리고**
클래스만으로 고른다 — 사과가 하나면 우연히 맞고, 둘이면 지목과 무관하게 점수 높은 쪽을
집는다. 증상이 "가끔 엉뚱한 걸 집는다"로만 나와서 원인을 못 찾는 종류다.

경로는 이미 전부 연결돼 있었다(막고 있던 건 이 파라미터 하나):
```
VLA bbox_center() → pixel/pixel_wh(JSON) → vla_command_node → /pick/target_pixel
  → task_manager._on_target_pixel → grasp_bridge 파라미터 pixel_x/y/w/h
  → grasp_bridge_node:425  pixel 로 개체 선정
```

- 실패 방향이 안전하다: 후보가 `match_tolerance_m`(0.06m) 밖이거나 2등과
  `ambiguity_margin_m`(0.02m) 안으로 붙어 모호하면 **grasp_bridge 가 호출을 실패시킨다**
- pixel 이 없으면 no-op — 클래스만 보내는 기존 지시는 그대로 동작한다
  (`parse_command` 의 정책 분기가 전부 `if 'pixel' in doc:` 안에 있다)
- 검정 2건: `test_launch_default_is_select_so_the_pixel_is_not_thrown_away` ·
  `test_select_without_a_pixel_is_a_no_op`

🔴 **실기 미검증** — 같은 클래스 2개로 T11 을 통과시켜야 한다.

⚠️ **손가락 포인팅은 인식하지 않는다.** perception 에 손/포인팅 추정 코드가 없고
`person` 은 `excluded_classes` 다. 지목은 **Tier 2 가 라벨 그려진 이미지를 보고 object_id 를
고르는 것**이고, 카메라에 찍힌 손가락을 모델이 읽어낼지는 검증된 바 없다.

### 5-C. P5 의 "정지 중 목적지만 수정" 처리 (계획서 §5 3-B)

"나머지는 테이블로"는 *무엇을* 바꾸라는 말이지 *움직이라*는 말이 아니다 → **revision 만 올리고
`PAUSED` 유지 + "테이블로 바꿨습니다. 계속할까요?" 되묻기**가 현재 사양.
반대 정책(수정 즉시 자동 재개)을 원하면 **이 한 곳만** 바꾸면 된다.

---

## 6. 이 세션이 밟은 함정 3개 — 다음 사람도 밟는다

### 6-A. `fsm_listening_timeout_sec` 이 세 곳에 복제돼 있고 하나가 어긋나 있었다

`vla_command.launch.py` 는 `60.0`, 정본 `task_manager.DEFAULT_TIMEOUTS[State.LISTENING]` 은 `110.0`.
노드 `declare_parameter` 는 맞아 있어서 **launch 로 띄울 때만** 어긋났다 — 단독 노드 실행으로는
재현이 안 되고 실기 경로에서만 나온다. 고쳤다(`7db2f7b`).

```
src/pick_fsm/pick_fsm/task_manager.py:56                    ← 정본
src/voice_processing/voice_processing/vla_command_node.py:376
src/voice_processing/launch/vla_command.launch.py:94
```
`test_{node,launch}_default_matches_fsm_listening_timeout` 두 건이 이 복제를 감시한다. **지우지 마라.**

### 6-B. 🔴 `M0609_VLA_system_new` 가 **구 워크스페이스의 venv 를 쓰고 있었다**

`_new/.venv/bin` 의 27개 파일이 `VIRTUAL_ENV=/home/kimkh/M0609_VLA_system/.venv`(구 ws)를 가리켰다.
증상이 없다 — 구 ws 가 아직 존재해서 그냥 동작한다. 하지만 구 ws 를 지우면 죽고, `pip install` 이
엉뚱한 데 들어간다.

패키지 125개는 `_new` 에 **실재**했으므로 경로 문자열만 고쳤다(재설치 없음):
```bash
grep -rl "/home/kimkh/M0609_VLA_system/" .venv/bin/ | xargs sed -i \
  's#/home/kimkh/M0609_VLA_system/#/home/kimkh/M0609_VLA_system_new/#g'
# 뒤의 '/' 가 `_new/` 를 매치에서 제외한다 (이중 치환 방지)
```
확인: `python3 -c "import openai; print(openai.__file__)"` 가 `_new` 를 가리켜야 한다.

**같은 부류를 하나 더 의심하라**: colcon 산출물엔 절대경로가 박힌다. `build/ install/ log/` 는
ws 를 복사·개명하면 **무조건 재생성**한다.

### 6-C. 🔴 `M0609_VLA_system_new` 는 `scripts/build.sh` 로만 빌드한다

맨손 `colcon build` 를 돌렸다가 두 가지가 동시에 터졌다(그 스크립트 주석에 이유가 적혀 있다):

1. `--symlink-install` 이 없으면 `build/` 가 **사본**이라 테스트가 **옛 코드**를 읽는다
   (수정했는데 테스트 결과가 안 바뀌는 증상 → 나를 한참 헤매게 했다)
2. `/usr/bin/colcon` 은 `/usr/bin/python3` 셰뱅을 박는다 → 모든 노드가 런타임에
   `ModuleNotFoundError: torch/openai` 로 죽는다 (2026-08-10 재현 기록)

확인법: `head -1 install/vla_system/lib/vla_system/agent_node` 가 `.venv` 를 가리켜야 한다.

> 참고: pytest 트레이스백이 `../M0609_VLA_system/src/...`(구 경로)로 찍히는 경우가 있는데
> **표시 아티팩트다.** 프로브로 확인했다 — 실제 실행 파일·모듈은 전부 `_new` 다.
> 놀라서 되돌리지 마라. 확인하려면 테스트 안에서 `__file__` 을 찍어보면 된다.

---

## 7. 절대 깨면 안 되는 것 (계획서 §3 불변식 — 구현 상태)

| | 불변식 | 지금 |
|---|---|---|
| I1 | 경계는 JSON 3채널뿐. `vla_interfaces` msg 를 cobot2_ws 로 안 넘긴다 | ✅ `MissionState`/`MissionCommand` 는 VLA 내부 전용 |
| I2 | VLM 은 `/pick/approve` 를 부르지 않는다 | ✅ `BLOCKED_CMDS` — 코드 경로 자체가 없다 |
| I3 | VLM 은 좌표를 안 보낸다 (`class` + 선택 `pixel`) | ✅ |
| I4 | 정지 경로에 LLM 이 끼지 않는다 | ✅ + `test_pause_mission_is_not_the_stop_button` |
| I5 | 물체 보정 중 자동 그리퍼 개방 금지 | ✅ `HOLDING_STATES`. `release_now` 는 **사람 명령**이라 예외가 아니다 |
| I6 | 동시 in-flight action 1개 | ✅ R1 + `pending_action` 게이트 |
| I7 | 상태 전이는 `states.py` 한 곳 | ✅ P4 도 여기만 고쳤다 |
| I8 | 계약 문서 사본 금지 — VLA 는 절대경로로 읽는다 | ✅ §14 도 cobot2_ws 정본에만 |
| I9 | 손목 카메라 코드 미구현, `regrasp_enabled=false` | ✅ P6 는 **자리만**. 파라미터는 주석, `AcquireTarget` 은 서버 없음, `regrasp_enabled` false. 검정 3건이 고정 |
| I10 | FSM 에 semantic 판단 금지 | ✅ |
| I11 | `PAUSED` 에서 자율 동작 금지 | ✅ 양쪽 다. FSM `State.PAUSED`(제한시간 없음) + supervisor. `pause_callback` 이 **둘 다** 멈춘다 |
| I12 | 시간 경과만으로 팔이 움직이는 경로 0개 | ✅ **기본값 기준.** `wait_place_timeout_sec` 0.0(끔)이 기본. yaml 에 양수를 넣으면 되돌아오므로 검정이 **기본값**을 지킨다 |
| I13 | 그리퍼는 팔이 **멈춰 선** 상태에서만 열린다 | ✅ 출발지 정확히 3곳: `PLACE`(정상 완료) · `WAIT_PLACE_TARGET` · `PAUSED`. 뒤 둘은 사람 명령. `test_RELEASE_로_가는_길은_팔이_멈춰_선_상태에서만_열린다` 가 집합을 고정한다. 종료 훅도 그리퍼를 안 건드린다 |

---

## 8. 관련 문서

- `md/plans/2026-08-12-vlm-fsm-integration.md` — P0 시점 진행 문서(커밋됨). 이 문서와 겹치면 **이쪽이 최신**
- `md/vla-bridge-contract.md` — **경계 계약 정본.** §13(WAIT_PLACE_TARGET) · §14(release_now, 미커밋).
  VLA 쪽은 절대경로로 이 파일을 읽는다. 사본 만들지 말 것
- `src/PACKAGES.md#pick_fsm` — FSM stateDiagram 정본. 전이표와 그림이 어긋나면 테스트가 깨진다
- `md/context/constraints.md` — 실기 실측 사실 단일 출처
- `~/M0609_VLA_system_new/src/vla_system/vla_system/agent/mission.py` — R1~R5 의 근거가 모듈 docstring 에 있다
