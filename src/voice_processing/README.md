# voice_processing

*`pick_fsm` 의 **지시 입력 층**. 사람 음성이든 외부 VLA 든, 결국 `/get_keyword`
(`std_srvs/Trigger`) 하나로 FSM 에 들어간다.*

> 🚨 여기서 지시를 보내면 **실기가 실제로 움직인다.** `vla_command_node`·`get_keyword` 둘 다
> `/pick/approve` 를 **부르지 않는다** — 승인은 언제나 사람이 누른다(rqt 버튼 또는
> `approve_listener_node`, 아래). 최종 안전장치는 물리 비상정지 버튼이다.

| 노드 | 입력 | 추가 의존성 |
|---|---|---|
| `vla_command_node` | `/vla/pick_command` (외부 PC 의 VLA, JSON) | **없음** — 표준 ROS 2 만 |
| `get_keyword` | 마이크 → wakeword → Whisper STT → LLM | `openai` `langchain-openai` `python-dotenv` `pyaudio` `openwakeword` `sounddevice` + `resource/.env` |
| `approve_listener_node` | 마이크 → wakeword → Whisper STT → 문구매칭 → `/pick/approve` | `openai` `pyaudio` `openwakeword` + `resource/.env` (`get_keyword` 와 동일) |

⚠️ **`get_keyword`·`approve_listener_node` 둘 다 마이크·웨이크워드를 쓴다** — 다만 서로
다른 FSM 상태(`LISTENING` vs `WAIT_APPROVAL`)에서만 활성화되므로 정상 경로에서는 겹치지
않는다. `get_keyword`·`vla_command_node`(둘 다 `/get_keyword` 제공)는 **동시에 띄우지 않는다.**

### 실행

```bash
ros2 run voice_processing get_keyword              # 마이크 경로 — launch 파일 없음
ros2 launch voice_processing vla_command.launch.py  # VLA 경로
ros2 run voice_processing approve_listener_node     # 음성 승인 — launch 파일 없음, 위 둘과 병행 가능
```

### 음성으로 그립 승인 (`approve_listener_node`)

`WAIT_APPROVAL` 상태일 때만 듣는다("hello 로키" 웨이크워드 → 승인 문구 인식되면
`/pick/approve` 호출). rqt 패널의 '승인' 버튼과 **완전히 같은 서비스를 부른다** — 이 노드는
그 옆에 음성 경로를 하나 더 놓는 것뿐이다. VLA 의 `cmd:"approve"` 차단(`BLOCKED_CMDS`)과는
무관 — 이 노드가 듣는 건 로봇 앞의 **사람**이지 외부 PC 가 아니다.

```bash
ros2 param set /approve_listener_node approve_phrases "승인,그립해,오케이 진행"
```

기본 승인 문구는 `승인,그립해,그립,진행해,진행,컨펌` — 일상 대화에 흔한 "네"/"응"류는
일부러 뺐다(실기 오작동 방지). 마이크 경로는 아직 실기 미검증.

`get_keyword` 는 서비스 호출이 오디오 스트림을 열고 웨이크워드("hello_rokey")가 뜰 때까지
블로킹한다. 뽑힌 키워드는 노드 터미널의 `Detected tools: [...]` 로그로 보거나, 단독 트리거로
직접 확인한다:

```bash
ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
```

응답 `message` 가 추출된 물체명(공백 join) — `task_manager` 는 이 중 **첫 단어만** target 으로 쓴다.
마이크 경로는 아직 실기 미검증.

레퍼런스(JSON 스키마·파라미터·결과 계약·검증 상태)는
**[`src/PACKAGES.md`](../PACKAGES.md#voice_processing)**.
설계 배경(역할 경계·대역폭·좌표계)은 **[`md/plans/2026-08-08-vla-integration.md`](../../md/plans/2026-08-08-vla-integration.md)**.
