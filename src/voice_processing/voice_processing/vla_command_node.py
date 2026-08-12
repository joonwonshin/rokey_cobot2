#!/usr/bin/env python3
r"""VLA 지시(JSON)를 `pick_fsm` 의 타겟으로 옮긴다.

외부 PC 의 VLA 가 "무엇을", 우리가 "어떻게"를 소유한다.
설계 출처: `md/plans/2026-08-08-vla-integration.md` §2(지시 채널) · §0(역할 경계) · §0-B(승인)

    ros2 launch voice_processing vla_command.launch.py
    ros2 topic pub -1 /vla/pick_command std_msgs/String \
        "data: '{\"cmd\":\"pick\",\"class\":\"apple\",\"request_id\":\"a17-3\"}'"

## 이 노드가 `voice_processing` 안에 있는 이유

`task_manager` 는 이미 **음성 노드 자리**를 갖고 있다 — `LISTENING` 상태에서
`/get_keyword`(`std_srvs/Trigger`)를 부르고 응답 `message` 의 **첫 단어**를 타겟으로 쓴다
(`task_manager.py` `_st_listening`). VLA 는 "사람 대신 말해주는 클라이언트"이므로
**그 자리에 그대로 꽂으면 `pick_fsm` 을 안 고쳐도 된다.** 새 상태도 새 msg 도 없다.

    VLA PC --/vla/pick_command(JSON)--> [이 노드] --/get_keyword(Trigger)--> task_manager
                                            |                                    |
                                            +----/vla/pick_result(JSON)<--/pick/state

## 이 노드는 push, FSM 은 pull 이다 — 어긋나는 곳이 전부 여기서 나온다

VLA 는 아무 때나 쏘고(push), FSM 은 `LISTENING` 에 들어와야 물어본다(pull). 그래서 이
노드는 **한 건짜리 래치**다: 지시를 붙잡고 있다가 FSM 이 물을 때 건넨다. 래치의 수명
(TTL)·"FSM 이 아직 듣고 있나"·"이 사이클이 언제 끝났나" 세 가지를 코드가 명시적으로
다루지 않으면 조용히 어긋난다. 아래 세 곳이 그 방어다:

  1. `_srv_keyword` 는 FSM 이 `LISTENING` 을 떠나면 **지시를 소비하지 않고 물러난다.**
     `task_manager._to()` 는 전이할 때 진행 중 future 를 버리므로(`_fut = None`), 버려진
     호출이 다음 지시를 가로채면 그 지시는 영영 결과가 안 나온다.
  2. 대기 마감은 `wait_timeout_sec` 과 **FSM 의 `LISTENING` 제한시간**(110 s) 중 이른 쪽이다.
     `/pick/state` 로 LISTENING 진입 시각을 알고 있으므로 남은 예산을 계산할 수 있다.
  3. `PERCEIVE` 인데 우리 지시가 안 팔렸으면 **FSM 이 다른 타겟으로 도는 중**이다
     (`pick_fsm` 을 `voice:=false` 로 띄운 경우). 조용히 두면 엉뚱한 물체를 집는다.

## rqt 패널 버튼도 같은 채널로 연다 — 단 `승인`은 뺀다

rqt 패널(`pick_fsm.rqt_panel`)의 시작/중단/리셋 버튼은 `/pick/start`·`/pick/abort`·
`/pick/reset`(전부 `std_srvs/Trigger`)을 그대로 부른다. 이 노드도 `cmd` 값으로 같은
서비스를 부르므로 음성/VLA 로 그 버튼들을 대신할 수 있다:

    {"cmd": "start"}
    {"cmd": "abort", "reason": "취소"}
    {"cmd": "reset"}
    {"cmd": "home"}

⚠️ `reset` 과 `home` 은 먹히는 상태가 다르다: `reset`(`/pick/reset`)은 **SAFE_STOP 에서만**
(안전정지 복구), `home`(`/pick/home`)은 **IDLE 에서만**(정상 대기 중 홈 복귀). 둘 다 성공하면
승인 없이 HOME 관절자세까지 실제로 움직인다 — 진행 중 사이클 도중에는 둘 다 거부한다.

## `place` 를 나중에 정하기 — `pick`(place 생략) → `WAIT_PLACE_TARGET` → `set_place`

`cmd:"pick"` 에 `place` 를 안 넣으면(그리고 `wait_place_when_omitted=true`, 기본), FSM 은
물체를 든 뒤 **자동으로 basket 에 놓지 않고** `WAIT_PLACE_TARGET` 에서 멈춰 목적지를
기다린다. 그때 목적지를 정해 보낸다:

    {"cmd": "pick", "class": "orange"}              # place 없음 → 들고 대기
    {"cmd": "set_place", "place": "table"}          # 대기 중 목적지 지정 → 내려놓기 진행
    {"cmd": "release_now"}                          # 대기 중 "그냥 거기 놔" → 이동 없이 RELEASE

`release_now`(`/pick/release_now`) 는 **이동하지 않고 지금 자리에서** 그리퍼를 연다 —
재촬영·재계획·IK 가 없다. `set_place` 와 마찬가지로 `WAIT_PLACE_TARGET` 에서만 먹는다.
이게 없으면 "집어서 들고만 있어줘" 다음의 "됐어, 놔"를 표현할 말이 없어서 탈출구가
basket 으로 보내기 아니면 `abort` 뿐이었다.
⚠️ `abort` 와 정반대다: abort 는 `HOLDING_STATES` 에서 그리퍼를 **일부러 안 연다**
(떨어뜨리는 것보다 물고 멈추는 게 안전하다). `release_now` 는 사람이 지금 그 자리를
**보고** 놔도 된다고 판단해 부르는 것이라 반대 방향이다.

`set_place` 는 **`WAIT_PLACE_TARGET` 에서만** 먹는다(아니면 거부).
🔴 **사람이 목적지를 안 정하면 그냥 계속 기다린다** — `wait_place_timeout_sec` 자동
내려놓기는 2026-08-12 사용자 결정으로 **기본 꺼졌다**(`0.0`). 시간이 지나서 팔이 저절로
움직이는 경로는 두지 않는다(계약 §13·§15). 옛 동작이 필요하면 yaml 에 양수를 넣는다.
`wait_place_when_omitted=false` 로 두면 이 경로가 꺼지고 예전처럼 place 생략 = basket
즉시 놓기가 된다.

## ✋ 멈춰 / 계속해 / 정리하고 끝내 — `pause` · `resume` · `stow` (2026-08-12 신설)

    {"cmd": "pause"}     # 즉시 멈춘다. 다음 명령이 올 때까지 대기
    {"cmd": "resume"}    # 하던 일을 잇는다 (멈춘 지점 기준으로 복귀)
    {"cmd": "stow"}      # 놓을 자리로 간 뒤 놓고 홈 복귀 — 종료 전 정리

`pause` 는 **어떤 상태에서든 성공으로 받는다.** 거부하면 안 되는 유일한 명령이라 멱등하게
뒀다 — "멈춰"에 실패가 돌아오면 사람이 다시 말하게 되고 그 사이 뭔가 시작될 수 있다.

⚠️ `abort` 와 다르다. `abort` 는 `SAFE_STOP` 까지 떨어져 `/pick/reset`+HOME 왕복이 필요한
**파괴적** 정지다. `pause` 는 `PAUSED` 에 머물고 "계속해" 한 마디로 잇는다. 진짜 위험이면
`abort`(또는 하드웨어 비상정지)를 쓰고, 사람이 잠깐 세우는 것이면 `pause` 다.

🔴 `PAUSED` 에서는 **시간이 지나도 아무 일도 일어나지 않는다** — 자동 재개·자동 내려놓기·
자동 홈·타임아웃 중단이 전부 없다. 나가는 길은 사람의 다음 명령뿐이다(계약 §15).

`stow` 는 종료 정리다. 물체를 들고 있으면 **놓을 자리로 먼저 가서** 놓고 홈으로 간다 —
"그리퍼를 열고 홈 복귀"를 글자 그대로 하면 지금 있는 자리에 떨어뜨리기 때문이다.
`SAFE_STOP` 에서만 거부한다(그쪽은 `reset` 이 정본 경로).

## 🔴 `/pick/approve`(승인 버튼)는 절대 부르지 않는다

`dry_run` 이 제거된 뒤 남은 소프트 안전장치는 `require_approval` 하나뿐이다. VLA/음성이
승인까지 보내면 안전장치가 0 이 된다(계획 §0-B). 그래서 승인 서비스는 **파라미터로도 열지
않았고**, `cmd: "approve"` 가 오면 **무조건 거부**한다(코드 경로 자체가 없다 — 값을 True 로
바꿔도 열리는 스위치가 아니라, 그런 스위치를 아예 안 만들었다). `auto_start`/`cmd:"start"`
는 `/pick/start`(작업 시작)까지고, 그 뒤 `WAIT_APPROVAL` 에서 실제로 팔이 움직이기 전에는
**사람이 rqt 패널이나 서비스로 직접 눌러야** 한다.

## ⚠️ `/get_keyword` 는 마이크 노드(`get_keyword`)도 제공한다

둘을 동시에 띄우면 어느 쪽이 응답할지 알 수 없다. **둘 중 하나만 띄운다.**
섞어 쓰려면 한쪽의 `keyword_service` 를 다른 이름으로 바꾼다.

## 지금 구현된 것 / 안 된 것

`class`(클래스 이름)는 끝까지 동작한다. `pixel`(어느 개체인가)도 **2026-08-11부터
`pixel_policy=select` 에서 선정에 쓰인다** — `grasp_bridge_node.select_by_point()`(계획
§5)가 받는다. 경로: 이 노드가 `/pick/target_pixel`(JSON)로 publish → `task_manager`가
받아서 PERCEIVE 진입 때 `grasp_bridge_node`에 `pixel_x/y/w/h` 파라미터로 밀어 넣는다
(`target_classes`와 같은 자리). `base_xy` 는 여전히 **검증만 하고 선정에 못 쓴다**
(계획 §5는 base XY 를 폴백 경로로만 남긴다 — VLA가 자기 카메라로 볼 때 쓰는 경로라
이 ws 카메라 좌표와 바로 안 맞는다).

`pixel_policy` 세 값의 뜻:
  - `warn`(기본): pixel 이 와도 무시하고 클래스만으로 진행, `ignored:["pixel"]` 로 회신
  - `reject`: pixel 이 오면 거부(개체가 모호할 위험을 아예 안 지겠다는 선택)
  - `select`: pixel 을 실제로 써서 개체를 고른다. 같은 class 후보가 씬에 없거나
    `match_tolerance_m`(브리지 파라미터, 기본 0.06m) 밖이면, 또는 2등 후보와
    `ambiguity_margin_m`(기본 0.02m) 안으로 붙어 모호하면 **grasp_bridge_node 가 그
    호출 자체를 실패시킨다** — 틀린 물체를 집는 것보다 안전하다.
🔴 **실기 미검증** — 코드·빌드·순수함수 테스트까지만 됐다. 실기로 PERCEIVE 한 사이클
전체(픽셀 지정 → 개체 선정 → grasp)를 관통시켜 본 적은 아직 없다.

`place` 는 2026-08-09 `pick_fsm.task_manager` 가 `PLACE_LOCATIONS`(basket/table/discard
세 고정 관절자세)을 갖게 되면서 **여기서도 받는다.** 좌표가 아니라 그 세 이름 중 하나만
받는다 — 이 값은 그대로 `/pick/place_location` 토픽으로 넘어간다(`_publish_place`,
`task_manager._on_place_location` 과 같은 계약). `basket`/`table`/`discard` 가 아닌 값은
거부한다. `table`/`discard` 관절값은 2026-08-11 실기 teach 완료됐다
(`pick_fsm/config/pick_fsm.yaml` 참고) — 셋 다 그대로 쓸 수 있다. 🔴 단 관절값만
교시됐고 pick_fsm 통합 사이클은 아직 미검증이다. VLA 쪽 `allow_unverified_place`
게이트는 그쪽 세션에서 풀어야 한다(cobot2_ws 는 세 값 다 이미 받는다).

**미구현 필드를 조용히 무시하지 않는다.** `pixel_policy` 가 그 처리를 정한다:
`warn` 이면 클래스만으로 진행하되 `/vla/pick_result` 의 `ignored` 로 되돌려주고, `reject` 면
거부한다. 같은 클래스 물체가 2개 이상 놓이는 순간 `warn` 은 확률적으로 다른 개체를 집으므로
**그때는 `reject` 로 바꾼다** — 계획 §5 `refuse_ambiguous_match` 와 같은 판단.

## TTL 은 받은 시각 기준이다 (`stamp_ns` 를 쓰지 않는다)

지시는 **다른 PC** 에서, 휴대폰 핫스팟을 건너서 온다(계획 §3-1). 두 PC 의 시계는 맞춰져
있지 않으므로 송신측 `stamp_ns` 로 나이를 재면 시계 오차가 그대로 TTL 오차가 된다.
`stamp_ns` 는 결과에 **에코만** 하고, 만료 판정은 이 노드가 받은 시각으로 한다.
"""

import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

#: 지시 JSON 과 결과 JSON 은 유실되면 안 된다 — 영상과 달리 초당 한 건도 안 되는 트래픽이다.
#: `/pick/state` 도 같은 프로파일이라야 `task_manager` 의 발행자(기본 depth 10)와 매칭된다.
#: ⚠️ VOLATILE 이라 **늦게 붙은 구독자는 그 전 메시지를 못 받는다.** 핫스팟이 끊겼다 붙은
#:    VLA 는 그 사이 결과를 놓치므로, 판정은 `request_id` 로 대조해야 한다.
COMMAND_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

#: `task_manager.TARGET_QOS` 와 **글자 그대로 같아야 한다** — `/pick/place_location` 구독자가
#: TRANSIENT_LOCAL 을 요구하므로(늦게 붙어도 마지막 값을 받는다), 발행자가 이보다 얕은
#: durability(기본 VOLATILE)면 아예 매칭이 안 된다(2026-08-08 place_location 도입 당시와
#: 같은 이유 — `task_manager.py` 상단 주석 참고).
PLACE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: FSM 이 실패로 끝났다고 볼 상태. 판정은 `/pick/state` 하나로만 한다 — FSM 에 결과 토픽을
#: 새로 만들면 그쪽 코드를 건드려야 하고, 그건 이 통합의 전제("FSM 보존")를 깬다.
FAIL_STATES = frozenset({'SPEAK_FAIL', 'ABORT', 'SAFE_STOP'})

#: 성공 판정. ⚠️ `RELEASE` **하나로는 안 된다** — `task_manager._to()` 는 상태에 **진입할 때**
#: 이름을 발행하고, 그리퍼를 열고 detach 하는 것은 그 **뒤**다. `RELEASE -> ABORT` 도 허용된
#: 전이라(`states.py`) 진입만 보고 성공이라 말하면 최대 20 s 이르고, 뒤따르는 ABORT 는
#: 보고조차 안 된다. 그래서 **`RELEASE` 를 지나 `HOME` 에 들어갔을 때** 성공으로 본다.
RELEASE_STATE = 'RELEASE'
DONE_STATE = 'HOME'

#: 이 상태들이 보이면 FSM 은 더 이상 우리 응답을 기다리지 않는다. `_st_listening` 의 호출은
#: 이미 버려졌으므로 대기 중이던 `/get_keyword` 는 **지시를 소비하지 않고** 물러나야 한다.
ABANDON_STATES = frozenset({'IDLE', 'ABORT', 'SAFE_STOP'})

#: FSM 이 지시 없이 인식으로 넘어간 상태. 여기 왔는데 우리 지시가 안 팔렸으면 FSM 은
#: **다른 타겟으로** 돌고 있다 (`pick_fsm` 을 `voice:=false` 로 띄운 경우가 대표적이다).
BYPASS_STATE = 'PERCEIVE'

#: FSM 이 사람 승인을 기다리는 상태. `/vla/pick_status` 의 `waiting_approval` 로 내보낸다 —
#: contract §4 가 열어둔 "LLM 이 승인 대기를 알면 좋다"에 답한다. 승인 자체는 여전히 사람
#: 몫이라 자동화 경로는 만들지 않는다(§0-B) — 이건 표시용 신호일 뿐이다.
WAIT_APPROVAL_STATE = 'WAIT_APPROVAL'

#: FSM 이 들어올린 뒤 place 를 기다리는 상태. `cmd:"set_place"` 는 이 상태에서만 먹는다 —
#: `task_manager.State.WAIT_PLACE_TARGET` 과 이름이 같아야 한다(패키지 경계라 값만 복제).
WAIT_PLACE_STATE = 'WAIT_PLACE_TARGET'

#: 로봇이 안전정지류라 명령을 못 받는 상태 코드. `pick_fsm.robot_safety_node.UNSAFE_STATES`
#: 가 정본 — 패키지 경계를 넘는 import 는 안 하므로(`PLACE_VALUES` 와 같은 이유) 값만 복제한다.
#: 저쪽이 바뀌면 손으로 맞춘다.
UNSAFE_STATE_CODES = frozenset({3, 5, 6, 9, 10})

#: 우리가 받아들이는 `cmd` 값. `pick_and_place` 는 같은 뜻으로 받는다 —
#: 이 FSM 의 pick 사이클은 어차피 `place_joints_deg` 에 놓는 것으로 끝난다.
PICK_CMDS = ('pick', 'pick_and_place')

#: rqt 패널 버튼 중 **안전한 것들**. `Trigger` 서비스를 그대로 호출한다 — LISTENING 래치를
#: 거치지 않고 즉시 나간다(사람이 아무 때나 rqt 버튼을 누르는 것과 같은 성격이라 pick 지시의
#: TTL·대기 로직이 안 맞는다). 파라미터 이름은 `<cmd>_service` 로 통일했다.
#: `home` 은 IDLE 에서만 먹는다(`task_manager._srv_home`) — reset(SAFE_STOP 전용)과 다르다.
#: `release_now` 는 `WAIT_PLACE_TARGET` 에서만 먹는다 — 상태 판정은 저쪽
#: (`task_manager._srv_release_now`)이 하고 여기선 그대로 전달만 한다. 상태 검사를
#: 양쪽에 두면 조용히 갈라진다(`set_place` 는 래치 때문에 예외적으로 여기서도 본다).
CONTROL_CMDS = ('start', 'abort', 'reset', 'home', 'release_now',
                'pause', 'resume', 'stow')

#: rqt 패널의 '승인' 버튼에 해당한다. **의도적으로 어떤 파라미터로도 못 연다** — 계획 §0-B.
#: cmd 로 오면 무조건 거부하고, 그 사유를 그대로 알려준다.
BLOCKED_CMDS = ('approve',)

#: `pixel_policy` 가 이상한 값이면 여기로 떨어진다. 안전한 쪽(거부)으로 넘어져야 한다 —
#: 오타 하나로 "같은 클래스 2개일 때의 유일한 방어"가 조용히 꺼지면 안 된다.
#: `select`(실제 선정)로 폴백하지 않는 이유가 이것이다 — 애매하면 안 집는 쪽이 낫다.
FALLBACK_PIXEL_POLICY = 'reject'

#: `pixel_policy` 로 받아들이는 값. `select` 는 2026-08-11 `select_by_point()` 구현과
#: 함께 추가됐다 — `grasp_bridge_node`가 준비되기 전까지는 `warn`/`reject` 만 뜻이 있었다.
PIXEL_POLICIES = ('warn', 'reject', 'select')

#: `pick_fsm.task_manager.PLACE_LOCATIONS` 의 키와 **반드시 같아야 한다.** 패키지 경계를
#: 넘는 import 는 안 하므로(둘 다 토픽/서비스로만 통신 — 헤더 참고) 값을 여기 복제해 둔다.
#: 저쪽에 위치를 추가/삭제하면 이 집합도 손으로 맞춰야 한다.
PLACE_VALUES = frozenset({'basket', 'table', 'discard'})


def classify_cmd(cmd: str) -> str:
    """`cmd` 문자열이 pick / control(안전) / blocked(승인) / unknown 중 무엇인지.

    라우팅만 하는 순수 함수라 노드 없이 테스트할 수 있다. `pick`·`unknown` 은 같은 값을
    돌려준다 — 둘 다 `parse_command()` 로 넘어가 거기서 "cmd 를 모른다"로 최종 거부된다.
    """
    if cmd in CONTROL_CMDS:
        return 'control'
    if cmd in BLOCKED_CMDS:
        return 'blocked'
    return 'pick'


def _pair(value, name: str):
    """길이 2 의 숫자쌍인지 본다. `(쌍, 사유)` — 사유가 있으면 거부다."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None, f'{name} 는 길이 2 의 배열이어야 한다 (받은 값: {value!r})'
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
        return None, f'{name} 의 원소가 숫자가 아니다 (받은 값: {value!r})'
    return (float(value[0]), float(value[1])), ''


def parse_command(raw: str, *, allowed_classes=(), pixel_policy: str = 'warn'):
    """`/vla/pick_command` 한 건을 검증한다. `(명령 dict, 사유)` 를 돌려준다.

    명령이 `None` 이면 **거부**이고 사유가 거부 이유다. 명령이 있으면 사유는 경고다
    (빈 문자열이면 경고 없음).

    ⚠️ **필드가 없거나 타입이 다르면 조용히 기본값으로 진행하지 않는다.** `dry_run` 이
    제거된 뒤로 잘못 받은 지시는 "엉뚱한 물체를 집는다"가 아니라 **"엉뚱한 좌표로 실제
    팔이 간다"** 다 (계획 §0-B).

    이 함수만은 rclpy 를 안 쓴다 — 노드를 띄우지 않고 스키마만 테스트할 수 있게 순수
    함수로 뒀다 (모듈 자체는 `import rclpy` 하므로 ROS 환경은 여전히 필요하다).
    """
    try:
        doc = json.loads(raw)
    except (ValueError, TypeError) as exc:
        return None, f'JSON 파싱 실패: {exc}'
    if not isinstance(doc, dict):
        return None, f'JSON 최상위가 객체가 아니다 ({type(doc).__name__})'

    cmd = str(doc.get('cmd', 'pick')).strip()
    if cmd not in PICK_CMDS:
        return None, f"cmd 를 모른다: {cmd!r} (아는 값: {'/'.join(PICK_CMDS)})"

    # VLA 쪽 SceneObject 는 `class_name` 이다 — 그쪽 필드 이름을 그대로 보내도 받는다.
    name = doc.get('class', doc.get('class_name', ''))
    if not isinstance(name, str) or not name.strip():
        return None, 'class 가 비어 있거나 문자열이 아니다'
    name = name.strip()
    # FSM 은 `/get_keyword` 응답을 공백으로 쪼개 **첫 단어만** 쓴다(`_st_listening`).
    # 공백이 섞이면 뒷부분이 조용히 사라지므로 여기서 끊는다. 콤마는 허용한다 —
    # `target_classes` 가 원래 콤마 목록이고 FSM 이 그대로 브리지에 넘긴다.
    if name.split() != [name]:
        return None, f'class 에 공백이 있다: {name!r} (여러 개는 콤마로: apple,orange)'

    # 빈 항목을 걸러내고 **정제한 값을 내보낸다.** 원문을 그대로 넘기면 `'apple,'` 이
    # `target_classes` 에 그대로 실려 브리지가 빈 클래스 하나를 찾게 된다.
    wanted = [part.strip() for part in name.split(',') if part.strip()]
    if not wanted:
        return None, f'class 에 쓸 만한 이름이 없다: {name!r}'
    if allowed_classes:
        outside = [w for w in wanted if w not in allowed_classes]
        if outside:
            return None, (f'{outside} 는 인식 대상이 아니다 — YOLO 가 애초에 안 찾으므로 '
                          f'절대 못 잡는다 (허용: {sorted(allowed_classes)})')

    # ⚠️ `doc.get('place')` 의 truthiness 로 보면 `{}` · `""` · `0` 이 통과한다.
    #    "미구현 필드를 조용히 무시하지 않는다"는 계약이므로 **키의 존재**로 판정한다 —
    #    타입이 안 맞거나 목록 밖 값이면 기본값(basket)으로 조용히 넘어가지 않고 거부한다.
    place = None
    if 'place' in doc:
        raw_place = doc['place']
        if not isinstance(raw_place, str) or raw_place.strip() not in PLACE_VALUES:
            return None, (f'place 는 {sorted(PLACE_VALUES)} 중 하나여야 한다 '
                          f'(받은 값: {raw_place!r})')
        place = raw_place.strip()

    ignored = []
    pixel = pixel_wh = None
    if 'pixel' in doc:
        pixel, why = _pair(doc['pixel'], 'pixel')
        if why:
            return None, why
        # 리사이즈된 프레임 위에서 찍은 좌표라면 기준 해상도 없이는 조용히 어긋난다.
        # 계획 §2: 값이 없으면 **거부한다**.
        if 'pixel_wh' not in doc:
            return None, 'pixel 을 보냈으면 pixel_wh(기준 해상도)도 보내야 한다'
        pixel_wh, why = _pair(doc['pixel_wh'], 'pixel_wh')
        if why:
            return None, why
        if pixel_wh[0] <= 0 or pixel_wh[1] <= 0:
            return None, f'pixel_wh 가 양수가 아니다: {pixel_wh}'
        if pixel_policy == 'reject':
            return None, 'pixel 개체 지정은 이 노드 설정(pixel_policy=reject)에서 막혀 있다'
        if pixel_policy == 'select':
            pass  # 아래 out['pixel'] 에 그대로 실어 _on_pick_command 가 브리지로 넘긴다
        else:
            ignored.append('pixel')
    if 'base_xy' in doc:
        ignored.append('base_xy')

    out = {
        'cmd': cmd,
        'class': ','.join(wanted),
        'request_id': str(doc.get('request_id', '')),
        # `stamp_ns` 는 결과에 에코해 상관관계 추적에 쓴다. `place` 는 None 이면
        # "지정 안 함"(호출부가 `/pick/place_location` 을 건드리지 않는다) — 파라미터
        # 기본값(`basket`)이 그대로 쓰인다(task_manager 쪽 계약, `_on_pick_command` 참고).
        # `pixel`/`pixel_wh` 는 검증만 통과하면 정책과 무관하게 채워진다(기존 계약 유지 —
        # `warn`에서도 값 자체는 내보내고 왔었다). **실제로 쓸지는 `ignored`로 판정한다**:
        # `pixel_policy=select` 일 때만 `ignored`에 'pixel'이 없다 — `_on_pick_command`가
        # 이 조건으로 브리지에 넘길지를 결정한다.
        'stamp_ns': doc.get('stamp_ns'),
        'pixel': pixel,
        'pixel_wh': pixel_wh,
        'place': place,
        'ignored': ignored,
    }
    warn = ''
    if ignored:
        warn = (f'{ignored} 는 아직 쓰지 않는다 — 클래스 이름만으로 고른다. '
                '같은 클래스 물체가 여럿이면 다른 개체를 집을 수 있다 '
                "(개체까지 지정하려면 pixel_policy:='select', 아예 막으려면 'reject')")
    return out, warn


class VlaCommandNode(Node):
    """VLA 지시를 붙잡고 있다가 FSM 이 물을 때 건네는 한 건짜리 래치."""

    def __init__(self):
        """파라미터를 선언하고, 인터페이스를 열고, 대기시간 예산을 검증한다."""
        super().__init__('vla_command_node')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('command_topic', '/vla/pick_command')
        self.declare_parameter('result_topic', '/vla/pick_result')
        self.declare_parameter('keyword_service', '/get_keyword')
        self.declare_parameter('state_topic', '/pick/state')
        # rqt 패널이 보여주는 상태를 그대로 미러링하는 VLA-facing 토픽. 아래 4개는 여기서
        # 읽어들이는 pick_fsm 내부 토픽이다(robot_safety_node / task_manager 가 발행).
        self.declare_parameter('status_topic', '/vla/pick_status')
        self.declare_parameter('robot_state_code_topic', '/pick/robot_state_code')
        self.declare_parameter('robot_state_text_topic', '/pick/robot_state_text')
        self.declare_parameter('target_active_topic', '/pick/target_active')
        self.declare_parameter('place_active_topic', '/pick/place_location_active')
        self.declare_parameter('start_service', '/pick/start')
        self.declare_parameter('abort_service', '/pick/abort')
        self.declare_parameter('reset_service', '/pick/reset')
        self.declare_parameter('home_service', '/pick/home')
        self.declare_parameter('release_now_service', '/pick/release_now')
        self.declare_parameter('pause_service', '/pick/pause')
        self.declare_parameter('resume_service', '/pick/resume')
        self.declare_parameter('stow_service', '/pick/stow')
        # `task_manager._on_place_location` 이 듣는 그 토픽이다 — 이름을 바꾸면 저쪽 launch
        # 인자도 같이 바꿔야 매칭된다.
        self.declare_parameter('place_location_topic', '/pick/place_location')
        # `task_manager._on_place_pending` 이 듣는 토픽 — pick 마다 "place 미지정?"을 알린다.
        self.declare_parameter('place_pending_topic', '/pick/place_pending')
        # true 면 place 없는 pick 을 FSM 이 WAIT_PLACE_TARGET 로 붙잡게 한다(place_pending=true).
        # false 면 예전처럼 곧장 파라미터 기본 위치(basket)에 놓는다 — 전환기 롤백 스위치.
        self.declare_parameter('wait_place_when_omitted', True)
        # `task_manager._on_target_pixel` 이 듣는 토픽 — pixel_policy=select 일 때만 쓴다.
        self.declare_parameter('target_pixel_topic', '/pick/target_pixel')
        # 🔴 승인 서비스는 파라미터로도 두지 않는다 — 있으면 언젠가 켜진다 (§0-B).
        self.declare_parameter('auto_start', False)
        self.declare_parameter('ttl_sec', 10.0)
        self.declare_parameter('wait_timeout_sec', 100.0)
        # `task_manager.DEFAULT_TIMEOUTS[State.LISTENING]` 의 사본이다. 저쪽이 정본이므로
        # 값을 바꿨다면 여기도 맞춰야 한다 — 파라미터로 뺀 이유가 그것이다.
        self.declare_parameter('fsm_listening_timeout_sec', 110.0)
        # 서비스 탐색·왕복에 쓰이는 여유. FSM 의 LISTENING 시계는 우리 서버가 뜨기 전부터
        # 돌기 시작한다(`task_manager._service()` 는 서버가 없으면 기다린다).
        self.declare_parameter('listening_margin_sec', 5.0)
        # ⚠️ 콤마 문자열이다. `[]` 를 기본값으로 쓰면 rclpy 가 BYTE_ARRAY 로 추론해서
        #    나중에 문자열 배열을 못 넣는다(2026-08-09 실측). 이 ws 의 `target_classes`
        #    (grasp_bridge_node)도 콤마 문자열이라 표기가 일치한다.
        self.declare_parameter('allowed_classes', '')
        self.declare_parameter('pixel_policy', 'warn')       # warn | reject | select

        policy = str(self.get_parameter('pixel_policy').value)
        if policy not in PIXEL_POLICIES:
            raise ValueError(f"pixel_policy 는 {'|'.join(PIXEL_POLICIES)} 다: {policy!r}")

        self._ttl = float(self.get_parameter('ttl_sec').value)
        self._wait = float(self.get_parameter('wait_timeout_sec').value)
        self._listen_budget = float(self.get_parameter('fsm_listening_timeout_sec').value)
        self._margin = float(self.get_parameter('listening_margin_sec').value)
        self._auto_start = bool(self.get_parameter('auto_start').value)
        self._wait_place_when_omitted = bool(
            self.get_parameter('wait_place_when_omitted').value)

        # 여기서 죽는 것이 맞다. 예산을 넘기면 **매 사이클** FSM 이 먼저 ABORT 하고
        # SAFE_STOP 에 들어가 `/pick/reset` 없이는 못 나온다 — 기동 때 한 번 막는 게 싸다.
        if self._wait >= self._listen_budget - self._margin:
            raise ValueError(
                f'wait_timeout_sec({self._wait}) 이 FSM 의 LISTENING 예산을 넘는다: '
                f'fsm_listening_timeout_sec({self._listen_budget}) - '
                f'listening_margin_sec({self._margin}) 보다 작아야 한다')

        # ── 상태 ──────────────────────────────────────────
        # 대기 지시는 **한 건만** 들고 있는다. 새 지시가 오면 덮어쓴다(경고를 찍는다) —
        # 큐로 쌓으면 사람이 마지막에 말한 것과 로봇이 집는 것이 어긋난다.
        self._cv = threading.Condition()
        self._pending = None            # (명령 dict, 받은 시각[monotonic])
        self._active = None             # FSM 에 넘어간 명령. 결과를 기다리는 중
        self._state = ''                # 마지막 `/pick/state`
        self._listening_since = None    # LISTENING 진입 시각[monotonic]
        self._saw_release = False       # 이번 사이클에서 RELEASE 를 지나왔는지
        self._closing = threading.Event()
        # rqt 미러링용 최신값. 표시 전용이라 락으로 감싸지 않는다 — 각 값은 단일 콜백만
        # 쓰고, `_publish_status()` 는 살짝 옛 값을 읽어도 무해하다(다음 발행이 곧 맞춘다).
        self._robot_text = ''           # 마지막 `/pick/robot_state_text`
        self._robot_code = None         # 마지막 `/pick/robot_state_code`
        self._active_target = ''        # `/pick/target_active` — FSM 이 지금 쓰는 타겟
        self._active_place = ''         # `/pick/place_location_active` — 지금 쓰는 위치

        self.result_pub = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), COMMAND_QOS)
        # QoS 는 `PLACE_QOS`(TRANSIENT_LOCAL) — task_manager 구독이 이걸 요구한다(위 정의부 참고).
        self.place_pub = self.create_publisher(
            String, str(self.get_parameter('place_location_topic').value), PLACE_QOS)
        # place 미지정 여부를 pick 마다 알린다. place 와 같은 TRANSIENT_LOCAL — 늦게 붙어도
        # 마지막 값을 받아야 `_st_idle` 이 사이클 시작 때 latch 할 수 있다.
        self.place_pending_pub = self.create_publisher(
            Bool, str(self.get_parameter('place_pending_topic').value), PLACE_QOS)
        # place 와 같은 QoS다 — `task_manager._on_target_pixel` 구독도 TARGET_QOS
        # (= PLACE_QOS 와 값이 같다)다.
        self.pixel_pub = self.create_publisher(
            String, str(self.get_parameter('target_pixel_topic').value), PLACE_QOS)
        # rqt 미러링 채널. result 와 같은 COMMAND_QOS(RELIABLE/VOLATILE) — VOLATILE 이라
        # 늦게 붙은 구독자는 마지막 값을 못 받지만, 아래 1 Hz 하트비트가 1 s 안에 현재 상태를
        # 다시 준다(+ keepalive 로 VLA 가 "cobot2_ws 응답 없음"을 staleness 로 감지). place 처럼
        # TRANSIENT_LOCAL 로 latch 하지 않는 이유: contract 가 핫스팟 링크에서 그 조합의
        # 블로킹 위험을 이미 지적했고, result 채널과 같은 프로파일로 맞춰 두는 게 안전하다.
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), COMMAND_QOS)
        self.create_subscription(
            String, str(self.get_parameter('command_topic').value),
            self._on_command, COMMAND_QOS, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter('state_topic').value),
            self._on_state, COMMAND_QOS, callback_group=cb)
        # 로봇 상태는 text/code 둘 다 받는다: text 는 표시용 이름(robot_safety_node 의 name
        # 테이블을 import 하지 않으려고 — 패키지 경계), code 는 `unsafe` 계산용.
        self.create_subscription(
            Int8, str(self.get_parameter('robot_state_code_topic').value),
            self._on_robot_code, COMMAND_QOS, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter('robot_state_text_topic').value),
            self._on_robot_text, COMMAND_QOS, callback_group=cb)
        # target_active/place_active 는 task_manager 가 TARGET_QOS(=PLACE_QOS, TRANSIENT_LOCAL)
        # 로 발행한다 — 구독자도 그 durability 라야 매칭된다(VOLATILE 이면 아예 안 붙는다).
        self.create_subscription(
            String, str(self.get_parameter('target_active_topic').value),
            self._on_target_active, PLACE_QOS, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter('place_active_topic').value),
            self._on_place_active, PLACE_QOS, callback_group=cb)
        # VOLATILE 보정 + keepalive. 상태가 안 바뀌어도 1 s 마다 현재 스냅샷을 내보낸다.
        self.create_timer(1.0, self._publish_status, callback_group=cb)
        self.create_service(
            Trigger, str(self.get_parameter('keyword_service').value),
            self._srv_keyword, callback_group=cb)
        self.start_cli = self.create_client(
            Trigger, str(self.get_parameter('start_service').value), callback_group=cb)
        self.abort_cli = self.create_client(
            Trigger, str(self.get_parameter('abort_service').value), callback_group=cb)
        self.reset_cli = self.create_client(
            Trigger, str(self.get_parameter('reset_service').value), callback_group=cb)
        self.home_cli = self.create_client(
            Trigger, str(self.get_parameter('home_service').value), callback_group=cb)
        self.release_now_cli = self.create_client(
            Trigger, str(self.get_parameter('release_now_service').value),
            callback_group=cb)
        self.pause_cli = self.create_client(
            Trigger, str(self.get_parameter('pause_service').value), callback_group=cb)
        self.resume_cli = self.create_client(
            Trigger, str(self.get_parameter('resume_service').value), callback_group=cb)
        self.stow_cli = self.create_client(
            Trigger, str(self.get_parameter('stow_service').value), callback_group=cb)
        #: `cmd` -> 클라이언트. `_handle_control()` 이 여기서 찾는다 — 서비스 이름은
        #: 위 파라미터로 바뀔 수 있어도 `cmd` 값(start/abort/reset/home)은 고정이다.
        self._control_clients = {
            'start': self.start_cli, 'abort': self.abort_cli, 'reset': self.reset_cli,
            'home': self.home_cli, 'release_now': self.release_now_cli,
            'pause': self.pause_cli, 'resume': self.resume_cli,
            'stow': self.stow_cli,
        }

        self.get_logger().info(
            f"준비됨 — {self.get_parameter('command_topic').value} 를 듣고 "
            f"{self.get_parameter('keyword_service').value} 로 답한다. "
            f'ttl={self._ttl:.0f}s wait={self._wait:.0f}s '
            f'auto_start={self._auto_start} pixel_policy={policy}')
        allowed = self._allowed()
        self.get_logger().info(
            f'허용 클래스: {sorted(allowed) if allowed else "(검사 안 함)"}')
        if self._auto_start:
            self.get_logger().warn(
                '⚠️ auto_start=true — 지시가 오면 이 노드가 /pick/start 를 부른다. '
                '승인(/pick/approve)은 여전히 사람 몫이다')

    # ────────────────────────────────────────────────────────
    def _allowed(self) -> set:
        """`allowed_classes`(콤마 문자열)를 집합으로. 비어 있으면 검사하지 않는다."""
        raw = str(self.get_parameter('allowed_classes').value)
        return {part.strip() for part in raw.split(',') if part.strip()}

    def _pixel_policy(self) -> str:
        """`pixel_policy` 를 읽는다. 값이 이상하면 **안전한 쪽(거부)** 으로 넘어진다.

        `__init__` 에서 한 번 검증했지만 `ros2 param set` 으로 런타임에 바뀔 수 있고,
        그때 오타 하나가 "같은 클래스 2개일 때의 유일한 방어"를 조용히 끄면 안 된다.
        """
        value = str(self.get_parameter('pixel_policy').value)
        if value in PIXEL_POLICIES:
            return value
        self.get_logger().error(
            f'pixel_policy 가 이상하다: {value!r} — 안전한 쪽으로 '
            f'{FALLBACK_PIXEL_POLICY} 처리한다', throttle_duration_sec=10.0)
        return FALLBACK_PIXEL_POLICY

    def _publish_result(self, cmd_or_id, accepted: bool, reason: str, result: str):
        """VLA 로 되돌려주는 유일한 채널. `request_id` 는 **그대로 echo** 한다.

        `cmd_or_id` 는 명령 dict 이거나 `request_id` 문자열이다 — 거부는 명령을 못 만든
        상태에서도 나가야 하므로 두 경우를 다 받는다.
        """
        cmd = cmd_or_id if isinstance(cmd_or_id, dict) else {}
        payload = {
            'request_id': cmd.get('request_id', cmd_or_id if isinstance(cmd_or_id, str) else ''),
            'accepted': accepted,
            # rejected | accepted | succeeded | failed | superseded
            'result': result,
            'reason': reason,
            'ignored': list(cmd.get('ignored', [])),
            'stamp_ns': cmd.get('stamp_ns'),
            'state': self._state,
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ────────────────────────────────────────────────────────
    def _on_command(self, msg):
        """`/vla/pick_command` 를 받는다. `cmd` 값으로 pick / 제어 / 차단 세 갈래로 나눈다."""
        try:
            doc = json.loads(msg.data)
            cmd_word = str(doc.get('cmd', 'pick')).strip() if isinstance(doc, dict) else ''
            rid = str(doc.get('request_id', '')) if isinstance(doc, dict) else ''
        except (ValueError, TypeError):
            doc, cmd_word, rid = None, '', ''

        # set_place 는 새 pick 을 시작하지 않는다 — 이미 물체를 든 채 대기 중인 FSM 에
        # 놓을 위치만 뒤늦게 건넨다. pick/control 어느 갈래도 아니라 여기서 먼저 가로챈다.
        if doc is not None and cmd_word == 'set_place':
            self._handle_set_place(doc, rid)
            return

        kind = classify_cmd(cmd_word) if doc is not None else 'pick'

        if kind == 'blocked':
            why = ('approve 는 음성/VLA 경로로 자동화하지 않는다 — require_approval 이 '
                  'dry_run 제거 뒤 남은 유일한 소프트 안전장치다(계획 §0-B). '
                  '/pick/approve 는 rqt 패널이나 서비스로 사람이 직접 부른다')
            self.get_logger().error(f"[{rid or '-'}] 지시 거부: {why}")
            self._publish_result(rid, False, why, 'rejected')
            return

        if kind == 'control':
            self._handle_control(cmd_word, rid, doc)
            return

        self._on_pick_command(msg.data)

    def _handle_control(self, cmd: str, rid: str, doc: dict):
        """rqt 패널의 시작/중단/리셋 버튼과 같은 서비스를 부른다. `승인`은 여기 없다."""
        cli = self._control_clients[cmd]
        if not cli.service_is_ready():
            why = f'{cli.srv_name} 가 아직 없다'
            self.get_logger().warn(f"[{rid or '-'}] {why}")
            self._publish_result(rid, False, why, 'rejected')
            return
        reason = str(doc.get('reason', '')).strip()
        self.get_logger().info(
            f"[{rid or '-'}] {cmd} 요청 전달" + (f" ({reason})" if reason else ''))
        fut = cli.call_async(Trigger.Request())

        def _done(f, cmd=cmd, rid=rid):
            res = f.result()
            if res is None:
                self._publish_result(rid, False, f'{cmd} 응답 없음', 'failed')
            elif res.success:
                self._publish_result(rid, True, res.message or f'{cmd} 성공', 'succeeded')
            else:
                self._publish_result(rid, False, res.message or f'{cmd} 거절됨', 'rejected')
        fut.add_done_callback(_done)

    def _handle_set_place(self, doc: dict, rid: str):
        """`WAIT_PLACE_TARGET` 에서 놓을 위치를 뒤늦게 채운다 — `/pick/place_location` 로 publish.

        pick 지시와 달리 래치를 안 거친다(새 사이클을 시작하지 않는다). FSM 이 지금 place 를
        기다리는 중일 때만 먹는다 — 아니면 거부한다(엉뚱한 시점에 목적지만 바뀌는 걸 막는다).
        상태 판정은 이 노드가 이미 받고 있는 `/pick/state`(`self._state`)로 한다.
        """
        place = doc.get('place')
        if not isinstance(place, str) or place.strip() not in PLACE_VALUES:
            why = f'place 는 {sorted(PLACE_VALUES)} 중 하나여야 한다 (받은 값: {place!r})'
            self.get_logger().warn(f"[{rid or '-'}] set_place 거부: {why}")
            self._publish_result(rid, False, why, 'rejected')
            return
        if self._state != WAIT_PLACE_STATE:
            why = (f'set_place 는 {WAIT_PLACE_STATE} 에서만 먹는다 '
                   f'(현재 {self._state or "?"})')
            self.get_logger().warn(f"[{rid or '-'}] set_place 거부: {why}")
            self._publish_result(rid, False, why, 'rejected')
            return
        self.place_pub.publish(String(data=place.strip()))
        self.get_logger().info(f"[{rid or '-'}] set_place: {place.strip()} — 내려놓기로 진행")
        self._publish_result(rid, True, f"놓을 위치 '{place.strip()}' 전달", 'accepted')

    def _on_pick_command(self, raw: str):
        """`cmd: pick|pick_and_place` 를 검증해 래치에 넣는다. 거부는 즉시 회신한다."""
        cmd, why = parse_command(raw, allowed_classes=self._allowed(),
                                 pixel_policy=self._pixel_policy())
        if cmd is None:
            # 거부는 조용히 하지 않는다. 여기서 안 알리면 VLA 는 지시가 사라진 것과
            # 로봇이 아직 안 움직인 것을 구분할 수 없다.
            try:
                rid = str((json.loads(raw) or {}).get('request_id', ''))
            except Exception:                                       # noqa: BLE001
                rid = ''
            self.get_logger().error(f'지시 거부: {why} — 원문: {raw[:200]}')
            self._publish_result(rid, False, why, 'rejected')
            return
        if why:
            self.get_logger().warn(f"[{cmd['request_id'] or '-'}] {why}")

        # place 미지정 → FSM 이 LIFT 후 set_place 를 기다리게 한다(place_pending=true).
        # 지정됐거나 롤백 스위치(wait_place_when_omitted=false)면 false 를 쏴 기존처럼 곧장
        # 파라미터 기본 위치(basket)에 놓는다. place 발행보다 **먼저** 쏜다(같은 순서 이유).
        pending = cmd['place'] is None and self._wait_place_when_omitted
        self.place_pending_pub.publish(Bool(data=pending))
        if pending:
            self.get_logger().info(
                f"[{cmd['request_id'] or '-'}] place 미지정 — LIFT 후 set_place 를 기다린다")

        if cmd['place'] is not None:
            # `task_manager._on_place_location` 이 이 값을 상태와 무관하게 즉시
            # `self._place_override` 에 반영한다 — 진행 중인 사이클에는 안 먹히고
            # (저쪽이 경고 로그를 낸다) 다음 `_st_idle` 부터 적용된다. 이 pick 지시가
            # 같은 사이클에서 그 place 를 쓰길 기대하므로, **래치에 넣기 전에** 먼저
            # 보낸다 — 순서를 바꾸면 이번 지시가 이전 place 값으로 진행될 수 있다.
            self.place_pub.publish(String(data=cmd['place']))
            self.get_logger().info(
                f"[{cmd['request_id'] or '-'}] 내려놓을 위치 지정: {cmd['place']}")

        if cmd['pixel'] is not None and 'pixel' not in cmd['ignored']:
            # place 와 같은 순서 이유(위 주석) — `task_manager._on_target_pixel` 은
            # **단발성**(다음 PERCEIVE 가 소비하면 지운다)이라 place 보다 오히려 타이밍이
            # 더 중요하다: 늦게 보내면 이 pick 지시가 이전 좌표를 못 받고 지나칠 수 있다.
            px, py = cmd['pixel']
            pw, ph = cmd['pixel_wh']
            self.pixel_pub.publish(String(data=json.dumps(
                {'x': px, 'y': py, 'w': pw, 'h': ph})))
            self.get_logger().info(
                f"[{cmd['request_id'] or '-'}] 개체 선정 좌표 전달: ({px:.0f},{py:.0f}) "
                f'/ 기준 {pw:.0f}x{ph:.0f}')

        with self._cv:
            old = self._pending[0] if self._pending is not None else None
            self._pending = (cmd, time.monotonic())
            self._cv.notify_all()
        if old is not None:
            self.get_logger().warn(
                f"이전 지시 '{old['class']}'(id={old['request_id'] or '-'}) 를 "
                f"'{cmd['class']}' 로 덮어쓴다 — 아직 FSM 이 가져가지 않았다")
            self._publish_result(old, False, '더 새로운 지시로 대체됨', 'rejected')
        self.get_logger().info(
            f"지시 수신: class='{cmd['class']}' id={cmd['request_id'] or '-'}")
        if self._auto_start:
            self._request_start()

    def _request_start(self):
        """지시가 자동으로 들어왔을 때(`auto_start`) `/pick/start` 를 비동기로 부른다.

        명시적 `cmd:"start"`(위 `_handle_control`)와 달리 결과를 `/vla/pick_result` 로
        회신하지 않는다 — 이건 pick 지시에 **얹혀** 나가는 부수 동작이라, 응답은 이미
        `_on_pick_command` 가 `accepted` 로 보낸 뒤다. 실패해도 지시 자체는 래치에 남아
        다음 `LISTENING` 때 여전히 유효하므로 fire-and-forget 로 둔다.
        """
        if not self.start_cli.service_is_ready():
            self.get_logger().warn(
                f'{self.start_cli.srv_name} 가 아직 없다 — 사람이 직접 start 해야 한다')
            return
        fut = self.start_cli.call_async(Trigger.Request())

        def _done(f):
            res = f.result()
            if res is None:
                self.get_logger().warn('/pick/start 응답 없음')
            elif not res.success:
                self.get_logger().info(f'/pick/start 거절: {res.message} '
                                       '(진행 중이면 정상 — 지시는 래치에 남는다)')
        fut.add_done_callback(_done)

    def _on_state(self, msg):
        """FSM 상태를 결과로 옮긴다. 판정은 `/pick/state` 하나로만 한다.

        `_pending`·`_active` 는 여기(구독 콜백)와 `_srv_keyword`(서비스 콜백)가 같이
        만진다 — 둘 다 `ReentrantCallbackGroup` 이라 다른 스레드에서 동시에 돈다.
        락 없이 두면 결과가 두 번 나가거나 한 번도 안 나갈 수 있다.
        """
        state = msg.data
        self._state = state
        publish = []                    # (명령, accepted, 사유, result) — 락 밖에서 발행한다

        with self._cv:
            if state == 'LISTENING':
                self._listening_since = time.monotonic()
            elif state in ABANDON_STATES:
                # 대기 중인 `/get_keyword` 를 깨워 지시를 소비하지 않고 물러나게 한다.
                self._listening_since = None
                self._cv.notify_all()

            if state == RELEASE_STATE:
                self._saw_release = True

            # FSM 이 지시를 안 가져간 채 인식으로 넘어갔다 = 다른 타겟으로 도는 중이다.
            if state == BYPASS_STATE and self._active is None and self._pending is not None:
                cmd, _ = self._pending
                self._pending = None
                publish.append((cmd, False,
                                'FSM 이 이 지시를 안 가져가고 PERCEIVE 로 갔다 — '
                                'pick_fsm 이 voice:=false 로 떠 있거나 다른 노드가 '
                                '/get_keyword 에 먼저 답했다', 'rejected'))

            active = self._active
            if active is not None:
                if state == DONE_STATE and self._saw_release:
                    self._active, self._saw_release = None, False
                    publish.append((active, True, f'{RELEASE_STATE} 를 지나 {DONE_STATE} 도달',
                                    'succeeded'))
                elif state in FAIL_STATES:
                    self._active, self._saw_release = None, False
                    publish.append((active, True, f'FSM 이 {state} 로 끝났다', 'failed'))

        for cmd, accepted, reason, result in publish:
            # ⚠️ 한 호출지점에서 severity 를 바꿔 가며 찍으면 안 된다 — rclpy 는 호출지점마다
            #    severity 를 캐시해서 `ValueError: Logger severity cannot be changed between
            #    calls` 로 **노드가 죽는다** (2026-08-09 실측, 스모크 중 크래시).
            line = f"[{cmd['request_id'] or '-'}] {result}: '{cmd['class']}' — {reason}"
            if result == 'succeeded':
                self.get_logger().info(line)
            else:
                self.get_logger().warn(line)
            self._publish_result(cmd, accepted, reason, result)

        # 상태가 바뀌었으니 rqt 미러도 바로 맞춘다(하트비트를 기다리지 않는다).
        self._publish_status()

    # ── rqt 미러링 ───────────────────────────────────────────
    def _on_robot_code(self, msg):
        self._robot_code = int(msg.data)
        self._publish_status()

    def _on_robot_text(self, msg):
        self._robot_text = msg.data
        self._publish_status()

    def _on_target_active(self, msg):
        self._active_target = msg.data
        self._publish_status()

    def _on_place_active(self, msg):
        self._active_place = msg.data
        self._publish_status()

    def _publish_status(self):
        """rqt 패널이 보여주는 상태를 `/vla/pick_status` 한 토픽으로 내보낸다.

        on-change(각 구독 콜백·`_on_state`) + 1 Hz 하트비트로 발행한다. VLA UI 의 staleness
        판정은 `stamp_ns` 차이가 아니라 **"N 초간 수신 없음"** 으로 해야 한다 — 두 PC 시계
        동기화를 가정하지 않는다(result 채널과 같은 원칙). `request_id` 는 지금 latch 된
        명령의 것이라 VLA 가 자기가 보낸 명령과 대조할 수 있다(없으면 빈 문자열).
        """
        active = self._active           # 참조 읽기는 원자적 — 살짝 옛 값이어도 표시용이라 무해
        payload = {
            'fsm': self._state,
            'robot': self._robot_text,
            'robot_code': self._robot_code,
            'target': self._active_target,
            'place': self._active_place,
            'request_id': active.get('request_id', '') if active else '',
            'waiting_approval': self._state == WAIT_APPROVAL_STATE,
            'unsafe': self._robot_code in UNSAFE_STATE_CODES,
            'stamp_ns': time.time_ns(),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ────────────────────────────────────────────────────────
    def _srv_keyword(self, _req, res):
        """FSM 의 `LISTENING` 이 부른다. 지시가 올 때까지 **기다린다**.

        즉시 실패로 답하면 FSM 이 `SPEAK_FAIL <-> LISTENING` 을 tick 주기로 왕복하다가
        `MAX_FAIL_STREAK` 로 IDLE 에 떨어진다 — 사람이 start 를 눌러 두고 VLA 지시를
        기다리는 정상 운용이 불가능해진다. 그래서 붙잡는다.

        마감은 `wait_timeout_sec` 과 **FSM 의 남은 LISTENING 예산** 중 이른 쪽이다.
        이 노드가 FSM 보다 늦게 떴으면 후자가 이미 줄어 있다(`task_manager._service()` 는
        서버가 없어도 기다리므로 그동안 LISTENING 시계가 돈다).
        """
        deadline = time.monotonic() + self._wait
        with self._cv:
            if self._listening_since is not None:
                deadline = min(deadline,
                               self._listening_since + self._listen_budget - self._margin)
            stale = None
            while True:
                if self._closing.is_set():
                    res.success, res.message = False, ''
                    return res
                # FSM 이 LISTENING 을 떠났으면 이 호출은 이미 버려졌다. 지시를 소비하면
                # 그 지시는 결과가 영영 안 나온다 — 래치에 그대로 두고 물러난다.
                if self._state in ABANDON_STATES:
                    self.get_logger().warn(
                        f'FSM 이 {self._state} 라 이 /get_keyword 호출은 버려졌다 — '
                        '지시를 소비하지 않고 물러난다')
                    res.success, res.message = False, ''
                    return res
                cmd = self._take_fresh()
                if cmd is not None:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    res.success, res.message = False, ''
                    self.get_logger().info('대기시간 안에 VLA 지시가 없었다 — 빈 응답')
                    return res
                self._cv.wait(remaining)
            # 앞 사이클의 결과를 못 본 채 다음 지시가 넘어가면 그 `request_id` 는 영영
            # 답이 없다. VLA 쪽에서 "보냈는데 아무 말이 없다"로 보이므로 여기서 끊는다.
            stale, self._active = self._active, cmd
            self._saw_release = False

        if stale is not None:
            self.get_logger().warn(
                f"이전 지시 id={stale['request_id'] or '-'} 의 결과를 못 봤다 — 대체 처리")
            self._publish_result(stale, True, '결과를 보기 전에 다음 지시가 시작됐다',
                                 'superseded')
        res.success, res.message = True, cmd['class']
        self.get_logger().info(
            f"FSM 에 전달: '{cmd['class']}' id={cmd['request_id'] or '-'}")
        self._publish_result(cmd, True, 'FSM 이 가져갔다 — 승인 대기는 사람 몫', 'accepted')
        return res

    def _take_fresh(self):
        """만료되지 않은 대기 지시를 꺼낸다. `self._cv` 를 잡은 채로 부른다."""
        if self._pending is None:
            return None
        cmd, received = self._pending
        age = time.monotonic() - received
        self._pending = None
        if age > self._ttl:
            # 지운다. 안 지우면 10 분 전 지시로 다음 픽이 나간다 (계획 §5).
            self.get_logger().warn(
                f"지시 만료: '{cmd['class']}' id={cmd['request_id'] or '-'} "
                f'({age:.1f}s > ttl {self._ttl:.0f}s)')
            self._publish_result(cmd, False,
                                 f'지정 만료 ({age:.1f}s > {self._ttl:.0f}s)', 'rejected')
            return None
        return cmd

    def close(self):
        """대기 중인 `/get_keyword` 콜백을 깨워 즉시 물러나게 한다.

        `Executor.shutdown()` 은 콜백이 끝날 때까지 기다린다 — 깨우지 않으면 Ctrl-C 가
        `wait_timeout_sec` 만큼(기본 50 s) 먹힌다.
        """
        self._closing.set()
        with self._cv:
            self._cv.notify_all()


def main(args=None):
    """노드를 띄우고 `MultiThreadedExecutor` 로 돌린다."""
    rclpy.init(args=args)
    node = VlaCommandNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
