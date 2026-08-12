"""The complete set of things the model is allowed to do.

This list *is* the control logic. There is no rule engine behind it deciding
which object matches "the red one" or how many apples "the apples" means --
the model reads the scene and calls one of these.

Schemas are in the flat Responses-API shape (``type``/``name`` at the top
level, not nested under ``function``).
"""

# Actions that hand control to the arm. The agent stops its tool loop after
# one of these: the next decision point is the action *completing*, which
# arrives as a robot state event, not as another tool round.
#
# pick_and_place only: cobot2_ws's pick_fsm always carries a pick through to
# place -- there is no "hold it and wait" or "put it down right here" on that
# side (vla-bridge-contract.md #7), so pick_and_hold/release had nowhere to
# go. They used to stay in the schema for vla_robot's own standalone arm
# control, but that path (robot_node.py) is gone now that cobot2_ws's pick_fsm
# is the only executor -- see 팀 컨벤션 문서 #3.
#
# 2026-08-11 update: cobot2_ws's pick_fsm now DOES have a "hold it and wait"
# state (WAIT_PLACE_TARGET, contract §13) for the specific case of a pick sent
# without `place` -- it lifts the object and waits for a follow-up `set_place`
# command instead of always finishing the cycle itself. set_place is a
# separate tool (not folded into pick_and_place) because it has no object_id
# of its own -- it always applies to whatever the arm is currently holding.
# 2026-08-12: pick_and_hold and release_held complete the hold path.
# pick_and_hold sends the same wire command as pick_and_place with place=null;
# it exists so "사과 집어줘" and "사과 바구니에 넣어줘" are different *calls*
# rather than the same call with a nullable field the model has to remember to
# leave empty. That field was being filled with basket by guesswork, which
# silently skipped the moment where the user gets asked.
#
# release_held is the other answer to WAIT_PLACE_TARGET: no destination at all,
# open where the arm stands. There is deliberately no `place_held` -- set_place
# already is that tool, and two tools emitting the same command is a coin flip
# the model has to win every turn.
MOTION_TOOLS = ("pick_and_place", "pick_and_hold", "set_place", "release_held")

# Actions that end the turn without moving anything.
TERMINAL_TOOLS = ("ask_clarification", "wait")

# Mission-level tools. These change what the supervisor *intends* to do; none of
# them moves the arm by itself, which is why they are not in MOTION_TOOLS.
#
# They exist because "나머지는 테이블로" was previously inexpressible: a mid-task
# utterance threw the whole mission away, so the only thing the model could do
# was restart from zero and re-pick what was already in the basket.
#
# pause_mission is NOT the stop button. The stop keyword is matched in the GUI
# and published straight to the robot without an LLM round-trip -- that path
# must never depend on a model deciding to call a tool.
MISSION_TOOLS = ("modify_mission", "cancel_mission", "pause_mission", "resume_mission")

# How far a mission correction reaches.
APPLY_SCOPES = ("CURRENT_AND_REMAINING", "REMAINING_ONLY")


def _say_argument(description: str) -> dict:
    """Every tool carries the sentence the user hears.

    Making it a required argument rather than hoping for free text alongside
    the call is the difference between the robot explaining itself and the
    robot going silent: a model that answers with a bare function call and no
    message leaves the user watching an arm stop for no stated reason.
    """
    return {
        "type": "object",
        "properties": {"say": {"type": "string", "description": description}},
        "required": ["say"],
        "additionalProperties": False,
    }


# vla-bridge-contract.md #5. table/discard 값 자체는 여기서 막지 않는다 -- 실기
# 검증(teach) 여부는 하드웨어 사정이지 스키마 사정이 아니다. 대신
# vla_pick_bridge_node가 allow_unverified_place(기본 false)로 실행을 막는다:
# 모델이 골라도 되고, 브리지가 실제로 보낼지는 따로 판단한다.
PLACE_VALUES = ("basket", "table", "discard")


def _pick_and_place_argument() -> dict:
    return {
        "type": "object",
        "properties": {
            "object_id": {
                "type": "string",
                "description": "scene의 visible_objects에 있는 id를 그대로 쓴다. 예: apple_17",
            },
            "place": {
                "type": ["string", "null"],
                "enum": list(PLACE_VALUES) + [None],
                "description": (
                    "어디에 놓을지. basket=장바구니, table=작업테이블 지정 자리, "
                    "discard=폐기 자리. table/discard는 사용자가 명시적으로 그 목적지를 "
                    "말했을 때만 골라라 -- 아직 실기에서 검증되지 않아 브리지가 거부할 "
                    "수 있고, 그러면 그 사실을 그대로 설명해라. "
                    "사용자가 목적지를 말하지 않았으면 null로 비워둬라 -- 그러면 로봇이 "
                    "물체를 든 채로 대기하다가 다음 판단에서 어디에 놓을지 물어볼 "
                    "기회가 주어진다(그때 set_place로 알려주면 된다). 짐작으로 basket을 "
                    "채우지 마라."
                ),
            },
            "say": {
                "type": "string",
                "description": "무엇을 왜 집는지 사용자에게 할 한 문장. 그대로 들린다.",
            },
        },
        "required": ["object_id", "place", "say"],
        "additionalProperties": False,
    }


def _set_place_argument() -> dict:
    return {
        "type": "object",
        "properties": {
            "place": {
                "type": "string",
                "enum": list(PLACE_VALUES),
                "description": (
                    "지금 팔이 들고 대기 중인 물체를 어디에 놓을지. basket=장바구니, "
                    "table=작업테이블 지정 자리, discard=폐기 자리."
                ),
            },
            "say": {
                "type": "string",
                "description": "어디에 놓는지 사용자에게 할 한 문장. 그대로 들린다.",
            },
        },
        "required": ["place", "say"],
        "additionalProperties": False,
    }


def _modify_mission_argument() -> dict:
    return {
        "type": "object",
        "properties": {
            "apply_scope": {
                "type": "string",
                "enum": list(APPLY_SCOPES),
                "description": (
                    "정정이 어디까지 미치는지. REMAINING_ONLY=아직 안 나간 것들만 "
                    "(기본이고 안전하다). CURRENT_AND_REMAINING=지금 팔이 들고 "
                    "가는 물체까지 목적지를 바꾼다 -- 사용자가 '지금 그것도'라고 "
                    "분명히 말했을 때만 골라라. 이미 놓은 것은 어느 쪽으로도 "
                    "되돌아가지 않는다."
                ),
            },
            "new_destination": {
                "type": ["string", "null"],
                "enum": list(PLACE_VALUES) + [None],
                "description": "바꿀 목적지. 목적지를 바꾸는 게 아니면 null.",
            },
            "remove_object_ids": {
                "type": "array",
                "items": {"type": "string"},
                "description": "작업에서 뺄 물체 id. 뺄 게 없으면 빈 배열.",
            },
            "add_object_ids": {
                "type": "array",
                "items": {"type": "string"},
                "description": "작업에 더할 물체 id. 더할 게 없으면 빈 배열.",
            },
            "say": {
                "type": "string",
                "description": "무엇을 어떻게 바꿨는지 사용자에게 할 한 문장.",
            },
        },
        "required": ["apply_scope", "new_destination", "remove_object_ids",
                     "add_object_ids", "say"],
        "additionalProperties": False,
    }


TOOLS = [
    {
        "type": "function",
        "name": "pick_and_place",
        "description": (
            "지정한 물체를 집어서 지정한 곳에 놓는다. 사용자가 담으라고/치우라고 명시한 "
            "물체에만 사용한다. 한 번에 하나만 호출할 수 있고, 동작이 끝나면 다시 판단 "
            "기회가 주어진다."
        ),
        "parameters": _pick_and_place_argument(),
        "strict": True,
    },
    {
        "type": "function",
        "name": "pick_and_hold",
        "description": (
            "지정한 물체를 집어서 **들고 대기한다.** 어디에 놓을지는 정하지 않는다. "
            "'사과 집어줘', '그거 들고 있어줘'처럼 사용자가 목적지를 말하지 않았을 때 "
            "이걸 쓴다 -- 목적지를 짐작해서 pick_and_place(place='basket')로 보내지 "
            "마라. 다 들면 로봇이 어디에 놓을지 물어볼 기회가 생기고, 그때 "
            "set_place(놓을 곳을 정함) 또는 release_held(그 자리에 놓음)로 답한다."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "object_id": {
                    "type": "string",
                    "description": "scene의 visible_objects에 있는 id를 그대로 쓴다.",
                },
                "say": {
                    "type": "string",
                    "description": "무엇을 왜 집는지 사용자에게 할 한 문장. 그대로 들린다.",
                },
            },
            "required": ["object_id", "say"],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "release_held",
        "description": (
            "지금 들고 대기 중인 물체를 **그 자리에 그대로 놓는다.** 팔은 움직이지 "
            "않는다. robot_state.status가 'waiting_place'일 때만 호출한다. "
            "'거기 놔', '됐어 내려놔'처럼 사용자가 목적지 없이 놓으라고 했을 때. "
            "어디에 놓을지 말했으면 이게 아니라 set_place다."
        ),
        "parameters": _say_argument("그 자리에 놓는다고 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "set_place",
        "description": (
            "robot_state.status가 'waiting_place'일 때만 호출한다 -- place 없이 보낸 "
            "pick_and_place가 물체를 든 채 목적지를 기다리는 중이라는 뜻이다. 지금 "
            "들고 있는 물체를 어디에 놓을지 정해서 호출한다. object_id는 필요 없다 "
            "-- robot_state.holding에 있는 물체에 그대로 적용된다."
        ),
        "parameters": _set_place_argument(),
        "strict": True,
    },
    {
        "type": "function",
        "name": "modify_mission",
        "description": (
            "진행 중인 여러 물체 작업의 **남은 부분**을 고친다. '나머지는 테이블로', "
            "'컵은 빼고', '저것도 같이' 처럼 작업을 취소하지 않고 바꾸는 말에 쓴다. "
            "이미 끝난 물체는 어떤 경우에도 되돌리지 않는다 -- 그건 새 작업이다. "
            "진행 중인 작업이 없으면 호출하지 마라."
        ),
        "parameters": _modify_mission_argument(),
        "strict": True,
    },
    {
        "type": "function",
        "name": "cancel_mission",
        "description": (
            "여러 물체 작업 전체를 그만둔다. 사용자가 '그만해', '됐어'처럼 남은 작업을 "
            "포기하라고 했을 때. 이미 끝낸 물체는 그대로 두고 남은 것만 버린다. "
            "지금 팔이 움직이고 있다면 cancel_current_action 을 먼저 부른다 -- "
            "이 도구는 팔을 세우지 않는다."
        ),
        "parameters": _say_argument("작업을 그만둔다고 사용자에게 할 한 문장. "
                                    "무엇까지 했는지 같이 말해라."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "pause_mission",
        "description": (
            "다음 물체로 넘어가는 것을 멈추고 사용자를 기다린다. 진행 중인 동작은 "
            "건드리지 않는다. **'멈춰'/'정지'에는 이 도구를 쓰지 마라** -- 그건 "
            "화면에서 바로 로봇으로 가는 별도 경로이고 이 판단을 기다리지 않는다. "
            "여기 쓸 자리는 '잠깐 생각해볼게'처럼 사용자가 다음 물체를 보류하는 경우다."
        ),
        "parameters": _say_argument("왜 기다리는지 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "resume_mission",
        "description": (
            "멈춰 있던 작업을 이어서 한다. 사용자가 '계속해', '진행해', '이어서'라고 "
            "했을 때. 멈춘 시점의 계획을 재사용하지 않고 지금 보이는 장면으로 다시 "
            "계획한다 -- 멈춘 사이에 테이블이 바뀌었을 수 있다."
        ),
        "parameters": _say_argument("이어서 무엇을 할지 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "cancel_current_action",
        "description": (
            "진행 중인 동작을 즉시 중단한다. 사용자가 방금 지시를 철회했거나 "
            "지금 향하고 있는 대상이 잘못됐다고 판단되면 다른 무엇보다 먼저 호출한다."
        ),
        "parameters": _say_argument("왜 멈추는지 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "reset_after_stop",
        "description": (
            "cobot2_ws가 정지 상태(SAFE_STOP)에 멈춰 있을 때만 호출한다 -- robot_state의 "
            "status가 'error'이고 details가 SAFE_STOP/정지 관련일 때가 그 신호다. 성공하면 "
            "사람 승인 없이 팔이 곧바로 HOME 자세로 실제로 움직인다. SAFE_STOP이 아닌데 "
            "호출하면 cobot2_ws가 그냥 거부한다. 사용자가 '리셋해줘'/'다시 시작해줘'처럼 "
            "명시적으로 요청했을 때만 호출하고, 추측으로 먼저 부르지 않는다."
        ),
        "parameters": _say_argument("리셋을 시도한다고 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "ask_clarification",
        "description": (
            "어떤 물체를 말하는지 애매하면 절대 추측하지 말고 이것을 호출해 되묻는다. "
            "후보 물체의 id를 함께 넘기면 화면에 번호가 붙은 사진으로 제시된다."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "question": {
                    "type": "string",
                    "description": "사용자에게 물어볼 한 문장.",
                },
                "object_ids": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": (
                        "헷갈리는 후보들의 id. 사용자가 '1번'이라고 답하면 이 배열의 "
                        "첫 번째를 가리킨다. 후보를 제시할 필요가 없으면 빈 배열."
                    ),
                },
            },
            "required": ["question", "object_ids"],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "wait",
        "description": (
            "지금 할 일이 없다. 사용자가 시킨 일을 다 했거나, 아직 아무 지시도 "
            "받지 않았거나, 다음 지시를 기다려야 할 때 호출한다."
        ),
        "parameters": _say_argument(
            "지금 상황과 무엇을 기다리는지 사용자에게 할 한 문장. "
            "예: '사과 담았어요. 더 필요한 거 있으세요?'"
        ),
        "strict": True,
    },
]

TOOL_NAMES = tuple(tool["name"] for tool in TOOLS)
