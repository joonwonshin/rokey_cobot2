"""Tier 1 rule layer, driven by a fake host and a table-driven parser.

No ROS and no network. The parser is a lookup table so a failure here is a
failure of the decision logic, never of the LLM's mood -- the two are worth
separating because only one of them is ours to fix.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_system.agent.rules import RuleStore                      # noqa: E402
from vla_system.agent.skill_tier import SceneItem, SkillTier      # noqa: E402


class FakeHost:
    def __init__(self, items):
        self.items = list(items)
        self.said, self.asked, self.picked, self.escalated = [], [], [], []
        self.notes, self.records, self.turns = [], [], 0

    def say(self, text): self.said.append(text)
    def ask(self, text): self.asked.append(text)
    def escalate(self, reason, text, mission_text): self.escalated.append(reason)
    def note(self, event): self.notes.append(event)
    def record(self, tier, detail=""): self.records.append((tier, detail))
    def turn_done(self): self.turns += 1
    def scene_items(self): return list(self.items)

    def pick(self, object_id, reason):
        self.picked.append(object_id)
        # A picked object leaves the table, exactly as the real scene reports.
        self.items = [i for i in self.items if i.object_id != object_id]


def parsed(intent="pick", classes=(), colors=(), ex=(), exc=(), rep=(),
           quantity="all", count=None, scope="now", reason="", destination=None):
    return {"intent": intent, "classes": list(classes), "colors": list(colors),
            "exclude_classes": list(ex), "exclude_colors": list(exc),
            "replaces": list(rep), "quantity": quantity, "count": count,
            "scope": scope, "reason": reason, "destination": destination}


def build(items, table, store=None):
    host = FakeHost(items)
    tier = SkillTier(host, store or RuleStore(),
                     lambda t: table.get(t, parsed(intent="other")))
    return host, tier


TABLE_SCENE = [
    SceneItem("cup_1", "cup", "white", rank=1),
    SceneItem("banana_2", "banana", "yellow", rank=2),
    SceneItem("apple_3", "apple", "red", rank=3),
]


def drive(tier, host, text):
    """One utterance, then let the mission run to completion."""
    tier.handle(text)
    for _ in range(10):
        if not tier.busy:
            break
        tier.on_action_finished()


# --------------------------------------------------------------- exclusion

def test_exclusion_is_not_inverted():
    """The failure this field exists for: "빼고" taken as "take"."""
    host, tier = build(TABLE_SCENE, {
        "바나나랑 컵 빼고 다 담아줘": parsed(ex=["banana", "cup"], quantity="all")})
    drive(tier, host, "바나나랑 컵 빼고 다 담아줘")
    assert host.picked == ["apple_3"]


def test_exclusion_by_colour():
    scene = [SceneItem("g", "apple", "green", rank=1),
             SceneItem("r", "apple", "red", rank=2)]
    host, tier = build(scene, {"초록 사과 빼고 사과 다 담아줘":
                               parsed(classes=["apple"], exc=["green"])})
    drive(tier, host, "초록 사과 빼고 사과 다 담아줘")
    assert host.picked == ["r"]


# ------------------------------------------------- prohibition vs the user

def test_prohibition_filters_broad_orders_silently():
    store = RuleStore()
    host, tier = build(TABLE_SCENE, {
        "컵은 깨지기 쉬우니까 앞으로 담지 마":
            parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
        "여기 있는 거 다 담아줘": parsed(quantity="all")}, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "여기 있는 거 다 담아줘")
    assert "cup_1" not in host.picked
    assert host.asked == []          # a broad order needs no confirmation


def test_named_prohibition_asks_then_complies():
    store = RuleStore()
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "컵 가져와": parsed(classes=["cup"], quantity="one")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "컵 가져와")
    assert host.asked, "naming a forbidden class must prompt, not silently swap"
    assert host.picked == []
    drive(tier, host, "응 가져와")
    assert host.picked == ["cup_1"]


def test_exception_does_not_erase_the_rule():
    store = RuleStore()
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "컵 가져와": parsed(classes=["cup"], quantity="one"),
             "여기 있는 거 다 담아줘": parsed(quantity="all")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "컵 가져와")
    drive(tier, host, "응 가져와")
    assert store.is_forbidden("cup", "white"), "one exception is not a retraction"


# ------------------------------------------------------------- corrections

def test_correction_overwrites_instead_of_piling_on():
    store = RuleStore()
    scene = [SceneItem("cup_red", "cup", "red", rank=1),
             SceneItem("cup_white", "cup", "white", rank=2),
             SceneItem("apple", "apple", "red", rank=3)]
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "아니 빨간 컵만 담지 마":
                 parsed("correct", ["cup"], ["red"], rep=["cup"], reason="깨지기 쉬워서"),
             "여기 있는 거 다 담아줘": parsed(quantity="all")}
    host, tier = build(scene, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "아니 빨간 컵만 담지 마")
    drive(tier, host, "여기 있는 거 다 담아줘")
    assert "cup_white" in host.picked, "the narrowed rule must free the white cup"
    assert "cup_red" not in host.picked


def test_correction_without_a_new_target_never_deletes():
    """The worst shape: old rule gone, nothing in its place, user told it worked."""
    store = RuleStore()
    table = {"컵은 담지 마": parsed("prohibit", ["cup"], reason="깨져서"),
             "아니 그게 아니라": parsed("correct", [], rep=["cup"])}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 담지 마")
    drive(tier, host, "아니 그게 아니라")
    assert store.is_forbidden("cup", "white"), "must not delete before escalating"
    assert host.escalated, "an unexpressible correction goes upstairs"


# ------------------------------------------------------------------ expiry

def test_stated_deadline_outranks_a_good_reason():
    store = RuleStore()
    host, tier = build(TABLE_SCENE, {
        "오늘은 컵이 젖어 있으니까 담지 마":
            parsed("prohibit", ["cup"], reason="젖어 있어서", scope="today")}, store)
    drive(tier, host, "오늘은 컵이 젖어 있으니까 담지 마")
    assert store.is_forbidden("cup", "white")
    store.end_session()
    assert not store.is_forbidden("cup", "white"), "'오늘은' must not outlive the session"


def test_yes_inside_a_dated_answer_is_not_a_standing_rule():
    """"오늘만 그렇게 해줘" reads as agreement and as a limit. The limit wins."""
    store = RuleStore()
    table = {"당분간 컵은 담지 마": parsed("prohibit", ["cup"], scope="ask"),
             "오늘만 그렇게 해줘": parsed("scope_answer")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "당분간 컵은 담지 마")
    assert host.asked, "a vague duration must be asked about"
    drive(tier, host, "오늘만 그렇게 해줘")
    assert store.is_forbidden("cup", "white")
    store.end_session()
    assert not store.is_forbidden("cup", "white")


# ------------------------------------------------------------------ safety

def test_hazard_needs_confirmation_and_forgets_it():
    scene = [SceneItem("s1", "scissors", "black", rank=1)]
    store = RuleStore()
    table = {"가위 가져와": parsed(classes=["scissors"], quantity="one")}
    host, tier = build(scene, table, store)
    drive(tier, host, "가위 가져와")
    assert host.asked and host.picked == []
    drive(tier, host, "응 가져와")
    assert host.picked == ["s1"]

    # A new session must ask again -- the acknowledgement is not stored.
    host2, tier2 = build(scene, table, store)
    drive(tier2, host2, "가위 가져와")
    assert host2.asked and host2.picked == []


def test_ambiguous_single_pick_escalates_rather_than_guessing():
    scene = [SceneItem("a1", "apple", "red", rank=1),
             SceneItem("a2", "apple", "red", rank=2)]
    host, tier = build(scene, {"사과 하나 가져와": parsed(classes=["apple"], quantity="one")})
    drive(tier, host, "사과 하나 가져와")
    assert host.picked == []
    assert "ambiguous" in host.escalated


def test_a_stale_scene_does_not_cause_a_second_pick():
    """The executor and the camera run on different clocks. A just-taken object
    is often still in the newest snapshot, and trusting it picked the same
    apple twice on a real ROS graph."""
    scene = [SceneItem("a1", "apple", "red", rank=1),
             SceneItem("a2", "apple", "red", rank=2)]
    host, tier = build(scene, {"사과 다 담아줘": parsed(classes=["apple"], quantity="all")})
    host.pick = lambda oid, reason: host.picked.append(oid)   # scene never updates
    tier.handle("사과 다 담아줘")
    for _ in range(6):
        if not tier.busy:
            break
        tier.on_action_finished()
    assert host.picked == ["a1", "a2"], f"each object once, got {host.picked}"


def test_mid_mission_utterance_goes_upstairs():
    host, tier = build(TABLE_SCENE, {"다 담아줘": parsed(quantity="all")})
    tier.handle("다 담아줘")
    tier.handle("아니 그건 말고")
    assert "mission_interrupted" in host.escalated


def test_mid_mission_utterance_pauses_instead_of_discarding():
    """The whole reason the supervisor exists. Before it, escalating threw the
    mission away, so Tier 2 could only restart from zero -- and re-pick what was
    already in the basket. What was done has to survive the interruption."""
    scene = [SceneItem("a1", "apple", "red", rank=1),
             SceneItem("a2", "apple", "red", rank=2)]
    host, tier = build(scene, {"사과 다 담아줘": parsed(classes=["apple"], quantity="all")})
    tier.handle("사과 다 담아줘")
    tier.on_action_finished()                 # a1 done, a2 dispatched
    tier.handle("아니 나머지는 테이블로")

    state = tier.supervisor.state
    assert state is not None, "mission was discarded, not paused"
    assert state.status == "PAUSED"
    assert state.completed_ids == ["a1"], f"completed lost: {state.completed_ids}"


# ------------------------------------------------------------- destination

def test_destination_reaches_the_mission():
    host, tier = build(TABLE_SCENE, {
        "사과 테이블에 올려줘": parsed(classes=["apple"], quantity="all",
                                       destination="table")})
    tier.handle("사과 테이블에 올려줘")
    assert tier.supervisor.state.destination == "table"


def test_no_destination_means_hold_not_basket():
    """A silent default to basket takes away the moment where the user gets
    asked. Empty is what makes the FSM park in WAIT_PLACE_TARGET."""
    host, tier = build(TABLE_SCENE, {
        "사과 담아줘": parsed(classes=["apple"], quantity="all")})
    tier.handle("사과 담아줘")
    assert tier.supervisor.state.destination == ""


def test_destination_is_the_only_new_rule_slot():
    """§6-C: every field added to RULE_SCHEMA is another thing the parser can
    fill in by guesswork. This pins the slot count so growing it is a decision
    somebody makes on purpose, not a drift."""
    from vla_system.agent.skill_tier import RULE_SCHEMA           # noqa: PLC0415

    assert set(RULE_SCHEMA["required"]) == {
        "intent", "classes", "colors", "exclude_classes", "exclude_colors",
        "replaces", "quantity", "count", "scope", "reason", "destination",
    }


# ------------------------------------------------- 판단형 수식어는 Tier 2 로

def test_rule_layer_announces_before_the_arm_moves():
    """말없이 움직이면 사용자는 "안 되는구나" 하고 같은 말을 다시 한다 (2026-08-12 실기).

    규칙 계층은 LLM 을 안 거쳐 빠른 게 장점인데, 그게 **아무 반응 없음**으로 보이면
    장점이 아니다. 되묻기 때는 원래 말하고 있었고, 정작 팔이 움직이는 경로만 조용했다.
    """
    host, tier = build(TABLE_SCENE, {
        "사과 담아줘": parsed(classes=["apple"], quantity="all")})
    tier.handle("사과 담아줘")
    assert host.picked, "집으러 가지도 않았다"
    assert host.said, "팔이 움직이는데 사용자에게 한 마디도 안 했다"


def test_the_prompt_sends_look_and_judge_phrases_upstairs():
    """🔴 2026-08-12 실기: "테이블에서 떨어질 것 같은 과일 집어줘" 가 그냥 "과일"이 되어
    엉뚱한 걸 집었다. LLM 턴이 0건 -- Tier 1 이 혼자 처리했고, Tier 1 은 사진을 안 본다.

    손가락 지목("이거")이 실기에서 잘 되는 것과 **같은 경로**여야 한다: 사진을 봐야 아는
    조건은 전부 Tier 2 로 넘어가야 한다. 규칙으로 판정 가능한 건 클래스·색·개수뿐이다.

    프롬프트 텍스트를 검사하는 이유: 파서는 LLM 이라 여기서 호출할 수 없고(테스트는
    표 기반 가짜 파서를 쓴다), 그 지시가 프롬프트에서 사라지는 것이 실제 회귀 경로다.
    """
    from vla_system.agent.skill_tier import RULE_PROMPT              # noqa: PLC0415

    assert "보고 판단해야 아는 수식어" in RULE_PROMPT
    for phrase in ("떨어질 것 같은", "제일 큰", "가장 가까운", "익은"):
        assert phrase in RULE_PROMPT, f"판단형 수식어 예시가 빠졌다: {phrase}"
    # 반대 방향도 지켜야 한다 -- 색·개수까지 Tier 2 로 보내면 규칙 계층이 무의미해진다.
    assert "빨간 사과 집어줘" in RULE_PROMPT


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
