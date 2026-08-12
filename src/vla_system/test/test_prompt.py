"""SYSTEM_PROMPT (Tier 2) 텍스트 검정.

프롬프트 텍스트를 직접 검사하는 이유: 이 지시들은 LLM 안에서만 효력이 있고
여기서 모델을 호출할 수는 없다. 실제 회귀는 "그 문장이 프롬프트에서 조용히
사라지는 것"이므로, 지금 할 수 있는 최선은 문구가 붙어 있는지 고정하는 것이다.
"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_system.agent.prompt import SYSTEM_PROMPT           # noqa: E402


def test_judgement_phrases_are_told_to_decide_not_ask():
    """🔴 2026-08-12 실기: "테이블에서 떨어질랑 말랑하는 과일 집어줘"에 사진까지
    받고도 "오렌지로 할까요, 사과로 할까요?"라고 되물었다.

    Tier 1 프롬프트(skill_tier.RULE_PROMPT)는 이런 판단형 수식어를 이미 other로
    Tier 2에 넘기도록 고쳐져 있었다 -- 그런데 Tier 2 프롬프트에는 "넘어오면 네가
    사진으로 판단하라"는 지시가 없어서, 모델이 "같은 종류가 여럿이라 애매함"
    규칙(#2)을 적용해 판단을 포기하고 후보를 나열했다. 반쪽만 고친 회귀다.
    """
    assert "판단해야" in SYSTEM_PROMPT
    for phrase in ("떨어질 것 같은", "제일 큰", "가장 가까운", "익은"):
        assert phrase in SYSTEM_PROMPT, f"판단형 수식어 예시가 빠졌다: {phrase}"
    assert "네가 할 일을 사용자에게 떠넘기는" in SYSTEM_PROMPT


def test_ambiguity_rule_does_not_override_a_stated_judgement_condition():
    """규칙 #2("같은 종류 여럿이면 되물어라")가 지시에 이미 있는 판단 기준을
    무시하고 종류만으로 되묻는 근거가 되면 안 된다는 예외가 명시돼 있는지."""
    normalized = " ".join(SYSTEM_PROMPT.split())
    assert "판단 기준" in normalized
    assert "종류만으로 되묻으라는 뜻이 아니다" in normalized


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
