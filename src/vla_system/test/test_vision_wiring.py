"""AgentNode가 사진을 언제 싣고 언제 빼는가.

test_vision_payload.py는 인코딩과 얹기를 본다. 여기서 보는 것은 **조건**이다 --
사람이 말했을 때만 싣고, 오래된 화면은 빼고, 파라미터로 끄면 토픽을 아예 안
잡는지. API를 부르지 않는다: LLM을 가짜로 바꿔 무엇이 실려 갔는지만 기록한다.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_interfaces.msg import SceneObject, SceneSnapshot          # noqa: E402
from vla_system.agent.llm import AgentResponse                     # noqa: E402
from vla_system.agent.vision import DATA_URL_PREFIX                # noqa: E402
from vla_system.nodes.agent_node import AgentNode                  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeLLM:
    """부르면 무엇이 실려 왔는지 적어 두고, 도구 없이 한 마디만 답한다."""

    def __init__(self):
        self.images = []
        self.frames = []

    def respond(self, items, image="", frames=None):
        self.images.append(image)
        self.frames.append(list(frames or []))
        return AgentResponse(text="네", calls=[])


def build(vision=True):
    # 생성 시점에 준다. vision_enabled는 구독을 만들지 말지를 정하므로
    # 만들어진 뒤에 바꿔봐야 늦다 -- launch도 같은 경로로 값을 넣는다.
    node = AgentNode(parameter_overrides=[
        rclpy.parameter.Parameter("skill_tier_enabled",
                                  rclpy.Parameter.Type.BOOL, False),
        rclpy.parameter.Parameter("vision_enabled",
                                  rclpy.Parameter.Type.BOOL, vision),
    ])
    node.close()
    llm = FakeLLM()
    node.get_llm = lambda: llm
    node.publish_reply = lambda kind, text, ids=None: None

    snapshot = SceneSnapshot()
    item = SceneObject()
    item.id, item.class_name, item.color = "apple_1", "apple", "red"
    item.track_id, item.position_valid = 1, True
    snapshot.objects.append(item)
    with node.lock:
        node.scene = snapshot
    return node, llm


def give_frame(node, age_seconds=0.0):
    from time import monotonic
    with node.lock:
        node.frame = np.full((480, 640, 3), 180, dtype=np.uint8)
        node.frame_monotonic = monotonic() - age_seconds


def test_what_the_user_said_carries_the_picture():
    node, llm = build()
    give_frame(node)
    node.decide({"type": "user_said", "text": "이거 집어줘"})
    assert llm.images and llm.images[0].startswith(DATA_URL_PREFIX)
    node.destroy_node()


def test_an_action_finishing_does_not():
    """미션 한 단계마다 사진을 사면 여러 개 담을 때 값이 배로 든다.
    끝났다는 통보는 애매할 수가 없으니 사진이 풀 것이 없다."""
    node, llm = build()
    give_frame(node)
    node.decide({"type": "action_finished", "action": "pick_and_place",
                 "result": "succeeded", "detail": ""})
    assert llm.images == [""]
    node.destroy_node()


def test_a_stale_frame_is_dropped_rather_than_sent():
    """카메라가 멎었는데 옛 사진을 보내면 모델은 이미 치워진 물체를 가리킨다."""
    node, llm = build()
    give_frame(node, age_seconds=30.0)
    node.decide({"type": "user_said", "text": "이거 집어줘"})
    assert llm.images == [""]
    node.destroy_node()


def test_with_no_camera_at_all_the_agent_still_answers():
    node, llm = build()
    node.decide({"type": "user_said", "text": "사과 집어줘"})
    assert llm.images == [""], "사진이 없다고 판단을 멈추면 안 된다"
    node.destroy_node()


def subscribed_topics(node) -> list[str]:
    return [subscription.topic_name for subscription in node.subscriptions]


def test_the_parameter_off_means_no_subscription_and_no_picture():
    """끄면 카메라 토픽을 잡지도 않는다 -- 30Hz 프레임을 받아 버리는 값이 없다."""
    node, llm = build(vision=False)
    assert not any("annotated" in name for name in subscribed_topics(node))
    give_frame(node)                      # 있어도 무시돼야 한다
    node.decide({"type": "user_said", "text": "이거 집어줘"})
    assert llm.images == [""]
    node.destroy_node()


def test_the_parameter_on_subscribes_to_the_labelled_frame():
    node, _ = build(vision=True)
    assert any("annotated" in name for name in subscribed_topics(node)), (
        "구독이 없으면 self.frame은 영원히 None이고, 사진 기능 전체가 조용히 죽는다")
    node.destroy_node()


# ------------------------------------- 움직이는 물체: 여러 장 (2026-08-12)

def give_history(node, ages):
    """`ages`(초 전) 만큼 지난 프레임들을 버퍼에 직접 채운다.

    `_remember_frame`을 쓰지 않는 이유: 그쪽은 간격 규칙이 있어서 테스트가
    실시간을 기다려야 한다. 여기서 보려는 것은 간격 규칙이 아니라 **뽑아 쓰는
    쪽의 조건**이다.
    """
    from time import monotonic
    now = monotonic()
    with node.lock:
        node.frame_history = [
            (now - age, np.full((480, 640, 3), 180, dtype=np.uint8))
            for age in sorted(ages, reverse=True)          # 오래된 것부터
        ]
        node.frame = node.frame_history[-1][1]
        node.frame_monotonic = node.frame_history[-1][0]


def test_several_frames_go_out_oldest_first_with_their_ages():
    node, llm = build()
    give_history(node, [1.0, 0.5, 0.0])
    node.decide({"type": "user_said", "text": "굴러가는 거 집어줘"})

    sent = llm.frames[0]
    assert len(sent) == 3
    ages = [age for age, _url in sent]
    assert ages == sorted(ages, reverse=True), "오래된 것부터 나가야 한다"
    assert all(url.startswith(DATA_URL_PREFIX) for _age, url in sent)
    assert llm.images[0] == "", "여러 장을 보낼 때 한 장짜리는 비어야 한다"
    node.destroy_node()


def test_one_frame_in_the_buffer_falls_back_to_the_single_picture():
    """기동 직후. 한 장을 여러 장인 척 보내면 안내만 붙고 정보는 그대로다."""
    node, llm = build()
    give_history(node, [0.0])
    node.decide({"type": "user_said", "text": "이거 집어줘"})
    assert llm.frames[0] == []
    assert llm.images[0].startswith(DATA_URL_PREFIX)
    node.destroy_node()


def test_a_stale_history_is_dropped_whole():
    """가장 최신 것조차 오래됐으면 전부 버린다 -- 한 장짜리와 같은 판단."""
    node, llm = build()
    give_history(node, [31.0, 30.5, 30.0])
    node.decide({"type": "user_said", "text": "굴러가는 거 집어줘"})
    assert llm.frames[0] == []
    assert llm.images[0] == ""
    node.destroy_node()


def test_frame_count_one_turns_the_feature_off():
    """되돌리는 스위치. 이 기능이 잘 안 되면 여기부터 1로 내린다."""
    node, llm = build()
    node.set_parameters([
        rclpy.parameter.Parameter("vision_frame_count",
                                  rclpy.Parameter.Type.INTEGER, 1)])
    give_history(node, [1.0, 0.5, 0.0])
    node.decide({"type": "user_said", "text": "굴러가는 거 집어줘"})
    assert llm.frames[0] == [], "꺼져 있는데 여러 장이 나갔다"
    assert llm.images[0].startswith(DATA_URL_PREFIX), "한 장 경로는 살아 있어야 한다"
    node.destroy_node()


def test_the_buffer_keeps_only_what_it_will_send():
    """카메라는 30Hz다. 매 프레임을 쌓으면 메모리만 먹는다 -- 넣을 때 거른다."""
    from time import monotonic
    node, _ = build()
    frame = np.full((480, 640, 3), 180, dtype=np.uint8)
    count = int(node.get_parameter("vision_frame_count").value)
    interval = float(node.get_parameter("vision_frame_interval_s").value)

    now = monotonic()
    with node.lock:
        for i in range(20):                    # 간격을 지킨 20장
            node._remember_frame(frame, now + i * interval)
    assert len(node.frame_history) == count

    with node.lock:
        before = len(node.frame_history)
        node._remember_frame(frame, now + 20 * interval + interval / 10)
    assert len(node.frame_history) == before, "간격 안에 들어온 프레임은 버린다"
    node.destroy_node()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
