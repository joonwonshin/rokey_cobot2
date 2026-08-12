"""라벨 사진을 모델에게 얹는 부분.

여기서 지키려는 것은 둘이다. **사진이 대화 기록에 들어가지 않을 것**(들어가면
지난 사진이 매 호출마다 따라 올라가 비용이 턴 수에 제곱으로 는다), 그리고
**못 보낼 상황에서 조용히 빠질 것**(카메라가 죽었는데 옛 사진을 보내면 모델은
없어진 물체를 자신 있게 가리킨다).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_system.agent.vision import (                                # noqa: E402
    DATA_URL_PREFIX,
    attach_frames,
    attach_image,
    encode_frame,
)


def frame(width=640, height=480):
    return np.full((height, width, 3), 200, dtype=np.uint8)


# --------------------------------------------------------------- encoding

def test_a_frame_becomes_a_data_url():
    url = encode_frame(frame())
    assert url.startswith(DATA_URL_PREFIX)
    assert len(url) > len(DATA_URL_PREFIX)


def test_a_wide_frame_is_narrowed_but_a_small_one_is_left_alone():
    """폭을 줄이는 것은 비용보다 라벨 가독성 때문이다 -- 뭉개지면 모델이 id를 지어낸다."""
    import cv2

    big = encode_frame(frame(1920, 1080), max_width=640)
    decoded = cv2.imdecode(
        np.frombuffer(__import__("base64").b64decode(big[len(DATA_URL_PREFIX):]),
                      dtype=np.uint8),
        cv2.IMREAD_COLOR)
    assert decoded.shape[1] == 640
    assert decoded.shape[0] == 360, "가로세로비가 유지돼야 한다"

    small = encode_frame(frame(320, 240), max_width=640)
    decoded = cv2.imdecode(
        np.frombuffer(__import__("base64").b64decode(small[len(DATA_URL_PREFIX):]),
                      dtype=np.uint8),
        cv2.IMREAD_COLOR)
    assert decoded.shape[1] == 320, "작은 프레임을 억지로 키우지 않는다"


def test_nothing_in_means_nothing_out():
    assert encode_frame(None) == ""
    assert encode_frame(np.zeros((0, 0, 3), dtype=np.uint8)) == ""


# ---------------------------------------------------------------- attaching

def test_the_picture_never_enters_the_conversation():
    """이 검정이 비용을 지킨다. attach_image는 사본에만 얹는다."""
    items = [{"role": "user", "content": "첫 턴"},
             {"role": "assistant", "content": "네"},
             {"role": "user", "content": "이거 집어줘"}]
    original = [dict(item) for item in items]

    sent = attach_image(items, "data:image/jpeg;base64,AAAA")

    assert items == original, "기록이 변형됐다 -- 다음 호출부터 사진이 따라다닌다"
    assert sent is not items
    assert isinstance(sent[-1]["content"], list)
    assert sent[-1]["content"][0] == {"type": "input_text", "text": "이거 집어줘"}
    assert sent[-1]["content"][1]["type"] == "input_image"


def test_only_the_newest_turn_carries_a_picture():
    items = [{"role": "user", "content": "첫 턴"},
             {"role": "user", "content": "둘째 턴"}]
    sent = attach_image(items, "data:image/jpeg;base64,AAAA")
    assert sent[0]["content"] == "첫 턴", "지난 턴은 텍스트 그대로여야 한다"


def test_no_picture_is_a_no_op():
    items = [{"role": "user", "content": "사과 집어줘"}]
    assert attach_image(items, "") is items


def test_a_non_user_tail_is_left_alone():
    """도구 결과 뒤에 사진을 끼워 넣으면 요청 자체가 깨진다. 덜 아는 편이 낫다."""
    items = [{"role": "user", "content": "사과 집어줘"},
             {"type": "function_call_output", "call_id": "x", "output": "{}"}]
    assert attach_image(items, "data:image/jpeg;base64,AAAA") is items


# ------------------------------------------- 여러 장 (움직이는 물체, 2026-08-12)

URL_A = "data:image/jpeg;base64,AAAA"
URL_B = "data:image/jpeg;base64,BBBB"
URL_C = "data:image/jpeg;base64,CCCC"


def _texts(parts):
    return [p["text"] for p in parts if p.get("type") == "input_text"]


def _images(parts):
    return [p["image_url"] for p in parts if p.get("type") == "input_image"]


def test_frames_keep_their_order_because_order_is_the_answer():
    """순서가 뒤집히면 "왼쪽으로 굴러간다"가 "오른쪽으로 굴러간다"가 된다.
    움직이는 물체를 보려고 여러 장을 보내는 것이므로 순서가 곧 답의 부호다."""
    items = [{"role": "user", "content": "굴러가는 거 집어줘"}]
    sent = attach_frames(items, [(1.0, URL_A), (0.5, URL_B), (0.0, URL_C)])
    assert _images(sent[-1]["content"]) == [URL_A, URL_B, URL_C]


def test_each_frame_is_labelled_with_how_long_ago_it_was():
    """시각 표시가 없으면 모델은 어느 것이 먼저인지 **추측한다**."""
    items = [{"role": "user", "content": "굴러가는 거 집어줘"}]
    sent = attach_frames(items, [(1.0, URL_A), (0.5, URL_B), (0.0, URL_C)])
    texts = " ".join(_texts(sent[-1]["content"]))
    assert "1.0초 전" in texts
    assert "0.5초 전" in texts
    assert "지금" in texts
    assert "마지막 사진이 가장 최신" in texts


def test_a_single_frame_carries_no_time_narration():
    """한 장일 때 시간 안내는 판단에 안 쓰이고 토큰만 쓴다."""
    items = [{"role": "user", "content": "이거 집어줘"}]
    sent = attach_frames(items, [(0.0, URL_A)])
    assert _images(sent[-1]["content"]) == [URL_A]
    assert _texts(sent[-1]["content"]) == ["이거 집어줘"], "안내 문구가 붙었다"


def test_frames_never_enter_the_conversation_either():
    """한 장짜리와 같은 계약 -- 기록에 들어가면 매 호출마다 따라 올라간다."""
    items = [{"role": "user", "content": "굴러가는 거 집어줘"}]
    original = [dict(item) for item in items]
    attach_frames(items, [(1.0, URL_A), (0.0, URL_B)])
    assert items == original


def test_attach_image_still_works_through_the_new_path():
    """기존 한 장 경로는 그대로여야 한다 -- 이 기능을 되돌릴 때 여기가 기준이다."""
    items = [{"role": "user", "content": "이거 집어줘"}]
    sent = attach_image(items, URL_A)
    assert sent[-1]["content"][0] == {"type": "input_text", "text": "이거 집어줘"}
    assert sent[-1]["content"][1] == {"type": "input_image", "image_url": URL_A}
    assert attach_image(items, "") is items


def test_empty_urls_are_dropped_not_sent_as_holes():
    """인코딩 실패한 프레임은 빈 문자열로 온다. 그대로 실으면 요청이 깨진다."""
    items = [{"role": "user", "content": "굴러가는 거"}]
    sent = attach_frames(items, [(1.0, ""), (0.0, URL_A)])
    assert _images(sent[-1]["content"]) == [URL_A]
    assert attach_frames(items, [(1.0, ""), (0.0, "")]) is items


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
