"""라벨이 그려진 카메라 화면을 모델에게 같이 보여주기 위한 인코딩.

왜 필요한가
----------
`scene`(JSON)만으로는 **"이거 집어줘"를 절대 풀 수 없다.** 그 발화에는 클래스도
색도 위치도 없다 -- 뜻은 전적으로 사람이 무엇을 가리키고 있느냐에 있고, 그건
화면에만 있다. 좌표를 더 촘촘히 실어 보내도 해결되지 않는 종류의 부족함이라
그림 자체를 보여주는 수밖에 없다.

무엇을 보내는가
-------------
`perception_node`가 이미 내보내는 `/vla/perception/annotated_image`다. 여기
그려진 박스 라벨은 `object_id(class, track_id)` -- **모델이 JSON에서 읽는 id와
같은 문자열이다**(`detector.draw_tracks`). 그래서 모델은 "화살표가 가리키는
박스"를 본 뒤 그 라벨을 그대로 도구 인자에 쓸 수 있다. 그림과 JSON을 잇는 것이
이 라벨 하나뿐이라, 라벨 형식을 바꾸면 이 기능이 조용히 망가진다.

무엇을 보내지 않는가
------------------
**기록에 남기지 않는다.** 대화 기록은 60개까지 쌓이는데 거기에 이미지가 섞이면
매 호출마다 예전 사진들이 전부 다시 올라간다 -- 턴이 늘수록 비용이 제곱으로
는다. `attach_image()`가 기록이 아니라 *호출 직전의 사본*에만 그림을 얹는
이유가 이것이다. 기록에는 텍스트만 남는다.

여러 장을 보낼 때 (2026-08-12)
----------------------------
**시간에 따라 위치가 변하는 물체**("굴러가는 공", "떨어지는 중인 과일")는 한 장
으로 판단할 수 없다 -- 정지 사진에서 "굴러가는 중"과 "멈춰 있음"은 구분되지
않는다. `attach_frames()`가 최근 몇 장을 시간 순서와 함께 얹는다.

장수는 3~4장을 기본으로 잡았다. 2장이면 트래킹 노이즈(박스 떨림)와 실제 이동이
안 갈리고 -- 추세를 보려면 최소 3점이 필요하다 -- 5장 이상은 이미지 토큰이 선형
으로 늘어 왕복이 느려지는데, 사람이 "떨어질 것 같다"고 말하는 수준의 판단에는
그 이상의 시간 해상도가 필요 없다.
"""

from __future__ import annotations

import base64

DATA_URL_PREFIX = "data:image/jpeg;base64,"


def encode_frame(bgr, max_width: int = 640, quality: int = 70) -> str:
    """BGR 프레임 -> data URL. 실패하면 빈 문자열.

    폭을 줄이는 것은 비용보다 **정확도** 때문이다. 640px에서 박스 라벨은 아직
    읽히지만 그 아래로 내려가면 `apple_1`과 `apple_7`이 뭉개진다 -- 그러면
    모델은 못 읽었다고 말하지 않고 그럴듯한 id를 지어낸다.
    """
    import cv2

    if bgr is None or getattr(bgr, "size", 0) == 0:
        return ""

    height, width = bgr.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        bgr = cv2.resize(bgr, (max_width, max(1, int(round(height * scale)))),
                         interpolation=cv2.INTER_AREA)

    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return ""
    return DATA_URL_PREFIX + base64.b64encode(buffer.tobytes()).decode("ascii")


def attach_image(items: list[dict], data_url: str) -> list[dict]:
    """마지막 user 항목에 그림을 얹은 **사본**을 돌려준다.

    원본 `items`는 건드리지 않는다 -- 그것이 대화 기록이고, 기록에 이미지가
    들어가면 다음 호출부터 계속 따라다닌다(모듈 설명 참고).

    얹을 자리가 없으면(마지막이 user가 아니거나 비어 있으면) 그냥 원본을
    돌려준다. 그림을 못 보내는 것은 이번 판단이 조금 덜 아는 것일 뿐이지만,
    엉뚱한 자리에 끼워 넣는 것은 요청 자체를 깨뜨린다.
    """
    return attach_frames(items, [data_url] if data_url else [])


def attach_frames(items: list[dict], frames: list[tuple[float, str]] | list[str]) -> list[dict]:
    """여러 장을 시간 순서와 함께 마지막 user 항목에 얹은 **사본**을 돌려준다.

    `frames`는 `(age_s, data_url)` 쌍의 리스트 -- `age_s`는 "몇 초 전 사진인가".
    호환을 위해 문자열 리스트도 받는데, 그때는 시각 안내 없이 그림만 얹는다.

    시각을 텍스트로 같이 넣는 이유
    ----------------------------
    사진 여러 장을 그냥 나열하면 모델은 **어느 것이 먼저인지 추측한다.** 그리고
    추측이 뒤집히면 "왼쪽으로 굴러가는 중"이 "오른쪽으로 굴러가는 중"이 된다 --
    움직이는 물체를 다루려고 여러 장을 보내는 것이므로 순서가 곧 답의 부호다.
    그래서 각 그림 **앞에** 그 사진이 몇 초 전 것인지 한 줄을 넣고, 마지막에
    "가장 마지막 사진이 현재"라고 못 박는다.

    한 장만 보낼 때는 안내를 넣지 않는다 -- 그 경우 시간 정보는 판단에 쓰이지
    않고 토큰만 쓴다.
    """
    if not frames or not items:
        return items

    last = items[-1]
    if last.get("role") != "user":
        return items

    content = last.get("content")
    if isinstance(content, str):
        parts: list[dict] = [{"type": "input_text", "text": content}]
    elif isinstance(content, list):
        parts = list(content)
    else:
        return items

    # (age, url) 과 url 둘 다 받는다.
    pairs: list[tuple[float | None, str]] = []
    for frame in frames:
        if isinstance(frame, tuple):
            age, url = frame
            pairs.append((float(age), url))
        else:
            pairs.append((None, frame))
    pairs = [(age, url) for age, url in pairs if url]
    if not pairs:
        return items

    multiple = len(pairs) > 1
    if multiple:
        parts.append({
            "type": "input_text",
            "text": (f"아래 사진 {len(pairs)}장은 같은 장면을 시간 순서대로 찍은 것이다"
                     " (오래된 것 -> 최신). 물체가 움직이고 있는지, 어느 방향으로"
                     " 얼마나 빠르게 움직이는지 판단할 때 쓴다."),
        })
    for age, url in pairs:
        if multiple and age is not None:
            label = "지금" if age < 0.05 else f"{age:.1f}초 전"
            parts.append({"type": "input_text", "text": f"[{label}]"})
        parts.append({"type": "input_image", "image_url": url})
    if multiple:
        parts.append({
            "type": "input_text",
            "text": "마지막 사진이 가장 최신이다. 물체의 현재 위치는 그것을 기준으로 판단해라.",
        })

    return items[:-1] + [{**last, "content": parts}]
