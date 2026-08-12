"""build_label_map 순수 함수 테스트.

ultralytics 는 도커 컨테이너에만 있으므로 노드 모듈이 호스트에서도 import 되어야 한다
(yolo_seg_node 가 ultralytics import 를 _load_model 안으로 미룬 이유). 이 테스트가
호스트에서 통과한다는 것 자체가 그 지연 import 가 유지되고 있다는 회귀 검사다.
"""

import os

import numpy as np
import pytest

from graspgenx_perception import yolo_seg_node
from graspgenx_perception.yolo_seg_node import (
    LABEL_OBJ_BASE, MAX_OBJECTS, AlreadyRunning, acquire_singleton, build_label_map,
    lock_path,
)


def test_empty_masks_gives_zero_label_map():
    labels, kept = build_label_map(None, 480, 640)
    assert labels.shape == (480, 640)
    assert labels.dtype == np.uint8
    assert not labels.any()

    assert not kept

    assert not build_label_map(np.empty((0, 480, 640), dtype=np.float32), 480, 640)[0].any()


def test_labels_start_at_101_and_increment():
    masks = np.zeros((2, 10, 10), dtype=np.float32)
    masks[0, 0:3, 0:3] = 1.0
    masks[1, 5:8, 5:8] = 1.0
    labels, kept = build_label_map(masks, 10, 10)
    assert kept == [0, 1]
    assert labels[1, 1] == LABEL_OBJ_BASE + 1
    assert labels[6, 6] == LABEL_OBJ_BASE + 2
    assert labels[9, 0] == 0


def test_threshold_is_half():
    masks = np.full((1, 4, 4), 0.5, dtype=np.float32)
    assert not build_label_map(masks, 4, 4)[0].any(), '0.5 는 임계 초과가 아니다'
    masks[:] = 0.51
    assert build_label_map(masks, 4, 4)[0].all()


def test_higher_confidence_wins_on_overlap():
    """입력은 신뢰도 내림차순이다 — 앞선 인스턴스가 위를 덮어야 한다."""
    masks = np.zeros((2, 4, 4), dtype=np.float32)
    masks[0, 0, 0] = 1.0      # 고신뢰, 작음
    masks[1] = 1.0            # 저신뢰, 전체
    labels, _ = build_label_map(masks, 4, 4)
    assert labels[0, 0] == LABEL_OBJ_BASE + 1, '고신뢰가 최상위 z-order'
    assert labels[3, 3] == LABEL_OBJ_BASE + 2


def test_fully_covered_instance_is_dropped_and_labels_stay_contiguous():
    """완전히 덮인 인스턴스를 남기면 라벨이 비어(101,103) 'obj_N=N번째' 가정이 깨진다."""
    masks = np.zeros((3, 4, 4), dtype=np.float32)
    masks[0] = 1.0            # 전체를 덮는 고신뢰
    masks[1, 0, 0] = 1.0      # 완전히 가려짐 -> 버려야 한다
    masks[2] = 0.0
    masks[2, :, :] = 0.0
    labels, kept = build_label_map(masks, 4, 4)
    assert kept == [0], '버려진 인스턴스는 kept 에서도 빠져야 클래스가 밀리지 않는다'
    assert sorted(np.unique(labels).tolist()) == [LABEL_OBJ_BASE + 1]


def test_min_pixels_drops_small_instances():
    masks = np.zeros((2, 10, 10), dtype=np.float32)
    masks[0, 0:5, 0:5] = 1.0   # 25픽셀
    masks[1, 9, 9] = 1.0       # 1픽셀
    labels, kept = build_label_map(masks, 10, 10, min_pixels=10)
    assert kept == [0]
    assert sorted(np.unique(labels).tolist()) == [0, LABEL_OBJ_BASE + 1]


def test_max_objects_truncates():
    masks = np.zeros((5, 4, 4), dtype=np.float32)
    for i in range(5):
        masks[i, i % 4, :] = 1.0
    labels, _ = build_label_map(masks, 4, 4, max_objects=2)
    assert len(np.unique(labels)) - 1 <= 2, '3번째부터는 칠하지 않는다'


def test_max_objects_is_clamped_to_uint8_safe_range():
    """상한을 파라미터로 키워도 uint8 랩어라운드(100+156 -> 0)로 못 넘어간다."""
    masks = np.zeros((1, 4, 4), dtype=np.float32)
    masks[0, 0, 0] = 1.0
    assert build_label_map(masks, 4, 4, max_objects=10_000)[0].max() <= 255
    assert LABEL_OBJ_BASE + MAX_OBJECTS <= 255


def test_shape_mismatch_raises_with_actionable_message():
    """retina_masks=True 가 빠지면 마스크가 모델 해상도로 나온다 — 조용히 넘어가면 안 된다."""
    masks = np.ones((1, 480, 640), dtype=np.float32)
    with pytest.raises(ValueError, match='retina_masks'):
        build_label_map(masks, 194, 259)


def test_max_objects_matches_capture_script():
    """형제 모듈 capture_graspgenx_scene.py 와 같은 상한이어야 라벨 규약이 일치한다."""
    import pathlib
    import re
    src = (pathlib.Path(__file__).resolve().parents[1]
           / 'graspgenx_perception' / 'capture_graspgenx_scene.py')
    if not src.exists():
        pytest.skip('capture_graspgenx_scene.py 가 없다')
    m = re.search(r'^MAX_OBJECTS\s*=\s*(\d+)', src.read_text(), re.M)
    assert m and int(m.group(1)) == MAX_OBJECTS


# --- 중복 인스턴스 방지 락 (acquire_singleton) ---------------------------------
# 배경: `docker exec` 에 --sig-proxy 가 없어 Ctrl-C 가 컨테이너 안까지 가지 않고,
# 재실행마다 yolo_seg_node 가 1개씩 쌓여 같은 토픽에 10중 발행까지 갔다 (2026-08-08).

def test_lock_path_is_keyed_by_topic_not_node_name():
    """키가 토픽이어야 한다. 노드 이름이면 `__node:=other` 가 락을 우회해 2중 발행한다."""
    assert lock_path('/yolo_seg/mask') == lock_path('/yolo_seg/mask')
    assert lock_path('/yolo_seg/mask') != lock_path('/cam2/mask')


def test_lock_path_contains_uid():
    """/tmp 는 sticky 라 남이 만든 파일을 못 연다 — uid 가 없으면 영구 차단이 생긴다."""
    assert f'-{os.getuid()}.lock' in lock_path('/yolo_seg/mask')


def test_second_acquire_on_same_topic_is_blocked(tmp_path, monkeypatch):
    monkeypatch.setattr(yolo_seg_node, 'LOCK_DIR', str(tmp_path))
    fd = acquire_singleton('/yolo_seg/mask')
    try:
        with pytest.raises(AlreadyRunning) as exc:
            acquire_singleton('/yolo_seg/mask')
        # 메시지가 실행 가능한 안내를 담아야 한다 — 빈 PID 는 `kill -INT ?` 가 된다.
        assert str(os.getpid()) in str(exc.value)
        assert 'kill -INT' in str(exc.value)
    finally:
        os.close(fd)


def test_different_topics_coexist(tmp_path, monkeypatch):
    """카메라 2대를 서로 다른 mask_topic 으로 돌리는 구성은 막지 않는다."""
    monkeypatch.setattr(yolo_seg_node, 'LOCK_DIR', str(tmp_path))
    a = acquire_singleton('/yolo_seg/mask')
    b = acquire_singleton('/cam2/mask')
    os.close(a)
    os.close(b)


def test_lock_is_released_when_holder_dies(tmp_path, monkeypatch):
    """flock 은 SIGKILL 에도 커널이 푼다 — stale 락이 남으면 안 된다."""
    monkeypatch.setattr(yolo_seg_node, 'LOCK_DIR', str(tmp_path))
    fd = acquire_singleton('/yolo_seg/mask')
    os.close(fd)                                   # 프로세스 사망과 같은 효과
    os.close(acquire_singleton('/yolo_seg/mask'))  # 다시 잡혀야 한다
