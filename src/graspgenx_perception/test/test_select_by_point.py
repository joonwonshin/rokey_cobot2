"""`select_by_point()` / `pixel_to_base()` 순수 함수 테스트 (카메라·GPU 불필요).

설계 출처: md/plans/2026-08-08-vla-integration.md §5.
"""

import numpy as np

from graspgenx_perception.capture_graspgenx_scene import pixel_to_base, select_by_point


def _scene():
    """obj_1 은 (0.30, 0.00) 부근, obj_2 는 (0.50, 0.20) 부근에 있는 4x4 라벨맵."""
    seg = np.zeros((4, 4), dtype=np.uint8)
    seg[0, 0] = 101       # obj_1
    seg[3, 3] = 102       # obj_2
    X = np.zeros((4, 4), dtype=np.float64)
    Y = np.zeros((4, 4), dtype=np.float64)
    X[0, 0], Y[0, 0] = 0.30, 0.00
    X[3, 3], Y[3, 3] = 0.50, 0.20
    label_map = {'obj_1': 101, 'obj_2': 102, 'table': 2}
    return seg, label_map, X, Y


def test_picks_nearest_within_radius():
    seg, label_map, X, Y = _scene()
    out, keep, diag = select_by_point(seg, label_map, X, Y, 0.30, 0.00,
                                      radius=0.06, margin=0.02)
    assert out is not None
    assert (out == 101).sum() == 1
    assert (out == 102).sum() == 0      # obj_2 는 지워졌다
    assert keep == {'obj_1': 101, 'table': 2}   # obj_ 아닌 항목은 안 지운다
    assert 'obj_1' in diag


def test_rejects_when_nothing_in_radius():
    seg, label_map, X, Y = _scene()
    out, keep, diag = select_by_point(seg, label_map, X, Y, tx=0.9, ty=0.9,
                                      radius=0.06, margin=0.02)
    assert out is None and keep is None
    assert '없음' in diag


def test_rejects_when_ambiguous():
    seg, label_map, X, Y = _scene()
    # obj_1(0.30,0.00) 과 obj_2(0.50,0.20) 사이 지점 -> 둘 다 큰 반경 안에 들어오고
    # 거리차가 margin 보다 작아 모호 판정이어야 한다.
    out, keep, diag = select_by_point(seg, label_map, X, Y, tx=0.40, ty=0.10,
                                      radius=0.30, margin=0.20)
    assert out is None and keep is None
    assert '모호' in diag


def test_ambiguity_check_disabled_with_neg_inf_margin():
    """refuse_ambiguous_match=False 는 margin=-inf 로 호출한다 — 항상 1등을 그대로 쓴다."""
    seg, label_map, X, Y = _scene()
    # obj_1(0.30,0.00) 에 더 가깝지만 obj_2(0.50,0.20) 도 radius 안에는 든다.
    out, keep, diag = select_by_point(seg, label_map, X, Y, tx=0.33, ty=0.03,
                                      radius=0.30, margin=float('-inf'))
    assert out is not None
    assert keep == {'obj_1': 101, 'table': 2}


def test_single_candidate_never_ambiguous():
    seg = np.zeros((2, 2), dtype=np.uint8)
    seg[0, 0] = 101
    X = np.zeros((2, 2)); Y = np.zeros((2, 2))
    X[0, 0], Y[0, 0] = 0.30, 0.00
    out, keep, diag = select_by_point(seg, {'obj_1': 101}, X, Y, 0.30, 0.00,
                                      radius=0.06, margin=0.02)
    assert out is not None and keep == {'obj_1': 101}


def test_single_candidate_kept_even_beyond_radius():
    """후보가 1개뿐이면 radius 밖이어도 그 하나를 쓴다 (설계 §9, 단일 후보 short-circuit).

    구멍/물체 이동으로 VLA 픽셀->base 가 radius 만큼 어긋나도, class 가 맞은 유일 후보를
    버리는 것보다 쓰는 게 낫다 — 애매함이 없으니 매칭 로직을 안 태운다.
    """
    seg = np.zeros((2, 2), dtype=np.uint8)
    seg[0, 0] = 101
    X = np.zeros((2, 2)); Y = np.zeros((2, 2))
    X[0, 0], Y[0, 0] = 0.30, 0.00
    # 지정점(0.9,0.9)은 유일 후보(0.30,0.00)에서 radius(0.06) 훨씬 밖 -- 그래도 통과해야 한다.
    out, keep, diag = select_by_point(seg, {'obj_1': 101}, X, Y, tx=0.9, ty=0.9,
                                      radius=0.06, margin=0.02)
    assert out is not None and keep == {'obj_1': 101}
    assert 'obj_1' in diag


def test_rejects_when_no_candidate_at_all():
    """obj_ 후보가 0개면(다른 class 만 남았거나 빈 씬) 당연히 거부."""
    seg = np.zeros((2, 2), dtype=np.uint8)
    X = np.zeros((2, 2)); Y = np.zeros((2, 2))
    out, keep, diag = select_by_point(seg, {'table': 2}, X, Y, 0.30, 0.00,
                                      radius=0.06, margin=0.02)
    assert out is None and keep is None


def test_pixel_to_base_uses_neighborhood_median_for_holes():
    depth = np.full((5, 5), 0.5, dtype=np.float32)
    depth[2, 3] = 0.0        # 지정 픽셀 자체가 구멍 -- 이웃 median(0.5)으로 메워져야 한다
    K = np.array([[500.0, 0, 2], [0, 500.0, 2], [0, 0, 1]])
    T = np.eye(4)
    pt = pixel_to_base(depth, K, T, u=3, v=2, half=2)
    assert pt is not None
    x, y = pt
    # (u-cx)*d/fx = (3-2)*0.5/500 = 0.001, (v-cy)*d/fy = 0
    assert abs(x - 0.001) < 1e-9 and abs(y) < 1e-9


def test_pixel_to_base_none_when_all_holes():
    depth = np.zeros((5, 5), dtype=np.float32)
    K = np.array([[500.0, 0, 2], [0, 500.0, 2], [0, 0, 1]])
    T = np.eye(4)
    assert pixel_to_base(depth, K, T, u=2, v=2, half=2) is None
