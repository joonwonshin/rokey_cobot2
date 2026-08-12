"""segment_from_labels() 의 국소 테이블 기준·돌출 다듬기 테스트 (카메라·GPU 불필요).

2026-08-09: yolo 경로(seg_source=yolo)에 테이블 높이 필터링이 아예 없던 것을 메웠다
(geometric 경로엔 이미 있었다). K·T 를 단순하게(fx=fy, T=identity) 잡아 손으로 검산
가능한 숫자로 만들었다 — 실제 값은 `check_segment.py` 스크래치로 먼저 실측 확인했다.
"""

import re

import numpy as np

from graspgenx_perception.capture_graspgenx_scene import (
    DEFAULTS, LABEL_OBJ_BASE, parse_class_dims, segment_from_labels,
)

H = W = 40
CX = CY = 20.0
FX = FY = 200.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
T_IDENTITY = np.eye(4)   # base = camera 프레임. Z = depth 그대로라 손으로 검산하기 쉽다


def _params(**over):
    p = dict(DEFAULTS)
    p.update(x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0, z_min=0.0, z_max=2.0,
             min_pixels=5, obj_radius_m=0.02, yolo_table_ring_m=0.05,
             yolo_min_ring_px=5, obj_min_h=0.015,
             # mask_erode_m 은 다른 메커니즘(반경/높이 크롭)을 검증하는 테스트와 얽히면
             # 안 되므로 기본으로 끈다 — 전용 테스트(test_mask_erode_*)에서 따로 켠다.
             mask_erode_m=0.0)
    p.update(over)
    return p


def _tilted_table(slope=0.5):
    """base 기준 X 를 따라 기운 테이블면. T=identity 라 depth 로 직접 표현한다.

    캘리브 잔차·실제 테이블 기울기가 있을 때를 흉내낸다 — 전역 스칼라 table_z 하나로는
    이 기울기를 못 따라가는 것이 지금 고치려는 문제다.
    """
    u = np.arange(W)
    uu, _ = np.meshgrid(u, np.arange(H))
    x_nom = (uu - CX) / FX
    return (1.0 + slope * x_nom).astype(np.float32)


def _heights_cm(diag):
    return {int(m.group(1)): float(m.group(2))
            for m in re.finditer(r'obj_(\d+):.*돌출높이=([\d.]+) cm', diag)}


def _table_sources(diag):
    return {int(m.group(1)): m.group(2)
            for m in re.finditer(r'obj_(\d+):.*테이블기준=[\d.]+ m\((\w+)\)', diag)}


def _two_objects_on_tilted_table():
    depth = _tilted_table(slope=0.5)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[17:23, 5:11] = LABEL_OBJ_BASE + 1     # 테이블이 낮은 쪽
    depth[17:23, 5:11] += 0.05
    labels[17:23, 29:35] = LABEL_OBJ_BASE + 2    # 테이블이 높은 쪽
    depth[17:23, 29:35] += 0.05
    return labels, depth


def test_local_table_ref_tracks_tilt():
    """실측: 두 물체 다 실제 돌출 5cm인데 테이블이 기울어 있다 -> 국소 기준이면 둘 다 5cm 근처."""
    labels, depth = _two_objects_on_tilted_table()
    seg, label_map, diag = segment_from_labels(labels, depth, K, T_IDENTITY, _params())
    assert seg is not None
    assert set(label_map) == {'obj_1', 'obj_2'}
    assert _table_sources(diag) == {1: '국소', 2: '국소'}
    heights = _heights_cm(diag)
    # 실측: obj_1=5.0cm, obj_2=6.0cm (링 안 배경 평균이라 완벽히 0은 아니다) — 둘 다 1.5cm 이내
    assert abs(heights[1] - 5.0) < 1.5
    assert abs(heights[2] - 5.0) < 1.5


def test_global_fallback_is_worse_under_tilt():
    """반경 크롭을 끄면(obj_radius_m=nan) 국소 링을 못 만들어 전역 중앙값으로 폴백한다.

    전역 스칼라 하나로는 기울어진 테이블의 양 끝에서 오차가 난다 — 이게 이 개선 전의
    동작(항상 전역, 사실은 필터링 자체가 없었다)과 같은 상황이다.
    """
    labels, depth = _two_objects_on_tilted_table()
    _, _, diag = segment_from_labels(labels, depth, K, T_IDENTITY,
                                     _params(obj_radius_m=float('nan')))
    assert _table_sources(diag) == {1: '전역', 2: '전역'}
    heights = _heights_cm(diag)
    # 실측: obj_1=2.6cm, obj_2=8.6cm (참값 5cm) — 국소 모드보다 뚜렷이 나쁘다
    assert max(abs(heights[1] - 5.0), abs(heights[2] - 5.0)) > 2.0


def test_table_level_bleed_is_trimmed_from_mask():
    """마스크 경계가 테이블로 새면(가림꼬리 등) obj_min_h 미만 픽셀이 seg 에서 잘린다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[17:23, 5:11] = LABEL_OBJ_BASE + 1     # 진짜 물체 (5cm 돌출)
    depth[17:23, 5:11] = 1.05
    labels[23, 5:11] = LABEL_OBJ_BASE + 1        # 같은 라벨인데 테이블 높이 그대로인 꼬리 한 줄
    depth[23, 5:11] = 1.00

    seg, label_map, diag = segment_from_labels(
        labels, depth, K, T_IDENTITY, _params(obj_radius_m=float('nan')))
    assert seg is not None
    assert 'obj_1' in label_map
    assert (seg[17:23, 5:11] == LABEL_OBJ_BASE + 1).all(), '진짜 물체 픽셀은 남아야 한다'
    assert (seg[23, 5:11] == 0).all(), '테이블 높이 꼬리는 잘려나가야 한다'
    assert _heights_cm(diag)[1] == 5.0


def test_ring_falls_back_to_global_when_no_room_for_background():
    """물체가 작업공간 대부분을 채우면 국소 링에 배경이 없어 전역으로 폴백한다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[2:38, 2:38] = LABEL_OBJ_BASE + 1
    depth[2:38, 2:38] = 1.05

    _, _, diag = segment_from_labels(labels, depth, K, T_IDENTITY, _params(obj_radius_m=0.02))
    assert _table_sources(diag).get(1) == '전역'


def test_no_finite_table_ref_when_scene_has_no_background():
    """박스 전체가 물체(배경 0개)면 테이블 기준을 못 잡는다 — 조용히 0 취급하지 않는다."""
    depth = np.full((H, W), 1.05, dtype=np.float32)
    labels = np.full((H, W), LABEL_OBJ_BASE + 1, dtype=np.uint8)

    _, _, diag = segment_from_labels(labels, depth, K, T_IDENTITY,
                                     _params(obj_radius_m=float('nan'), min_pixels=5))
    assert '없음(배경 0개)' in diag


# ── class_dims: 클래스별 실측 치수 ───────────────────────────
def test_parse_class_dims_happy_path():
    dims, warns = parse_class_dims('bottle:0.033:0.20, apple:0.04:0.08')
    assert dims == {'bottle': (0.033, 0.2), 'apple': (0.04, 0.08)}
    assert warns == []


def test_parse_class_dims_bad_entries_are_dropped_not_fatal():
    dims, warns = parse_class_dims(
        'bottle:0.033:0.20,garbage,apple:x:0.08,neg:-0.01:0.1')
    assert dims == {'bottle': (0.033, 0.2)}     # 나머지 셋은 각자 다른 이유로 버려진다
    assert len(warns) == 3


def test_parse_class_dims_empty():
    assert parse_class_dims('') == ({}, [])


def test_known_class_uses_measured_radius_and_trims_contamination():
    """실측 반경으로 크롭 폭을 넓히고, 실측 높이+margin 을 넘는 오염 픽셀을 잘라낸다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[10:16, 15:21] = LABEL_OBJ_BASE + 1
    depth[10:16, 15:21] = 1.20                  # 실제 콜라병: 20cm 돌출
    labels[16, 15:21] = LABEL_OBJ_BASE + 1       # 같은 라벨에 섞인 오염(이웃/팔): 40cm
    depth[16, 15:21] = 1.40

    p = _params(obj_radius_m=0.02, class_dims='bottle:0.10:0.20',   # 전역 기본값은 좁다
                class_dims_margin_m=0.03)
    seg, label_map, diag = segment_from_labels(
        labels, depth, K, T_IDENTITY, p, class_map={LABEL_OBJ_BASE + 1: 'bottle'})

    assert 'obj_1' in label_map
    assert (seg[16, 15:21] == 0).all(), '실측 높이+margin 을 넘는 오염 픽셀은 잘려야 한다'
    assert (seg[10:16, 15:21] == LABEL_OBJ_BASE + 1).all(), '진짜 물체는 남아야 한다'
    assert '실측 bottle=20.0 cm' in diag


def test_unknown_class_has_no_upper_bound():
    """class_dims 에 없는 클래스는 상한 없이(기존 동작) 키 큰 물체도 통째로 남는다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[10:16, 15:21] = LABEL_OBJ_BASE + 1
    depth[10:16, 15:21] = 1.30                  # 30cm — bottle 이었다면 상한(0.23m)에 걸렸을 것

    p = _params(obj_radius_m=0.05, class_dims='bottle:0.10:0.20')
    seg, label_map, _ = segment_from_labels(
        labels, depth, K, T_IDENTITY, p, class_map={LABEL_OBJ_BASE + 1: 'mystery_object'})
    assert 'obj_1' in label_map
    assert (seg[10:16, 15:21] == LABEL_OBJ_BASE + 1).all()


def test_shallow_detection_warns_about_missing_depth():
    """돌출높이가 실측의 절반 미만이면 depth 결손(반사면 등)을 의심하는 경고를 남긴다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[10:16, 15:21] = LABEL_OBJ_BASE + 1
    depth[10:16, 15:21] = 1.05                  # 5cm 밖에 안 잡힘 (실측 20cm 의 절반 미만)

    p = _params(obj_radius_m=0.05, class_dims='bottle:0.10:0.20')
    _, _, diag = segment_from_labels(
        labels, depth, K, T_IDENTITY, p, class_map={LABEL_OBJ_BASE + 1: 'bottle'})
    assert '실측 대비 얕음' in diag


def test_class_dims_defaults_to_off():
    """class_dims 를 안 주면(기본값 '') 전과 동일하게 동작한다 — 회귀 방지."""
    labels, depth = _two_objects_on_tilted_table()
    seg, label_map, diag = segment_from_labels(labels, depth, K, T_IDENTITY, _params())
    assert set(label_map) == {'obj_1', 'obj_2'}
    assert '실측' not in diag


def test_mask_erode_shrinks_object_boundary():
    """mask_erode_m 은 마스크 테두리를 실측 거리만큼 깎는다 — 안쪽만 남는다.

    depth=1.05, fx=200 -> 픽셀 피치 ~5.25mm/px. erode_m=0.005 -> erode_px round(0.95)=1
    -> 6x6 블록의 테두리 1px 링이 지워지고 안쪽 4x4만 남는다.
    """
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[10:16, 15:21] = LABEL_OBJ_BASE + 1
    depth[10:16, 15:21] = 1.05

    p = _params(obj_radius_m=float('nan'), mask_erode_m=0.005)  # 반경 크롭은 끄고 erosion만 본다
    seg, label_map, _ = segment_from_labels(labels, depth, K, T_IDENTITY, p)

    assert 'obj_1' in label_map
    assert (seg[11:15, 16:20] == LABEL_OBJ_BASE + 1).all()   # 안쪽 4x4는 남는다
    assert (seg[10, 15:21] == 0).all()      # 윗변
    assert (seg[15, 15:21] == 0).all()      # 아랫변
    assert (seg[10:16, 15] == 0).all()      # 왼변
    assert (seg[10:16, 20] == 0).all()      # 오른변


def test_mask_erode_zero_is_noop():
    """mask_erode_m=0(기본, _params override)이면 기존 동작과 같다 — 회귀 방지."""
    labels, depth = _two_objects_on_tilted_table()
    seg, label_map, _ = segment_from_labels(
        labels, depth, K, T_IDENTITY, _params(mask_erode_m=0.0))
    assert set(label_map) == {'obj_1', 'obj_2'}


def test_mask_erode_can_remove_small_object_entirely():
    """erode_m 이 물체 반폭보다 크면 min_pixels 밑으로 깎여 라벨 자체가 사라진다."""
    depth = np.full((H, W), 1.0, dtype=np.float32)
    labels = np.zeros((H, W), dtype=np.uint8)
    labels[10:16, 15:21] = LABEL_OBJ_BASE + 1
    depth[10:16, 15:21] = 1.05

    p = _params(obj_radius_m=float('nan'), mask_erode_m=0.02)  # 20mm — 6px(~30mm) 물체를 통째로 깎음
    seg, label_map, _ = segment_from_labels(labels, depth, K, T_IDENTITY, p)
    assert label_map == {}
