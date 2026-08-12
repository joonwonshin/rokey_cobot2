"""grasp_bridge_node 의 순수 함수 검증 (카메라·GPU·로봇 불필요).

    source install/setup.bash
    python3 src/graspgenx_perception/test/manual_grasp_bridge.py

pytest 가 수집하지 않도록 `manual_` 접두어를 쓴다 — 최상위에서 assert 를 돌리는
스크립트형이라 `colcon test` 에 섞이면 안 된다 (형제 파일 manual_roundtrip.py 와 같은 규칙).
"""
import numpy as np
from scipy.spatial.transform import Rotation

from graspgenx_perception.grasp_bridge_node import EXTRA_DEFAULTS, pose_msg, select

# ---- 1. pose_msg: 4x4 -> quaternion. 부호 하나가 grasp 자세를 뒤집는다 ----
rng = np.random.default_rng(1)
worst = 0.0
for _ in range(300):
    R = Rotation.random(random_state=int(rng.integers(1 << 30)))
    T = np.eye(4)
    T[:3, :3] = R.as_matrix()
    T[:3, 3] = rng.normal(size=3)
    m = pose_msg(T)
    q = np.array([m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])
    # q 와 -q 는 같은 회전이다 — 행렬로 되돌려 비교한다
    worst = max(worst, np.abs(Rotation.from_quat(q).as_matrix() - T[:3, :3]).max())
    assert np.allclose([m.position.x, m.position.y, m.position.z], T[:3, 3])
print(f'pose_msg vs scipy: 최대오차 {worst:.2e}')
assert worst < 1e-9, worst
# 회전 없음 -> 항등 쿼터니언
m0 = pose_msg(np.eye(4))
assert np.allclose([m0.orientation.x, m0.orientation.y, m0.orientation.z, m0.orientation.w],
                   [0, 0, 0, 1]), m0.orientation

# ---- 2. select(): 도달·접근축·점수 필터가 실제로 거르는가 ----
p = dict(EXTRA_DEFAULTS)
p['min_score'] = 0.5
p['max_reach_m'] = 0.900
p['max_approach_z'] = -0.3
p['target'] = ''


def grasp(t, approach_down=True, roll=0.0):
    """approach_down 이면 grasp 의 +Z 가 아래를 향한다(R[2,2] = -1)."""
    T = np.eye(4)
    base = Rotation.from_euler('x', 180.0 if approach_down else 0.0, degrees=True)
    T[:3, :3] = (base * Rotation.from_euler('z', roll, degrees=True)).as_matrix()
    T[:3, 3] = t
    return T


good = grasp([0.5, 0.0, 0.1])
far = grasp([1.2, 0.0, 0.1])                      # 도달 초과
up = grasp([0.5, 0.0, 0.1], approach_down=False)  # 접근축이 위
objects = {
    'obj_1': {'grasps': [good.tolist(), far.tolist(), up.tolist()],
              'scores': [0.6, 0.99, 0.99], 'widths': [0.041, 0.088, 0.077], 'n_pts': 500},
}
label, T, score, width, g_ok, s_ok, w_ok, diag = select(objects, p)
print(diag)
assert label == 'obj_1' and score == 0.6, (label, score)
assert len(g_ok) == 1, f'도달 초과/접근축 위 가 살아남았다: {len(g_ok)}'
assert np.allclose(T[:3, 3], [0.5, 0.0, 0.1])
# 폭은 **고른 grasp 의** 것이어야 한다. 필터에 걸린 후보(0.088/0.077)의 폭이 새어나오면
# 그리퍼가 엉뚱한 폭으로 닫는다 — 포즈는 맞는데 안 잡히는, 가장 헷갈리는 실패다.
assert width == 0.041, width
assert list(w_ok) == [0.041], list(w_ok)

# 점수 미달만 있는 경우 -> 선택 없음
low = {'obj_1': {'grasps': [good.tolist()], 'scores': [0.2], 'n_pts': 500}}
assert select(low, p)[0] is None

# 후보 0개 -> 선택 없음(예외 아님)
empty = {'obj_1': {'grasps': [], 'scores': [], 'widths': [], 'n_pts': 500}}
assert select(empty, p)[0] is None

# 구버전 워커(widths 키 없음) -> 폭 0.0 = "모름". 소비자가 기본값으로 대체한다.
# 여기서 그럴듯한 값을 지어내면 그리퍼가 조용히 헛닫힌다.
now = {'obj_1': {'grasps': [good.tolist()], 'scores': [0.6], 'n_pts': 500}}
assert select(now, p)[3] == 0.0, select(now, p)[3]

# widths 길이가 grasps 와 안 맞는 경우 -> 0.0("모름")으로 낮추고 계속 돈다(죽지 않는다).
# `select()` 는 모듈 함수라 `self.get_logger()` 를 쓸 수 없다 — logger 인자가 없으면
# NameError 로 죽었던 버그(2026-08-09)가 재발하면 여기서 걸린다.
mismatched = {'obj_1': {'grasps': [good.tolist()], 'scores': [0.6],
                        'widths': [0.05, 0.06], 'n_pts': 500}}
r = select(mismatched, p)
assert r[0] == 'obj_1' and r[3] == 0.0, r

# 여러 물체 중 점수 최고를 고른다
two = {
    'obj_1': {'grasps': [good.tolist()], 'scores': [0.6], 'n_pts': 500},
    'obj_2': {'grasps': [grasp([0.6, 0.1, 0.1]).tolist()], 'scores': [0.8], 'n_pts': 500},
}
assert select(two, p)[0] == 'obj_2', select(two, p)[0]

# target 을 주면 그 물체만
p2 = dict(p)
p2['target'] = 'obj_1'
assert select(two, p2)[0] == 'obj_1'

print('PASS')
