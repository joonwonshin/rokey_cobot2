"""합성 장면(테이블 평면 + 상자 2개)으로 segment()/write_scene() 검증.

실행:  source install/setup.bash
       python3 src/graspgenx_perception/test/manual_capture_scene.py [출력디렉토리]
       (시스템 파이썬. 카메라·로봇·GPU 불필요)

2단계 검증의 1단계다. 2단계는 여기서 쓴 디렉토리를 GraspGenX venv 의 loader 로
실제로 읽어보는 것 — 형제 파일 manual_scene_roundtrip.py 참고.

pytest 가 수집하지 않도록 `manual_` 접두어를 쓴다 — 최상위에서 `sys.argv` 를 읽고
`rclpy.init()` 까지 부르는 스크립트형이라 `colcon test` 에 섞이면 안 된다.
"""
import os
import sys

import numpy as np

from graspgenx_perception.capture_graspgenx_scene import (
    DEFAULTS, quat_to_matrix, segment, to_base, write_scene,
)

# ---- 1. quat_to_matrix: tf_transformations 없이 직접 만든 유일한 함수다 ----
# scipy 와 대조한다. 부호/전치가 틀리면 grasp 가 통째로 엉뚱한 데 뜬다.
from scipy.spatial.transform import Rotation  # noqa: E402

rng = np.random.default_rng(0)
q = rng.normal(size=(200, 4))
q /= np.linalg.norm(q, axis=1, keepdims=True)
err = max(np.abs(quat_to_matrix(*qi) - Rotation.from_quat(qi).as_matrix()).max() for qi in q)
print(f'quat_to_matrix vs scipy: 최대오차 {err:.2e}')
assert err < 1e-12, err
# 정규화되지 않은 쿼터니언도 (s = 2/n 로 흡수) 정상이어야 한다
assert np.allclose(quat_to_matrix(*(3.0 * q[0])), Rotation.from_quat(q[0]).as_matrix())

H, W = 200, 200
K = np.array([[600., 0., 100.], [0., 600., 100.], [0., 0., 1.]])
# ⚠️ 대칭행렬(diag(1,-1,-1) 류)을 쓰면 R.T == R 이라 **전치 실수를 테스트가 못 잡는다.**
# 아래로 보되 요(yaw)·롤을 섞은 비대칭 회전을 쓴다.
T = np.eye(4)
T[:3, :3] = quat_to_matrix(*Rotation.from_euler('xyz', [177., 3., 25.], degrees=True).as_quat())
T[:3, 3] = [0.5, 0., 1.0]
assert not np.allclose(T[:3, :3], T[:3, :3].T), '비대칭 회전이어야 전치 실수를 잡는다'

# ---- 2. to_base: GraspGenX loader 와 같은 식인가 (다른 형태로 써서 대조) ----
_d = np.full((4, 5), 1.2, np.float32)
_d[1, 1] = 0.9
_uv = np.stack(np.meshgrid(np.arange(5), np.arange(4)), -1)
_pc = np.stack([(_uv[..., 0] - K[0, 2]) * _d / K[0, 0],
                (_uv[..., 1] - K[1, 2]) * _d / K[1, 1], _d], -1).reshape(-1, 3)
_ref = (T[:3, :3] @ _pc.T).T + T[:3, 3]            # R @ p + t (명시적으로 다르게 쓴 형태)
assert np.allclose(to_base(_d, K, T).reshape(-1, 3), _ref, atol=1e-6), 'to_base 가 R @ p + t 와 다르다'

# ---- 3. segment(): 세그멘테이션 논리. 변환 정확성은 위 2번이 이미 책임진다 ----
depth = np.full((H, W), 1.00, np.float32)      # 테이블(카메라가 기울어 base 에선 살짝 경사)
depth[:8, :] = 0.0                             # 무효 depth 띠 -> 배경 라벨 0 이 존재해야 한다
depth[100:140, 30:70] = 0.95                   # 물체1: 카메라 쪽으로 5cm
depth[100:140, 120:160] = 0.94                 # 물체2: 6cm

xyz = to_base(depth, K, T)
p = dict(DEFAULTS)
p['table_z'] = float('nan')
p['obj_min_h'] = 0.025                         # 기울기(수 mm)보다 크고 물체(5cm)보다 작게
for k, lo, hi in (('x', 0, 0), ('y', 1, 1), ('z', 2, 2)):
    p[f'{k}_min'] = float(xyz[..., lo].min()) - 0.01
    p[f'{k}_max'] = float(xyz[..., hi].max()) + 0.01

seg, label_map, diag = segment(depth, K, T, p)
print(diag)
u = sorted(np.unique(seg).tolist())
print('unique(seg) =', u)
assert u == [0, 2, 101, 102], f'라벨이 배경0+테이블2+물체2개가 아니다: {u}'
assert label_map == {'ground': 0, 'table': 2, 'obj_1': 101, 'obj_2': 102}, label_map
assert (seg[110:130, 40:60] == 101).all() and (seg[110:130, 130:150] == 102).all()

# 붙어 있는 두 물체는 하나로 뭉쳐야 한다(설계상 한계를 고정한다)
d2 = np.full((H, W), 1.00, np.float32)
d2[:8, :] = 0.0
d2[100:140, 30:160] = 0.95
_, lm2, _ = segment(d2, K, T, dict(p))
assert lm2 == {'ground': 0, 'table': 2, 'obj_1': 101}, lm2

# obj_max_h 로 자른 키 큰 것이 table 라벨로 새지 않아야 한다
d3 = depth.copy()
d3[40:70, 40:70] = 0.60                        # 40cm 높이 = 로봇 팔 흉내
p3 = dict(p)
p3['obj_max_h'] = 0.12
seg3, lm3, _ = segment(d3, K, T, p3)
assert lm3 == {'ground': 0, 'table': 2, 'obj_1': 101, 'obj_2': 102}, lm3
assert (seg3[45:65, 45:65] == 0).all(), '키 큰 것이 table(2) 로 새고 있다'

# 파일 4개를 실제로 쓴다 — 2단계에서 GraspGenX loader 가 이 디렉토리를 읽는다
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/graspgenx_scene_test/00'
rgb = np.zeros((H, W, 3), np.uint8)
rgb[..., 0] = 200                              # 빨간 톤 — RGB/BGR 뒤바뀜을 눈으로 잡으려고
write_scene(out, depth, rgb, seg, K, T, label_map,
            [p['x_min'], p['y_min'], p['z_min'], p['x_max'], p['y_max'], p['z_max']])
for f in ('depth.npy', 'rgb.png', 'seg.png', 'meta_data.json'):
    assert os.path.exists(os.path.join(out, f)), f
print(f'wrote {out}')

# ROS 파라미터 타입 강제 — `-p scene:=00`(YAML이 정수 0으로 파싱)이 죽지 않아야 한다
import rclpy  # noqa: E402
from graspgenx_perception.capture_graspgenx_scene import SceneCapture  # noqa: E402

rclpy.init(args=['test', '--ros-args',
                 '-p', 'scene:=00',        # INTEGER 0 으로 들어온다 -> '00'
                 '-p', 'obj_min_h:=0',     # INTEGER 0 이 DOUBLE 자리에 -> 0.0
                 '-p', 'use_tf:=false'])
node = SceneCapture()
rp = node.params()
print('params: scene=%r obj_min_h=%r use_tf=%r' % (rp['scene'], rp['obj_min_h'], rp['use_tf']))
assert rp['scene'] == '00', rp['scene']
assert isinstance(rp['obj_min_h'], float) and rp['obj_min_h'] == 0.0, rp['obj_min_h']
assert rp['use_tf'] is False
assert isinstance(rp['min_pixels'], int) and isinstance(rp['depth_topic'], str)

# depth 단위: 16UC1 은 mm 라 /1000, 32FC1 은 이미 m.
# 이 나눗셈을 빠뜨리면 로봇이 1.6 km 밖을 향하는데 세그멘테이션 테스트로는 안 잡힌다.
from cv_bridge import CvBridge  # noqa: E402

_br = CvBridge()
node.depths = []
node._on_depth(_br.cv2_to_imgmsg(np.full((2, 2), 1234, np.uint16), encoding='16UC1'))
assert abs(node.depths[-1][0, 0] - 1.234) < 1e-6, node.depths[-1][0, 0]
node._on_depth(_br.cv2_to_imgmsg(np.full((2, 2), 1.234, np.float32), encoding='32FC1'))
assert abs(node.depths[-1][0, 0] - 1.234) < 1e-6, node.depths[-1][0, 0]

# 시간축 중앙값: 소수 프레임의 튀는 값은 지워지고, 대부분 비어 있던 픽셀은 0 으로 남아야 한다
frames = [np.full((2, 2), 1.0, np.float32) for _ in range(10)]
frames[0][0, 0] = 5.0            # 이상치 1회 -> 중앙값이 무시
frames[1][0, 0] = 5.0            # 이상치 2회 -> 여전히 무시
for f in frames[:8]:
    f[1, 1] = 0.0                # 10프레임 중 2회만 유효 -> min_valid_ratio 0.5 미달
node.depths = frames
merged, used = node.merged_depth(10, 0.5)
print('merged:', merged.tolist(), 'used=', used)
assert used == 10
assert abs(merged[0, 0] - 1.0) < 1e-6, merged[0, 0]   # 이상치 제거
assert merged[1, 1] == 0.0, merged[1, 1]              # 유효 프레임 부족 -> 버림
assert abs(merged[0, 1] - 1.0) < 1e-6

node.destroy_node()
rclpy.shutdown()

print('PASS')
