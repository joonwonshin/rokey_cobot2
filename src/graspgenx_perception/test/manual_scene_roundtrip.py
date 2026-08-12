"""2단계: capture_graspgenx_scene.py 가 쓴 디렉토리를 GraspGenX **자체 loader**로 읽는다.

왜 필요한가: 스키마를 소스 읽어서 맞췄을 뿐 실제로 로드해본 적이 없다.
카메라가 오기 전에 포맷 오류를 여기서 잡는다 — 실기 시간에 디버깅하지 않으려고.

실행 (GraspGenX venv 안에서. 이 파일만 rclpy 가 아니라 `graspgenx` 를 import 한다):
    cd ~/cobot2_ws/isaac_ros-dev/src/GraspGenX
    uv run python ~/cobot2_ws/src/graspgenx_perception/test/manual_scene_roundtrip.py \
        /tmp/graspgenx_scene_test/00

`manual_` 접두어가 필수다 — 호스트 시스템 파이썬엔 `graspgenx` 가 없어서 pytest 가
수집하면 `colcon test` 가 통째로 깨진다.
"""
import sys

import numpy as np
from graspgenx.utils.scene_loaders import (
    build_scene_pc_excluding_object,
    collect_scene_items,
    load_realworld_scene,
)

scene_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/graspgenx_scene_test/00'
parent = scene_dir.rsplit('/', 1)[0]

# ① demo_scene_pc.py 가 부모 디렉토리에서 씬을 찾는 경로와 같은 함수
items = collect_scene_items(parent)
print('collect_scene_items ->', items)
assert items and items[0][0] == 'realworld', f'realworld 포맷으로 인식되지 않았다: {items}'

# ② 실제 로드
scene = load_realworld_scene(scene_dir)
print('scene keys :', sorted(k for k in scene if not k.startswith('_')))
print('objects    :', {k: len(v["pc"]) for k, v in scene['objects'].items()})
print('scene_xyz  :', scene['scene_xyz'].shape)
assert set(scene['objects']) == {'obj_1', 'obj_2'}, scene['objects'].keys()

# ③ loader 가 낸 base 좌표 == 우리가 파일에 의도한 좌표인가.
#    기대값을 상수로 박지 않는다(카메라 자세가 바뀌면 바로 낡는다).
#    depth+K+camera_pose 로 여기서 독립적으로 다시 계산해 대조한다.
import json  # noqa: E402
import os  # noqa: E402

from PIL import Image  # noqa: E402

md = json.load(open(os.path.join(scene_dir, 'meta_data.json')))
K = np.asarray(md['intrinsics'], float)
T = np.asarray(md['camera_pose'], float)
depth = np.load(os.path.join(scene_dir, 'depth.npy')).astype(np.float32)
seg2d = np.asarray(Image.open(os.path.join(scene_dir, 'seg.png')), dtype=np.int32)
vv, uu = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
p_cam = np.stack([(uu - K[0, 2]) * depth / K[0, 0],
                  (vv - K[1, 2]) * depth / K[1, 1], depth], -1)
ref = (T[:3, :3] @ p_cam.reshape(-1, 3).T).T + T[:3, 3]     # R @ p + t
ref = ref.reshape(depth.shape + (3,))

for name in ('obj_1', 'obj_2'):
    pc = scene['objects'][name]['pc']
    m = (seg2d == md['label_map'][name]) & (depth > 0)
    want = ref[m]
    print(f'  {name}: loader {len(pc)} pts / 참조 {int(m.sum())} pts, '
          f'중앙값 z {np.median(pc[:, 2]):+.4f} vs {np.median(want[:, 2]):+.4f}')
    assert len(pc) == int(m.sum()), (len(pc), int(m.sum()))
    d = np.abs(np.sort(pc, axis=0) - np.sort(want, axis=0)).max()
    assert d < 1e-4, f'{name} loader 좌표가 참조와 {d:.2e} m 어긋난다'

# ④ 충돌 필터가 쓰는 "대상 제외 씬" — 대상 점이 빠지고 나머지(테이블·다른 물체)는 남아야 한다
full = len(scene['scene_xyz'])
for name in ('obj_1', 'obj_2'):
    rest = build_scene_pc_excluding_object(scene, name)
    removed = full - len(rest)
    print(f'  scene minus {name}: {len(rest)} pts (제거 {removed})')
    assert removed == len(scene['objects'][name]['pc']), (removed, len(scene['objects'][name]['pc']))

# ⑤ 라벨 없는 픽셀(배경 0)도 씬 점군에 남는지 = 장애물이 라벨 없이도 충돌에 잡히는 근거
labeled = sum(len(v['pc']) for v in scene['objects'].values())
print(f'  전체 {full} pts 중 라벨된 물체 {labeled} pts → 나머지 {full - labeled} pts 가 장애물 역할')
assert full - labeled > 0

print('PASS')
