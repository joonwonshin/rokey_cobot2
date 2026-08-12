#!/usr/bin/env python3
"""all-zeros 자세에서 cuRobo가 자기충돌이라고 보는 **구 쌍**을 이름으로 찍는다.

게이트 E에서 cuMotion이 계획 10회 모두 `INVALID_START_STATE_SELF_COLLISION`으로 실패했다.
"어디가 겹치는지"를 모르면 XRDF를 손댈 수 없다 — 반지름을 줄일지, ignore 쌍을 더할지가
갈린다. 이 스크립트는 XRDF 구를 base_link 기준으로 펴서 링크쌍별 최대 침투량을 낸다.

컨테이너 안에서:
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    python3 /workspaces/cobot2_ws/scripts/diag_self_collision.py

기본 자세는 all-zeros다. `--q 0.5 -0.4 1.2 0 0.9 0`처럼 주면 다른 자세도 본다.
"""

import argparse
from collections import defaultdict

import numpy as np
import torch
import yaml

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType

XRDF = '/workspaces/isaac_ros-dev/m0609/m0609_rg2.xrdf'
URDF = '/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf'


def ignore_pairs(xrdf_path):
    """XRDF self_collision.ignore 를 정렬된 frozenset 쌍 집합으로."""
    spec = yaml.safe_load(open(xrdf_path))
    pairs = set()
    for link, others in spec['self_collision']['ignore'].items():
        for other in others:
            pairs.add(frozenset((link, other)))
    return pairs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xrdf', default=XRDF)
    parser.add_argument('--urdf', default=URDF)
    parser.add_argument('--q', type=float, nargs='*', default=None,
                        help='관절값(rad). 생략하면 all-zeros')
    parser.add_argument('--top', type=int, default=20, help='침투 큰 순으로 몇 쌍까지 볼지')
    args = parser.parse_args()

    tad = TensorDeviceType()
    cfg = CudaRobotModelConfig.from_robot_yaml_file(args.xrdf, urdf_path=args.urdf,
                                                    tensor_args=tad)
    model = CudaRobotModel(cfg)
    dof = model.get_dof()

    q = torch.zeros((1, dof), device=tad.device, dtype=tad.dtype)
    if args.q:
        assert len(args.q) == dof, f'--q 는 {dof}개여야 한다'
        q[0] = torch.tensor(args.q, device=tad.device, dtype=tad.dtype)

    spheres = model.get_robot_as_spheres(q)[0]
    centers = np.array([s.position for s in spheres])
    radii = np.array([s.radius for s in spheres])

    # 구 인덱스 → 링크 이름. link_sphere_idx_map은 링크 인덱스를 구마다 하나씩 담는다.
    kc = cfg.kinematics_config
    sphere_link_idx = kc.link_sphere_idx_map.cpu().numpy().ravel()
    idx_to_link = {v: k for k, v in kc.link_name_to_idx_map.items()}
    sphere_link = [idx_to_link.get(int(i), f'link#{int(i)}') for i in sphere_link_idx]

    ignored = ignore_pairs(args.xrdf)

    # 링크쌍별 최대 침투량(= r_i + r_j - d). 양수면 겹친 것이다.
    worst = defaultdict(lambda: (-1e9, None))
    n = len(spheres)
    for i in range(n):
        for j in range(i + 1, n):
            li, lj = sphere_link[i], sphere_link[j]
            if li == lj:
                continue
            d = float(np.linalg.norm(centers[i] - centers[j]))
            penetration = radii[i] + radii[j] - d
            key = frozenset((li, lj))
            if penetration > worst[key][0]:
                worst[key] = (penetration, (i, j, d))

    colliding = [(k, v) for k, v in worst.items() if v[0] > 0.0]
    colliding.sort(key=lambda kv: -kv[1][0])

    print(f'자세 q = {q.cpu().numpy().ravel()}')
    print(f'구 {n}개 · 링크 {len(set(sphere_link))}개 · XRDF ignore 쌍 {len(ignored)}개')
    print('=' * 92)
    print(f'{"링크쌍":54s} {"침투(mm)":>10s}  {"ignore?":>8s}')
    print('-' * 92)

    unignored = 0
    for key, (penetration, detail) in colliding[:args.top]:
        a, b = sorted(key)
        is_ignored = key in ignored
        if not is_ignored:
            unignored += 1
        print(f'{a + " ↔ " + b:54s} {penetration * 1000:10.1f}  '
              f'{"무시됨" if is_ignored else "🔴 검사됨":>8s}')

    print('-' * 92)
    total_unignored = sum(1 for k, v in colliding if k not in ignored)
    print(f'겹치는 링크쌍 {len(colliding)}개 중 ignore에 없는 것 {total_unignored}개')
    if total_unignored:
        print('→ 이 쌍들이 INVALID_START_STATE_SELF_COLLISION의 원인이다.')
        print('  인접 링크라 원래 겹치는 게 정상이면 XRDF self_collision.ignore에 추가하고,')
        print('  떨어져 있어야 할 링크가 겹쳤다면 구 반지름/중심이 틀린 것이다.')
    else:
        print('→ 이 자세에서는 검사 대상 쌍 중 겹치는 것이 없다. 원인은 다른 데 있다.')


if __name__ == '__main__':
    main()
