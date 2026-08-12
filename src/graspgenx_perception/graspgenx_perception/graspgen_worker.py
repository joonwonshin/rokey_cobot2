#!/usr/bin/env python3
"""GraspGenX GPU 워커 — 모델을 한 번만 올려놓고 stdin 으로 씬을 받는다.

실행 (반드시 GraspGenX venv 안에서. 보통은 grasp_bridge_node 가 자식 프로세스로 띄운다):
    cd ~/cobot2_ws_new/isaac_ros-dev/src/GraspGenX
    uv run python \
      ~/cobot2_ws_new/src/graspgenx_perception/graspgenx_perception/graspgen_worker.py \
      --gripper onrobot_RG2

이 파일은 패키지 디렉토리에 있지만 **rclpy 를 import 하지 않는다.** 형제 모듈이 아니라
`grasp_bridge_node` 가 경로로 실행하는 별도 프로세스다(`worker_script` 파라미터).

프로토콜 (한 줄 = 한 요청):
    stdin  <- 씬 디렉토리 경로 한 줄
    stdout -> JSON 한 줄  {"ok":true,"objects":{"obj_1":{"grasps":[[4x4]...],
                                                 "scores":[...],"widths":[...],"n_pts":N}}}
             실패하면 {"ok":false,"error":"..."}
    "widths"[i] = grasps[i] 의 닫힘축에서 잰 **물체 폭 [m]** (grasp 와 1:1). grasp_widths() 참고.
    ⚠️ 모델 로딩이 끝나면 stdout 에 {"ok":true,"ready":true} 한 줄을 먼저 뱉는다.
       진행 로그는 전부 **stderr** 로 간다 — stdout 은 JSON 전용이다.

왜 ZMQ 가 아닌가: 서버 모드는 `uv sync --extra serve`(pyzmq·msgpack·msgpack-numpy)를,
클라이언트 쪽은 시스템 파이썬에 같은 것을 요구한다. 양끝을 우리가 다 만드는 지금은
파이프 한 줄이면 충분하고 **새 의존성이 0개다**. GPU 를 다른 PC 로 뺄 때 ZMQ 로 갈아탄다
(그때 바뀌는 건 이 파일과 노드의 전송 계층뿐이고, 씬 포맷·필터는 그대로다).

grasp 는 **씬의 camera_pose 가 적용된 프레임**(= 우리 캡처에선 base_link)으로 나온다.
scene_loaders.load_realworld_scene:86 이 점을 world 로 보내기 때문이다.
"""

import argparse
import json
import os
import sys
import traceback

# ⚠️ import 보다 먼저 해야 한다. GraspGenX 와 그 의존성들이 stdout 으로 print 하는데
#    (체크포인트 경로, 그리퍼 로딩, OpenGL 초기화 …) 그러면 JSON 파서가 첫 줄에서 죽는다.
#    stdout 은 우리 JSON 전용으로 잠그고 나머지는 전부 stderr 로 보낸다.
_STDOUT = sys.stdout
sys.stdout = sys.stderr

import numpy as np  # noqa: E402
import trimesh  # noqa: E402

from graspgenx import get_checkpoints_version_dir  # noqa: E402
from graspgenx.grasp_server import GraspGenXSampler  # noqa: E402
from graspgenx.samplers import run_planner_on_batch  # noqa: E402
from graspgenx.utils.checkpoint_io import load_model_cfg  # noqa: E402
from graspgenx.utils.collision_filter import filter_colliding_grasps  # noqa: E402
from graspgenx.utils.scene_loaders import (  # noqa: E402
    build_scene_pc_excluding_object,
    load_realworld_scene,
)


#: 그리퍼가 닫히는 축은 grasp 프레임의 **+X** 다(+Z 는 접근축이다 — 여기 쓰면 폭 대신 물체
#: 높이를 잰다). 근거 둘: `onrobot_RG2/config.json` sweep_volume.extents = [0.102, 0.02, 0.028]
#: 로 개구 폭 0.102 가 **X 성분**이고, `md/context/constraints.md:1041` 이 같은 규약을
#: `robot.py:59` 기준으로 못박았다. ComputeGrasp.srv:19 도 같은 말이다.
GRASP_CLOSING_AXIS = 0


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def grasp_widths(obj_pc, grasps, lo=2.0, hi=98.0):
    """물체 점군을 각 grasp 의 닫힘축(+X)에 투영한 폭 [m]. `len(grasps)` 개, grasp 와 1:1.

    **왜 OBB `half_extent` 를 안 쓰는가**: `_compute_obb`(graspmoe.py:150-159)는 world Z 축
    요 회전만 한다 — `half_extent` 는 [수평 최소면적 사각형 변A, 변B, 높이]이고 그 축은
    grasp 의 닫힘축과 정렬돼 있지 않다. 게다가 점 <10개·OBB 계산 실패 경로에서 `obb=None`
    이고(graspmoe.py:383, 616, 785), diffusion 브랜치 grasp 는 자유 자세라 물체당 하나뿐인
    OBB 로 대표할 수 없다. 투영은 그 셋을 전부 피하고 브랜치 종류와도 무관하다.

    퍼센타일 2/98 은 상류 `_obb_from_angle` 과 같은 값이다 — depth 튀는 점 몇 개가 폭을
    부풀리면 그리퍼가 물체를 놓친다. min/max 로 바꾸지 말 것.

    ⚠️ 이 값은 **보이는 면**의 폭이다. 단일 시점 depth 라 점군이 카메라 쪽 껍질뿐이고,
       닫힘축이 카메라 시선과 나란하면 뒷면이 없어 **실제보다 좁게** 나온다(→ 과도하게 조임).
       또 캡처의 `obj_radius_m`(기본 0.05) 크롭이 측정 가능 폭을 그 2배에서 잘라낸다.
    """
    g = np.asarray(grasps, dtype=float).reshape(-1, 4, 4)
    pc = np.asarray(obj_pc, dtype=float).reshape(-1, 3)
    if len(g) == 0 or len(pc) == 0:
        return np.zeros(len(g), dtype=float)
    # (N,3) @ (3,M) -> (N,M). 원점 t 는 뺄 필요가 없다 — 폭은 투영값의 **차이**라 상쇄된다.
    proj = pc @ g[:, :3, GRASP_CLOSING_AXIS].T
    lo_v, hi_v = np.percentile(proj, [lo, hi], axis=0)
    return np.maximum(0.0, hi_v - lo_v)


def emit(obj):
    print(json.dumps(obj), file=_STDOUT, flush=True)


#: `demo_scene_pc.py:292` 와 같은 값. 안 맞추면 로봇 팔·바닥 밖 튄 점까지 뷰어에 실려
#: 카메라가 자동으로 멀리 줌아웃되고, 정작 물체는 화면 구석 점 몇 개로 찌그러진다.
LIVE_VIZ_BOUNDS = ([-1.5, -1.25, -0.15], [1.5, 1.25, 2.0])


def push_live_scene(vis, scene):
    """캡처 프레임(전체 배경 점군)을 뷰어에 그린다. `demo_scene_pc.py:285-298` 와 동일 규약
    (같은 네임스페이스 `pc_scene`, 같은 point size) — 그 데모를 본 사람에게 낯설지 않다.

    매 compute() 마다 **`vis.scene.reset()`으로 이전 프레임을 통째로 지운다.** 안 지우면
    이전 캡처의 물체·grasp 가 유령처럼 겹쳐 남는다 — "지금 이 순간의 캡처"만 보여주는 게
    라이브 피드의 요점이다(여러 프레임을 겹쳐 보고 싶으면 이 함수를 쓰면 안 된다).
    """
    vis.scene.reset()
    xyz, rgb = scene['scene_xyz'], scene['scene_rgb']
    lo, hi = LIVE_VIZ_BOUNDS
    m = np.all((xyz > lo) & (xyz < hi), axis=1)
    from graspgenx.utils.viser_utils import visualize_pointcloud
    visualize_pointcloud(vis, 'pc_scene', xyz[m], rgb[m], size=0.0025)


def push_live_object(vis, label, obj_pc, obj_rgb, grasps, conf, gripper):
    """물체 하나(점군 + 살아남은 grasp 후보 + 1등 콜리전 메시)를 뷰어에 그린다.

    `demo_scene_pc.py:visualize_grasps_for_object` 의 축약판이다 — diff/obb 서브네임스페이스,
    OBB 와이어프레임, top-5 메시는 뺐다(라이브 피드는 매 요청마다 다시 그리므로 무거우면
    끊긴다). 점수 색 규약(빨강=낮음, 초록=높음, 1등은 파란 굵은선)은 그대로 유지해
    한 번 본 사람이 바로 읽을 수 있게 했다.
    """
    from graspgenx.utils.viser_utils import (
        get_color_from_score, visualize_mesh, visualize_pointcloud, visualize_x_grasp,
    )
    ns = f'obj/{label}'
    visualize_pointcloud(vis, f'{ns}/pc', obj_pc, obj_rgb, size=0.004)
    if len(grasps) == 0:
        return
    colors = get_color_from_score(conf, use_255_scale=True)
    best = int(np.argmax(conf))
    for j, g in enumerate(grasps):
        color = [0, 100, 255] if j == best else colors[j]
        lw = 5.0 if j == best else 1.5
        visualize_x_grasp(vis, f'{ns}/pred_grasps/grasp_{j:03d}', g,
                          color=color, gripper_info=gripper, linewidth=lw)
    visualize_mesh(vis, f'{ns}/top_grasp_mesh', gripper.collision_mesh,
                   color=[0, 100, 255], transform=grasps[best])


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gripper', default='onrobot_RG2')
    ap.add_argument('--checkpoints', default=None)
    ap.add_argument('--assets_dir', default=None)
    ap.add_argument('--planner', default='graspmoe', choices=['diffusion', 'graspmoe'])
    ap.add_argument('--num_grasps', type=int, default=64)      # 8GB VRAM 기준
    ap.add_argument('--grasp_threshold', type=float, default=0.5)
    # 상판 위 물건은 옆에서 못 잡는다 — 옆면 후보를 만들면 73%가 테이블과 충돌한다(2026-08-05 실측)
    ap.add_argument('--moe_obb_density', default='dense',
                    choices=['sparse', 'dense', 'dense-topandside'])
    ap.add_argument('--collision_threshold', type=float, default=0.02)
    ap.add_argument('--max_scene_points', type=int, default=8192)
    ap.add_argument('--num_collision_samples', type=int, default=2000)
    ap.add_argument('--min_obj_points', type=int, default=100)
    # `scripts/demo_scene_pc.py` 와 같은 viser 뷰어를 매 compute() 마다 자동 갱신한다.
    # 그쪽은 모델을 매번 새로 로드해서 씬 하나 보는 데도 모델 로딩(수십 초)이 걸리는데,
    # 이 워커는 이미 뜬 채로 매 요청을 처리하므로 그 비용이 0 이다 — 발표·디버깅용 실시간 피드.
    ap.add_argument('--live_viz', action='store_true')
    ap.add_argument('--live_viz_port', type=int, default=8080)
    return ap.parse_args()


def main():
    args = parse_args()
    repo_root = os.environ.get('GRASPGENX_ROOT') or os.getcwd()
    ckpt = args.checkpoints or str(get_checkpoints_version_dir())
    log(f'[worker] checkpoints: {ckpt}')
    cfg = load_model_cfg(os.path.join(ckpt, 'gen'), os.path.join(ckpt, 'dis'), None, None)

    sampler = GraspGenXSampler(
        cfg, args.gripper,
        assets_dir=args.assets_dir or os.path.join(repo_root, 'assets'))
    gripper = sampler.get_gripper_info()
    surf, _ = trimesh.sample.sample_surface(
        gripper.collision_mesh, args.num_collision_samples)
    surf = np.asarray(surf, dtype=np.float32)
    log(f'[worker] gripper={args.gripper} 준비 완료')

    vis = None
    if args.live_viz:
        # 뷰어가 안 뜨거나 포트가 이미 쓰이고 있어도 grasp 연산 자체는 계속 돼야 한다 —
        # 발표/디버깅 부가기능이 본 기능을 막으면 본말전도다.
        try:
            from graspgenx.utils.viser_utils import create_visualizer
            vis = create_visualizer(port=args.live_viz_port)
        except Exception as e:                                   # noqa: BLE001
            log(f'[worker] live_viz 기동 실패({type(e).__name__}: {e}) — 시각화 없이 계속한다')

    emit({'ok': True, 'ready': True})

    for line in sys.stdin:
        scene_dir = line.strip()
        if not scene_dir:
            continue
        try:
            emit(handle(scene_dir, sampler, surf, args, gripper=gripper, vis=vis))
        except Exception as e:                                   # noqa: BLE001
            # traceback 을 버리면 "ValueError: zero-size array ..." 만 남아 어느 줄인지
            # 못 찾는다(2026-08-09에 실제로 반나절 날렸다). stderr 로만 보낸다 —
            # stdout 은 JSON 전용이라 여기에 쓰면 노드 쪽 파서가 죽는다.
            log(f'[worker] 실패: {type(e).__name__}: {e}\n'
                + ''.join(traceback.format_exc()))
            emit({'ok': False, 'error': f'{type(e).__name__}: {e}'})


def handle(scene_dir, sampler, gripper_surface_points, args, gripper=None, vis=None):
    scene = load_realworld_scene(scene_dir, min_obj_points=args.min_obj_points)
    labels = list(scene['objects'].keys())
    if vis is not None:
        # planner 가 수 초~수십 초 걸리는 동안 발표 화면이 멈춰 보이면 안 된다 — 배경 점군은
        # 먼저 그려서 "지금 이 프레임을 보고 있다" 는 걸 바로 보여준다(demo_scene_pc.py 순서와 같다).
        try:
            push_live_scene(vis, scene)
        except Exception as e:                                   # noqa: BLE001
            log(f'[worker] live_viz 씬 렌더 실패({type(e).__name__}: {e}) — 계속한다')
    if not labels:
        return {'ok': True, 'objects': {}, 'note': 'segmented object 없음'}

    results = run_planner_on_batch(
        [scene['objects'][k]['pc'] for k in labels], sampler,
        planner=args.planner,
        grasp_threshold=args.grasp_threshold,
        num_grasps=args.num_grasps,
        topk_num_grasps=-1,
        moe_obb_density=args.moe_obb_density,
        # planner 기본값은 (-8,-6,-4,-2,-1,0) 인데 demo_scene_pc.py 는 "-2,0" 을 쓴다.
        # 2026-08-05 에 68 grasp 를 얻은 건 demo 쪽 값이라 그걸 따른다.
        moe_z_offsets_cm=(-2.0, 0.0),
    )

    out = {}
    for label, (grasps, conf, tags, _obb) in zip(labels, results):
        if len(grasps) == 0:
            # 후보가 0개여도 물체 점군은 그린다 — "세그는 됐는데 grasp 가 안 나온다" 도
            # 디버깅 정보다. 여기서 빠지면 화면에서 물체가 통째로 사라져 세그 자체가
            # 실패한 것과 구분이 안 된다.
            if vis is not None:
                try:
                    push_live_object(vis, label, scene['objects'][label]['pc'],
                                     scene['objects'][label]['rgb'],
                                     np.zeros((0, 4, 4)), np.zeros(0), gripper)
                except Exception as e:                           # noqa: BLE001
                    log(f'[worker] live_viz {label} 렌더 실패({type(e).__name__}: {e}) — 계속한다')
            out[label] = {'grasps': [], 'scores': [], 'widths': [],
                          'n_pts': len(scene['objects'][label]['pc'])}
            continue
        scene_pc = build_scene_pc_excluding_object(scene, label)
        if len(scene_pc) > args.max_scene_points:
            idx = np.random.choice(len(scene_pc), args.max_scene_points, replace=False)
            scene_pc = scene_pc[idx]
        keep = filter_colliding_grasps(
            scene_pc=scene_pc, grasp_poses=grasps,
            collision_threshold=args.collision_threshold,
            gripper_surface_points=gripper_surface_points)
        grasps, conf = grasps[keep], conf[keep]
        tags = [t for t, k in zip(tags, keep) if k]
        # 충돌 필터 **뒤에** 잰다 — 버려질 grasp 까지 재면 배열 길이가 어긋나 소비자가
        # 엉뚱한 후보의 폭을 쓴다. 폭은 grasp 와 항상 1:1 이어야 한다.
        widths = grasp_widths(scene['objects'][label]['pc'], grasps)
        # 충돌 필터가 전부 걸러내면 widths 가 빈 배열이라 min()/max() 가 던진다.
        span = (f'폭 {widths.min() * 1000:.0f}~{widths.max() * 1000:.0f} mm'
                if len(widths) else '폭 —')
        log(f'[worker] {label}: {len(grasps)} free '
            f'(diff={tags.count("diff")}, obb={tags.count("obb")}), {span}')
        if vis is not None:
            try:
                push_live_object(vis, label, scene['objects'][label]['pc'],
                                 scene['objects'][label]['rgb'], grasps, conf, gripper)
            except Exception as e:                               # noqa: BLE001
                log(f'[worker] live_viz {label} 렌더 실패({type(e).__name__}: {e}) — 계속한다')
        out[label] = {
            'grasps': np.asarray(grasps, dtype=float).tolist(),
            'scores': np.asarray(conf, dtype=float).tolist(),
            'widths': np.asarray(widths, dtype=float).tolist(),
            'n_pts': int(len(scene['objects'][label]['pc'])),
        }
    return {'ok': True, 'objects': out}


if __name__ == '__main__':
    main()
