#!/usr/bin/env python3
"""그립 명령 하나로 촬영→세그멘테이션→GraspGenX→선택까지 도는 ROS 노드.

    ros2 service call /grasp/compute std_srvs/srv/Trigger

한 번 부를 때마다:
    1. depth N프레임 수집 → 픽셀별 중앙값 병합
    2. base 프레임 작업공간 박스로 세그멘테이션(신경망 0개)
    3. 씬 4파일을 `out_dir`(기본 `<repo>/data/graspgenx_scene`, 호출마다 타임스탬프 하위
       디렉토리)에 영구 저장하고 **GPU 워커**에 경로 한 줄 전달 — 판단 근거를 나중에도
       열어볼 수 있어야 한다(2026-08-07, 예전엔 임시 디렉토리에 썼다가 즉시 지웠다)
    4. 워커가 준 grasp(=base_link 프레임)를 도달·접근축·점수로 거르고 1개 선택
    5. `/grasp/best`(PoseStamped) + `/grasp/candidates`(PoseArray) 발행,
       서비스 응답 message 에 요약

왜 워커가 별도 프로세스인가:
    GraspGenX 는 torch 가 있는 uv venv 에서만 import 되고, rclpy 는 시스템 파이썬이다.
    이 워크스페이스는 **ROS 노드에 venv 를 쓰지 않는다** — 무거운 ML 은 격리하고 통신한다.
    워커는 모델을 한 번만 올려두므로 요청당 비용은 추론 시간뿐이다.

⚠️ 이 노드는 로봇을 움직이지 않는다. grasp 포즈를 계산해 발행할 뿐이다.
   실행은 MoveIt 쪽에서 사람이 승인한 뒤에 한다.
"""

import datetime
import json
import os
import shutil
import subprocess
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from graspgenx_perception.capture_graspgenx_scene import (
    LABEL_OBJ_BASE, SceneCapture, best_labels, default_out_dir, filter_labels_by_class,
    pixel_to_base, segment, select_by_point, workspace_mask, write_scene,
)

try:
    from pick_fsm_msgs.srv import ComputeGrasp
except ImportError:                                          # pragma: no cover
    # 선택적 의존이다 — 없으면 legacy Trigger 경로만 연다(__init__ 참고).
    ComputeGrasp = None

EXTRA_DEFAULTS = {
    # 워커
    'graspgenx_root': os.path.expanduser('~/cobot2_ws_new/isaac_ros-dev/src/GraspGenX'),
    'worker_script': os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'graspgen_worker.py'),
    'gripper': 'onrobot_RG2',
    'worker_timeout_sec': 60.0,
    # ── 워커 기동 인자 (아래 4개는 `graspgen_worker.py` 의 argparse 기본값과 같은 값) ──
    # ⚠️ **워커 기동 시점에 argv 로 고정된다.** 상주 프로세스라 `ros2 param set` 만으로는
    #    안 바뀐다 — 바꾼 뒤 워커를 죽여야(브리지가 다음 요청에서 다시 띄운다) 적용된다.
    # 이걸 노출하는 이유: 2026-08-09 실기에서 `[collision] 0/43 grasps collision-free` 로
    # 후보가 전멸했는데, 임계값을 낮춰볼 수단이 노드 쪽에 하나도 없었다.
    # 🔴 `collision_threshold` 는 **그리퍼 표면점과 씬점의 최소거리**다. 씬 점군에는
    #    seg 라벨과 무관하게 유효 depth 가 전부 들어가므로 **테이블도 장애물이다**
    #    (`constraints.md:1203`). 납작한 물체를 위에서 잡으면 손끝이 테이블에서 2cm 안에
    #    들어와 **정상 grasp 까지 전멸**한다 — 0개가 나오면 여기부터 의심한다.
    'collision_threshold': 0.02,
    'num_grasps': 64,               # 8GB VRAM 기준. 올리면 OOM 위험(아래 주석)
    'grasp_threshold': 0.5,
    'moe_obb_density': 'dense',     # sparse | dense | dense-topandside
    # true 면 워커가 `scripts/demo_scene_pc.py` 와 같은 viser 웹뷰어를 띄우고, 매 compute()
    # 마다 방금 캡처한 씬(실제 포인트클라우드) + grasp 후보 + 1등 콜리전 메시로 자동 갱신한다.
    # 브라우저에서 http://<이 머신>:live_viz_port 로 본다. **기본 켜짐**(2026-08-09) —
    # 발표·디버깅에 상시 쓰기로 함. 끄려면 `live_viz:=false`(런치) 또는
    # `ros2 param set /grasp_bridge_node live_viz false`(워커 재기동 전에만 먹는다).
    'live_viz': True,
    'live_viz_port': 8080,
    # 선택 정책 (계획서 §1-3 5번). 전부 실측 튜닝값이다
    # GraspGenX grasp 의 원점은 **그리퍼 base** 이고 손끝(TCP)은 +Z 로 이만큼 떨어져 있다
    # (RG2 config.json "fingertip"[2] = 0.18). 접근이 30° 기울면 base 는 물체에서 9cm 옆에
    # 앉는다 — 2026-08-05 에 이걸 "좌표 오차"로 오진했다. 그래서 TCP 를 같이 발행·기록한다.
    'tcp_offset_m': 0.18,
    'min_score': 0.5,
    'max_reach_m': 0.900,       # M0609 도달. constraints.md 의 URDF 실측
    'max_approach_z': -0.3,     # grasp +Z(접근축)가 아래를 향하는 것만: R[2,2] < 이 값
    'target': '',               # 비우면 점수 최고 물체. 라벨 이름(obj_2)으로 고른다
    # **클래스 이름**으로 대상을 좁힌다 (예: 'apple' 또는 'apple,cup'). 비우면 전부.
    # 리스트가 아니라 콤마 문자열인 이유: rcl YAML 파서가 리스트 안 타입 혼합·빈 리스트에서
    # 죽는 사례를 이 ws 에서 반복해 밟았다(CLAUDE.md §4). 문자열은 그 함정이 없다.
    # seg_source='yolo' 전용 — geometric 경로는 클래스를 모른다.
    # `ros2 param set` 으로 런타임 변경이 먹는다. extra() 가 compute() 마다 다시 읽는다
    # (yolo_seg_node 의 classes 와 다른 점이다. 그쪽은 __init__ 에서 한 번만 읽는다).
    'target_classes': '',
    # ── 개체 선정(select_by_point) — 2026-08-11 구현 ────────
    # 같은 class 후보가 2개 이상일 때 "어느 개체"를 고를 좌표. `task_manager`가 매
    # PERCEIVE 마다 밀어 넣는다(target_classes 와 같은 자리) — VLA/클릭이 없는 보통의
    # pick 은 nan 이 그대로 오므로(= "지정 없음") 기존 동작(점수 최고)과 동일하다.
    # nan 을 "off" 센티널로 쓰는 이유는 이 파일의 `table_z`/`class_dims` 관례와 같다.
    # 설계 출처: md/plans/2026-08-08-vla-integration.md §5.
    'pixel_x': float('nan'), 'pixel_y': float('nan'),
    # `pixel_x/y` 가 어느 해상도 기준인지. compute() 는 이 값이 **지금 depth 프레임의
    # 실제 해상도**와 다르면 스케일링을 추측하지 않고 거부한다(vla-bridge-contract.md §2).
    'pixel_w': float('nan'), 'pixel_h': float('nan'),
    # 지정점에서 이 반경[m] 안의 물체만 후보로 본다. VLA `system.yaml`의
    # `match_tolerance_m`(0.06)과 값을 맞췄다 — 두 시스템이 다른 허용오차를 쓰면
    # "VLA 는 지목했는데 우리가 못 찾는다"가 난다. UNVERIFIED: 실기 튜닝값 아님, 초안값.
    'match_tolerance_m': 0.06,
    # true 면 2등 후보가 이 마진[m] 안으로 붙어 있을 때 **선택을 거부한다**(모호함 자체를
    # 신뢰하지 않는다). false 면 거리만으로 1등을 그대로 쓴다. UNVERIFIED: 초안값.
    'ambiguity_margin_m': 0.02,
    'refuse_ambiguous_match': True,
}


def pose_msg(T):
    """4x4 -> geometry_msgs/Pose. 회전은 행렬 -> 쿼터니언(Shepperd)."""
    m, p = T[:3, :3], Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
        float(x), float(y), float(z), float(w))
    return p


def tcp_of(g, offset):
    """grasp 4x4 -> 손끝 위치. TCP = 원점 + offset * (grasp 의 +Z축).

    GraspGenX 의 grasp 원점은 그리퍼 base 다(`robot.py:59`, constraints.md).
    **이걸 물체 위치로 읽으면 안 된다** — 접근이 기울수록 옆으로 벗어난다.
    """
    g = np.asarray(g, dtype=float).reshape(-1, 4, 4)
    return g[:, :3, 3] + offset * g[:, :3, 2]


def select(objects, p, logger=None):
    """워커 결과 -> (label, T, score, width, g_ok, s_ok, w_ok, 진단문자열). 없으면 앞 7개가 None.

    grasp 는 이미 base_link 프레임이다(씬의 camera_pose 를 로더가 적용한다).
    `width` 는 **고른 grasp 의** 닫힘축 폭 [m] 이다(`graspgen_worker.grasp_widths`).
    후보마다 다르므로 물체 대표값이 아니다 — 대안으로 갈아탈 때 폭도 같이 갈아타야 한다.
    """
    lines, best = [], None
    for label, d in sorted(objects.items()):
        g = np.asarray(d['grasps'], dtype=float).reshape(-1, 4, 4)
        s = np.asarray(d['scores'], dtype=float)
        # 구버전 워커는 widths 를 안 낸다. 0.0 은 "모름" 이고, 소비자(FSM)가 자기
        # default_width_m 으로 대체한다 — 여기서 그럴듯한 값을 지어내면 그리퍼가 조용히 헛닫힌다.
        w = np.asarray(d.get('widths') or [0.0] * len(g), dtype=float)
        if len(w) != len(g):
            # 워커 쪽은 grasps/scores/widths 를 같은 keep 마스크로 슬라이스해 항상 길이가
            # 맞지만, 여기선 그 불변식을 신뢰만 하고 확인하지 않았었다(cross-review 2026-08-09).
            # 안 맞으면 아래 w[i]/w[ok] 가 IndexError 나 "boolean index did not match" 로
            # 죽는데, 스택트레이스만 남고 원인이 안 드러난다 — "모름"으로 낮춰서 계속 돈다.
            if logger is not None:
                logger.warn(
                    f'{label}: widths 길이 {len(w)} != grasps {len(g)} — 워커 응답이 깨졌다. '
                    '폭을 전부 "모름"(0.0) 으로 낮춘다')
            w = np.zeros(len(g), dtype=float)
        if len(g) == 0:
            lines.append(f'  {label}: 후보 0')
            continue
        ok = s >= p['min_score']
        reach = np.linalg.norm(g[:, :3, 3], axis=1)
        ok &= reach < p['max_reach_m']
        ok &= g[:, 2, 2] < p['max_approach_z']     # +Z 접근축이 아래를 향하는 것만
        # 위치를 같이 찍는다 — 이게 없으면 "엉뚱한 물체를 골랐다"를 눈치챌 방법이 없다.
        # 2026-08-05: 사과가 bounds 밖이라 다른 덩어리가 선택됐는데 개수만 보고는 못 알아챘다.
        # base 가 아니라 **손끝**을 찍는다. base 는 접근 각도에 따라 물체에서 크게 벗어난다.
        c = tcp_of(g, p['tcp_offset_m']).mean(axis=0)
        lines.append(f'  {label}: {len(g)}개 -> 점수 {int((s >= p["min_score"]).sum())} '
                     f'-> 도달 {int((reach < p["max_reach_m"]).sum())} '
                     f'-> 접근축 {int(ok.sum())} 통과  '
                     f'손끝≈({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})')
        if not ok.any():
            continue
        i = int(np.argmax(np.where(ok, s, -np.inf)))
        if p['target'] and label != p['target']:
            continue
        if best is None or s[i] > best[2]:
            best = (label, g[i], float(s[i]), float(w[i]), g[ok], s[ok], w[ok])
    if best is None:
        return (None,) * 7 + ('\n'.join(lines) or '물체 없음',)
    return (*best, '\n'.join(lines))


class GraspBridge(SceneCapture):
    def __init__(self):
        super().__init__()
        for k, v in EXTRA_DEFAULTS.items():
            self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        self.lock = threading.Lock()
        self.worker = None
        self.result = None          # 마지막 compute() 결과 (포즈·폭·대안). lock 안에서만 만진다
        cb = ReentrantCallbackGroup()
        self.pub_best = self.create_publisher(PoseStamped, '/grasp/best', 10)
        # RViz 육안 확인용. /grasp/best 는 그리퍼 base 라 물체 옆에 뜬다 — 이쪽이 물체 위다
        self.pub_tcp = self.create_publisher(PoseStamped, '/grasp/best_tcp', 10)
        self.pub_all = self.create_publisher(PoseArray, '/grasp/candidates', 10)
        self.srv = self.create_service(Trigger, '/grasp/compute', self.on_compute,
                                       callback_group=cb)
        # ComputeGrasp 는 pick_fsm_msgs 가 빌드돼 있을 때만 연다. 이 패키지가 없다고 브리지가
        # 통째로 안 뜨면 legacy Trigger 경로까지 같이 죽는다 — 그쪽은 이 메시지가 필요 없다.
        self.srv_grasp = None
        if ComputeGrasp is None:
            self.get_logger().warn(
                'pick_fsm_msgs 를 import 하지 못해 /grasp/compute_grasp 를 열지 않는다. '
                '폭(width_m)은 이 서비스로만 나간다 — Trigger 응답에는 담을 필드가 없다')
        else:
            self.srv_grasp = self.create_service(
                ComputeGrasp, '/grasp/compute_grasp', self.on_compute_grasp, callback_group=cb)
        self.get_logger().info(
            '준비됨 — ros2 service call /grasp/compute std_srvs/srv/Trigger'
            + ('  |  /grasp/compute_grasp pick_fsm_msgs/srv/ComputeGrasp (폭 포함)'
               if self.srv_grasp else ''))

    def extra(self):
        out = dict(self.params())
        for k, default in EXTRA_DEFAULTS.items():
            v = self.get_parameter(k).value
            out[k] = str(v) if isinstance(default, str) else type(default)(v)
        return out

    def start_worker(self, p):
        """모델 로딩은 여기서 한 번만. ready 한 줄을 기다린다."""
        if self.worker and self.worker.poll() is None:
            return True
        cmd = ['uv', 'run', '--no-sync', 'python', p['worker_script'],
               '--gripper', p['gripper'],
               '--collision_threshold', str(p['collision_threshold']),
               '--num_grasps', str(int(p['num_grasps'])),
               '--grasp_threshold', str(p['grasp_threshold']),
               '--moe_obb_density', str(p['moe_obb_density'])]
        if p['live_viz']:
            cmd += ['--live_viz', '--live_viz_port', str(int(p['live_viz_port']))]
        self.get_logger().info(f"워커 기동(모델 로딩 수십 초): {' '.join(cmd)}")
        self.worker = subprocess.Popen(
            cmd, cwd=p['graspgenx_root'], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=None, text=True, bufsize=1)
        line = self.worker.stdout.readline()
        try:
            ok = json.loads(line).get('ready') is True
        except json.JSONDecodeError:
            ok = False
        if not ok:
            self.get_logger().error(f'워커 기동 실패 (첫 줄: {line!r}) — stderr 를 확인한다')
            return False
        self.get_logger().info('워커 준비 완료')
        return True

    def ask_worker(self, scene_dir, timeout):
        self.worker.stdin.write(scene_dir + '\n')
        self.worker.stdin.flush()
        box = {}
        t = threading.Thread(target=lambda: box.setdefault('l', self.worker.stdout.readline()))
        t.daemon = True
        t.start()
        t.join(timeout)
        if 'l' not in box:
            raise TimeoutError(f'워커 응답 {timeout}s 초과')
        return json.loads(box['l'])

    def on_compute(self, request, response):
        """legacy: std_srvs/Trigger. 포즈는 응답이 아니라 `/grasp/best` 로 나간다."""
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, '이미 계산 중이다'
            return response
        try:
            response.success, response.message = self.compute()
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'{type(e).__name__}: {e}')
            response.success, response.message = False, f'{type(e).__name__}: {e}'
        finally:
            self.lock.release()
        return response

    def on_compute_grasp(self, request, response):
        """pick_fsm_msgs/ComputeGrasp — 포즈·**폭**·대안을 응답에 담는 정본 계약.

        Trigger 경로와 계산은 완전히 같다(`compute()` 하나만 있다). 다른 건 폭이 실려
        나간다는 것뿐이다 — Trigger 응답에는 폭을 담을 필드가 없어서 소비자가
        `default_width_m` 상수로 때웠다.
        """
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, '이미 계산 중이다'
            return response
        try:
            # 요청이 대상을 지정하면 그 호출에 한해 파라미터를 덮는다(파라미터 자체는 안 건드린다
            # — 덮어쓰면 다음 Trigger 호출이 조용히 남의 타겟으로 돈다).
            over = {}
            if request.target:
                over['target_classes'] = request.target
            if request.min_confidence > 0.0:
                over['min_score'] = float(request.min_confidence)
            response.success, response.message = self.compute(over)
            if response.success and self.result:
                r = self.result
                response.grasp_pose = r['pose']
                response.width_m = r['width_m']
                response.confidence = r['confidence']
                response.alternatives = r['alternatives']
                response.alternative_widths = r['alternative_widths']
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'{type(e).__name__}: {e}')
            response.success, response.message = False, f'{type(e).__name__}: {e}'
        finally:
            self.lock.release()
        return response

    def compute(self, overrides=None):
        """계산 -> (성공, 메시지). 성공하면 `self.result` 에 포즈·폭·대안을 남긴다.

        반환값을 (성공, 메시지) 로 유지하는 이유: 실패 return 이 10군데 가까이 흩어져 있어
        arity 를 늘리면 전부 손대야 하고, 한 군데만 놓치면 조용히 튜플 언팩에서 죽는다.
        `self.result` 는 호출자가 잡은 `self.lock` 안에서만 읽고 쓴다.
        """
        self.result = None
        p = self.extra()
        p.update(overrides or {})
        if not self.start_worker(p):
            return False, '워커 기동 실패'

        # 1) 최신 프레임만 쓴다 — 이전 요청 때 쌓인 것을 섞지 않는다
        self.depths.clear()
        self.yolo_labels_history.clear()
        self.yolo_classes.clear()
        n = max(1, p['frames'])
        # ⚠️ 여기서 rclpy.spin_once 를 부르면 안 된다 — 이미 executor 가 이 노드를 돌리고 있고
        #    콜백 안에서 다시 spin 하면 재진입으로 엉킨다. 구독 콜백은 다른 스레드에서
        #    (서비스만 ReentrantCallbackGroup, 구독은 기본 그룹) 채워주므로 기다리기만 한다.
        end = time.monotonic() + p['timeout_sec']
        while rclpy.ok() and not self.ready(n, p) and time.monotonic() < end:
            time.sleep(0.02)
        if not self.ready(n, p):
            return False, (f'수신 실패 (depth {len(self.depths)}/{n}, '
                           f'color={self.color is not None}, info={self.K is not None}, '
                           f'tf={self.tf_ready(p)})')

        T_base_cam = self.camera_pose()
        if T_base_cam is None:
            return False, 'TF lookup 실패'
        depth, _ = self.merged_depth(n, p['min_valid_ratio'])
        if self.info_wh != (depth.shape[1], depth.shape[0]):
            return False, f'camera_info {self.info_wh} != depth {depth.shape[::-1]}'

        # 2) 세그멘테이션
        # seg_source='yolo' 면 최신 한 장이 아니라 방금 모은 n장 중 탐지 픽셀이 가장 많은
        # 프레임을 쓴다(best_labels) — grasp 연산(수십 초)에 비하면 프레임 몇 장 더 보는
        # 비용은 무시할 만하고, 흔들린 한 컷 때문에 seg 가 통째로 비는 걸 줄인다.
        use_yolo = p.get('seg_source') == 'yolo'
        wanted = {s.strip() for s in p['target_classes'].split(',') if s.strip()}
        if wanted and not use_yolo:
            # 조용히 무시하면 "사과만 잡는다"고 믿은 채 전부 연산한다 — 가장 나쁜 실패다.
            return False, (f"target_classes='{p['target_classes']}' 는 seg_source='yolo' 에서만 "
                           f"쓴다. 지금은 '{p['seg_source']}' 라 클래스를 알 수 없다")
        hist = self.yolo_labels_history
        cdiag = None
        if use_yolo and wanted:
            # 클래스맵이 딸린 프레임만 후보로 둔다. 라벨맵은 RELIABLE 로 나가지만 구독이
            # BEST_EFFORT 라 두 토픽이 따로 떨어질 수 있는데, 하필 라벨만 온 프레임이
            # "픽셀 최다"로 뽑히면 필터가 통째로 무력해진다 — 조용히 전부 연산하는 게 최악이다.
            hist = [f for f in hist if f[0] in self.yolo_classes]
            if not hist:
                return False, (f"target_classes='{p['target_classes']}' 인데 "
                               f"{p['class_topic']} 를 한 프레임도 못 받았다. yolo_seg_node 가 "
                               '이 클래스맵을 발행하는 버전인지 확인할 것')
            # ⚠️ **거르고 나서 고른다.** 순서를 뒤집으면 best_labels 가 전체 클래스 픽셀 합으로
            # 고르므로, dining table 처럼 큰 물체가 점수를 지배해 대상(사과)이 깜빡인 컷이
            # "최선"으로 뽑히고 필터가 그걸 전부 지운다 → 사과가 있었는데 grasp 0개
            # (cross-review 2026-08-08). 프레임 10장 필터링은 grasp 연산 수십 초에 비해 공짜다.
            diags = {}
            cands = []
            for s, im in hist:
                out, d = filter_labels_by_class(im, self.yolo_classes[s], wanted)
                cands.append((s, out))
                diags[s] = d
            frame_stamp, labels = best_labels(cands)
            cdiag = diags[frame_stamp]
        else:
            # 아래 5) 의 발행 stamp 와 다른 값이다 — 이건 **고른 라벨 프레임**의 stamp 다.
            frame_stamp, labels = best_labels(hist) if use_yolo else (None, None)
        class_map = self.yolo_classes.get(frame_stamp, {}) if use_yolo else {}
        if use_yolo:
            self.get_logger().info('클래스맵: ' + (', '.join(
                f'obj_{v - LABEL_OBJ_BASE}={n}' for v, n in sorted(class_map.items())) or '없음'))
        if cdiag:
            self.get_logger().info(cdiag)
            if not (labels > LABEL_OBJ_BASE).any():
                # segment_from_labels 는 빈 seg 를 None 이 아니라 전부 0 으로 돌려준다. 그대로
                # 두면 씬을 쓰고 워커를 왕복한 뒤 '물체 없음'만 가서 원인이 안 보인다.
                #
                # 🔴 **YOLO 가 실제로 낸 클래스를 같이 찍는다.** 예전엔 "이름이 맞는지 확인"
                #    까지만 말해서 오타를 의심하게 만들었는데, 진짜 원인은 보통
                #    "detect 목록에 없어서 YOLO 가 애초에 그걸 안 찾고 있었다" 쪽이다.
                #    이 둘은 눈으로 구분이 안 된다 — 실제로 본 목록이 있어야 갈린다.
                seen = sorted({n for m in self.yolo_classes.values() for n in m.values()})
                return False, (
                    f"target_classes='{p['target_classes']}' 에 해당하는 물체가 없다.\n"
                    f"  YOLO 가 이번에 실제로 낸 클래스: {seen or '없음'}\n"
                    '  → 이 목록에 타겟이 없으면 오타가 아니라 **탐지 대상 문제**다: '
                    'config/objects.yaml 의 `detect` 에 그 이름을 넣고 yolo_seg_node 를 '
                    '다시 띄울 것 (그 파일은 __init__ 에서 한 번만 읽는다).\n'
                    '  → 목록에 있는데도 실패했다면 그때가 오타/대소문자를 볼 차례다.\n'
                    f'{cdiag}')

        seg, label_map, diag = segment(depth, self.K, T_base_cam, p, labels, class_map)
        if seg is None:
            return False, diag
        self.get_logger().info(diag)

        # 2-b) 개체 선정(select_by_point). class 필터 뒤·워커 호출 전 — 여기서 걸러야
        # GraspGenX 연산 자체가 후보 하나로 줄어든다(진짜 병목은 워커 수십 초).
        # `pixel_x/y`가 nan(= 지정 없음)이면 그냥 지나간다 — 기존 "점수 최고" 동작 그대로.
        if np.isfinite(p['pixel_x']) and np.isfinite(p['pixel_y']):
            if not (np.isfinite(p['pixel_w']) and np.isfinite(p['pixel_h'])):
                return False, 'pixel_x/y 는 있는데 pixel_w/h(기준 해상도)가 없다'
            cur_w, cur_h = int(depth.shape[1]), int(depth.shape[0])
            if (int(p['pixel_w']), int(p['pixel_h'])) != (cur_w, cur_h):
                # 스케일링을 추측하지 않는다(vla-bridge-contract.md §2) — 다른 해상도로
                # 찍은 좌표를 그대로 쓰면 조용히 엉뚱한 물체를 가리킨다.
                return False, (f"pixel_wh({int(p['pixel_w'])}x{int(p['pixel_h'])}) != "
                               f'지금 depth 해상도({cur_w}x{cur_h})')
            u, v = int(round(p['pixel_x'])), int(round(p['pixel_y']))
            if not (0 <= u < cur_w and 0 <= v < cur_h):
                return False, f'pixel ({u},{v}) 가 해상도 {cur_w}x{cur_h} 밖이다'
            pt = pixel_to_base(depth, self.K, T_base_cam, u, v)
            if pt is None:
                return False, f'pixel ({u},{v}) 주변에 유효 depth 가 없다(구멍)'
            tx, ty = pt
            _, X, Y, _ = workspace_mask(depth, self.K, T_base_cam, p)
            margin = p['ambiguity_margin_m'] if p['refuse_ambiguous_match'] else float('-inf')
            seg2, label_map2, sel_diag = select_by_point(
                seg, label_map, X, Y, tx, ty, p['match_tolerance_m'], margin)
            self.get_logger().info(f'개체 선정: pixel=({u},{v}) -> base=({tx:+.3f},{ty:+.3f}) '
                                   f'— {sel_diag}')
            if seg2 is None:
                return False, f'개체 선정 실패: {sel_diag}'
            seg, label_map = seg2, label_map2

        # 3) 워커 호출. 씬은 항상 영구 저장한다 — GraspGenX가 뭘 보고 판단했는지(rgb/seg/meta)를
        #    호출이 끝난 뒤에도 열어볼 수 있어야 오검출을 진단할 수 있다(2026-08-07, 임시
        #    디렉토리+즉시 삭제였을 때는 이게 불가능했다). scene id 를 호출마다 마이크로초
        #    타임스탬프로 잡아 이전 호출 기록을 덮어쓰지 않는다(초 단위면 빠른 재시도가
        #    충돌할 수 있어 %f 를 붙인다, cross-review 2026-08-07) — `scene` 파라미터를
        #    🔴 `time.strftime` 은 `%f` 를 모른다 — 디렉토리 이름에 **리터럴 `%f` 가 그대로**
        #    남아 마이크로초 구분이 전혀 안 되고 있었다(2026-08-09, 저장된 씬이 전부
        #    `20260809_161221_%f`). 같은 초의 재시도가 앞 씬을 덮어써 진단 근거가 사라진다.
        #    `%f` 는 `datetime.strftime` 에만 있다 — 그래서 `datetime` 을 쓴다.
        #    명시하면 그 이름을 그대로 쓴다(재현용 고정 씬을 만들 때). 단 `scene='00'`(기본값과
        #    같은 문자열)을 "명시"해도 구분할 수 없어 타임스탬프로 대체된다 — 고정 씬이
        #    필요하면 `00` 이외의 이름을 쓸 것.
        # ⚠️ 워커에 넘기는 경로는 **반드시 절대경로**다. 이 노드의 cwd 는 사용자가 ros2 를
        #    실행한 곳이고 워커의 cwd 는 graspgenx_root 다(start_worker). 상대경로를 넘기면
        #    워커가 <graspgenx_root>/output/00 을 찾다가
        #    "FileNotFoundError: No meta_data.json in output/00" 으로 죽는다 (2026-08-07).
        scene = (p['scene'] if p['scene'] != '00'
                 else datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        scene_dir = os.path.abspath(os.path.expanduser(
            os.path.join(p['out_dir'] or default_out_dir(), scene)))
        self.get_logger().info(f'씬 저장(영구): {scene_dir}')
        try:
            write_scene(scene_dir, depth, self.color, seg, self.K, T_base_cam, label_map,
                        [p['x_min'], p['y_min'], p['z_min'],
                         p['x_max'], p['y_max'], p['z_max']])
        except Exception:
            # 파일 4개 중 일부만 쓰다 실패하면(디스크 풀 등) 반쪽짜리 씬이 남아
            # 나중에 진짜 기록과 섞인다 — 실패 시 통째로 지운다(cross-review 2026-08-07).
            shutil.rmtree(scene_dir, ignore_errors=True)
            raise
        res = self.ask_worker(scene_dir, p['worker_timeout_sec'])
        if not res.get('ok'):
            return False, f"워커 오류: {res.get('error')}"

        # 4) 선택
        label, T, score, width_m, g_ok, _s_ok, w_ok, sel_diag = select(
            res.get('objects', {}), p, self.get_logger())
        self.get_logger().info(f'선택 단계:\n{sel_diag}')
        if label is None:
            return False, f'조건을 통과한 grasp 0개\n{sel_diag}'

        # 5) 발행
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, p['base_frame']
        msg.pose = pose_msg(T)
        self.pub_best.publish(msg)
        T_tcp = T.copy()
        T_tcp[:3, 3] = tcp_of(T, p['tcp_offset_m'])[0]
        tcp_msg = PoseStamped()
        tcp_msg.header, tcp_msg.pose = msg.header, pose_msg(T_tcp)
        self.pub_tcp.publish(tcp_msg)
        arr = PoseArray()
        arr.header = msg.header
        arr.poses = [pose_msg(g) for g in g_ok]
        self.pub_all.publish(arr)

        # 대안 폭을 같이 담는다. 폭은 **후보마다 다르다**(닫힘축이 후보마다 달라서다) —
        # 1등 폭만 주면 소비자가 대안으로 갈아탄 뒤에도 1등 폭으로 닫아 헛집는다.
        alts = []
        for g in g_ok:
            a = PoseStamped()
            a.header = msg.header
            a.pose = pose_msg(g)
            alts.append(a)
        self.result = {
            'pose': msg,
            'width_m': float(width_m),
            'confidence': float(score),
            'alternatives': alts,
            'alternative_widths': [float(v) for v in w_ok],
        }

        t = T[:3, 3]
        tcp = tcp_of(T, p['tcp_offset_m'])[0]
        # 'obj_2' 만 찍으면 어느 물체였는지 로그로 되짚을 수 없다 — 클래스를 붙여 남긴다.
        cls = class_map.get(label_map.get(label))
        label = f'{label}({cls})' if cls else label
        return True, (f'{label} 선택: score={score:.3f}, '
                      f'**손끝**=({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}) {p["base_frame"]}, '
                      f'그리퍼base=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}), '
                      f'접근축 z={T[2, 2]:+.2f}, 폭={width_m * 1000:.1f} mm, '
                      f'후보 {len(g_ok)}개\n{sel_diag}')

    def destroy_node(self):
        if self.worker and self.worker.poll() is None:
            try:
                self.worker.stdin.close()
                self.worker.wait(timeout=5)
            except Exception:                                    # noqa: BLE001
                self.worker.kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = GraspBridge()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
