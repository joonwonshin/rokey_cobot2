"""YOLO 인스턴스 세그멘테이션을 ROS 토픽으로 붙이는 노드.

원본 실험 스크립트(`yoloseg.py`)는 pyrealsense2 로 카메라를 직접 열고 `show=True` 로
띄웠다. 여기서는 두 가지를 바꿨다:

  1. **카메라를 직접 열지 않는다.** RealSense 장치는 한 프로세스만 잡을 수 있는데
     이 워크스페이스는 `realsense2_camera` 가 이미 물고 있다(graspx 파이프라인이
     `/camera/camera/aligned_depth_to_color/*` 를 쓴다). 직접 열면 둘 중 하나가 죽는다.
     그래서 컬러 토픽을 구독한다.
  2. **`show=True` 대신 overlay 토픽.** GUI 창은 컨테이너 안에서 X11 포워딩에 묶이고
     헤드리스에서 죽는다. 오버레이는 파라미터로 켜는 이미지 토픽으로 뺐다.

ultralytics 는 **호스트 시스템 파이썬에 없다** (torch 가 numpy 를 끌어올려 apt
cv_bridge 를 깬다 — `~/.claude/CLAUDE.md` §3). 도커 컨테이너 안에서만 돈다.
그래서 import 를 `_load_model()` 안으로 미룬다 — 이 모듈 자체는 호스트에서도
import 되고, 순수 함수 테스트가 GPU 없이 돈다.
"""

import fcntl
import json
import os

import cv2
import yaml
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
# `import rclpy` 만으로는 `rclpy.logging` 이 안 붙는다 (AttributeError). 명시적으로 가져온다.
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# 라벨맵 규약은 형제 모듈 capture_graspgenx_scene.py 와 맞춘다.
# GraspGenX 로더가 `obj_` 접두어 라벨만 보므로 obj_1 -> 101, obj_2 -> 102 ...
LABEL_OBJ_BASE = 100
# 라벨맵이 uint8 이라 100+156 은 조용히 0 으로 랩어라운드한다.
# 형제 파일 capture_graspgenx_scene.py:84 와 **같은 값이어야 한다**.
MAX_OBJECTS = 155

# 컬러 스트림 QoS. depth=1 인 이유: 추론이 입력 fps 보다 느릴 때 depth 가 크면
# 큐에 쌓인 **묵은** 프레임을 순서대로 처리하게 되고, 그 마스크가 최신 depth 와 짝지어진다.
# 최신 프레임만 보는 게 맞다 (sensor_data 기본 depth 는 5 = 30fps 에서 최대 166ms 지연).
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# 가중치가 없으면 여기서 찾는다. object_detection 패키지가 .pt 를 install 에 심는다
# (`.gitignore` 의 `*.pt` 때문에 커밋되지는 않는다 — README "가중치" 절 참고).
DEFAULT_WEIGHT_PKG = 'object_detection'
DEFAULT_WEIGHT_NAME = 'yolo11n-seg.pt'

LOCK_DIR = '/tmp'


class AlreadyRunning(RuntimeError):
    """같은 토픽에 발행하는 인스턴스가 이미 있다. 버그가 아니라 정상 실패 경로다."""


def lock_path(topic: str) -> str:
    """락 파일 경로. **uid 를 파일명에 넣는 것이 핵심이다.**

    `/tmp` 는 sticky(1777) 라 다른 계정이 먼저 만든 파일은 열지도 지우지도 못한다. 이 랩탑은
    OS 계정을 여러 개 쓰므로(`kimkh`/`jjh`/`rokey`/…) 공용 이름을 쓰면 남이 남긴 파일 하나가
    **영구 차단**이 된다. uid 를 붙이면 그 상황 자체가 생기지 않는다.
    """
    key = topic.strip('/').replace('/', '-') or 'default'
    return os.path.join(LOCK_DIR, f'yolo_seg_node-{key}-{os.getuid()}.lock')


def acquire_singleton(topic: str) -> int:
    """`topic` 에 발행하는 인스턴스가 이미 있으면 `AlreadyRunning`. 반환한 fd 는 열어 둬야 한다.

    **키가 노드 이름이 아니라 토픽인 이유**: 지키려는 불변식이 "이 토픽의 publisher 는 하나"
    이기 때문이다. 이름으로 잠그면 `__node:=other` 로 이름만 바꾼 인스턴스가 락을 우회하고도
    같은 토픽에 2중 발행하고(막아야 하는데 통과), 반대로 카메라 2대를 서로 다른 `mask_topic`
    으로 돌리는 정당한 구성이 막힌다(통과해야 하는데 막힘). 둘 다 방향이 반대다.

    ROS 2 는 이름이 겹치는 노드를 막지 않는다 — 경고 한 줄만 찍고 둘 다 돈다. 그러면 소비자는
    프레임마다 다른 인스턴스의 마스크를 받고, 어느 것인지 구분할 방법이 없다.

    이게 실제로 터졌다: `docker exec` 에는 `--sig-proxy` 가 없어서(docker 29.7.0) 호스트
    Ctrl-C 가 컨테이너 안까지 가지 않고, 재실행할 때마다 고아가 1개씩 쌓였다
    (2026-08-08 실측 10개 · RAM 12GB · VRAM 2.6GB · CPU 8코어). 운영 절차는
    `scripts/graspx_container.sh` 로 고쳤지만, 그걸 안 거치고 띄우는 경로가 남아 있으므로
    노드 자신이 마지막으로 한 번 더 막는다.

    **한계**: 호스트와 컨테이너는 `/tmp` 를 공유하지 않으므로(바인드 마운트는 ws 와
    `/tmp/.X11-unix` 뿐) 이 락은 **경계를 넘어서는 못 막는다.** 같은 ROS 도메인이라 중복은
    성립하는데 락은 모른다. 지금은 이 노드가 컨테이너에서만 도니(호스트엔 ultralytics 가 없다)
    실제 문제가 아니지만, 호스트에서도 돌게 되면 이 가정이 깨진다.

    flock 은 프로세스가 SIGKILL 로 죽어도 커널이 풀어준다 — stale 락이 남지 않는다.
    """
    path = lock_path(topic)
    try:
        # uid 별 경로라 남의 파일과 겹치지 않지만, /tmp 가 없거나 읽기전용인 환경도 있다.
        # 여기서 터지면 "중복 방지 장치가 못 떴다"는 뜻이라 원문을 그대로 올린다.
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    except OSError as exc:
        raise RuntimeError(f'중복 방지 락을 열 수 없다: {path} ({exc})') from exc
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        # 이미 연 fd 에서 읽는다 — 경로를 다시 여는 것보다 실패 경로가 하나 적다.
        owner = os.pread(fd, 32, 0).decode(errors='replace').strip() or '?'
        os.close(fd)
        raise AlreadyRunning(
            f"'{topic}' 에 발행하는 인스턴스가 이미 PID {owner} 로 돌고 있다 ({path}). "
            '둘이 같은 토픽에 발행하면 소비자가 어느 쪽 마스크인지 구분하지 못한다. '
            '먼저 끝내고 다시 띄운다:\n'
            f'  kill -INT {owner}    # 컨테이너 안이면 docker exec <container> kill -INT {owner}\n'
            '  scripts/graspx_container.sh <launch 인자> 로 띄우면 이 정리가 자동이다') from None
    # write 를 먼저 하고 truncate 를 뒤에 한다. 반대로 하면 그 사이에 읽은 경합자가 빈 파일을
    # 보고 `kill -INT ?` 라는 실행 불가능한 안내를 받는다.
    pid = f'{os.getpid()}\n'.encode()
    os.pwrite(fd, pid, 0)
    os.ftruncate(fd, len(pid))
    return fd


def build_label_map(masks: np.ndarray, height: int, width: int,
                    max_objects: int = MAX_OBJECTS, min_pixels: int = 0):
    """(N,H,W) 인스턴스 마스크 -> (라벨맵 uint8, kept).

    - 입력은 **신뢰도 내림차순**(ultralytics 규약)이라고 본다. 역순으로 칠해서
      고신뢰 인스턴스가 최상위 z-order 를 갖게 한다.
    - 겹침으로 픽셀이 0개가 되거나 `min_pixels` 미만으로 깎인 인스턴스는 버리고,
      남은 것에 **101 부터 연속으로** 다시 매긴다. 라벨이 비면(101,103,…) 소비자의
      "obj_N = N번째 물체" 가정이 깨진다.
    - `kept[j]` 는 라벨 `101+j` 가 원래 몇 번째 인스턴스였는지다. 이 재번호 매기기가
      끝나면 클래스 정보로 되돌아갈 길이 사라지므로(라벨맵은 정수 하나뿐) 여기서
      같이 돌려준다 — 호출자가 `res.boxes.cls[kept[j]]` 로 클래스를 붙인다.
    """
    labels = np.zeros((height, width), dtype=np.uint8)
    if masks is None or len(masks) == 0:
        return labels, []

    # retina_masks=True 가 보장하는 것을 코드로 확인한다. 어긋나면 아래 불리언 인덱싱이
    # IndexError 로 터지는데, 여기서 먼저 잡아 원인이 드러나는 메시지를 남긴다.
    if tuple(masks.shape[1:]) != (height, width):
        raise ValueError(
            f'마스크 해상도 {tuple(masks.shape[1:])} != 이미지 {(height, width)}. '
            'predict(retina_masks=True) 가 빠졌는지 확인할 것')

    max_objects = max(0, min(int(max_objects), MAX_OBJECTS))
    sel = masks[:max_objects] > 0.5

    # uint8 로는 겹침 해소 중간값(최대 155)을 담을 수 있지만, 압축 전 임시 id 라
    # 의미가 다르므로 별도 배열을 쓴다.
    tmp = np.zeros((height, width), dtype=np.uint8)
    for i in range(len(sel) - 1, -1, -1):    # 뒤(저신뢰)부터 칠하고 앞(고신뢰)이 덮는다
        tmp[sel[i]] = i + 1

    next_id, kept = 0, []
    for i in range(len(sel)):
        keep = tmp == i + 1
        if int(keep.sum()) < max(1, min_pixels):
            continue                          # 완전히 덮였거나 너무 작다 — 버린다
        next_id += 1
        labels[keep] = LABEL_OBJ_BASE + next_id
        kept.append(i)
    return labels, kept


def load_objects(path: str) -> dict:
    """`config/objects.yaml` -> {'detect': [이름...], 'pick_default': str}.

    형식 오류는 전부 예외로 올린다. 조용히 기본값으로 되돌아가면 "좁게 탐지하려고 적었는데
    전부 탐지되고 있다"를 아무도 눈치채지 못한다 — 이 파일을 만든 목적이 사라진다.
    """
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        raise ValueError(f'{path}: 최상위가 매핑이 아니다 (detect:/pick_default: 를 쓴다)')
    detect = doc.get('detect') or []
    if not isinstance(detect, list) or not all(isinstance(x, str) for x in detect):
        raise ValueError(f'{path}: `detect` 는 클래스 **이름** 문자열의 목록이어야 한다 '
                         '(COCO 인덱스가 아니다)')
    pick = doc.get('pick_default', '')
    if pick is None:
        pick = ''
    if not isinstance(pick, str):
        raise ValueError(f'{path}: `pick_default` 는 문자열이어야 한다 '
                         "(비우려면 '' — 콤마로 여러 개도 된다)")
    return {'detect': [s.strip() for s in detect if s.strip()], 'pick_default': pick.strip()}


def names_to_coco_ids(names, model_names) -> list:
    """클래스 이름 목록 -> 가중치의 인덱스 목록. 모르는 이름이 있으면 예외.

    🔴 인덱스를 사람이 적지 않게 하려고 있는 함수다. `banana=46` 같은 표를 문서와 설정에
    베껴 두면 가중치를 바꾼 날 조용히 어긋난다 — 변환은 **실제로 올라간 가중치**로만 한다.
    """
    inv = {str(v): int(k) for k, v in dict(model_names).items()}
    unknown = [n for n in names if n not in inv]
    if unknown:
        raise ValueError(
            f'이 가중치가 모르는 클래스 이름: {unknown}\n'
            f'가중치가 아는 이름 {len(inv)}개: {sorted(inv)}\n'
            '대소문자를 구분한다. 공구류는 COCO 80종에 없다 — 파인튜닝이 필요하다.')
    return sorted({inv[n] for n in names})


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.declare_parameter('model_path', '')
        # 탐지 대상의 정본. 비우면 아래 `classes`(COCO 인덱스) 를 쓰는 옛 경로로 돌아간다.
        self.declare_parameter('objects_file', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('mask_topic', '/yolo_seg/mask')
        self.declare_parameter('label_topic', '/yolo_seg/labels')
        # 라벨맵은 정수 하나라 "obj_2 가 사과였다"를 담을 수 없다. 클래스는 여기로 뺀다.
        self.declare_parameter('class_topic', '/yolo_seg/classes')
        self.declare_parameter('overlay_topic', '/yolo_seg/overlay')
        self.declare_parameter('publish_overlay', False,
                               ParameterDescriptor(dynamic_typing=True))
        # 오버레이는 기본으로 JPEG 로 낸다. 848x480 bgr8 은 1.16MB 라 UDP 전용 경로
        # (fastdds_udp_only.xml)로는 15Hz 를 못 버틴다 — 실측 3.75Hz 까지 떨어졌다.
        # 같은 화면이 JPEG q80 이면 36KB 다(실제 씬 기준, 33배). 토픽 이름을
        # `<overlay_topic>/compressed` 로 두면 rqt_image_view 가 compressed 전송으로 인식한다.
        # dynamic_typing: launch 나 CLI 로 넘어온 값은 YAML 로 파싱된다.
        # `device:=0` 은 STRING 선언에 INTEGER 가 들어와 InvalidParameterTypeException 이고,
        # `conf:=1` 은 DOUBLE 선언에 INTEGER 다. 타입은 선언이 아니라 읽을 때 맞춘다
        # (capture_graspgenx_scene.py:107 과 같은 이유).
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('overlay_compressed', True, dyn)
        self.declare_parameter('overlay_jpeg_quality', 80, dyn)
        self.declare_parameter('conf', 0.25, dyn)
        self.declare_parameter('device', '0', dyn)   # ultralytics 가 'cpu'/'0,1' 도 받는다
        # 빈 리스트를 그냥 넘기면 rclpy 가 BYTE_ARRAY 로 추론해 정수 목록을 못 넣는다
        # (`-p classes:="[0,39]"` -> InvalidParameterTypeException).
        # capture_graspgenx_scene.py:111 이 같은 이유로 dynamic_typing 을 쓴다.
        self.declare_parameter('classes', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('max_objects', MAX_OBJECTS, dyn)
        self.declare_parameter('min_pixels', 0, dyn)

        self.conf = float(self.get_parameter('conf').value)
        self.device = str(self.get_parameter('device').value)
        classes = self.get_parameter('classes').value
        self.classes = [int(c) for c in classes] if classes else None
        self.max_objects = max(0, min(int(self.get_parameter('max_objects').value), MAX_OBJECTS))
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self.overlay_compressed = bool(self.get_parameter('overlay_compressed').value)
        self.jpeg_quality = int(self.get_parameter('overlay_jpeg_quality').value)
        self._warned_truncate = False

        # 중복 기동은 **가중치 로드 전에** 끊는다 — 로드는 수 초 + VRAM 300MB 라 실패가 비싸다.
        # 파라미터를 다 읽은 뒤여야 `mask_topic` 을 키로 쓸 수 있어서 여기가 가장 이른 자리다.
        self._lock_fd = acquire_singleton(self.get_parameter('mask_topic').value)

        self.bridge = CvBridge()
        self.model = self._load_model(self.get_parameter('model_path').value)
        self._apply_objects_file()

        self.mask_pub = self.create_publisher(Image, self.get_parameter('mask_topic').value, 10)
        self.label_pub = self.create_publisher(Image, self.get_parameter('label_topic').value, 10)
        # 라벨맵과 **같은 header.stamp** 를 실어 보낸다. 소비자(grasp_bridge)는 여러 프레임을
        # 모아 그중 하나를 고르므로, 최신값 하나만 들고 있으면 고른 프레임과 짝이 어긋난다.
        self.class_pub = self.create_publisher(
            String, self.get_parameter('class_topic').value, 10)
        overlay_topic = self.get_parameter('overlay_topic').value
        self.overlay_pub = None
        if self.publish_overlay:
            if self.overlay_compressed:
                self.overlay_pub = self.create_publisher(
                    CompressedImage, overlay_topic + '/compressed', 10)
                self.overlay_topic_name = overlay_topic + '/compressed'
            else:
                self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)
                self.overlay_topic_name = overlay_topic
        self.image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, self.image_topic, self.image_callback, IMAGE_QOS)

        # "노드는 떴는데 아무것도 안 나온다"가 가장 흔한 실패다 (카메라 미기동,
        # 토픽명 불일치, 컨테이너 경계에서 데이터가 안 넘어옴). 조용히 기다리지 않는다.
        self._frames = 0
        self.create_timer(5.0, self._watchdog)

        self.get_logger().info(
            f'yolo_seg_node: {self.image_topic} -> mask/labels '
            f'(device={self.device}, conf={self.conf}, overlay={bool(self.overlay_pub)})')
        if self.overlay_pub is None:
            self.get_logger().info(
                '오버레이는 꺼져 있다 — /yolo_seg/overlay 토픽이 없다. '
                '보려면 -p publish_overlay:=true')
        else:
            self.get_logger().info(f'overlay -> {self.overlay_topic_name}')

    def _watchdog(self):
        if self._frames:
            self._frames = 0
            return
        self.get_logger().warn(
            f'5초간 {self.image_topic} 를 한 장도 못 받았다. 카메라가 떠 있는지, 토픽명이 맞는지, '
            '컨테이너면 FASTRTPS_DEFAULT_PROFILES_FILE 이 양쪽에 걸렸는지 확인할 것')

    def _apply_objects_file(self):
        """`objects_file` 의 `detect` 이름들로 `self.classes` 를 덮어쓴다.

        **모델을 올린 뒤에** 불러야 한다 — 이름→인덱스 변환에 `model.names` 가 필요하다.
        파일을 지정했는데 못 읽으면 **죽는다.** 조용히 전체 탐지로 돌아가면 작업대의 다른
        물체까지 전부 후보가 되는데, 좁히려고 이 파일을 쓴 사람은 그걸 모른다.
        """
        path = str(self.get_parameter('objects_file').value or '')
        if not path:
            return
        cfg = load_objects(path)                # 실패하면 그대로 올라가 노드가 죽는다
        if not cfg['detect']:
            raise ValueError(f'{path}: `detect` 가 비어 있다. 전부 탐지하려면 '
                             'objects_file 을 빈 문자열로 두고 이 파일을 쓰지 않는다')
        if self.classes:
            # 두 곳에서 같은 걸 정하면 어느 쪽이 이겼는지 로그 없이는 알 수 없다.
            self.get_logger().warn(
                f'`classes`({self.classes}) 와 `objects_file` 이 둘 다 주어졌다 — '
                '파일이 이긴다. classes 인자를 지울 것')
        self.classes = names_to_coco_ids(cfg['detect'], self.model.names)
        self.get_logger().info(
            f"탐지 대상 {cfg['detect']} -> COCO {self.classes}  (출처: {path})")

    def _load_model(self, model_path: str):
        from ultralytics import YOLO  # 호스트엔 없다 — 모듈 로드 시점이 아니라 여기서 터뜨린다

        if not model_path:
            share = get_package_share_directory(DEFAULT_WEIGHT_PKG)
            model_path = os.path.join(share, 'resource', DEFAULT_WEIGHT_NAME)
        # 존재 확인을 우리가 먼저 한다. ultralytics 는 basename 이 공식 에셋명이면
        # 없는 경로를 받아도 조용히 네트워크에서 받아온다 — 그러면 어떤 가중치가
        # 올라갔는지 로그만 보고는 알 수 없다.
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'가중치가 없다: {model_path}\n'
                '`.gitignore` 의 `*.pt` 때문에 커밋되지 않는다 — README "가중치" 절 참고')
        model = YOLO(model_path)
        if model.task != 'segment':
            raise RuntimeError(
                f"'{model_path}' 는 task={model.task} 다. seg 가중치가 아니면 마스크가 안 나온다.")
        self.get_logger().info(f'model={model_path} classes={len(model.names)}')
        return model

    def class_payload(self, res, kept, header) -> dict:
        """라벨값 -> 클래스 이름 매핑을 JSON 으로. 소비자는 stamp_ns 로 라벨맵과 짝짓는다.

        `res.boxes` 와 `res.masks` 는 인덱스가 정렬돼 있다(ultralytics 8.4.113 실측:
        마스크 2개면 `boxes.cls` 도 길이 2). `kept[j]` 가 라벨 `101+j` 의 원래 인덱스다.
        탐지가 0개여도 발행한다 — "아직 못 받았다"와 "아무것도 없다"를 구분해야 한다.
        """
        stamp = header.stamp
        objects = []
        boxes = getattr(res, 'boxes', None)
        if boxes is not None and kept:
            cls_ids = boxes.cls.cpu().numpy().astype(int)
            confs = boxes.conf.cpu().numpy()
            for j, i in enumerate(kept):
                if i >= len(cls_ids):      # 마스크/박스 개수가 어긋나면 조용히 밀리지 않게 끊는다
                    self.get_logger().warn(f'boxes({len(cls_ids)}) < masks 인덱스 {i} — 클래스 생략')
                    break
                objects.append({
                    'label': LABEL_OBJ_BASE + j + 1,
                    'class': str(res.names.get(int(cls_ids[i]), cls_ids[i])),
                    'cls_id': int(cls_ids[i]),
                    'conf': round(float(confs[i]), 3),
                })
        return {
            'stamp_ns': int(stamp.sec) * 10**9 + int(stamp.nanosec),
            'frame_id': header.frame_id,
            'objects': objects,
        }

    def image_callback(self, msg: Image) -> None:
        # 콜백에서 나간 예외는 rclpy 가 잡지 않아 spin() 밖으로 튀고 노드가 죽는다.
        # 한 프레임 실패(인코딩 불일치·CUDA OOM)로 노드를 잃지 않는다.
        self._frames += 1
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = bgr.shape[:2]

            # retina_masks=True 가 필수다. 없으면 masks.data 가 letterbox 된 모델 입력
            # 해상도로 나온다 (194x259 입력 -> 480x640 마스크, ultralytics 8.4.113 실측).
            # 그 상태로 라벨맵에 인덱싱하면 IndexError 다 — build_label_map 이 먼저 잡는다.
            res = self.model.predict(bgr, retina_masks=True, conf=self.conf, device=self.device,
                                     classes=self.classes, verbose=False)[0]

            masks = None if res.masks is None else res.masks.data.cpu().numpy()
            if masks is not None and len(masks) > self.max_objects and not self._warned_truncate:
                self.get_logger().warn(
                    f'인스턴스 {len(masks)}개 > max_objects={self.max_objects} — 나머지는 버린다')
                self._warned_truncate = True
            labels, kept = build_label_map(masks, h, w, self.max_objects, self.min_pixels)
            # 이것도 try 안이다 — torch 인덱싱이라 여기서 터질 수 있고, 밖에 두면 위 주석이
            # 약속한 "프레임 하나 버리고 계속"이 이 코드에만 적용되지 않는다.
            class_json = json.dumps(self.class_payload(res, kept, msg.header), ensure_ascii=False)
        except Exception as exc:                       # noqa: BLE001 - 프레임 하나 버리고 계속
            self.get_logger().error(f'프레임 처리 실패: {exc}')
            return

        label_msg = self.bridge.cv2_to_imgmsg(labels, encoding='mono8')
        label_msg.header = msg.header
        self.label_pub.publish(label_msg)
        self.class_pub.publish(String(data=class_json))

        # labels > 0 과 정보량이 같다. sam_mask_node 의 0/255 mono8 계약과 맞춰
        # 세그멘테이션 백엔드를 바꿔 끼울 수 있게 남긴다 (README "토픽" 참고).
        mask_msg = self.bridge.cv2_to_imgmsg(
            np.where(labels > 0, 255, 0).astype(np.uint8), encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if self.overlay_pub is not None:
            plotted = res.plot()          # 박스 + 마스크 + 클래스명/점수를 함께 그린다
            if self.overlay_compressed:
                ok, buf = cv2.imencode('.jpg', plotted,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    overlay_msg = CompressedImage()
                    overlay_msg.format = 'jpeg'
                    overlay_msg.data = buf.tobytes()
                else:
                    self.get_logger().warn('JPEG 인코딩 실패 — 오버레이를 건너뛴다')
                    return
            else:
                overlay_msg = self.bridge.cv2_to_imgmsg(plotted, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    rc = 0
    try:
        # 생성도 try 안이다. 가중치 부재·detect 가중치·ultralytics 부재는 **정상 실패
        # 경로**인데, 밖에 두면 그때 rclpy.shutdown() 이 실행되지 않는다.
        node = YoloSegNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                   # Ctrl-C / SIGTERM 은 정상 종료다
    except AlreadyRunning as exc:
        # 이것도 정상 실패 경로다. 스택트레이스로 내보내면 `ros2 launch` 가 줄마다
        # `[yolo_seg_node-1]` 을 붙여 정작 읽어야 할 안내가 파이썬 프레임에 묻힌다.
        get_logger('yolo_seg_node').fatal(str(exc))
        rc = 1
    finally:
        if node is not None:
            node.destroy_node()
        # SIGTERM(`timeout`, docker stop)으로 죽으면 시그널 핸들러가 이미 컨텍스트를
        # 내린 뒤라 다시 부르면 RCLError 로 터진다 — 정상 종료가 스택트레이스를 남긴다.
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
