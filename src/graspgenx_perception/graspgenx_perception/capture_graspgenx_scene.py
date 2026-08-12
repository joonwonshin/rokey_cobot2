#!/usr/bin/env python3
"""RealSense 한 프레임을 GraspGenX의 real_world 포맷(파일 4개)으로 떨어뜨린다.

왜 노드가 아니라 파일인가:
  `demo_scene_pc.py`가 M2T2 real_world 디렉토리를 그대로 먹는다
  (scene_loaders.py:50 load_realworld_scene). 즉 ROS 배선을 하기 전에
  **패키지가 제공하는 데모 스크립트 그대로** 실기 데이터를 검증할 수 있다.
  브리지 노드는 이 경로가 뚫린 다음에 짠다.

출력 (loader가 요구하는 이름 그대로):
  <out_dir>/<scene>/depth.npy       float32, **미터**, (H,W)
  <out_dir>/<scene>/rgb.png
  <out_dir>/<scene>/seg.png         uint8 라벨맵cd /workspaces/isaac_ros-dev
  colcon build --symlink-install \
    --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
  <out_dir>/<scene>/meta_data.json  intrinsics 3x3 / camera_pose 4x4 / label_map / scene_bounds

camera_pose 는 tf2 lookup(base_link <- camera_color_optical_frame)이다.
  - **npy 직접 로드 금지** — 2026-08-02에 OpenCV optical 규약 때문에 90° 틀어진 이력이 있다.
  - loader가 이 4x4로 점을 world로 보내므로(scene_loaders.py:86), 이걸 넣으면
    GraspGenX가 말하는 "world"가 곧 base_link 가 된다.
  - 마스크가 **컬러 정렬** depth 기준이라 부모는 camera_color_optical_frame 이다(depth optical 아님).

세그멘테이션(seg.png)은 신경망 0개다:
  base 프레임 작업공간 박스 안 + 테이블면보다 obj_min_h 이상 높은 픽셀 →
  cv2.connectedComponents 로 덩어리 분리. 붙어 있는 물체는 하나로 뭉친다.
  ponytail: YOLO-seg/DBSCAN 은 이 경로가 grasp를 내는 걸 본 뒤에 붙인다.
            물체를 서로 떨어뜨려 놓으면 이걸로 충분하다.

⚠️ self-filter 가 없다. 로봇 팔이 작업공간 박스 안에 들어와 있으면 물체로 잡힌다
   — 캡처할 때 팔을 박스 밖으로 치워두거나 bounds 로 잘라낸다.
"""

import json
import os
import warnings

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

# 전부 실측 튜닝값이다. 도면값 아님 — 첫 캡처 로그를 보고 다시 잡는다.
DEFAULTS = {
    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
    'info_topic': '/camera/camera/aligned_depth_to_color/camera_info',
    'color_topic': '/camera/camera/color/image_raw',
    'base_frame': 'base_link',
    'camera_frame': 'camera_color_optical_frame',
    'out_dir': '',            # 비우면 <repo>/data/graspgenx_scene
    'scene': '00',            # <out_dir>/<scene>/ 아래로 쓴다
    'use_tf': True,           # False면 camera_pose=단위행렬 (world=카메라 프레임)
    # base_link 기준 작업공간 박스 [m]. 로봇 베이스가 테이블에 앉아 있다고 보고 잡은 값 — UNVERIFIED
    'x_min': 0.10, 'x_max': 0.90,
    'y_min': -0.50, 'y_max': 0.50,
    'z_min': -0.05, 'z_max': 0.70,
    'table_z': float('nan'),  # nan이면 박스 안 z 중앙값으로 자동 추정 (geometric 경로 전용)
    # 테이블면 위 이 높이부터 물체로 본다. **2026-08-09부터 yolo 경로도 이 값을 쓴다** —
    # segment_from_labels() 가 각 물체 주변 국소 테이블 기준(아래 yolo_table_ring_m)으로
    # 마스크 하단의 가림꼬리·경계오염 픽셀을 잘라낸다. 물체 자체가 낮으면(<15mm) 안 잡힌다.
    'obj_min_h': 0.02,
    # 로봇 팔·그리퍼는 작업공간 박스 **안**에 있어 xy 로 뺄 수 없다 — 높이로 자른다.
    # 0.12 는 2026-08-08 실측 튜닝값이다(씬 cmp_geo): 그리퍼가 상판 위 17.9~33.0 cm 에
    # 걸쳐 4182 px 짜리 obj_1 로 잡혔고, 같은 씬의 사과는 상판 위 최대 7.2 cm 였다.
    # 그 사이를 자르면 그리퍼만 사라진다. **기하 경로 전용이다 — yolo 경로는 이 값을
    # 절대 쓰지 않는다.** 서 있는 물체(콜라병 등, 20cm 넘음)가 이 기본값에 걸려 통째로
    # 잘려나가기 때문이다 — yolo 는 이미 클래스로 걸렀으므로 로봇 팔이 obj_N 이 될 일이 없다.
    # ⚠️ 그래도 남는 구멍: COCO 0 = person 이고 README 실측에 person 검출이 기록돼 있다 —
    # `classes` 기본값이 전체라 사람·팔이 obj_N 으로 들어올 수 있다. yolo 경로에서 이걸
    # 막는 수단은 `classes`(탐지 제한)와 `target_classes`(연산 제한)뿐이다.
    'obj_max_h': 0.12,
    # ── yolo 경로 전용: 물체별 "국소 테이블 기준" ────────────
    # 전역 table_z(박스 전체 중앙값) 하나로는 카메라 잔차·테이블 기울기가 있을 때
    # 물체마다 오차가 달라진다. 대신 obj_radius_m 밖 ~ +이 값 안의 배경(비물체) 픽셀만
    # 모아 그 물체 **주변**의 테이블 높이를 따로 잰다. obj_radius_m 이 nan 이면 이 링을
    # 만들 수 없어 전역 table_z 로 폴백한다.
    'yolo_table_ring_m': 0.03,
    # 링 안 배경 픽셀이 이보다 적으면(작업공간 가장자리·물체가 빽빽할 때) 국소값을
    # 못 믿고 전역 table_z 로 폴백한다.
    'yolo_min_ring_px': 20,
    # yolo 경로 전용: 각 물체 마스크를 실측 거리 기준 이만큼[m] 안쪽으로 깎는다
    # (erode_footprint_m). obj_radius_m(원형 크롭, 물체 중심 기준)과 역할이 겹치는 것 같지만
    # 다르다 — obj_radius_m 은 매끈한 원이라 물체 실제 윤곽(예: 각진 병)의 오목한 부분을 못
    # 따라간다. 이건 YOLO 마스크가 원래 그리는 윤곽을 그대로 깎아 들어간다. 물체의 평균
    # depth 로 그 지역 실측 픽셀 피치(depth/fx)를 구해 erode_m 을 픽셀 수로 환산해서 깎으므로,
    # 이미지 해상도·물체까지 거리와 무관하게 항상 "약 이만큼[mm]"을 깎는다(erode_footprint_m
    # 의 docstring 참고 — base XY 격자 리샘플링은 작은 물체를 통째로 지우는 버그가 나서 폐기).
    # 이 장면은 가림이 없다는 전제(2026-08-10 사용자 확인)라 경계를 통째로 깎아도 손해가
    # 없다 — occlusion 이 있는 씬이면 이 값을 올리면 물체 자체가 줄어들 수 있으니 주의.
    # UNVERIFIED: 3mm 는 아직 실기로 잰 값이 아니다. 첫 캡처 로그(물체별 px 수 변화, 특히
    # 작은 물체가 min_pixels 밑으로 떨어지는지)를 보고 다시 잡는다. 0이면 끄기(기존 동작).
    'mask_erode_m': 0.003,
    'min_pixels': 300,        # 이보다 작은 덩어리는 버린다(노이즈)
    # 단일 시점 depth 는 물체 뒤 가림영역을 전경 깊이로 메운다(occlusion shadow).
    # 그 꼬리가 덩어리에 붙어 OBB 를 부풀리고, 심하면 grasp 가 0개가 된다
    # (2026-08-05: 사과 8cm 가 12.6x18.8cm 로 잡혀 후보 0). 덩어리 중앙에서 이 반경 밖을 자른다.
    # RG2 개구가 0.102 m 라 이보다 큰 물체는 어차피 못 잡는다. nan 이면 끄기.
    # 0.05 는 씬 10(사과) 실측 튜닝값이다: nan→8개, 0.07→4개, **0.05→32개**, 0.04→1개.
    # 너무 조이면 OBB 가 작아져 후보가 사라진다. 씬 하나로 잡은 값이니 다른 물체에선 다시 본다.
    'obj_radius_m': 0.05,
    # ── yolo 경로 전용: 클래스별 실측 치수(선택) ─────────────
    # 'class:radius_m:height_m,...' 콤마 목록. **여기 있는 클래스는 obj_radius_m/obj_max_h
    # 전역값 대신 이 실측치를 쓴다.** 형태가 고정된 공산품(콜라병·컵 등)에서 정확도가 오른다 —
    # 자연물(사과·바나나 등, 개체마다 형태가 다르다)은 넣지 않는 편이 낫다.
    # 리스트가 아니라 문자열인 이유는 target_classes 와 같다(CLAUDE.md §4 rcl yaml 리스트 함정).
    # 정본은 ws 루트 `config/objects.yaml` 의 `dimensions:` — `graspx.launch.py` 가 이 문자열로
    # 변환해 넘긴다. 모르는 클래스거나 비어 있으면 **기존 동작 그대로**(전역 반경 + 상한 없음).
    # ⚠️ 이걸로 depth 자체의 결손(반사면이라 점이 아예 안 찍히는 것)은 못 고친다 — 그건
    # 물체별 마스크를 다듬는 것뿐이지 없는 점을 채워주지 않는다. 근본 해법은 알려진 형상을
    # 정합(ICP)해 메우는 것이고, 이건 그 전 단계의 값싼 개선이다.
    'class_dims': '',
    # 실측 height 위로 이만큼까지는 허용한다 — 측정 오차·바닥 접지 오차·국소 테이블 기준의
    # 잔차를 흡수한다. 너무 좁히면 진짜 물체 꼭대기까지 잘려나간다.
    'class_dims_margin_m': 0.03,
    'frames': 5,             # depth 를 이만큼 모아 픽셀별 중앙값. 정지 장면이라 가능하다
    'min_valid_ratio': 0.5,   # 프레임 중 이 비율 이상에서 유효해야 그 픽셀을 쓴다
    'timeout_sec': 10.0,
    # 세그멘테이션 백엔드. 'geometric' = 작업공간 박스 + connectedComponents (신경망 0개),
    # 'yolo' = yolo_seg 노드가 내는 라벨맵을 그대로 쓴다. 라벨 규약(101,102,...)이 같아
    # 변환이 필요 없다. yolo 는 학습한 클래스만 잡으므로 공구 seg 모델이 없으면 geometric 이 낫다.
    # 2026-08-08: 잡을 물체를 target_classes 로 지정해 하나씩 돌리는 운용으로 확정 —
    # geometric 은 클래스를 모른다(위 target_classes 주석 참고). 기본값을 yolo 로 바꿨다.
    'seg_source': 'yolo',
    'label_topic': '/yolo_seg/labels',
    # yolo_seg_node 가 내는 "라벨값 -> 클래스 이름" 매핑. target_classes 필터가 이걸 쓴다.
    'class_topic': '/yolo_seg/classes',
}

# LABEL_TABLE 은 **사람용**이다. GraspGenX 로더는 `obj_` 접두어가 아닌 라벨을 전부 무시하고
# (scene_loaders.py:95), 씬 점군에는 라벨과 무관하게 유효 depth 가 전부 들어간다.
# 즉 라벨 2와 라벨 0은 GraspGenX 에게 동일하다 — 오버레이로 "박스가 상판을 덮었나"를 눈으로
# 확인하려고 남긴다. 지우면 디버깅 수단이 사라진다.
LABEL_TABLE = 2
LABEL_OBJ_BASE = 100          # obj_1 -> 101, obj_2 -> 102 ... (샘플 데이터와 같은 규약)
MAX_OBJECTS = 155             # seg.png 가 uint8 이라 100+156 은 조용히 0으로 랩어라운드한다
MAX_DEPTH_BUFFER = 120        # 실패 경로에서 depth 프레임이 무한히 쌓이는 것을 막는다


def stamp_ns(stamp):
    """builtin_interfaces/Time -> int ns. 라벨맵과 클래스맵을 짝짓는 유일한 키다."""
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


def quat_to_matrix(x, y, z, w):
    """쿼터니언 -> 3x3. tf_transformations 가 이 랩탑에 없어 직접 만든다."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class SceneCapture(Node):
    def __init__(self):
        super().__init__('graspgenx_scene_capture')
        # dynamic_typing: `-p scene:=00` 은 YAML 이 정수 0 으로 파싱해 STRING 선언과 충돌한다.
        # `-p obj_min_h:=0` (DOUBLE 선언에 INTEGER) 도 같은 예외로 죽는다.
        # 타입은 선언이 아니라 params() 에서 DEFAULTS 기준으로 맞춘다.
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        self.bridge = CvBridge()
        self.depths = []            # 프레임별 원본. 마지막에 픽셀별 중앙값으로 합친다
        self.color = None
        self.color_is_raw = False   # raw가 오면 compressed 를 덮어쓰지 않는다
        self.K = None
        self.info_frame = None
        self.info_wh = None

        p = self.params()
        # 정지한 장면을 한 컷 뜨는 것이라 exact sync 를 쓰지 않는다. 각 토픽의 최신값을 쓴다.
        self.create_subscription(
            Image, p['depth_topic'], self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, p['color_topic'], self._on_color, qos_profile_sensor_data)
        # bag 은 컬러가 compressed 로만 녹화돼 있었다(constraints.md). 라이브 런치도 raw 를
        # 안 낼 수 있으므로 둘 다 구독하고 raw 를 우선한다 — 없는 토픽 구독은 무해하다.
        self.create_subscription(
            CompressedImage, p['color_topic'] + '/compressed',
            self._on_color_compressed, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, p['info_topic'], self._on_info, qos_profile_sensor_data)
        # seg_source='yolo' 일 때만 쓴다. 없는 토픽 구독은 무해하므로 항상 걸어둔다.
        self.yolo_labels_history = []   # [(stamp_ns, 라벨맵)] — best_labels() 가 최고를 고른다
        # stamp_ns -> {라벨값: 클래스 이름}. **이 클래스 자체는 안 쓴다** — grasp_bridge_node
        # 의 target_classes 가 쓴다. 이 노드를 CLI 로 단독 실행하면 클래스 필터는 없다.
        self.yolo_classes = {}
        self.create_subscription(
            Image, p['label_topic'], self._on_labels, qos_profile_sensor_data)
        self.create_subscription(
            String, p['class_topic'], self._on_classes, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_labels(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.yolo_labels_history.append((stamp_ns(msg.header.stamp), img))
        # depth 버퍼와 같은 상한을 재사용한다 — 실패 경로에서 무한히 쌓이는 것을 막는 목적이 같다.
        if len(self.yolo_labels_history) > MAX_DEPTH_BUFFER:
            del self.yolo_labels_history[:-MAX_DEPTH_BUFFER]

    def _on_classes(self, msg):
        """라벨맵과 **다른 토픽**이라 최신값 하나만 들면 짝이 어긋난다 — stamp 로 보관한다.

        `std_msgs/String` 은 범용 타입이라 아무나 이 토픽에 아무 문자열이나 발행할 수 있다.
        형태가 어긋난 것 하나로 구독 콜백이 죽으면 노드가 통째로 내려가므로 넓게 잡는다.
        """
        try:
            d = json.loads(msg.data)
            parsed = {int(o['label']): str(o['class']) for o in d['objects']}
            stamp = int(d['stamp_ns'])
        except (ValueError, TypeError, KeyError, AttributeError):
            return                       # 프레임 하나 버린다. 라벨맵 경로는 영향받지 않는다
        self.yolo_classes[stamp] = parsed
        if len(self.yolo_classes) > MAX_DEPTH_BUFFER:
            for k in list(self.yolo_classes)[:-MAX_DEPTH_BUFFER]:
                del self.yolo_classes[k]

    def _on_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # 16UC1 은 mm, 32FC1 은 이미 m. 이 나눗셈을 빠뜨리면 로봇이 1.6 km 밖을 향한다.
        self.depths.append(img.astype(np.float32) / 1000.0
                           if img.dtype == np.uint16 else img.astype(np.float32))
        # color/info 가 안 오면 ready() 가 영원히 False 라 타임아웃까지 쌓인다(1280x720 이면 GB 단위).
        if len(self.depths) > MAX_DEPTH_BUFFER:
            del self.depths[:-MAX_DEPTH_BUFFER]

    def merged_depth(self, n_frames, min_valid_ratio):
        """픽셀별 중앙값. D435i 는 밝고 무늬 없는 상판에서 프레임마다 다른 곳이 튄다 —
        단일 프레임을 쓰면 그 노이즈가 그대로 '물체'가 된다. 장면이 정지해 있으므로
        시간축 중앙값이 가장 싼 해법이다(구멍 메우기 + 잡음 제거를 동시에)."""
        st = np.stack(self.depths[-n_frames:])
        valid = st > 0
        cnt = valid.sum(0)
        with warnings.catch_warnings():
            # 전 프레임이 0인 픽셀은 All-NaN 이 정상이다(바로 아래에서 0으로 만든다)
            warnings.simplefilter('ignore', category=RuntimeWarning)
            med = np.nanmedian(np.where(valid, st, np.nan), axis=0)
        med[~np.isfinite(med)] = 0.0
        med[cnt < max(1, int(round(min_valid_ratio * len(st))))] = 0.0
        return med.astype(np.float32), len(st)

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.color_is_raw = True

    def _on_color_compressed(self, msg):
        if self.color_is_raw:
            return
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self.color = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _on_info(self, msg):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        # 잘못된 camera_info(예: depth/camera_info)를 물리면 fx 가 ~640 vs ~900 으로 다른데
        # 아무 에러 없이 통과한다. 프레임 이름과 해상도를 뒤에서 대조하려고 들고 있는다.
        self.info_frame = msg.header.frame_id
        self.info_wh = (int(msg.width), int(msg.height))

    def params(self):
        """파라미터를 DEFAULTS 의 타입으로 강제한다. dynamic_typing 의 짝이다."""
        out = {}
        for k, default in DEFAULTS.items():
            val = self.get_parameter(k).value
            if isinstance(default, str):
                # scene:=00 -> 정수 0 으로 들어온다. 샘플 데이터 규약(00, 01)대로 두 자리로 되돌린다.
                out[k] = f'{val:02d}' if isinstance(val, int) and k == 'scene' else str(val)
            elif isinstance(default, bool):
                out[k] = bool(val)
            else:
                out[k] = type(default)(val)
        return out

    def tf_ready(self, p):
        """TF 를 대기 조건에 넣는다.

        `Buffer.lookup_transform(timeout=…)` 은 이 구성에서 **무효다**: TransformListener 를
        spin_thread 없이 만들었으므로 /tf_static 구독이 이 노드에 붙어 있고, Humble 의 Buffer 는
        타임아웃 동안 sleep 폴링만 할 뿐 executor 를 돌리지 않는다 → 그 3초 동안 새 TF 가
        들어올 수 없다. depth 10프레임(≈330ms)이 /tf_static 디스커버리보다 먼저 차면
        헛기다린 뒤 실패하고 **이미 찍은 프레임을 버린다.** 그래서 spin 루프 안에서 확인한다.
        """
        if not p['use_tf']:
            return True
        return self.tf_buffer.can_transform(
            p['base_frame'], p['camera_frame'], rclpy.time.Time())

    def ready(self, n_frames, p):
        return (len(self.depths) >= n_frames and self.color is not None
                and self.K is not None and self.tf_ready(p))

    def camera_pose(self):
        """base_frame <- camera_frame 4x4. 실패하면 None."""
        p = self.params()
        if not p['use_tf']:
            self.get_logger().warn(
                'use_tf=False — camera_pose 가 단위행렬이 된다(world=카메라 광학 프레임). '
                '⚠️ x_min/x_max/y_*/z_* 는 base 기준 값인데 그대로 카메라 좌표에 적용되므로 '
                '대개 "박스 안 유효 depth 0개"가 된다. bounds 도 카메라 기준으로 다시 줄 것.')
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(
                p['base_frame'], p['camera_frame'],
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0))
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'TF lookup 실패: {e}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T


def to_base(depth, K, T_base_cam):
    """(H,W) depth[m] + K + 4x4 -> (H,W,3) base 프레임 XYZ.

    GraspGenX 쪽 구현과 **반드시 같아야 한다**:
      scene_loaders.depth_to_camera_xyz:24-33 (optical: x 오른쪽, y 아래, z 전방)
      scene_loaders.transform_xyz:36-38      (xyz @ R.T + t)
    한 줄로 분리해 둔 이유는 테스트가 이 식만 따로 겨냥할 수 있게 하려는 것이다 —
    `R.T` 를 `R` 로 바꾸는 실수는 사과 위치를 통째로 옮기면서도 조용하다.
    """
    H, W = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    xyz_cam = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], axis=-1)
    return xyz_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]


def pixel_to_base(depth, K, T_base_cam, u, v, half=2):
    """단일 픽셀 (u,v) -> base XY (z 는 버린다). 유효 depth 가 없으면 None.

    `select_by_point()`가 XY 평면 거리로만 매칭하므로 z 는 필요 없다. 클릭/VLA 픽셀
    바로 그 자리의 depth 가 구멍(0)일 수 있어 `(2*half+1)^2` 이웃의 median 을 쓴다
    (설계 출처: md/plans/2026-08-08-vla-integration.md §5 "5×5 median"). `to_base()`와
    같은 광학 규약(x 오른쪽, y 아래, z 전방) — 둘을 따로 유지하면 한쪽만 고치는 사고가 난다.
    """
    H, W = depth.shape
    v0, v1 = max(0, v - half), min(H, v + half + 1)
    u0, u1 = max(0, u - half), min(W, u + half + 1)
    valid = depth[v0:v1, u0:u1]
    valid = valid[valid > 0]
    if valid.size == 0:
        return None
    d = float(np.median(valid))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    xyz_cam = np.array([(u - cx) * d / fx, (v - cy) * d / fy, d])
    xyz_base = xyz_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]
    return float(xyz_base[0]), float(xyz_base[1])


def select_by_point(seg, label_map, X, Y, tx, ty, radius, margin):
    """지정 base XY `(tx, ty)` 에 가장 가까운 obj 하나만 남긴다 -> (seg, label_map, 진단).

    설계 출처: md/plans/2026-08-08-vla-integration.md §5(`select_by_point` 초안).
    `obj_N` 대신 base XY 를 선택 키로 쓰는 이유: 라벨 번호는 프레임마다 바뀌어
    VLA(다른 프로세스·다른 프레임 촬영 시점)와 대조가 안 되지만 좌표는 프레임 독립이다.

    각 후보의 위치는 마스크 픽셀의 base X/Y **중앙값**(centroid 근사)이다. 후보가 2개 이상일
    때만 거리·모호성으로 가린다: `radius` 안에 1등이 없으면 거부, 2등과의 거리차가 `margin`
    미만이면 모호로 보고 **역시 거부한다**(`refuse_ambiguous_match`, 끄려면 `margin=-inf`).
    **후보가 1개뿐이면 radius/margin 검사를 건너뛰고 그 하나를 쓴다** — 픽셀은 같은 class
    여럿을 구분하려고 받는 값이라 후보가 하나면 애매함 자체가 없다(설계 §9). 유일 후보를
    좌표가 좀 어긋났다고(구멍/물체 이동) 버리는 것보다 그 하나를 쓰는 게 낫다. (후보 0개는 거부.)

    `label_map` 의 `obj_` 로 시작하지 않는 항목(`ground`/`table` 등)은 지우지 않는다 —
    GraspGenX 점군에는 라벨과 무관하게 유효 depth 가 전부 들어가므로, 선택 안 된 물체의
    점을 지워도 충돌 판정용 점군 자체는 그대로 남아야 한다(seg 라벨만 0 으로 되돌린다).
    """
    cand = []
    for name, v in label_map.items():
        if not name.startswith('obj_'):
            continue
        m = seg == v
        if not m.any():
            continue
        cx, cy = float(np.median(X[m])), float(np.median(Y[m]))
        cand.append((float(np.hypot(cx - tx, cy - ty)), name, v))
    cand.sort(key=lambda t: t[0])
    if not cand:
        return None, None, f'({tx:+.3f},{ty:+.3f}) 근처에 후보(obj_) 자체가 없음'
    # 후보가 여럿일 때만 거리·모호성으로 가린다. 1개뿐이면 애매함이 없어(픽셀은 같은 class
    # 여럿을 구분하려는 값) radius/margin 검사를 건너뛴다 — 설계 §9. 유일 후보를 radius 밖
    # 이라고 버리는 것보다, class 가 맞은 그 하나를 쓰는 게 낫다.
    if len(cand) > 1:
        if cand[0][0] > radius:
            return None, None, f'({tx:+.3f},{ty:+.3f}) 반경 {radius:.3f}m 안에 물체 없음'
        if cand[1][0] - cand[0][0] < margin:
            return None, None, (f'모호: {cand[0][1]}({cand[0][0]:.3f}m) vs '
                                f'{cand[1][1]}({cand[1][0]:.3f}m) — 안 집는 게 낫다')
    out = seg.copy()
    for _, _, v in cand[1:]:
        out[out == v] = 0
    keep = {k: v for k, v in label_map.items() if not k.startswith('obj_') or v == cand[0][2]}
    return out, keep, f'{cand[0][1]} 선택 (지정점에서 {cand[0][0]:.3f}m)'


def workspace_mask(depth, K, T_base_cam, p):
    """base 프레임 작업공간 박스 안의 유효 depth 픽셀 마스크. 기하/yolo 경로가 공유한다."""
    xyz = to_base(depth, K, T_base_cam)
    X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    in_box = ((depth > 0)
              & (X >= p['x_min']) & (X <= p['x_max'])
              & (Y >= p['y_min']) & (Y <= p['y_max'])
              & (Z >= p['z_min']) & (Z <= p['z_max']))
    return in_box, X, Y, Z


def erode_footprint_m(m, depth, K, erode_m):
    """마스크 `m`을 `erode_m`[m] 만큼(실측 거리 기준) 안쪽으로 깎는다.

    ⚠️ 첫 시도는 base 프레임 X,Y 로 마스크 픽셀을 성긴 격자에 다시 뿌려 거기서 깎는
    것이었다 — 실측(2026-08-10, 이 함수의 이전 버전으로 재현)해보니 depth 픽셀 간격이
    격자 셀보다 성겨서(픽셀마다 점 하나씩만 찍히고 그 사이는 빈칸) erode 커널이 고립된
    점을 전부 지워버려 **작은 물체가 통째로 사라졌다**(36px 마스크가 erode 3mm에 0px).
    그래서 리샘플링 없이 **이미지 고유 격자**(빈틈이 없다)에서 깎는다: 물체의 평균 depth
    로 그 지역의 실측 픽셀 피치(depth/fx, m/px)를 구해 erode_m 을 픽셀 수로 환산한다.
    카메라가 테이블을 비스듬히 보면(eye-to-hand) 시선 방향 성분이 살짝 섞이지만
    (cos(기울기) 배 정도 언더/오버 컷), 격자 리샘플링의 빈틈 버그보다는 훨씬 안전하다.
    """
    if erode_m <= 0 or not m.any():
        return m
    fx = K[0, 0]
    mean_depth = float(depth[m].mean())
    if mean_depth <= 0:
        return m
    pitch = mean_depth / fx      # m/px 근사
    erode_px = max(1, int(round(erode_m / pitch)))
    kernel = np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
    return cv2.erode(m.astype(np.uint8), kernel) > 0


def segment_from_labels(labels, depth, K, T_base_cam, p, class_map=None):
    """yolo_seg 라벨맵(101,102,...)을 그대로 씬 seg 로 쓴다.

    라벨 규약이 이미 같으므로 하는 일은 네 가지다:
      1. **유효 depth 로 마스킹.** GraspGenX 는 depth 역투영으로 점군을 만든다. depth 가 0 인
         픽셀에 라벨이 붙어 있으면 그 물체의 점 수만 부풀고 점군에는 안 들어간다.
      2. **마스크 침식(erosion), 실측 거리 기준.** 반경 크롭(3번)은 물체 중심 기준 원형
         크롭이라 반경 안쪽은 그대로 통과한다 — 스테레오 depth 의 flying pixel(물체~배경
         경계에서 depth 가 섞이는 현상)은 그 반경 *안쪽* 마스크 경계에서 생기므로 반경
         크롭으로는 못 거른다. `mask_erode_m`(미터)만큼 마스크 경계를 통째로 깎는다
         (erode_footprint_m) — 물체의 평균 depth 로 그 지역 실측 픽셀 피치를 구해 미터를
         픽셀 수로 환산하므로 거리·해상도에 상관없이 "약 이만큼[mm]"을 일관되게 깎는다.
         **가림이 없는 씬 전제**(2026-08-10)라 경계를 깎아도 물체 자체가 사라지지 않는다.
      3. 반경 크롭. `class_map`으로 그 물체의 클래스를 알고 `p['class_dims']`에 실측치가
         있으면 **그 물체의 실측 반경**을 쓴다 — 없으면 전역 `obj_radius_m`(기존 동작).
      4. **물체마다 자기 주변의 국소 테이블 높이를 재서 그 위 obj_min_h 미만인 픽셀을
         잘라낸다.** 마스크 경계가 테이블로 살짝 새거나, 단일 시점 depth 의 가림꼬리가
         물체 밑에 붙는 걸 막는다. 클래스별 실측 height 를 알면 그 값 + 여유(margin) 위도
         같이 잘라낸다(이웃 물체·팔의 오염 방지) — **모르면 상한을 안 건다.** 서 있는 물체
         (콜라병 등)를 기하 경로 기준값(12cm) 하나로 자르면 통째로 잘려나가기 때문이다.
      5. 남은 라벨을 label_map 으로 정리 + 물체별 돌출 높이(+실측 대비 편차)를 진단에 남긴다.
    역할 분담은 **YOLO 가 "어느 물체인지", 기하가 "닿을 수 있는 곳·얼마나 튀어나왔는지"** 다.

    ⚠️ `class_dims` 는 depth 자체의 결손(반사면이라 점이 아예 안 찍히는 것)을 못 고친다 —
    마스크를 다듬을 뿐 없는 점을 채워주지 않는다. 돌출높이가 실측보다 한참 작게 나오면
    그 신호(진단문자열의 "⚠️ 실측 대비 얕음")로 depth 결손을 의심한다.
    """
    if labels is None:
        return None, None, ('seg_source=yolo 인데 라벨맵을 못 받았다. '
                            'yolo_seg_node 가 떠 있는지, label_topic 이 맞는지 확인할 것')
    if labels.shape != depth.shape:
        return None, None, (f'라벨맵 {labels.shape} != depth {depth.shape}. '
                            'yolo_seg 의 image_topic 이 depth 와 같은 정렬 해상도인지 확인할 것')

    # 유효 depth **와 작업공간 박스**의 교집합만 남긴다.
    # 박스를 안 걸면 배경(바닥·벽·먼 물체)까지 라벨에 들어가 물체 점군이 폭발한다 —
    # 2026-08-06 에 GraspGenX 가 41.7GB 를 할당하려다 죽었다.
    # 역할 분담: **YOLO 는 "어느 물체인지", 기하는 "닿을 수 있는 곳인지"** 를 정한다.
    in_box, X, Y, Z = workspace_mask(depth, K, T_base_cam, p)
    if not in_box.any():
        return None, None, '작업공간 박스 안에 유효 depth 가 0개다 — bounds 나 TF 를 의심한다'
    seg = np.where(in_box, labels, 0).astype(np.uint8)

    # 국소 테이블 기준 계산용 "배경(=물체 아닌 것)" 스냅샷. 아래 루프가 seg 를 지워
    # 나가기 **전에** 떠 둬야 한다 — 안 그러면 먼저 다듬은 물체의 흔적이 나중 물체의
    # 배경 판정에 섞이거나, 반대로 아직 안 다듬은 물체가 남의 링에 배경으로 잡힌다.
    bg0 = in_box & (seg <= LABEL_OBJ_BASE)
    global_table_z = float(np.median(Z[bg0])) if bg0.any() else float('nan')

    class_map = class_map or {}
    class_dims, dim_warns = parse_class_dims(p.get('class_dims', ''))
    dims_margin = float(p.get('class_dims_margin_m', 0.03))

    label_map, kept = {}, []
    default_radius = p['obj_radius_m']
    ring = p['yolo_table_ring_m']
    min_ring_px = int(p['yolo_min_ring_px'])
    obj_min_h = p['obj_min_h']
    for v in sorted(int(x) for x in np.unique(seg) if x > LABEL_OBJ_BASE):
        cls_name = class_map.get(v)
        dims = class_dims.get(cls_name) if cls_name else None
        radius = dims[0] if dims else default_radius

        m = seg == v
        erode_m = float(p.get('mask_erode_m', 0.0))
        if erode_m > 0:
            eroded = erode_footprint_m(m, depth, K, erode_m)
            seg[m & ~eroded] = 0
            m = eroded
        px = int(m.sum())
        cx = cy = float('nan')
        if np.isfinite(radius) and px >= p['min_pixels']:
            # 기하 경로와 **같은 반경 크롭**(반경은 클래스별 실측치가 있으면 그걸 쓴다).
            # COCO 라벨은 dining table 처럼 화면의 상당 부분을 한 인스턴스로 덮는데, 그
            # 점군을 그대로 넘기면 GraspGenX 가 죽는다(2026-08-06: 67,879 px 라벨에서
            # 41.7GB 할당 시도). RG2 개구가 0.102 m 라 그보다 큰 물체는 어차피 못 잡는다.
            cx, cy = float(np.median(X[m])), float(np.median(Y[m]))
            m = m & (np.hypot(X - cx, Y - cy) <= radius)
            seg[(seg == v) & ~m] = 0
            px = int(m.sum())
        if px < p['min_pixels']:
            seg[seg == v] = 0
            continue

        # ── 국소 테이블 기준: (이 물체의) radius 밖 ~ +yolo_table_ring_m 안의 배경 픽셀
        #    중앙값. 반경 크롭이 꺼져 있거나(radius=nan) 링 안 배경이 모자라면 전역으로 폴백.
        table_z, table_src = global_table_z, '전역'
        if np.isfinite(radius) and np.isfinite(cx):
            dist = np.hypot(X - cx, Y - cy)
            ring_mask = bg0 & (dist > radius) & (dist <= radius + ring)
            if int(ring_mask.sum()) >= min_ring_px:
                table_z, table_src = float(np.median(Z[ring_mask])), '국소'

        # ── 돌출 다듬기: 국소(또는 전역) 테이블 바로 위 얇은 층을 뺀다(하한, 항상).
        #    클래스별 실측 height 를 알면 그 위 margin 을 넘는 픽셀도 뺀다(상한, 선택) —
        #    이웃 물체·팔 그림자가 이 라벨에 섞여 들었을 때만 걸리고, 정상 범위는 안 건드린다.
        if np.isfinite(table_z):
            drop = m & (Z <= table_z + obj_min_h)
            if dims is not None:
                drop |= m & (Z > table_z + dims[1] + dims_margin)
            if drop.any():
                seg[drop] = 0
                m = seg == v
                px = int(m.sum())
            if px < p['min_pixels']:
                seg[seg == v] = 0
                continue

        idx = v - LABEL_OBJ_BASE
        label_map[f'obj_{idx}'] = v
        h = float(Z[m].max() - table_z) if np.isfinite(table_z) else float('nan')
        kept.append((idx, px, h, table_src, table_z, cls_name, dims))

    diag = [f'yolo 라벨맵 사용: 박스 안 픽셀 {int(in_box.sum())}, '
            f'라벨 {len(kept)}개 채택 (min_pixels={p["min_pixels"]})']
    for w in dim_warns:
        diag.append(f'  ⚠️ {w}')
    for idx, px, h, table_src, tz, cls_name, dims in kept:
        h_str = f'{h * 100:.1f} cm' if np.isfinite(h) else '?'
        tz_str = f'{tz:.4f} m({table_src})' if np.isfinite(tz) else '없음(배경 0개)'
        line = f'  obj_{idx}: {px:5d} px  테이블기준={tz_str}  돌출높이={h_str}'
        if dims is not None and np.isfinite(h):
            expect_cm = dims[1] * 100.0
            line += f' (실측 {cls_name}={expect_cm:.1f} cm)'
            if h < dims[1] * 0.5:
                line += ' ⚠️ 실측 대비 얕음 — depth 결손(반사면 등) 의심'
        diag.append(line)
    if not kept:
        diag.append('  ⛔ 채택 0개 — yolo 가 아무것도 못 잡았거나 depth 와 겹치는 픽셀이 없다')
    return seg, label_map, '\n'.join(diag)


def best_labels(frames):
    """`[(stamp_ns, 라벨맵), ...]` 중 물체 픽셀(라벨값 > 100)이 가장 많은 것 -> 같은 튜플.

    grasp 연산(수십 초)에 비하면 프레임 몇 장 더 보는 비용은 무시할 만하다 — 탐지가
    한 프레임에서만 흔들려도(조명 반사, 순간 블러) 최선의 컷을 쓰게 한다.
    ponytail: 프레임을 픽셀별로 합치지 않는다(depth median과 다르게 라벨은 정수 클래스ID라
    두 프레임을 섞으면 의미 없는 값이 나온다) — "라벨 픽셀 수 최대"인 프레임을 그대로 쓴다.
    stamp 를 같이 돌려주는 이유: 클래스맵이 별도 토픽이라 **고른 프레임의** stamp 로
    찾아야 짝이 맞는다. 배열만 돌려주면 어느 프레임이었는지 되찾을 수 없다.
    """
    if not frames:
        return None, None
    return max(frames, key=lambda f: int((f[1] > LABEL_OBJ_BASE).sum()))


def parse_class_dims(raw: str):
    """`'class:radius_m:height_m,...'` -> `({클래스: (반경, 높이)}, 경고 목록)`.

    형식이 틀린 항목은 버리고 경고에 남긴다 — 씬 캡처를 막을 정도로 치명적이지 않다
    (`objects.yaml`의 `detect`/`pick_default` 오타와 다르게, 이 값은 정확도를 조금 낮출
    뿐 아무것도 못 잡게 만들지는 않는다). 빈 문자열이면 `({}, [])` — 호출자는 전역
    `obj_radius_m`으로 폴백하고 상한을 안 건다(기존 동작).
    """
    dims, warns = {}, []
    for part in raw.split(','):
        part = part.strip()
        if not part:
            continue
        fields = part.split(':')
        if len(fields) != 3:
            warns.append(f"class_dims 항목 형식 오류(class:radius_m:height_m 아님): {part!r}")
            continue
        name, r_s, h_s = (f.strip() for f in fields)
        try:
            r, h = float(r_s), float(h_s)
        except ValueError:
            warns.append(f'class_dims 항목의 숫자를 못 읽었다: {part!r}')
            continue
        if not (r > 0 and h > 0):
            warns.append(f'class_dims 항목이 양수가 아니다: {part!r}')
            continue
        dims[name] = (r, h)
    return dims, warns


def filter_labels_by_class(labels, class_map, wanted):
    """`wanted` 클래스에 속하지 않는 라벨을 0 으로 지운다 -> (라벨맵, 진단문자열).

    **워커 호출 전에** 거른다. `select()` 의 `target` 은 이미 계산된 결과에서 고르는 것이라
    GraspGenX 가 물체 전부를 연산한 뒤다(물체당 수 초~수십 초). 여기서 지우면 그 연산
    자체가 사라진다 — "지정한 물체만 연산"의 의미가 이쪽이다.
    """
    keep, drop = [], []
    out = labels.copy()
    for v in (int(x) for x in np.unique(labels) if x > LABEL_OBJ_BASE):
        name = class_map.get(v)
        if name in wanted:
            keep.append(f'obj_{v - LABEL_OBJ_BASE}={name}')
        else:
            out[out == v] = 0
            drop.append(f'obj_{v - LABEL_OBJ_BASE}={name or "?"}')
    diag = (f'클래스 필터 target_classes={sorted(wanted)}: '
            f'남김 [{", ".join(keep) or "없음"}] / 버림 [{", ".join(drop) or "없음"}]')
    return out, diag


def segment(depth, K, T_base_cam, p, yolo_labels=None, class_map=None):
    """작업공간 박스 + 높이 임계 + connectedComponents -> (seg uint8, label_map, 진단문자열).

    p['seg_source'] == 'yolo' 면 기하 대신 yolo_labels 를 쓴다 (segment_from_labels).
    `class_map`(라벨값 -> 클래스 이름)은 yolo 경로에서 클래스별 실측 치수를 찾는 데만 쓴다 —
    기하 경로는 클래스를 모르므로 무시한다.
    """
    if p.get('seg_source') == 'yolo':
        return segment_from_labels(yolo_labels, depth, K, T_base_cam, p, class_map)
    H, W = depth.shape
    in_box, X, Y, Z = workspace_mask(depth, K, T_base_cam, p)
    if not in_box.any():
        return None, None, '작업공간 박스 안에 유효 depth 가 0개다 — bounds 나 TF 를 의심한다'

    table_z = p['table_z']
    auto = not np.isfinite(table_z)
    if auto:
        table_z = float(np.median(Z[in_box]))

    obj = in_box & (Z > table_z + p['obj_min_h'])
    max_h = p['obj_max_h']
    if np.isfinite(max_h):
        # 로봇 팔은 작업공간 박스 안에 있어 xy 로 뺄 수 없다. 높이로 자른다.
        obj &= Z < table_z + max_h
    # 3x3 열림 — depth 가장자리의 한두 픽셀짜리 점들이 덩어리로 잡히는 걸 막는다
    obj = cv2.morphologyEx(obj.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    n, comp = cv2.connectedComponents(obj.astype(np.uint8), connectivity=8)

    seg = np.zeros((H, W), np.uint8)
    # `~obj` 로 칠하면 obj_max_h 에 잘린 키 큰 것(로봇 팔·사람)까지 table 라벨을 받는다.
    # 실제로 table 라벨의 z 최대가 +0.376 m 로 찍혔다(2026-08-05). 낮은 것만 테이블이다.
    seg[in_box & (Z <= table_z + p['obj_min_h'])] = LABEL_TABLE
    label_map = {'ground': 0, 'table': LABEL_TABLE}
    kept = []
    radius = p['obj_radius_m']
    for c in range(1, n):
        m = comp == c
        px = int(m.sum())
        if px < p['min_pixels']:
            continue
        if np.isfinite(radius):
            # 중앙값 중심 — 꼬리가 붙어 있어도 평균보다 덜 끌려간다(2026-08-05 실측:
            # 꼬리 포함 덩어리의 중앙값이 실제 사과 중심에서 1.7cm, 평균은 그 이상).
            mx, my = float(np.median(X[m])), float(np.median(Y[m]))
            m &= np.hypot(X - mx, Y - my) <= radius
            px = int(m.sum())
            if px < p['min_pixels']:
                continue
        if len(kept) >= MAX_OBJECTS:      # uint8 랩어라운드 방지. 예외 없이 조용히 틀린다
            break
        idx = len(kept) + 1
        seg[m] = LABEL_OBJ_BASE + idx
        label_map[f'obj_{idx}'] = LABEL_OBJ_BASE + idx
        # 중심 좌표가 없으면 "어느 게 사과인지" 를 알 방법이 없다 — 튜닝의 유일한 근거다
        kept.append((idx, px, float(Z[m].max() - table_z),
                     float(X[m].mean()), float(Y[m].mean()), float(Z[m].mean())))

    diag = [f"table_z={table_z:.4f} m ({'자동추정' if auto else '지정'}), "
            f"박스 안 픽셀 {int(in_box.sum())}, 물체 후보 덩어리 {n - 1}개 중 {len(kept)}개 채택"]
    # ⚠️ "표면중심"이지 물체 중심이 아니다. 단일 시점이라 보이는 면만 평균되고,
    #    반지름 r 인 구라면 시선축을 따라 카메라 쪽으로 약 2r/3 만큼 앞에 앉는다
    #    (사과 r=2.5cm -> 1.7cm). **이 편차를 캘리브 오차로 오해해 보정하지 말 것.**
    for idx, px, h, ox, oy, oz in kept:
        diag.append(f'  obj_{idx}: {px:5d} px  표면중심 base=({ox:+.3f}, {oy:+.3f}, {oz:+.3f}) m  '
                    f'테이블 위 최대높이 {h * 100:.1f} cm')
    if not kept:
        diag.append('  ⛔ 채택 0개 — obj_min_h 를 낮추거나 min_pixels 를 줄이거나 bounds 를 확인한다')
    return seg, label_map, '\n'.join(diag)


def default_out_dir():
    """`out_dir` 파라미터가 비었을 때 쓰는 저장 위치.

    설치 경로가 계정마다 다를 수 있어(cobot2_ws 위치) 하드코딩하지 않는다 — 이 파일
    자신의 경로에서 두 단계 위로 올라가 패키지 소스 루트를 찾는다.

    ⚠️ `abspath`가 아니라 **`realpath`를 써야 한다.** `--symlink-install`(이 워크스페이스
    기본값)에서 `install/setup.bash`는 `build/<pkg>/<pkg>/`를 PYTHONPATH에 먼저 얹고,
    그 경로의 각 파일은 `src/`를 가리키는 심볼릭 링크다. `abspath(__file__)`는 심볼릭
    링크를 풀지 않으므로 결과가 `build/graspgenx_perception/data/graspgenx_scene`가
    되는데, 이 디렉토리는 `colcon build`/`rm -rf build`로 지워지는 임시 산출물이라
    저장한 씬이 조용히 사라진다(2026-08-07, `python3 -c` 로 직접 재현·확인). `realpath`로
    링크를 풀면 진짜 소스 트리(`src/graspgenx_perception`) 밑으로 저장된다.
    """
    repo = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    return os.path.join(repo, 'data', 'graspgenx_scene')


def write_scene(out, depth, color_rgb, seg, K, T, label_map, bounds):
    """loader가 요구하는 파일 4개를 쓴다. 카메라 없이도 테스트할 수 있게 분리했다."""
    os.makedirs(out, exist_ok=True)
    np.save(os.path.join(out, 'depth.npy'), depth.astype(np.float32))
    # imwrite 는 실패해도 예외 없이 False 만 준다 — 그냥 두면 파일 2개짜리 디렉토리를
    # 만들어 놓고 "저장 완료"라고 보고하고, 실패는 한참 뒤 GraspGenX 로더에서 터진다.
    for name, img in (('rgb.png', cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)),
                      ('seg.png', seg)):
        if not cv2.imwrite(os.path.join(out, name), img):
            raise IOError(f'{os.path.join(out, name)} 쓰기 실패')
    with open(os.path.join(out, 'meta_data.json'), 'w') as f:
        json.dump({
            'intrinsics': np.asarray(K).tolist(),
            'camera_pose': np.asarray(T).tolist(),
            'label_map': label_map,
            'scene_bounds': list(bounds),
        }, f, indent=2)
    return out


def run(node):
    """캡처 본체. 성공 0, 실패 1. 정리(shutdown)는 main 이 책임진다."""
    p = node.params()
    n_frames = max(1, p['frames'])
    end = node.get_clock().now() + rclpy.duration.Duration(seconds=p['timeout_sec'])
    while rclpy.ok() and not node.ready(n_frames, p) and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready(n_frames, p):
        node.get_logger().error(
            f"수신 실패 (depth {len(node.depths)}/{n_frames} 프레임, "
            f"color={node.color is not None}, info={node.K is not None}, "
            f"tf={node.tf_ready(p)}) — 카메라 런치와 ROS_DOMAIN_ID 를 확인한다. "
            f"tf 만 False 면 {p['base_frame']} <- {p['camera_frame']} 이 없는 것이다")
        return 1

    T = node.camera_pose()
    if T is None:
        return 1

    depth, used = node.merged_depth(n_frames, p['min_valid_ratio'])
    color, K = node.color, node.K
    node.get_logger().info(
        f'depth {used} 프레임 중앙값 병합 — 유효 픽셀 '
        f'{100.0 * (depth > 0).mean():.1f}% (단일 프레임 '
        f'{100.0 * (node.depths[-1] > 0).mean():.1f}%)')

    if color.shape[:2] != depth.shape[:2]:
        node.get_logger().error(
            f'컬러 {color.shape[:2]} != depth {depth.shape[:2]} — '
            'aligned_depth_to_color 토픽이 맞는지 확인한다')
        return 1
    # camera_info 가 depth 와 짝인지 본다. 엉뚱한 info(예: depth/camera_info)를 물리면
    # fx 가 ~640 vs ~900 으로 달라 grasp 가 통째로 어긋나는데 아무 에러도 안 난다.
    if node.info_wh != (depth.shape[1], depth.shape[0]):
        node.get_logger().error(
            f'camera_info {node.info_wh} != depth {(depth.shape[1], depth.shape[0])} — '
            f"info_topic({p['info_topic']}) 이 depth 와 짝이 아니다")
        return 1
    if p['use_tf'] and node.info_frame != p['camera_frame']:
        node.get_logger().warn(
            f"camera_info.frame_id='{node.info_frame}' 인데 camera_frame 파라미터는 "
            f"'{p['camera_frame']}' 이다 — TF 를 다른 프레임에서 뜨고 있을 수 있다")

    # yolo 는 최신 한 장이 아니라 최근 n_frames 장 중 탐지 픽셀이 가장 많은 프레임을 쓴다
    # (best_labels 참고) — depth 를 n_frames 만큼 모으는 동안 어차피 라벨도 같이 쌓인다.
    frame_stamp, labels = (best_labels(node.yolo_labels_history[-n_frames:])
                           if p.get('seg_source') == 'yolo' else (None, None))
    # class_dims(클래스별 실측 치수)를 쓰려면 그 프레임의 클래스맵이 필요하다 — 라벨맵과
    # 별도 토픽이라 stamp 로 짝을 맞춘다(grasp_bridge_node.compute() 와 같은 패턴).
    class_map = (node.yolo_classes.get(frame_stamp, {})
                 if p.get('seg_source') == 'yolo' else {})
    seg, label_map, diag = segment(depth, K, T, p, labels, class_map)
    if seg is None:
        node.get_logger().error(diag)
        return 1

    out = os.path.join(p['out_dir'] or default_out_dir(), p['scene'])
    write_scene(out, depth, color, seg, K, T, label_map,
                [p['x_min'], p['y_min'], p['z_min'],
                 p['x_max'], p['y_max'], p['z_max']])

    # 유효 depth 가 0개면 `min()` 이 "zero-size array to reduction operation minimum"
    # 으로 던진다 — **표시용 로그 한 줄 때문에 캡처 전체가 죽는다.** 그런 프레임이야말로
    # (카메라 가림·전면 무효 depth) 로그가 필요한 순간이라 여기서 막는다.
    valid = depth[depth > 0]
    rng = f'{valid.min():.3f}~{valid.max():.3f} m' if valid.size else '유효 depth 0개 ⚠️'
    node.get_logger().info(
        f"{diag}\n저장: {out}\n"
        f"  depth {depth.shape} {depth.dtype} 범위 {rng}\n"
        f"  camera_pose t={np.round(T[:3, 3], 4).tolist()}\n"
        f"다음: cd isaac_ros-dev/src/GraspGenX && uv run python scripts/demo_scene_pc.py "
        f"--sample_data_dir {os.path.dirname(out)} --gripper_name onrobot_RG2 "
        f"--scene {p['scene']} --moe_obb_density dense")
    return 0


def main():
    rclpy.init()
    node = SceneCapture()
    try:
        return run(node)
    finally:
        # 예외(잘못된 -p 값, imwrite 실패 등)로 빠져나가도 컨텍스트를 남기지 않는다
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
