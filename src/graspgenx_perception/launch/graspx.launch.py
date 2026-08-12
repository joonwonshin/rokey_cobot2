"""graspgenx_perception 의 인식·파지 계산 층을 띄운다.

**`run_yolo` 와 `run_bridge` 를 한 머신에서 동시에 true 로 두면 안 된다.**
`yolo_seg_node` 는 ultralytics 때문에 **컨테이너 전용**이고, `grasp_bridge_node` 는
GraspGenX 워커를 `uv` 로 띄우는데 **컨테이너에 uv 가 없다**. 그래서 반씩 나눠 띄운다:

  # A) YOLO 세그 (기본값) — 컨테이너와 호스트를 각각 띄운다
  #   컨테이너 (run_yolo=true, run_bridge=false 가 기본이라 인자 없이 그대로 써도 된다):
  #   FASTRTPS_DEFAULT_PROFILES_FILE 필수 (없으면 프레임 0장)
  #   탐지 대상은 ws 루트 `config/objects.yaml` 의 detect 가 정본이다 — 인자로 안 준다
  ros2 launch graspgenx_perception graspx.launch.py
  #   호스트: run_bridge 는 기본 false 라 켜야 하고, yolo_seg_node 는 컨테이너 쪽 것만 쓴다
  ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true
  #   ⚠️ pick_fsm 과 같이 쓸 거면 여기 `target_classes:=`/`seg_source:=` 를 주지 않는다 —
  #      task_manager 가 PERCEIVE 마다 자기 타겟으로 덮어쓴다(2026-08-09). 타겟은
  #      `/pick/target` 이나 rqt 패널에서 고른다. 브리지 단독으로 돌릴 때만 여기서 준다.
  #   live_viz 기본 true — 뜨는 순간부터 http://<이 머신>:8080 에 매 캡처가 자동으로
  #      그려진다(실제 포인트클라우드 + grasp 후보 + 1등 콜리전 메시). 끄려면 live_viz:=false.

  # B) 기하 세그 — 호스트 한 대로 끝난다. 물체 클래스를 모른다(target_classes 못 씀)
  ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true \
      seg_source:=geometric

카메라(`realsense2_camera`)와 로봇 bringup 은 **여기서 띄우지 않는다.** 로봇 bringup 은
실기 모션이라 사람이 직접 실행해야 하고, 카메라는 다른 파이프라인과 공유하기 때문이다.

전제:
  1. 호스트에서 `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`
  2. 호스트에서 로봇 bringup (base_link <- camera_color_optical_frame TF 가 필요하다)
  3. 양쪽 `ROS_DOMAIN_ID=93` (컨테이너는 이미 박혀 있고, 호스트에서 export 를 빠뜨린다)
  4. **컨테이너**에 `FASTRTPS_DEFAULT_PROFILES_FILE` (호스트 쪽은 없어도 됐다 — README 실측)

seg_source:
  geometric — 작업공간 박스 + connectedComponents. 신경망 0개. 클래스를 모른다
  yolo      — yolo_seg_node 의 `/yolo_seg/labels` 를 그대로 쓴다. **기본값**
              (2026-08-08 확정: 잡을 물체를 target_classes 로 지정해 하나씩 돌린다)
"""

import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_objects_file() -> str:
    """ws 루트의 `config/objects.yaml`. 없으면 `''`(기능 끔).

    패키지 안이 아니라 ws 루트에 두는 이유는 그 파일 머리말에 적어 뒀다 — 요지는
    `--symlink-install` 이어도 패키지 config 는 `build/` 복사본이라 고쳐도 안 먹는다는 것.
    share = <ws>/install/<pkg>/share/<pkg> 라 4단계 위가 ws 루트다. 이 배치가 아닐 수도
    있으니 존재 확인을 하고, `COBOT2_OBJECTS` 로 언제든 덮어쓸 수 있게 둔다.
    """
    env = os.environ.get('COBOT2_OBJECTS')
    if env:
        return env
    share = get_package_share_directory('graspgenx_perception')
    path = os.path.abspath(os.path.join(share, '..', '..', '..', '..',
                                        'config', 'objects.yaml'))
    return path if os.path.isfile(path) else ''


def default_class_dims() -> str:
    """`objects.yaml`의 `dimensions:` -> `'class:radius_m:height_m,...'` 문자열.

    형태가 고정된 공산품(병·컵 등)의 실측 반경·높이만 적는다 — 자연물(사과 등)은
    개체마다 형태가 달라 클래스 전체를 대표하는 값을 못 잡으므로 안 적는 편이 낫다.
    경로 규칙은 `default_objects_file()`과 같다. 항목 하나가 이상해도(radius/height
    누락 등) 그 항목만 건너뛰고 나머지는 살린다 — 파싱 자체는
    `capture_graspgenx_scene.parse_class_dims()`가 한 번 더 하므로 여기서 엄격하게
    막을 필요는 없다(다만 여기서 걸러 두면 노드 로그가 아니라 런치 로그에서 먼저 보인다).
    """
    path = default_objects_file()
    if not path or not os.path.isfile(path):
        return ''
    try:
        doc = yaml.safe_load(open(path, encoding='utf-8')) or {}
    except Exception as exc:                                        # noqa: BLE001
        print(f'[graspx.launch] {path} 를 못 읽었다 ({exc}) — class_dims 비움')
        return ''
    parts = []
    for name, v in (doc.get('dimensions') or {}).items():
        r, h = (v or {}).get('radius'), (v or {}).get('height')
        if r is None or h is None:
            print(f'[graspx.launch] dimensions.{name} 에 radius/height 가 없다 — 건너뜀')
            continue
        parts.append(f'{name}:{r}:{h}')
    return ','.join(parts)


ARGS = {
    'seg_source': ('yolo', "'geometric' 또는 'yolo'"),
    'run_yolo': ('true', 'yolo_seg_node 를 띄울지 (seg_source=yolo 면 반드시 true). **기본값**'),
    # 기본 false — 컨테이너(uv 없음)에서 무심코 같이 뜨는 걸 막는다. 호스트에서 브리지를
    # 띄우려면 run_bridge:=true 를 명시한다 (위 A) 예시).
    'run_bridge': ('false', 'grasp_bridge_node 를 띄울지'),
    'image_topic': ('/camera/camera/color/image_raw', 'yolo_seg 가 구독할 컬러 토픽'),
    'publish_overlay': ('true', '오버레이(JPEG) 발행 — rqt 로 보려면 true'),
    'device': ('0', "'0'=첫 GPU, 'cpu'"),
    'conf': ('0.1', 'YOLO 신뢰도 임계'),
    # COCO 80종의 **인덱스** 목록. 비우면 전체. banana=46, apple=47, cup=41, bottle=39,
    # bowl=45, orange=49, scissors=76 (2026-08-07 yolo11n-seg.pt 의 model.names 로 확인).
    # 이 가중치엔 공구 5종이 없다 — 없는 물체는 어떤 인덱스로도 못 잡는다.
    # 탐지 대상의 정본. 이름으로 적고 노드가 가중치로 인덱스를 뽑는다 — 아래 `classes` 는
    # 이 파일이 없을 때만 쓰는 옛 경로다(인덱스를 손으로 적는 쪽).
    'objects_file': (default_objects_file(),
                     "탐지 대상·기본 타겟의 정본 yaml. 비우면 아래 classes 를 쓴다"),
    'classes': ('[]', "objects_file 을 안 쓸 때의 COCO **인덱스** 필터. "
                      "예: classes:='[46,47,49]' (banana, apple, orange)"),
    'min_pixels': ('300', '이보다 작은 덩어리는 물체로 안 본다'),
    # `classes` 와 역할이 다르다: `classes` 는 **무엇을 탐지할지**(yolo, 넓게 두는 값),
    # `target_classes` 는 **무엇을 잡을지**(bridge, 좁게 두는 값). 브리지는 대상 외 라벨을
    # 워커에 넘기기 전에 지우므로 grasp 연산(물체당 수 초~수십 초)이 실제로 줄어든다.
    'target_classes': ('', "grasp 를 계산할 클래스 이름. 콤마 구분, 비우면 전부. 예: 'apple,cup'"),
    # seg_source=geometric 전용. 이걸 안 걸면 그리퍼가 obj_1 로 잡힌다(2026-08-08 실측).
    'obj_max_h': ('0.12', '상판 위 이 높이를 넘는 픽셀은 버린다 — 로봇 팔/그리퍼 self-filter'),
    # seg_source=yolo 전용(2026-08-09). objects.yaml 의 dimensions 가 정본 — 여기 직접
    # 문자열로 줄 수도 있지만 그러면 두 곳에 값이 생겨 어긋난다(target_classes 와 같은 함정).
    'class_dims': (default_class_dims(),
                   "'class:radius_m:height_m,...'. 여기 있는 클래스는 obj_radius_m/obj_max_h "
                   '전역값 대신 실측치를 쓴다. 형태 고정 물체만(자연물은 넣지 않는다)'),
    'class_dims_margin_m': ('0.03', 'class_dims 의 height 위로 이만큼까지 허용(측정오차 흡수)'),
    'out_dir': ('', '씬 4파일 저장 위치. 비우면 <repo>/data/graspgenx_scene '
                    '(2026-08-07부터 항상 영구 저장 — 임시 디렉토리 아님)'),
    # 워커 기동 인자라 워커가 이미 떠 있으면 안 먹는다 — 브리지·워커를 새로 띄울 때만 유효.
    # **기본 켜짐**(2026-08-09) — 발표·디버깅에 상시 쓰기로 함. 끄려면 live_viz:=false.
    'live_viz': ('true', "compute() 마다 viser 웹뷰어(실제 포인트클라우드+grasp 후보)를 "
                         "자동 갱신. http://<이 머신>:live_viz_port"),
    'live_viz_port': ('8080', 'live_viz 웹뷰어 포트'),
}


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ARGS}

    yolo = Node(
        package='graspgenx_perception', executable='yolo_seg_node', name='yolo_seg_node',
        output='screen',
        condition=IfCondition(cfg['run_yolo']),
        parameters=[{
            'image_topic': cfg['image_topic'],
            'publish_overlay': cfg['publish_overlay'],
            'device': cfg['device'],
            'conf': cfg['conf'],
            # 런치 인자는 문자열이라 그냥 넘기면 노드에 STRING 으로 도착하고
            # `[int(c) for c in classes]` 가 문자 단위로 돌다 ValueError 로 죽는다.
            # ParameterValue(value_type=List[int]) 가 YAML 로 파싱해 정수 배열로 만든다
            # (2026-08-07 `LaunchContext` 로 '[]'/'[46,47]' 둘 다 직접 평가해 확인).
            'classes': ParameterValue(cfg['classes'], value_type=List[int]),
            'objects_file': cfg['objects_file'],
            'min_pixels': cfg['min_pixels'],
        }],
    )

    # 이 노드는 로봇을 움직이지 않는다 — grasp 포즈를 계산해 발행할 뿐이다.
    # 실행 파일 이름에 `.py` 가 없다: 소스가 패키지 안으로 들어오면서 console_scripts
    # 진입점이 됐다 (2026-08-07, setup.py 주석 참고).
    bridge = Node(
        package='graspgenx_perception', executable='grasp_bridge_node', name='grasp_bridge_node',
        output='screen',
        condition=IfCondition(cfg['run_bridge']),
        parameters=[{
            'seg_source': cfg['seg_source'],
            'min_pixels': cfg['min_pixels'],
            'obj_max_h': cfg['obj_max_h'],
            'out_dir': cfg['out_dir'],
            # 문자열로 선언한다 — 리스트였다면 rcl YAML 파서의 타입 함정을 또 밟는다
            # (CLAUDE.md §4). ParameterValue 로 감쌀 필요도 없다.
            'target_classes': cfg['target_classes'],
            'class_dims': cfg['class_dims'],
            'class_dims_margin_m': ParameterValue(cfg['class_dims_margin_m'], value_type=float),
            'live_viz': ParameterValue(cfg['live_viz'], value_type=bool),
            'live_viz_port': ParameterValue(cfg['live_viz_port'], value_type=int),
        }],
    )

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [yolo, bridge]
    )
