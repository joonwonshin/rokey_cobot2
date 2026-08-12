# cobot2_ws 정리 — pick_fsm 기준 패키지 지도 + 삭제 후보

CLAUDE.md 5절 "패키지 지도" TODO에 대한 답. 근거는 `grep`으로 실제 import/launch/package.xml
의존을 추적한 것 — 코멘트에 이름이 나온다고 "쓰인다"로 치지 않았다.

## 1. 활성 파이프라인 (pick_fsm 기준, 삭제 금지)

```
pick_fsm (task_manager, robot_safety_node)
 ├─ pick_fsm_msgs            ComputeGrasp 인터페이스
 ├─ voice_processing         /get_keyword 서비스 (task_manager 가 client)
 ├─ cobot_rg2/rg2/*          bringup/moveit (M0609+RG2+D435i), doosan-robot2·onrobot-ros2 벤더 드라이버 포함
 ├─ cumotion                 planning_pipeline:=isaac_ros_cumotion 옵션 경로 (README 9절, pick_fsm.yaml 주석)
 └─ (grasp_source=compute_grasp 경로) graspgenx_perception
     └─ object_detection     ⚠️ 코드는 안 쓰지만 **패키지 자체는 필요** —
                              yolo_seg_node.py 가 get_package_share_directory('object_detection')
                              로 가중치(.pt) 경로를 찾는다 (graspgenx_perception/package.xml:22)
```

검증 방법: `grep -rn "object_detection\." src/graspgenx_perception` → `DEFAULT_WEIGHT_PKG` 한 곳뿐,
`SegmentationNode`/`get_segmentation` 호출은 어디서도 없음.

## 2. 삭제 후보 (pick_fsm 파이프라인 어디서도 안 씀)

| 패키지 | 근거 | 명령 |
|---|---|---|
| `pick_and_place_voice` | 이미 `COLCON_IGNORE` — object_detection/robot_control/voice_processing 통짜 복사본, 완전 죽은 코드 | `git rm -r src/pick_and_place_voice` |
| `pick_and_place_text` | 독자 프로토타입. 어떤 package.xml/launch/import 도 참조 안 함 (grep 0건) | `git rm -r src/pick_and_place_text` |
| `robot_control` (최상위) | pick_fsm 은 이 패키지를 **import 하지 않는다.** task_manager.py:107-108 주석의 `JReady`/`BUCKET_POS`는 "값의 출처" 언급일 뿐 — CLAUDE.md 4절에 "pick_fsm.yaml 값을 여기에 맞추지 말 것"이라고 이미 명시(둘이 다른 게 정상). RG2 저수준 제어는 `pick_fsm/rg2.py`가 대체 | `git rm -r src/robot_control` |
| `rokey` | ~~빈 패키지, entry_points가 존재하지 않는 모듈을 가리킴~~ **정정(2026-08-08 삭제 실행 중 발견)**: `rokey/basic/{get_current_pos,jog_complete}.py`는 실제로 존재하는 독립 실습용 스크립트였다(`find -maxdepth 2`로 depth-3 파일을 못 본 내 오판). 삭제 근거는 이것 대신 — **pick_fsm/graspgenx_perception/cobot_rg2 등 활성 패키지 중 `import rokey`/`from rokey`가 0건**(grep 확인)이라는 점 하나로 충분하다 | `git rm -r src/rokey` |
| `od_msg` | 유일한 소비자가 `object_detection/segmentation.py`(죽은 코드, 위 1절)와 `robot_control`/`pick_and_place_text`(둘 다 위에서 삭제 대상). graspgenx_perception/pick_fsm 은 안 씀 | `git rm -r src/od_msg` (object_detection도 같이 정리할 때만) |
| `usb_cam` | 카메라는 `realsense2_camera`(cobot_rg2/rg2/m0609_rg2_bringup/launch/camera.launch.py)로 구동. `usb_cam` 문자열 참조가 ws 전체에 0건 — 서드파티 V4L 드라이버 전체가 그냥 얹혀만 있음 | `git rm -r src/usb_cam` |
| `webcam_perception` | C270 실험 트랙, graspgenx가 YOLO-seg로 대체(사용자 확인, 2026-08-08). `pick_fsm/graspgenx_perception/루트` README 어디에도 `webcam`/`C270`/`sam_mask` 언급 0건 | `git rm -r src/webcam_perception` |

### object_detection — 패키지는 남기되 죽은 코드만 제거

`resource/`(가중치 share 경로)만 graspgenx_perception이 쓴다. `object_detection/{detection,yolo,realsense,segmentation,yolo_seg}.py`와 `setup.py`의 두 entry_point(`object_detection`, `object_detection_seg`)는 아무도 실행 안 함.
→ **od_msg를 지우려면 object_detection도 같이 손대야 한다** (segmentation.py가 od_msg를 import).

```bash
# object_detection을 "가중치 보관용 빈 패키지"로 축소
cd src/object_detection
git rm object_detection/detection.py object_detection/yolo.py \
       object_detection/realsense.py object_detection/segmentation.py \
       object_detection/yolo_seg.py
# setup.py entry_points 두 줄, package.xml의 <exec_depend>od_msg</exec_depend> 도 같이 지운다 (수동 편집)
```

## 3. 보류 — 삭제 전 확인 필요

- **`cumotion`**: pick_fsm 기본 경로(`planning_pipeline:=ompl`)에서는 안 쓰지만, README 9-1b 이전 절과
  `config/testcommand.md`가 이 패키지를 전제로 함 → **GPU 파이프라인을 계속 쓸 계획이면 유지**.

## 4. 루트 잡동사니 (src 밖, 확장자 없는 메모/터미널 캡처 5개)

전부 git-tracked. 코드 아니고 일회성 메모 — 삭제하거나 `md/`로 옮긴다.

| 파일 | 내용 |
|---|---|
| `common_memo` | 5바이트, "memo" |
| `docker_gpu` | isaac_ros 컨테이너 attach 방법 메모 |
| `test_grap_plan` | 실기 점검 명령 스니펫 (T0~T3) |
| `typescript` | `script` 명령 오작동 캡처 (의미 없는 터미널 로그) |
| `yolo_seg_grasp` | graspx 실행 명령 스니펫 |

```bash
git rm typescript common_memo          # 순수 잡음
git mv docker_gpu md/context/docker_gpu.md
git mv test_grap_plan md/context/test_grap_plan.md
git mv yolo_seg_grasp md/context/yolo_seg_grasp.md
```

## 5. CLAUDE.md 갱신 필요

1절 "`src/`에 패키지 9개 존재" — 실제로는 14개(정리 후 대략 8~9개로 줄어듦). 정리 실행 후 이 줄을
현재 목록으로 다시 쓸 것.

---
확신도: 추론(grep·package.xml·launch 파일로 근거 확인, 단 colcon build/실행으로 재확인은 안 함)
내가 채워넣은 가정: (1) "안 쓰임" 판정 기준을 grep 0건으로 잡음 — 동적 import(문자열 조립)는 못 잡는다 (2) webcam_perception/cumotion을 "보류"로 분류 (3) 루트 잡동사니 5개를 "메모"로 판단
확인 요청: webcam_perception 계속 씀?
