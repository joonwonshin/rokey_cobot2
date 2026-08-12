<!-- meta
updated: 2026-08-08
status:  live
owns:    문서를 어디에 쓰는지 규칙 · md/ 전체 문서 지도
-->

# 문서 지도 — `md/` 무엇이 어디에 있나

> **이 파일이 문서의 진입점이다.** 새 문서를 만들면 여기에 한 줄 추가한다.
> 규칙: **같은 사실을 두 문서에 쓰지 않는다.** 아래 "단일 출처" 열이 그 사실을 소유한 문서다.
> 다른 문서는 값을 베끼지 말고 링크한다 — 사본은 반드시 갈라진다([[ws/cobot2/errors-log]] 참고).
>
> ws 루트의 `README.md`는 **팀원용 실행 절차**다(재현 명령 · 인자표 · 기능확인). 이 문서와 역할이 다르다.
> 패키지 레퍼런스(인터페이스·파라미터·빌드·검증 상태)는 **`src/PACKAGES.md`**다(2026-08-09,
> 패키지별 README 4개를 통합 — `cobot_rg2`·`cumotion`·`graspgenx_perception`·`pick_fsm`
> 폴더의 `README.md`는 이제 그 문서를 가리키는 포인터만 남아 있다). 이 세 문서(루트 실행
> 가이드 / `src/PACKAGES.md` 레퍼런스 / `md/` 지식베이스)가 이 ws의 문서 전체다.

## 상시 문서 (계속 갱신)

| 문서 | 단일 출처 | 갱신 방식 |
|---|---|---|
| [state.md](state.md) | **현재 상태 · 다음 할 일 · 열려 있는 이슈** | 현재 상태로 **덮어쓴다**. 로그처럼 쌓지 않는다 |
| [context/constraints.md](context/constraints.md) | **실기·현장에서 확인된 사실** (하드웨어, TF, MoveIt, QoS, 리소스) | 확인될 때마다 추가. 추측은 쓰지 않는다 |
| [errors-log.md](errors-log.md) | **오류·오판·번복 이력** | 날짜별로 **쌓는다**. 덮어쓰지 않는다 |

세 문서의 경계: **지금 어떤가 → `state.md`** / **무엇이 참인가 → `constraints.md`** /
**무엇을 틀렸나 → `errors-log.md`**.

## 절차 문서 (그 작업의 명령이 여기에만 있다)

| 문서 | 단일 출처 |
|---|---|
| [rosbag-d435i.md](rosbag-d435i.md) | **D435i rosbag 녹화·재생 명령 전부.** `state.md`·`archive/2026-08-01-session.md`에 두지 않는다 |
| [review_moveit.md](review_moveit.md) | **MoveIt2 OMPL + Octomap** 파이프라인 수행 기록, 채택 설정 스냅샷, cuRobo 비교 설계 |
| [detect_graspx.md](detect_graspx.md) | **GraspGenX 설계 원본** — 출력 규약(§3), 폭 계산·1/10mm 함수(§5), 상류 버그(§6). `state.md`는 여기로 포인터만 둔다 |
| [context/test_grap_plan.md](context/test_grap_plan.md) | 🔄 **합쳐짐(2026-08-08) → `config/testcommand.md` "경로 B".** 값의 정본이 아니다. **비교용으로만** 남겼다 — 두 문서가 같은 명령을 다른 파라미터로 적고 있던 3건(`bringup` 의 `rviz`, `camera` 해상도, `moveit` 의 `cumotion`)을 실기에서 정해야 해서다 |
| [context/yolo_seg_grasp.md](context/yolo_seg_grasp.md) | YOLO 세그 컨테이너/호스트 2터미널 실행 스니펫 + COCO 인덱스 메모. 루트 `yolo_seg_grasp` 에서 이동(2026-08-08) |
| [context/docker_gpu.md](context/docker_gpu.md) | isaac_ros 컨테이너 attach·중지·재기동 메모. 루트 `docker_gpu` 에서 이동(2026-08-08) |

## 패키지 실기 로그 (레퍼런스는 `src/PACKAGES.md`)

패키지별 README에 쌓이던 날짜별 실기 디버깅 로그를 2026-08-09 통합 작업에서 여기로 옮겼다.
"지금 참인 값"은 `src/PACKAGES.md`, "그 값이 왜 그렇게 됐는지"는 아래.

| 문서 | 단일 출처 |
|---|---|
| [cumotion-experiment-log.md](cumotion-experiment-log.md) | **cumotion 패키지 실기 실험 로그** — 루프 결함 발견·수정, 그리퍼 자기충돌 발견, 복셀 붕괴 조사 |
| [graspgenx-perception-notes.md](graspgenx-perception-notes.md) | **graspgenx_perception 실기 검증·설계 검토 로그** — DDS 방향성 버그, 컨테이너 인스턴스 누적, TensorRT 검토, "다음 방향" 설계 |

## 계획 문서

| 문서 | 상태 |
|---|---|
| [M0609_perception_motion_sprint_plan.md](M0609_perception_motion_sprint_plan.md) | 🟢 현행 — Day1~5 스프린트 원본. **어느 PC에서 하느냐는 아래 두 문서가 나눈다** |
| [plans/2026-08-01-pc-role-split.md](plans/2026-08-01-pc-role-split.md) | 🟢 현행 — 개인PC(CPU) vs GPU PC 역할 분담 |
| [plans/2026-08-03-octomap-integration.md](plans/2026-08-03-octomap-integration.md) | ✅ **완료(08-03)** — 결과는 `review_moveit.md`. 계획서 자체는 이력으로 보존 |
| [plans/2026-08-03-c270-webcam-plan.md](plans/2026-08-03-c270-webcam-plan.md) | ⏸ 미착수 — C270 eye-in-hand. GPU 불필요. ⚠️ **본문 §1이 전제하던 `src/usb_cam`은 2026-08-08 삭제됨**(어디서도 안 씀) — 재개하려면 `v4l2_camera`부터 다시 판단 |
| [plans/2026-08-03-gpu-dependent-candidates.md](plans/2026-08-03-gpu-dependent-candidates.md) | ⏸ 보류 — GPU 머신 확보 전엔 **어떤 명령도 실행하지 않는다** |
| [plans/2026-08-04-gpu-rental-checklist.md](plans/2026-08-04-gpu-rental-checklist.md) | 🟢 **종료(2026-08-05, 로컬 GPU PC로 전환).** 대여 GPU 세션은 끝났지만 **밟은 지뢰 §6**과 컨테이너 실측 사실 §8은 로컬 Isaac ROS 작업에서도 계속 유효 |
| [plans/2026-08-05-foundationpose-graspgenx-pick.md](plans/2026-08-05-foundationpose-graspgenx-pick.md) | 🟢 **현행 원본** — 음성 타겟팅 + GraspGenX + 동적 회피 아키텍처 (같은 날 3차 개정) |
| [plans/2026-08-05-cumotion-bringup.md](plans/2026-08-05-cumotion-bringup.md) | 🔴 **진행 중** — 게이트 A~F. §4-3 "cuMotion은 MoveIt octomap을 안 본다"가 nvblox 필수화의 근거 |
| [plans/2026-08-07-graspgenx-target-matching.md](plans/2026-08-07-graspgenx-target-matching.md) | 🟢 현행 — "GraspGenX가 원하는 물체를 감지했는지" 배선 상태·남은 작업 |
| [plans/2026-08-08-ws-cleanup.md](plans/2026-08-08-ws-cleanup.md) | ✅ **완료(08-08)** — pick_fsm 기준 패키지 지도 + 삭제 실행(7개 패키지 제거, object_detection 죽은 코드 정리). 결과는 `CLAUDE.md` 1절 |
| [plans/2026-08-08-vla-integration.md](plans/2026-08-08-vla-integration.md) | 🟢 **범위 확정(2026-08-08, 3차 개정) · 미착수.** 단일 출처: `~/M0609_VLA_system` ↔ 이 ws **역할 경계**(로봇 행동은 우리, VLA 는 지시만·외부 PC·휴대폰 핫스팟), **지시 채널 계약**(커스텀 msg 0개), **VLA PC 로 넘길 카메라 토픽·대역폭 산정**(§3-3), **물체 선정(target selection) 설계** |
| [plans/2026-08-09-cumotion-verify.md](plans/2026-08-09-cumotion-verify.md) | 🟢 **미착수 · 실기 대기(2026-08-09 작성).** 개인PC에서 소스만 읽고 세운 cuMotion 재계획 트리거 · VRAM 배분 가설의 검증 계획(T1~T7). 예상값은 전부 ⚠️ 미검증 — `rokey` 필요 |
| [plans/2026-08-09-yolo-seg-finetune.md](plans/2026-08-09-yolo-seg-finetune.md) | 🟢 **미착수 · 계획만(2026-08-09).** COCO 가중치로는 우리 물체를 못 잡는 것(`state.md` 0-b의 잔여 블로커)을 뚫는다. **자동 어노테이션은 새 툴 없이** `capture_graspgenx_scene.py`의 depth 세그 + `scripts/seg_to_yolo.py`. 캡처·학습 모두 `rokey` 필요. 미결정 4건 |
| [plans/2026-08-10-presentation.md](plans/2026-08-10-presentation.md) | 🟢 **미착수 · 계획만(2026-08-09 작성).** `md/`+git 이력을 시각화해 추후 발표자료로 만들기 위한 범위·데이터 소스 매핑. §3-2 = **실기 세션에서 같이 회수할 데이터 목록**. ⚠️ **파일명의 08-10은 발표일이 아니다**(비공개) — 마감으로 읽지 말 것 |
| [plans/archive/2026-08-05-graspgenx-gpu-sprint.md](plans/archive/2026-08-05-graspgenx-gpu-sprint.md) | 🗄 **이력** — 2026-08-05 GraspGenX 실기 파이프라인 관통으로 목적 달성. 전제였던 "팀 공유 RTX 4070 좌석"은 폐기(실제는 로컬 RTX 4060 8GB) |

## 산출물 · 이력

| 문서 | 상태 |
|---|---|
| [2026-08-03-notebooklm-digest.md](2026-08-03-notebooklm-digest.md) | 📤 NotebookLM 소스용 다이제스트(08-03 스냅샷). 여기 적힌 값은 갱신되지 않는다 — 정본은 위 상시 문서다 |
| [2026-08-07-nvblox-curobo-digest.md](2026-08-07-nvblox-curobo-digest.md) | 📤 NotebookLM 소스용. **단일 출처: nvblox·cuRobo·cuMotion·GraspGenX 알고리즘 설명, MoveIt vs cuRobo 비교(개념·연산구조).** 실행 명령은 여기 없다(`config/testcommand.md`) |
| [isaac_ros_nvblox_setup.md](isaac_ros_nvblox_setup.md) | ⏸ Docker/udev 셋업(§1~5)만 유효. **nvblox 실행 절차 본체는 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6이 단일 출처** |
| [archive/2026-08-01-session.md](archive/2026-08-01-session.md) | 🗄 **동결(08-01 세션 기록).** 런치·녹화 명령은 **폐기** — `rosbag-d435i.md`를 볼 것 |
| [archive/2026-08-03-session-dashboard.html](archive/2026-08-03-session-dashboard.html) | 🗄 08-03 세션 성과 요약(시각화). 값은 갱신 안 됨 — 정본은 `review_moveit.md`·`errors-log.md` |
| [plans/archive/2026-08-05-graspgenx-gpu-sprint.md](plans/archive/2026-08-05-graspgenx-gpu-sprint.md) | 🗄 (위 계획 문서 표 참고) |
| `decisions/` | (비어 있음) ADR을 쓰게 되면 `<번호>-<제목>.md` |
| `journal/` | 그날그날의 시간 진행 로그. §"상시 문서" 옆 참고 |

## 문서를 쓸 때

- 작업 문서는 **`md/` 한 곳만** 쓴다(커밋됨). `docs/`는 PDF 서고 전용이며 gitignore다.
- 위키링크는 경로를 포함한다: `[[ws/cobot2/state]]` — `state.md`가 여러 ws에 있다.
- **자주 바뀌는 물리량(카메라 거리, 튜닝값)은 문서에 적지 않고 읽는 명령을 적는다.**
