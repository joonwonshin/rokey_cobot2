# 문서 지도 — 로봇 쪽(`docs/fsm/`) 무엇이 어디에 있나

> **이 파일이 진입점이다.** 새 문서를 만들면 여기에 한 줄 추가한다.
>
> 규칙: **같은 사실을 두 문서에 쓰지 않는다.** 아래 "단일 출처" 열이 그 사실을 소유한
> 문서다. 다른 문서는 값을 베끼지 말고 링크한다 — 사본은 반드시 갈라진다.
>
> 저장소 루트 `README.md` 는 **설치·실행 절차**다. 이 문서와 역할이 다르다.
> 패키지 레퍼런스(인터페이스·파라미터·빌드·검증 상태)는 **`src/PACKAGES.md`** 다.

## 경계 · 제약 (가장 먼저 읽을 것)

| 문서 | 단일 출처 |
|---|---|
| [vla-bridge-contract.md](vla-bridge-contract.md) | **판단 계층 ↔ FSM 경계 계약.** JSON 스키마·`cmd` 값·상태 대응표. 여기서 어긋나면 계약이 이긴다 |
| [context/constraints.md](context/constraints.md) | **실기로 확인한 사실.** 도면·문서와 다른 실측값. 하드웨어 동작이 예상과 다르면 여기부터 본다 |
| [context/unknowns.md](context/unknowns.md) | 아직 모르는 것 · 확인이 필요한 가정 |
| [context/team.md](context/team.md) | 팀·환경 제약 |

## 인식 · 파지

| 문서 | 단일 출처 |
|---|---|
| [detect_graspx.md](detect_graspx.md) | **GraspGenX 설계 원본** — 출력 규약(§3), 폭 계산·1/10 mm 함수(§5), 상류 버그(§6) |
| [graspgenx-perception-notes.md](graspgenx-perception-notes.md) | GraspGenX 컨테이너 운용 — 인스턴스 정리·진단 명령 |
| [rosbag-d435i.md](rosbag-d435i.md) | **D435i rosbag 녹화·재생 명령 전부** |

## 모션 · 플래닝

| 문서 | 단일 출처 |
|---|---|
| [review_moveit.md](review_moveit.md) | MoveIt 구성 검토 · octomap 통합 결과 |
| [isaac_ros_nvblox_setup.md](isaac_ros_nvblox_setup.md) | nvblox 설치·구성 |
| [cumotion-experiment-log.md](cumotion-experiment-log.md) | cuMotion 실험 기록 |
| [launch-params.md](launch-params.md) | launch 인자 정리 |
| [context/movegroup_rmpflow_review.md](context/movegroup_rmpflow_review.md) | move_group · RMPflow 검토 |
| [context/docker_gpu.md](context/docker_gpu.md) | 컨테이너 GPU 접근 |

## 통합 · 아키텍처

| 문서 | 단일 출처 |
|---|---|
| [0811_integ_digest.md](0811_integ_digest.md) | 통합 시점 요약 |
| [M0609_perception_motion_sprint_plan.md](M0609_perception_motion_sprint_plan.md) | 인식·모션 통합 설계 |

## 조사 요약 (외부 자료 정리)

| 문서 | 내용 |
|---|---|
| [2026-08-03-notebooklm-digest.md](2026-08-03-notebooklm-digest.md) | 초기 조사 정리 |
| [2026-08-07-nvblox-curobo-digest.md](2026-08-07-nvblox-curobo-digest.md) | nvblox · cuRobo |
| [2026-08-11-rmpflow-reactive-motion-digest.md](2026-08-11-rmpflow-reactive-motion-digest.md) | RMPflow 반응형 모션 |
