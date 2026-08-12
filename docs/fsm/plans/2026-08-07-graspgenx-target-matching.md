<!-- meta
updated: 2026-08-07
status:  live
owns:    "GraspGenX가 원하는 물체를 감지했는지" 배선 상태 · 남은 작업
-->

# Plan: GraspGenX 목표 물체 판별 — 코드 감사 결과 (2026-08-07)

**아키텍처는 이미 정해져 있다.** [[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]]
§1·§1-3·Phase 0-A 가 "음성 target → YOLO-seg 인스턴스 마스크 → GraspGenX" 설계를 이미 끝냈다.
이 문서는 **새 설계가 아니라**, `fsm_grasp_modify` 커밋(2026-08-07)까지 실제 코드를 읽고
그 설계 중 **뭐가 아직 배선 안 됐는지**를 확정한 것이다.

---

## 확인된 사실 (코드 읽기, 2026-08-07)

| # | 사실 | 근거 |
|---|---|---|
| 1 | `grasp_bridge_node.select()`에 `p['target']` 필터가 있다(라벨 문자열 일치 비교) | `grasp_bridge_node.py:118-119` |
| 2 | 그 라벨은 의미 있는 클래스명이 아니라 `obj_1, obj_2…` — YOLO 인스턴스 순번일 뿐이다 | `yolo_seg_node.py:34` `LABEL_OBJ_BASE=100`, `graspgen_worker.py:111` `labels = list(scene['objects'].keys())` |
| 3 | `target`은 **노드 파라미터**(launch-time 정적값)다. per-request로 못 바꾼다 | `grasp_bridge_node.py` `EXTRA_DEFAULTS['target']`, `/grasp/compute`는 `std_srvs/Trigger`라 요청 필드가 없다 |
| 4 | `task_manager`가 실제로 쓰는 `legacy_trigger` 경로는 `Trigger.Request()`를 보낸다 — target을 실을 필드 자체가 없다 | `task_manager.py:421` |
| 5 | target을 실을 수 있는 `compute_grasp`/`ComputeGrasp` 경로는 클라이언트 코드만 있다 — 서버가 ws 어디에도 없다 | `pick_fsm_msgs`는 `.srv` 정의뿐, `create_service(ComputeGrasp, ...)` grep 0건 |
| 6 | YOLO는 이미 노드로 떠 있다(`yolo_seg_node`)지만 `grasp_bridge_node`의 `seg_source` 기본값은 `geometric`(신경망 0개) — `yolo`로 바꿔도 라벨이 사실 3번 문제라 이름 매칭이 안 된다 | `grasp_bridge_node.py:143-148`(EXTRA_DEFAULTS), `graspx.launch.py:26` |

**결론**: "지정한 이름의 물체를 골랐는지 판단하는 코드"는 없다. 있는 건 "N번째로 검출된 덩어리 중
특정 순번만 통과시키는" 필터뿐이고, 그 순번을 음성 인식 결과와 연결하는 배선도 없다.
[[ws/cobot2/state]] 0-b가 이 상태를 "미해결"로 이미 적어뒀다 — 이번 감사로 **확인**됐을 뿐 새로 발견된 건 아니다.

---

## 남은 일 — 08-05 계획 Phase 0-A/D 를 지금 코드 기준으로 좁힌 것

08-05 계획은 "YOLO-seg가 target 클래스의 인스턴스 마스크를 낸다"를 전제로 짰다. 지금
`yolo_seg_node`는 **탐지는 하지만 클래스 이름을 라벨맵에 안 싣는다**(마스크는 `obj_N`
순번뿐, 원래 YOLO가 낸 클래스 이름·신뢰도가 유실된다). 세 조각이 필요하다:

1. **`yolo_seg_node` — 클래스 이름을 라벨과 함께 발행.**
   지금 `build_label_map()`은 마스크만 리라벨링하고 `res.boxes.cls`/`model.names`를 버린다.
   `{obj_id: class_name}` 매핑을 별도 토픽(또는 라벨 메시지 확장)으로 내야 한다.
2. **`grasp_bridge_node` — `target`을 요청 필드로 받고, 클래스 이름으로 필터.**
   `p['target']`을 노드 파라미터에서 `/grasp/compute` 요청 인자로 옮기거나(Trigger→커스텀
   서비스 전환 필요 — 5번 항목과 겹친다), 최소한 1번의 매핑을 받아 `select()`에서
   `label`이 아니라 `class_name`으로 비교하도록 바꾼다.
3. **`task_manager` — 음성 target을 실제로 전달.** 지금 `legacy_trigger` 경로는 `self.target`을
   아예 안 보낸다. `ComputeGrasp` 서버를 새로 만들거나(정본 계약, 하지만 아직 없음),
   `legacy_trigger`의 Trigger 서비스에 target을 실을 방법을 만들어야 한다(서비스 타입 변경 필요).

**순서 권고**: 1 → 2 먼저(그리퍼·MoveIt 없이 이 PC에서도 검증 가능 — YOLO 컨테이너만 있으면 됨).
3번(서비스 계약 변경)은 로봇 없이도 가능하지만 `pick_fsm`·`graspgenx_perception` 두 패키지를
같이 건드리므로 1·2가 끝나 라벨→이름 매핑이 실제로 도는 걸 본 뒤에 하는 게 手戻り가 적다.

---

## 이 계획이 다루지 않는 것

- YOLO 모델을 마트 품목으로 재학습하는 것(08-05 계획 A-3, 마지막 수단)
- 컨테이너 → 호스트 데이터 유실 버그(`graspgenx_perception/README.md` "🔴 미해결", 별개 이슈 —
  `seg_source:=yolo`를 실제로 켜기 전에 이것부터 풀어야 한다)
- 손 배제·다물체 혼재 등 나머지 아키텍처 — 08-05 계획이 이미 다룸, 재론 안 함

---
확신도: 검증됨 — 표의 6개 사실은 전부 이 세션에서 해당 파일을 직접 읽어 확인했다(실행하지 않음).
내가 채워넣은 가정: 없음 — "남은 일" 3개 항목은 기존 설계(08-05 계획)와 현재 코드의 차이를 그대로 좁힌 것이라 새 가정을 추가하지 않았다.
확인 요청: 남은 일 1~3번 중 지금 세션에서 바로 시작할까, 아니면 이 문서만 남기고 다음으로 미룰까?
