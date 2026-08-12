# 2026-08-10 align_grasp_object — RViz 파지목표 vs 실물 3cm 평면오차 실기 진단

이 세션(대화 기반, 실기 미착수)에서 세운 진단 계획. 여기 적힌 원인 후보는 전부 `⚠️ 미검증`이다 —
실기에서 뒤집히면 이 문서를 갱신한다 (docs 규칙: 로그처럼 쌓지 말고 현재 상태로 덮어쓴다).

## 0. 증상

- RViz에 시각화된 파지목표 물체(그리퍼가 향하는 위치)와 실제 물체 위치 사이에 **평면(XY) 방향
  약 3cm** 오차 관측.
- 사용자 확인: `MotionPlanning → Scene Geometry`에 뜨는 **콜리전 구(sphere)** 얘기다 — octomap
  복셀이 아니라 GraspGenX가 계산한 grasp 결과 쪽.
- octomap도 같은 카메라로 평평한 테이블 위 물체(예: 사과, 높이 ≈5cm)를 감지하고 있으니,
  이 데이터로 GraspGenX 쪽 위치를 교차검증/보정할 수 있는지가 질문의 핵심.

## 1. 파이프라인 요약 (소스로 확인, 재확인 불필요)

```
capture_graspgenx_scene.py: depth(n프레임 median) + K + T_base_cam(TF: base_link←camera_color_optical_frame)
        → segment() → GraspGenX worker(uv venv, 별도 프로세스) → grasp 4x4 (이미 base_link 프레임)
grasp_bridge_node.select(): min_score/max_reach/approach_z 필터 → best grasp
grasp_bridge_node.tcp_of(): TCP = grasp원점 + tcp_offset_m(0.18) × grasp의 +Z축 → /grasp/best_tcp
pick_fsm/task_manager.py: geometry.to_gripper_base() 요 90° 보정 (grasp원점=rg2_base_link, tool0 아님)
        → moveit.make_object(obj_id, tcp, object_radius_m=0.04) → CollisionObject(SPHERE) 등록
        → 같은 TCP로 ik_async(ee_link=rg2_base_link) → move_to_joints_async
```

핵심: **RViz 구 위치 = 실제 IK/모션플래닝 목표(TCP)와 동일값이다.** 시각적 문제가 아니라
실제 파지 정확도에 직결된다 — 콜리전 구만 따로 보정하면 눈속임만 된다(§4 참고).

## 2. 원인 후보 3가지

| # | 후보 | 근거 | 옥토맵 비교로 잡히는가 |
|---|---|---|---|
| ① | eye-to-hand 캘리브(`T_base_cam`) 잔차 | `constraints.md:952` 1.65m에서 캘리브 상수 오차 41mm 실측 기록. 자릿수가 3cm과 일치 | **아니오** — 옥토맵도 같은 T_base_cam을 쓰므로 GraspGenX와 같은 방향·크기로 같이 어긋난다. 비교해도 델타가 0으로 나온다 |
| ② octomap 복셀 양자화 | `sensors_3d.yaml:30` `octomap_resolution: 0.05` = 5cm 복셀. 렌더링 자체가 ±2.5cm까지 스냅됨 | 해당 없음 — 이건 GraspGenX 쪽이 아니라 옥토맵 렌더링 고유 오차. 오히려 옥토맵을 "정답"으로 쓰면 **5cm 물체를 5cm 복셀로 재는 셈**이라 기준 자체가 거칠다 |
| ③ 단일 시점 표면중심 편향 (`2r/3`) | `constraints.md:1179` 이미 문서화. eye-to-hand로 카메라가 옆에서 비스듬히 보므로 이 편향이 Z뿐 아니라 XY에도 실린다 | **예** — momentary n프레임 캡처 고유 편향이라, 시간축으로 다른 옥토맵(장시간 누적)과 비교하면 드러난다 |

①과 ③은 옥토맵으로 갈라낼 수 있는 성질이 다르다 — **먼저 어느 쪽인지 실측으로 갈라야** 옥토맵
비교 기능을 만들지 말지 정할 수 있다.

## 3. 실기 진단 순서

### 3-1. 계통오차 vs 노이즈 판별 (코드 변경 0, 가장 먼저)

같은 물체를 고정해두고 `/grasp/compute_grasp`(또는 `/grasp/compute`)를 5~10회 반복 호출.

- [ ] 매 호출 `/grasp/best_tcp` 위치를 기록
- [ ] 물체 위치는 실측(줄자 또는 알려진 좌표에 배치)
- [ ] **판정**:
  - 매번 같은 방향·크기로 어긋남(분산 작음) → **① 캘리브 계통오차** 쪽. §3-2로
  - 호출마다 델타가 흔들림(분산 큼) → **③ momentary 노이즈** 쪽. §3-3로

### 3-2. ①로 판정된 경우 — 옥토맵 비교는 하지 않는다

- 옥토맵과 비교해도 델타가 0으로 나와 "고칠 게 없다"는 잘못된 결론에 이르기 쉽다 — 시도하지 말 것
- `constraints.md:986`에 이미 미뤄둔 "실측은 알려진 좌표 물체로" 재캘리브 검증 항목을 수행
- 필요 시 `eye2hand_calibration.py` 재실행 (마운트 강성·`square_size` 재측정부터 의심,
  `constraints.md:69`)

### 3-3. ③로 판정된 경우 — 저비용 대책부터

- [ ] **1차**: `grasp_bridge_node`의 `p['frames']`(캡처 프레임 수)를 늘려서 median 안정성 확인.
      구현 비용 0(파라미터 값만 변경) — 옥토맵 비교보다 먼저 시도
- [ ] 그래도 3cm이 안 줄면 **2차**: 옥토맵(또는 원본 `/camera/camera/depth/color/points`를
      직접 시간평균) 기반 표면중심 재추정 도입 검토

  옥토맵 기반으로 갈 경우 주의점:
  - `octomap_resolution: 0.05`는 5cm 물체 비교용으로 거칠다 — 비교 전용으로 별도 해상도
    (예: 0.01)를 쓰거나, 옥토맵 대신 `/camera/camera/depth/color/points` 원본을 직접
    시간축 누적 평균하는 쪽이 더 정직한 신호
  - `GetPlanningScene(components=OCTOMAP)`으로 직렬화된 octree를 받아 디코딩 + 최근접 복셀
    탐색 로직이 필요 — 지금 파이프라인에 없는 신규 개발 항목
  - **보정 대상은 콜리전 구가 아니라 `grasp_bridge_node`의 `tcp_of()` 이후 실제 grasp
    TCP여야 한다.** 콜리전 구(`moveit_bridge.make_object`)만 보정하면 IK 목표는 그대로라
    파지 정확도는 안 바뀐다(§1 참고)
  - 실행 목표를 바꾸는 변경이므로 실기 검증 없이 커밋 금지, cross-review 필수

## 4. 미해결/확인 필요

- [ ] §3-1 반복호출 실측 — 아직 안 함
- [ ] 3cm이 항상 같은 방향(예: 카메라에서 먼 쪽 vs 가까운 쪽)인지도 같이 기록할 것 —
      방향이 일정하면 ①(캘리브, 부호 있는 바이어스)일 가능성이 더 커진다
- [ ] RViz Octomap Render Mode를 끈 상태에서도 3cm이 그대로인지 (콜리전 구 단독 확인,
      이전 대화에서 이미 제안했으나 이번 확인에서 옥토맵 얘기가 아니라고 정정됨 — 이 항목은
      "콜리전 구가 물체와 3cm 어긋난다"는 원 관측을 재확인하는 용도로만 남긴다)
