<!-- meta
updated: 2026-08-11
status:  digest — 이 날 VLA↔FSM 통합 세션에서 쌓인 지식 요약. 값의 정본은 각 코드/문서이고
         (아래 "참조") 이 파일은 "그날 무엇을 배웠나"를 한자리에 모은 것이다. 로그처럼
         불어나지 않는다 — 사실이 바뀌면 정본을 고치고 여기 요약도 덮어쓴다.
owns:    없음(파생 문서). 🟢=이 날 실기로 확인, 🔴=미검증
-->

# 0811 통합 digest — VLA 노드 ↔ pick_fsm

> **한 줄**: 외부 PC 의 VLA 가 "어느 **개체**를 집을지"를 픽셀로 지목하면, cobot2_ws 가 그
> 픽셀을 base XY 로 바꿔 같은 클래스 여러 개 중 하나만 골라 GraspGenX 로 넘기는 경로
> (`select_by_point`)를 구현하고 **실기로 PERCEIVE 까지 관통**시켰다.

관련 정본:
계획 문서(비공개)(설계·히스토리 단일 출처, §5 가 select_by_point 정본) ·
[경계 계약 문서](vla-bridge-contract.md)(외부 repo 와의 계약) ·
[`src/PACKAGES.md#voice_processing`](../src/PACKAGES.md) ·
[실기 제약 문서](context/constraints.md)(실기 상수)

---

## 0. cobot2_ws 쪽 발표/실무 하이라이트 (§1~8 요약, 상세는 각 절 참조)

> VLA 쪽 요약은 §9.1~9.3(그쪽 세션 작성) 참고 — 이 절은 대칭 구조로 cobot2_ws 쪽만 정리.

### 0.1 로보틱스 지식

- **픽셀이 아니라 base XY 로 매칭**했더니, 같은 물리 지점이 캡처마다 `obj_1→obj_2`로
  라벨이 뒤바뀌어도 정확히 같은 물체를 찾았다 (§2). 라벨 id 매칭이었다면 재촬영마다
  다른 물체를 집었을 것 — **프레임에 안정적인 값(좌표)으로 매칭하고, 프레임마다
  재부여되는 값(라벨)에 상태를 싣지 않는다**는 일반 원칙의 실측 사례.
- **grasp 원점 ≠ TCP**: GraspGenX 는 그리퍼 base 를 원점으로 주지만 실제 접촉점은
  거기서 +Z 0.18m(RG2 fingertip) 떨어져 있다. 원점을 물체 위치로 오인하면 접근 각도가
  기울수록 옆으로 벗어난다 (§4).
- **테이블도 collision 점군에 들어간다** — segmentation 과 무관하게 유효 depth 전부가
  들어가서, 납작한 물체를 위에서 잡으면 손끝이 테이블 2cm 안으로 들어와 정상 grasp 까지
  0개로 전멸할 수 있다. "후보 0개"가 나오면 먼저 의심할 지점 (§4).
- **depth 노이즈는 재촬영 한 번으로 흡수**된다 — 같은 물체·같은 자리에서 collision-free
  비율이 0%↔4%로 흔들렸는데, `_perceive_failed` 의 자동 재시도가 설계대로 동작해 살아남 (§4).

### 0.2 발표 강조 포인트

- **설계 판단을 실측으로 증명한 사례**(§2): "픽셀 대신 base XY 로 매칭한다"는 결정이
  이론이 아니라, 같은 세션 안에서 라벨이 실제로 뒤바뀌는 걸 잡아내고 그럼에도 7mm
  오차로 정확히 맞았다는 재현 가능한 데이터로 뒷받침됨.
- **전 구간(PERCEIVE)까지 실기 관통 + 실패 경로도 안전 종료**: MoveIt 미기동 상태에서도
  `_move()` 가 `/move_action` 부재를 감지해 SCENE_PREP 타임아웃 → ABORT → SAFE_STOP 으로
  스스로 멈췄다(§3, §6) — "모른다"를 "아무거나 실행"이 아니라 "안전 정지"로 처리하는
  설계가 실제로 작동함을 보여준 순간.
- **정적 리뷰가 라이브 테스트로 못 잡는 버그를 잡음**(§7): 성공 경로만 탄 오늘 실기
  관통에서는 안 보였던 픽셀 override 누수(실패 경로로 빠지면 좌표가 다음 사이클로 새는
  버그)를 cross-review 가 코드 정독만으로 찾아 수정 → build PASS, test 113/113. "실기가
  통과했다"와 "버그가 없다"는 다르다는 걸 보여주는 근거.

### 0.3 ws 연결에 필수적이었던 실무 팁 (재사용 가능)

- **기존 상태머신을 안 고치고 끼워넣기**: FSM 이 이미 갖고 있던 `LISTENING → /get_keyword`
  자리에 VLA 를 "사람 대신 말해주는 클라이언트"로 꽂아서 FSM 본체 변경 없이 통합 (§1).
  외부 시스템을 붙일 때 새 진입점을 만들기 전에 기존 진입점을 재사용할 수 있는지 먼저 본다.
- **QoS 는 글자 그대로 맞아야 한다**: `TRANSIENT_LOCAL depth=1` 을 양쪽이 정확히 같은
  값으로 안 쓰면 에러 없이 그냥 조용히 매칭이 안 된다 (§5) — "아무 반응이 없다" 증상의
  1번 용의자.
  ROS_DOMAIN_ID 불일치도 동일한 실패 모드(§9.3, VLA 쪽 실측)이므로 세트로 의심한다.
- **TTL 은 송신 타임스탬프가 아니라 수신 시각 기준**: 서로 다른 PC(시계 안 맞음) 간
  통신이라 `stamp_ns` 는 에코 용도로만 쓰고 나이는 받는 쪽이 잰다 (§5).
- **SetParameters 로 넘기는 float 은 반드시 DOUBLE 타입 명시**(`float_param`) — 이 ws
  는 int/float 혼동으로 노드가 죽은 이력이 반복돼 이번에도 명시적으로 방어함 (§5, 팀 컨벤션 문서 공통 규칙).
- **공유 랩탑에서는 노드 기동 전에 `/dsr01/*`·`/move_action` 이 이미 떠 있는지 먼저
  확인**한다 — 다른 세션이 로봇을 살려둔 채 작업 중일 수 있다 (§6, 오늘 실제로 겪음).
- **"지시 없음"은 하나의 센티널(nan)로 통일**: `pixel_x/y/w/h = nan` 이 이 ws 의
  `table_z`/`class_dims` 관례와 동일해서, 지정 없는 보통 pick 이 별도 분기 없이 기존
  "점수 최고" 동작으로 자연스럽게 흐른다 (§4).

---

## 1. 통합 아키텍처 — push(VLA) vs pull(FSM) 를 잇는 한 건짜리 래치

**역할 경계**: VLA(외부 PC, `~/M0609_VLA_system`)가 "무엇을", cobot2_ws 가 "어떻게"를 소유한다.
FSM 은 이미 `LISTENING` 상태에서 `/get_keyword`(Trigger)를 부르는 **음성 노드 자리**를 갖고
있어서, VLA 를 "사람 대신 말해주는 클라이언트"로 그 자리에 꽂으면 **FSM 을 안 고쳐도 된다.**

```
VLA PC ──/vla/pick_command(JSON)──▶ vla_command_node ──/get_keyword(Trigger)──▶ task_manager
   │  {class, pixel, pixel_wh,           │ (pixel_policy=select 일 때만)              │
   │   request_id, place, stamp_ns}      ├──/pick/target_pixel(JSON x,y,w,h)──▶ _on_target_pixel
   │                                     ├──/pick/place_location──────────────▶ _on_place_location
   ◀──/vla/pick_result(JSON)────────────┘                                         │
                                                              PERCEIVE 진입: _push_bridge()
                                                              SetParameters(pixel_x/y/w/h,
                                                              target_classes, seg_source)
                                                                            │
                                                            grasp_bridge_node.compute():
                                                            segment() → pixel_to_base()
                                                            → select_by_point() → GraspGenX
```

**왜 이렇게**: VLA 는 아무 때나 쏘고(push), FSM 은 `LISTENING` 에 와야 묻는다(pull). 그래서
`vla_command_node` 는 지시를 붙잡고 있다 FSM 이 물을 때 건네는 **한 건짜리 래치**다. 어긋나는
지점(TTL·"FSM 이 아직 듣나"·"이 사이클이 끝났나")을 코드가 명시적으로 방어한다.

---

## 2. 핵심 설계 판단 — 왜 픽셀이 아니라 base XY 로 매칭하나 🟢

VLA 와 cobot2_ws 는 **다른 프로세스·다른 프레임**에서 돈다. 그래서:

- VLA 는 **픽셀**로 지목한다(자기가 보는 이미지 좌표).
- cobot2_ws 는 그 픽셀을 **브리지에서 딱 한 번** base XY 로 바꾼다(카메라 TF 사용,
  `pixel_to_base()`). 이후 매칭은 전부 base XY 최근접이다.
- **`obj_N` 라벨 id 로 매칭하지 않는다** — 라벨은 캡처마다 재부여되어 프레임 의존적이다.

**오늘 이 판단이 옳았음을 실측이 증명했다**: 같은 픽셀 `(749,383)` 이 첫 캡처에선 `obj_1`,
재촬영 뒤엔 `obj_2` 로 **라벨이 뒤바뀌었는데도**, base XY 최근접(+0.23~0.25, +0.055 근처)이
매번 같은 물리 물체를 정확히 찾았다. 라벨 id 로 매칭했다면 재촬영에서 엉뚱한 걸 집었을 것이다.

---

## 3. 오늘 실기로 검증된 것 🟢 (domain 93, 실 D435i)

**전 구간 관통(PERCEIVE 까지)**: `vla_command_node`(pixel_policy=select) + `task_manager`
(voice:=false) + `grasp_bridge_node`(seg_source=geometric) 동시 기동.

| 단계 | 실측 결과 |
|---|---|
| VLA 흉내 `pixel:[749,383], pixel_wh:[1280,720]` | `/pick/target_pixel = {x:749,y:383,w:1280,h:720}` |
| task_manager 수신 | `개체 선정 좌표 수신: (749,383) / 기준 1280x720` |
| PERCEIVE push | `브리지 설정: target_classes='(전부)', seg_source=geometric, pixel=(749,383)` |
| 첫 캡처 | obj_1 선정(base +0.227,+0.056, 지정점 0.022m) → **collision 0/157** → 후보 0 → 재촬영 |
| 재촬영(자동 재시도 1/2) | **obj_2 선정(base +0.247,+0.054, 지정점 0.007m)** → collision 9/239 → 성공 |
| GraspGenX | `score=0.701, 손끝=(+0.235,+0.036,+0.025), 폭=39.9mm` |
| SCENE_PREP | MoveIt 없음 → 10s 타임아웃 → **안전하게 ABORT→SAFE_STOP** |

- 실물체 6~9개 씬에서 지정 픽셀 개체를 **7mm 오차**로 선정.
- 실패 경로도 검증: 배경 픽셀 `(5,5)` → `base=(-4.521,-3.974)` → `반경 0.060m 안에 물체 없음`
  으로 **워커 호출 전 조기 거부**(GPU 낭비 없음).

**카메라 실측**(오늘, color 1280x720 → aligned_depth 도 1280x720 추종):
- K = `fx909.53 fy909.20 cx659.54 cy370.20`
- TF `base_link→camera_color_optical_frame` Translation `[1.264, -0.053, 0.760]`
- `table_z` 자동추정 `-0.0099 ~ -0.0142 m` (캡처마다 흔들린다)

---

## 4. 로보틱스 지식 (통합하며 확인/재확인)

- **GraspGenX grasp 원점 = 그리퍼 base, TCP 는 +Z 로 0.18m**(RG2 fingertip). grasp 를 물체
  위치로 읽으면 접근이 기울수록 옆으로 벗어난다 — 그래서 TCP 를 따로 계산·발행한다. 🟢
- **테이블도 장애물이다**: collision 점군엔 seg 와 무관하게 유효 depth 가 전부 들어간다
  (`collision_threshold=0.02m`). 납작한 물체를 위에서 잡으면 손끝이 테이블 2cm 안에 들어와
  **정상 grasp 까지 전멸**할 수 있다 — 0개가 나오면 여기부터 의심. 🟢(오늘 첫 캡처 0/157)
- **depth 노이즈로 grasp 후보가 요동친다**: 같은 물체·같은 자리에서 collision-free 비율이
  0%↔4%(0/157 → 9/239) 로 흔들렸다. **재촬영 한 번**으로 살아났다 — `_perceive_failed` 의
  자동 재촬영 재시도가 이걸 흡수한다(설계대로 동작 🟢).
- **pixel_to_base 의 depth 구멍 방어**: 지정 픽셀이 구멍(depth=0)이면 이웃 5×5 median 으로
  메운다. 전부 구멍이면 None(선정 실패). 🟢(단위테스트)
- **모호성 거부**: 지정점 반경 `match_tolerance_m`(0.06m, VLA `system.yaml` 과 맞춤) 밖이면
  거부, 2등 후보가 `ambiguity_margin_m`(0.02m, **초안값 🔴**) 안으로 붙으면 "모호"로 거부.
  틀린 물체를 집는 것보다 안 집는 쪽이 안전하다는 판단(`refuse_ambiguous_match`).
- **nan 센티널**: "지정 없음"은 `pixel_x/y/w/h = nan` 으로 표현(이 ws 의 `table_z`/`class_dims`
  관례와 동일). 지정 없는 보통 pick 은 nan 이 그대로 와서 기존 "점수 최고" 동작과 같다. 🟢

---

## 5. ROS / DDS / 인프라 함정

- **도메인 격리**: 이 랩탑은 여러 계정·세션이 **같은 domain 93**을 공유한다. 단독 노드
  테스트는 빈 도메인(예: 77)으로 격리했고, 실카메라가 필요할 땐 93 을 공유했다(사용자가
  "다른 세션은 cumotion 비전이라 안 겹친다" 확인). 🟢
- **QoS 매칭**: `/pick/target_pixel`·`/pick/place_location` 은 `TRANSIENT_LOCAL depth=1`
  (늦게 붙어도 마지막 값 수신). 발행자가 이보다 얕은 durability 면 **조용히 매칭 안 됨.**
  `vla_command_node` 의 `PLACE_QOS` 와 `task_manager` 의 `TARGET_QOS` 는 **글자 그대로 같아야**
  한다. 🟢
- **TTL 은 받은 시각 기준**: 지시는 다른 PC 에서 핫스팟 건너 온다 — 시계가 안 맞으므로 송신
  `stamp_ns` 로 나이를 재지 않는다(에코만). 🟢(설계)
- **`/get_keyword` 중복**: 마이크 노드(`get_keyword`)와 `vla_command_node` 둘 다 이 서비스를
  제공 → **동시에 띄우면 어느 쪽이 답할지 모름.** 하나만 띄운다.
- **`ros2 launch ... target:=`(빈 값)은 문법 오류**로 거부된다. 빈 타겟은 인자를 아예 빼고
  파라미터 기본값(자동)에 맡긴다. 🟢(오늘 밟음)
- **SetParameters float 은 DOUBLE 타입 명시**(`float_param`) — 이 ws 는 int/float 혼동으로
  노드가 죽은 이력 다수. 🟢
- **launch 미선언 인자는 경고 없이 무시**된다(`dry_run:=`, `target_classes:=` 등). 🟢

---

## 6. 안전 — 오늘 겪은 것과 규칙

- **승인 게이트**: `dry_run` 은 2026-08-09 제거됨. 남은 소프트 안전장치는 `require_approval`
  하나이고, VLA/음성은 `/pick/approve` 를 **절대 부르지 않는다**(`BLOCKED_CMDS` — 코드 경로
  자체가 없다). 최종 안전장치는 물리 비상정지 버튼.
- 🔴→규칙 **`/dsr01/*` 저수준 드라이버가 이미 떠 있을 수 있다**: 오늘 task_manager 를 띄울 때
  다른 세션이 올려둔 `/dsr01/controller_manager` 등이 살아 있었다(내가 안 올림). `SAFE_STOP→
  HOME` 전이가 관절이동을 시도하는 상태라 잠깐 놀랐으나, **실제 모션은 안 나갔다** — `_move()`
  가 `/move_action`(MoveIt) 준비를 요구하는데 그게 없어서(오늘 `moveit.launch.py` 미기동)
  SCENE_PREP 타임아웃으로 안전하게 멈췄다. **교훈: task_manager 기동 전에 `/dsr01/*` 와
  `/move_action` 존재를 먼저 확인한다**(공유 랩탑이라 다른 세션이 로봇을 살려둔 채 작업 중일
  수 있다). → `constraints.md` 승격 후보.

---

## 7. cross-review 지적과 대응 (HIGH 1건)

- **HIGH(수정 완료 🟢)**: 단발성 픽셀 override(`_pixel_override`)가 성공 경로
  (`_push_bridge`)에서만 삭제되고 `_pushed` 는 매 전이에서 리셋 → **소비 전에 SPEAK_FAIL/
  ABORT 로 빠지면 좌표가 다음 사이클로 새서** 클래스만 지시한 다음 pick 이 엉뚱한 개체를
  고른다. 수정: `_to()` 에서 SPEAK_FAIL/ABORT 진입 시 override 를 지운다(`_st_idle` 시작점
  클리어는 vla 의 "픽셀→start" legit 픽셀을 race 로 지우므로 피함). build PASS, test 113/113.
  - 오늘 라이브 관통은 성공 경로만 탔어서 이 누수는 안 보였다 — **정적 리뷰가 잡았다.**
- note 2건(미수정, 판단 후 플래그): centroid median 이 depth 구멍에 이론상 당겨질 수 있음
  (오늘 7mm 정확도라 실질 영향 미미) / 1회성 픽셀 토픽의 `TRANSIENT_LOCAL` latch 가 노드
  재시작 시 stale 을 줄 수 있음(QoS 계약이라 안 바꿈).

---

## 8. 남은 것 🔴

- **MoveIt 이후 전 구간**(SCENE_PREP → PLAN → 실제 파지·place)은 오늘 안 돌렸다. `moveit.
  launch.py` + 로봇 bringup 을 다 띄운 진짜 pick 한 사이클 미검증.
- **실제 VLA PC 연결** 미검증 — 오늘은 `ros2 topic pub` 로 흉내. 핫스팟·도메인·DDS 도달성
  전부 미실측.
- **같은 클래스 다중 개체 씬**에서의 모호 거부(`ambiguity_margin_m=0.02` 초안값) 실기 튜닝.
- `base_xy` 경로는 여전히 검증만 하고 선정에 안 씀(캘리브가 `match_tolerance_m` 예산 밖).
- `select_by_point` 을 SPEAK_FAIL/ABORT 로 빠뜨리는 **누수 실패경로 자체의 실기 재현**은 안 함
  (코드 경로로만 확인 후 수정).

---

## 9. VLA 쪽(`~/M0609_VLA_system`) 기여 — 아키텍처·경계 설계

> 이 절은 VLA ws 세션이 작성. FSM(cobot2_ws) 쪽 상세는 위 1~8절(그쪽 세션 작성) 참고.
> 두 ws는 **서로 다른 독립 git clone**(같은 remote, 다른 브랜치)이라 코드 공유가 안 되고
> 오직 `/vla/pick_command`↔`/vla/pick_result`(JSON) 하나로만 붙는다 — 그래서 "경계 설계"
> 자체가 이 통합의 핵심 난이도였다.

### 9.1 로보틱스/시스템 지식 — 발표 포인트

- **push(비동기 지시) vs pull(동기 질의) 을 잇는 "한 건짜리 래치" 패턴** 🟢: VLA 는 사람이
  말하는 순간 아무 때나 지시를 쏘지만(push), FSM 은 자기 상태머신이 `LISTENING`에 와야만
  묻는다(pull, `/get_keyword` Trigger 서비스). 서로 다른 트리거 모델의 두 시스템을 잇는
  범용 패턴: 중간에 **최신 지시 1건만 붙잡고 있다가 상대가 물을 때 건네는 래치**를 두면
  둘 다 안 고친다. 이 패턴은 VLA/FSM 뿐 아니라 "이벤트 드리븐 생산자 + 폴링 소비자"를
  잇는 어떤 조합에도 재사용 가능 — 발표에서 일반화해서 말하기 좋은 대목.
- **좌표계 소유권은 한쪽에만 둔다** 🟢: VLA 는 자기 카메라 프레임의 **픽셀**만 알고,
  base XY 로의 변환은 **브리지에서 딱 한 번만** 일어난다(FSM 쪽 `pixel_to_base()`).
  VLA 는 자체 `table_homography`로 좌표를 계산할 수 있었지만 **의도적으로 실행에 안 씀
  (GUI 디버그 표시 전용)** — 두 프로세스가 각자 좌표를 계산해서 섞으면 캘리브레이션
  드리프트가 나도 어느 쪽이 틀렸는지 못 가른다. "같은 물리량을 두 곳에서 계산하지 않는다"는
  일반 원칙을 좌표 변환이라는 구체 사례로 보여줄 수 있음.
- **request_id 없이 위치 기반 상관(correlation)** 🟢: `/pick/state` 는 지금 어느 pick 의
  진행 상황인지 식별자가 없다. VLA 는 "한 번에 하나만 처리한다"는 FSM 의 설계 전제를
  근거로 **위치 기반 상관**(현재 pending_action 하나에 전부 매핑)으로 우회했다 — 상관
  키가 없을 때 시스템의 동시성 불변식을 근거로 정합성을 확보하는 예시. 단, 이 전제가
  깨지면(FSM 이 두 pick 을 동시에 돌리게 되면) 조용히 틀린 매핑이 된다는 게 명시적
  리스크로 남아 있음(§4번 열린 확인 요청).
- **판단(what)과 실행(how)의 소유권을 코드로 분리** 🟢: VLA 가 "어느 클래스를, 어디로"만
  판단하고 좌표 변환·충돌 회피·grasp 계산·모션은 전부 FSM 쪽. 이 경계를 지키려고 이번
  세션에 VLA 쪽 **단독 실행 스택 전체(로봇 이동·그리퍼·손목 grasp 계산, 17개 파일)를
  삭제**했다 — 안 쓰는 코드를 남겨두면 "이론상 두 실행 경로가 있다"는 상태가 되어 사고
  발생 시 어느 경로가 실제로 돈 건지 감사(audit)가 불가능해진다. 죽은 코드 삭제가
  단순 정리가 아니라 **안전 감사 가능성(auditability)**의 문제라는 게 발표 포인트.

### 9.2 안전 설계 — 코드 레벨 강제와 UI 레벨 신뢰의 경계

- **승인은 코드로 막고, 위험한 명령은 설명(prompt)으로만 막는다는 비대칭을 명시적으로
  인지하고 설계함** 🔴: `/pick/approve` 는 VLA 코드에 호출 경로 자체가 없다
  (`BLOCKED_CMDS`) — 모델이 뭘 하든 물리적으로 못 부른다, **강한 보장**. 반면 `reset`
  (SAFE_STOP 에서만 유효하고 성공 시 사람 승인 없이 HOME 관절이동까지 실제로 실행됨)은
  코드 레벨 상태 확인 없이 **LLM 툴 설명 문장**에만 기대어 "SAFE_STOP 일 때만 불러라"를
  지킨다 — **약한 보장**, FSM 쪽이 SAFE_STOP 밖에서 거부하는 걸 유일한 안전판으로 신뢰.
  cross-review 로 이 비대칭을 잡아 문서화만 해두고(코드는 안 고침, 실기 미검증) 사용자
  판단으로 넘김. "안전장치는 어디서 강제되는가"를 레이어별로 구분해 말할 수 있는 사례.
- **진행 상태 표시가 판단을 재트리거하지 않도록 방어** 🟢: `/pick/state` 구독으로 FSM 의
  중간 단계(LIFT/PLACE/WAIT_APPROVAL 등)를 GUI 에 실시간 표시하게 했는데, 이 스트림이
  에이전트의 의사결정 루프에 새 이벤트로 들어가면 "표시용 상태"가 "새 명령 트리거"로
  둔갑할 위험이 있다. `current_action` 이 채워진 진행중 상태는 `last_result` 를 비워
  발행해 `agent_node` 가 `if not last_result or current_action: return` 으로 명시적으로
  무시하게 했다 — **관측(observability)과 제어(control)의 경로를 코드로 분리**한 것.

### 9.3 ws 연결에 필수적이었던 것 — 실무 팁

- **경계는 커스텀 msg 가 아니라 JSON 문자열 하나** — `vla_interfaces`(커스텀 ROS msg)를
  cobot2_ws 로 안 넘기기로 확정. 이유: 커스텀 msg 를 경계로 쓰면 두 독립 clone 이
  **msg 버전으로 묶여** 한쪽만 스키마를 바꿔도 둘 다 다시 빌드해야 한다. JSON 이면
  스키마 진화(필드 추가)가 한쪽만 배포해도 하위 호환으로 굴러간다.
- **계약서는 한쪽 repo 에만 정본을 두고 나머지는 절대경로로 참조** — `vla-bridge-contract.md`
  정본은 cobot2_ws 에만 있고 VLA 쪽엔 사본을 안 둔다(2026-08-10 사본 두 개가 어긋난
  사고 이후 확정한 규칙, 팀 컨벤션 문서 §0/§2). 두 repo 가 물리적으로 분리돼 있을 때 "계약
  문서 복사 붙여넣기"가 제일 먼저 깨지는 지점이라는 걸 실사고로 확인함.
- **`ROS_DOMAIN_ID` 명시 불일치가 조용히 아무 통신도 안 되게 만든다** 🟢: VLA 쪽은
  domain 미지정(=0 기본값), cobot2_ws 관행은 93 — 왕복 스모크에서 **양쪽 다 명시적으로
  93 을 줘야** 서로 보였다. 에러 메시지 없이 그냥 아무것도 안 오는 실패 모드라 디버깅
  비용이 큼 — "왜 아무 반응이 없지" 류 증상을 보면 QoS 다음으로 domain ID 를 의심.
- **QoS(특히 durability)는 양쪽이 문자 그대로 같아야** 🟢: `TRANSIENT_LOCAL depth=1` 로
  맞춘 토픽들은 발행자가 이보다 얕은 durability 면 조용히 매칭 안 됨 — FSM 쪽에서 먼저
  기본값(QoS 미지정)으로 두고 VLA 가 맞춰 구독하는 순서로 진행(§1 표 참고), 반대로
  VLA 가 먼저 강한 가정을 세우면 FSM 쪽 변경 하나로 조용히 깨질 수 있어 **"기본값은 어느
  쪽이 소유하는가"를 문서에 명시**해뒀다(`fsm-state-integration.md`).
- **독립 clone 간 "확인 요청"은 개수를 최소화해서 목록으로 던진다** — FSM 쪽에 매번
  왕복 대화로 묻는 대신, VLA 쪽이 스스로 구현을 다 끝내고 **"이 4개만 확인해달라"**는
  요청서(`fsm-state-integration.md`)를 만들어 넘겼다. 세션이 물리적으로 분리된 두 에이전트
  간 협업에서 왕복 비용을 줄이는 실무 패턴 — "제안 없이 질문만" 대신 "구현 다 하고
  가정 목록만 검증받기".

### 9.4 오늘 실기로 검증된 것(VLA 쪽) 🟢 / 남은 것 🔴

- 🟢 브리지 경계 스모크: `place` 필드 파싱, TTL 만료→rejected, accepted 경로
  (`/get_keyword` 수동 호출) — `pick_fsm` 자체는 안 띄우고 `vla_command_node` 단독으로.
- 🟢 빌드/테스트: 이번 세션 변경(실행 스택 삭제 17개 파일 + 상태 연동) 후
  `./scripts/build.sh` PASS, 139 tests passed.
- 🔴 실제 VLA PC(핫스팟 경유)↔cobot2_ws 왕복 미검증 — 오늘은 같은 랩탑에서
  `ros2 topic pub` 으로 흉내만 냄.
- 🔴 `/pick/state` 발행을 VLA 가 실제로 수신해 GUI 에 뜨는지 미확인(QoS 는 코드로만
  맞춤, 실기 확인 안 됨).
- 🔴 `reset` 명령의 SAFE_STOP→HOME 실제 왕복 미검증.
