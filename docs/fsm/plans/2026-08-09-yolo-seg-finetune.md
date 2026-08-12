<!-- meta
updated: 2026-08-09 (작성)
status:  live (미착수 — 계획만. 캡처·학습 모두 GPU 실기 머신 `rokey` 필요)
owns:    YOLO11-seg 파인튜닝 방법 · 자동 어노테이션 경로 · 필요 요소 체크리스트
-->

# YOLO11-seg 파인튜닝 계획 — 우리 물체를 직접 학습시킨다

> 관련: [[ws/cobot2/state]] "다음 할 일" 0-b · [[ws/cobot2/plans/2026-08-07-graspgenx-target-matching]] ·
> [[ws/cobot2/plans/2026-08-10-presentation]] · 패키지 레퍼런스는 `src/PACKAGES.md`
> (2026-08-09 이관 — 패키지별 README는 포인터만 남는다), 날짜별 로그는 [[ws/cobot2/graspgenx-perception-notes]]
> 이 세션(`kimkh-17U70N-GA70K`, 개인PC·GPU 없음)에서 **실행으로 확인한 것만** "검증됨"으로 적는다.

---

## 0. 왜 하나 — 이건 성능 개선이 아니라 **막힌 것을 뚫는 일**이다

지금 `yolo_seg_node`는 COCO 80종으로 학습된 `yolo11n-seg.pt`를 쓴다. 우리가 실제로 집을
물체가 COCO에 없으면 **어떤 클래스 인덱스로도 못 잡는다** (`graspx.launch.py:55` — "이 가중치엔
공구 5종이 없다"). 그래서 지금까지 데모는 사과·바나나·컵처럼 COCO에 우연히 있는 물체로만 됐다.

- `seg_source` 기본값이 2026-08-08부터 **`yolo`**다 (`capture_graspgenx_scene.py:91`).
  즉 지금 파이프라인의 물체 인식은 **전적으로 이 가중치에 걸려 있다**.
- `target_classes`(잡을 물체를 이름으로 지정)는 `seg_source='yolo'`에서만 동작한다
  (`grasp_bridge_node.py:243-245`). `geometric` 폴백으로 내려가면 클래스를 잃는다.
- state.md "다음 할 일" **0-b(물체 이름 지정)**의 잔여 블로커가 정확히 이것이다.

→ **파인튜닝은 0-b를 닫는 일이다.** "GraspGenX 성능 과시"는 그 위에 얹히는 부수 효과다.

---

## 1. 결론 — 어노테이션 툴은 사지도, 설치하지도 않는다. **이미 이 repo에 있다**

`capture_graspgenx_scene.py`의 `segment()`(`:409`)가 depth로 물체 마스크를 이미 만든다:

```
작업공간 박스(base_link 기준 x/y/z bounds) ∩ 테이블면보다 obj_min_h 위 ∩ obj_max_h 아래
  → cv2.morphologyEx OPEN 3x3 (가장자리 노이즈 제거)
  → cv2.connectedComponents → 라벨 101, 102, ...
```

**신경망 0개 · 사람 클릭 0회 · GPU 0.** 고정 eye-to-hand D435i + 평면 작업대라는 이 ws의
조건에 정확히 맞고, 이미 실기에서 grasp를 뽑아낸 경로라 좌표계·TF·해상도 정합이 검증돼 있다.

남은 일은 라벨맵 → YOLO-seg 폴리곤 변환 하나였고, `scripts/seg_to_yolo.py`로 작성했다
(2026-08-09, `--self-check` PASS — **이번 세션에 실행함**).

### 검토했지만 안 쓰기로 한 것

| 후보 | 왜 안 쓰나 |
|---|---|
| SAM2 / `ultralytics.data.annotator.auto_annotate` | 박스를 줄 detector가 먼저 필요하다. 우리 물체는 COCO에 없으니 순환 문제. depth가 이미 마스크를 주는데 GPU 모델을 얹을 이유가 없다 |
| YOLOE 텍스트/비주얼 프롬프트 | 위와 같음. `ultralytics 8.4.76`에 `YOLOE`·`YOLOEVPSegPredictor` 존재는 확인함(검증됨) — **투명·박막 물체로 depth가 깨질 때의 폴백**으로만 남긴다 |
| Roboflow Auto Label / CVAT 서버 / Label Studio | 유료 또는 서버 세팅. 검수용으로 CVAT는 나중에 고려 가능하나 지금 규모(클래스 2~5개)엔 과하다 |

---

## 2. 방법 — 4단계

### 2-1. 수집 (실기 `rokey` 필요)

**한 번에 한 종류만** 작업대에 놓고, 위치·자세·조명을 바꿔가며 씬을 반복 캡처한다.
클래스 라벨을 사람이 프레임마다 안 붙여도 되는 이유가 이것이다 — 그 배치의 클래스는 하나뿐이다.

```bash
export ROS_DOMAIN_ID=93
ros2 launch m0609_rg2_bringup camera.launch.py        # 카메라 + 캘리브 TF
# 로봇 bringup 도 필요하다 — base_link ← camera_color_optical_frame TF 가 없으면 캡처가 실패한다
ros2 run graspgenx_perception capture_graspgenx_scene --ros-args \
    -p out_dir:=$HOME/cobot2_ws/data/ft_bolt -p scene:=0001   # ⚠️ 미검증(명령 미실행)
```

- ⚠️ **로봇 팔을 작업공간 박스 밖으로 치운다.** 이 경로엔 self-filter가 없어서 팔이 물체로
  라벨링된다(`capture_graspgenx_scene.py` docstring). `obj_max_h`(기본 0.12)로 높이 컷은
  걸리지만 완전하지 않다.
- 물체를 **서로 떨어뜨려** 놓는다. 붙어 있으면 `connectedComponents`가 한 덩어리로 만든다.
- 캡처마다 `scene` 이름을 바꾼다(안 바꾸면 덮어쓴다).

### 2-2. 라벨 변환 (머신 무관, GPU 불필요)

```bash
python3 scripts/seg_to_yolo.py data/ft_bolt dataset --cls 0 --name bolt
python3 scripts/seg_to_yolo.py data/ft_nut  dataset --cls 1 --name nut
# -> dataset/{images,labels}/train/*, dataset/data.yaml (--cls 별로 names 누적)
```

`seg.png`의 라벨값 >100인 덩어리만 폴리곤으로 뽑고(table=2, ground=0은 무시), `min_pixels`
미만은 버린다. 자체 검증: `python3 scripts/seg_to_yolo.py --self-check` → **PASS(2026-08-09 실행)**.

### 2-3. 학습 (GPU 필요 — `rokey` RTX 4060)

```bash
yolo segment train model=yolo11n-seg.pt data=dataset/data.yaml \
     epochs=100 imgsz=640 batch=8 \
     copy_paste=0.5 degrees=180 fliplr=0.5 flipud=0.5 hsv_v=0.5   # ⚠️ 미검증(미실행)
```

🔴 **`copy_paste`가 이 계획에서 가장 중요한 한 줄이다.** 기본값이 `0.0`이고(2026-08-09
`get_cfg()`로 확인 — 검증됨), 우리 데이터는 **한 장면에 물체 하나 + 배경이 항상 같은 작업대**라
그냥 학습하면 배경에 오버피팅된다. 데모에서 물체 여러 개를 늘어놓는 순간 깨진다.
`copy_paste`가 마스크를 잘라 다른 장면에 붙여 "여러 물체가 섞인 장면"을 합성해준다.
`degrees=180`은 eye-to-hand 고정 시점에서 물체만 회전하는 우리 상황에 맞다.

### 2-4. 배선

학습 산출물 `runs/segment/train*/weights/best.pt`를 `yolo_seg_node`가 찾는 경로에 놓는다.

- 가중치는 `object_detection` 패키지의 share 경로에서 로드된다(`yolo_seg_node.py:49-52`,
  `DEFAULT_WEIGHT_NAME='yolo11n-seg.pt'`). `.gitignore`의 `*.pt` 때문에 **커밋되지 않는다** —
  가중치 배포는 별도 수단이 필요하다(§4).
- 파인튜닝 모델은 클래스 인덱스가 COCO와 **완전히 다르다**. `classes:='[46,47]'`(banana/apple)
  같은 기존 인자를 그대로 쓰면 엉뚱한 것을 잡거나 아무것도 못 잡는다 → **런치 인자·문서·
  `target_classes` 예시를 전부 새 클래스 이름으로 갱신해야 한다.**

---

## 3. 필요한 요소 체크리스트

| 구분 | 항목 | 지금 상태 |
|---|---|---|
| 하드웨어 | `rokey` 머신 (RTX 4060) | 이 세션에서 접근 불가 — 개인PC임 |
| 하드웨어 | D435i + 캘리브 TF (`T_cam2base.npy` 최신) | 캘리브는 2026-08-03 이후 재측정 이력 있음. **파인튜닝 캡처 직전에 TF 유효성 확인 필요** |
| 하드웨어 | 로봇 bringup (TF 공급원) | 모션은 필요 없다. TF만 있으면 된다 |
| 하드웨어 | **학습시킬 물체 실물** | ⛔ **아직 정해지지 않았다 (§5 미결정 1번)** |
| 소프트웨어 | `ultralytics` (컨테이너 `od_kimkh`) | 컨테이너에 있음. 개인PC 호스트엔 8.4.76 설치돼 있음(검증됨) |
| 소프트웨어 | `scripts/seg_to_yolo.py` | ✅ 작성 완료, self-check PASS |
| 소프트웨어 | 캡처를 여러 장 자동 반복하는 수단 | ⛔ **없다.** `capture_graspgenx_scene`은 **한 번에 한 씬**만 뜬다 (§5 미결정 2번) |
| 데이터 | 클래스당 이미지 수 | 목표 200~300장. **근거 없는 감이다** — 고정 카메라·고정 배경이라 적게 든다는 판단이지 실측 아님 |
| 데이터 | 검증셋 | ⛔ 없다. `seg_to_yolo.py`는 `val: images/train`으로 쓴다 — **mAP를 믿으면 안 된다** |
| 시간 | 수집 | 클래스당 30~60분 (추측) |
| 시간 | 학습 | RTX 4060 / 300장 / 100 epoch / yolo11n-seg 기준 30분~1시간 (추측, 미측정) |

---

## 4. 위험 요소 — 먼저 알고 들어간다

1. **⏰ 실기 세션 경합** — 발표 마감이 아니라 **`rokey` 머신 시간**이 진짜 제약이다.
   이 작업은 실기 세션을 하루치 이상 먹고(수집→학습→배선→재검증), 같은 머신을 쓰는
   [[ws/cobot2/plans/2026-08-09-cumotion-verify]] T1~T7과 경합한다. 순서를 먼저 정할 것.
   **발표 자료 관점에서는 이 작업이 오히려 자산이다** — COCO에 없는 물체를 직접 학습시켜
   잡는 before/after가 지금 발표에 없는 정량 축을 채운다([[ws/cobot2/plans/2026-08-10-presentation]] §3-2).
   > ⚠️ **2026-08-09 정정**: 이 항목은 원래 "발표가 내일(08-10)이라 시간이 없다"였다.
   > 근거는 `2026-08-10-presentation.md`의 파일명·제목뿐이었는데 **그건 발표일이 아니다**(발표일 비공개).
   > 이 ws의 plan 파일명 날짜는 **작성일** 규약이다 — 파일명을 마감일로 읽은 것이 오독이었다.
2. **자동 라벨은 초안이다.** depth 세그가 틀린 프레임(그림자, 팔 침범, 물체 붙음)이 그대로
   학습 데이터가 되면 모델이 그 오류를 학습한다. → **`rgb.png` 위에 `seg.png`를 겹쳐 눈으로
   훑는 단계가 반드시 필요하다.** 지금 그 도구가 없다(§5 미결정 3번).
3. **가중치는 git에 안 들어간다**(`.gitignore` `*.pt`). `rokey`에서 학습한 `best.pt`를 어떻게
   보관·배포할지 정해야 한다. 정하지 않으면 다음 세션에 사라진다.
4. **배경 오버피팅** — §2-3의 `copy_paste` 참고. 완화책이지 해결책은 아니다.
   여유가 있으면 작업대에 천을 깔거나 조명을 바꿔 배경 변형을 물리적으로 넣는 게 확실하다.
5. **투명·박막 물체면 이 계획 전체가 무너진다.** depth가 안 잡히면 마스크가 안 나온다.
   그때가 YOLOE/SAM2로 올라갈 유일한 정당한 시점이다.
6. 클래스 인덱스가 바뀌므로 **기존 문서·런치 예시의 COCO 인덱스가 전부 거짓이 된다**(§2-4).

---

## 5. 결정해야 할 것 (사용자 확인 필요)

1. **어떤 물체를 학습시키나?** 종류와 개수. depth로 잡히는 재질인지(투명/광택/검정 무광 주의).
2. **여러 씬 캡처를 어떻게 반복하나?** `capture_graspgenx_scene`은 1회 1씬이다. 선택지:
   (a) 사람이 물체를 옮기고 명령을 반복 실행 — 도구 0개, 지루함
   (b) `scene` 이름을 자동 증가시키며 N초 간격으로 도는 얇은 루프 추가 — 20줄
   → 30장 넘어가면 (b)가 낫다고 본다.
3. **라벨 육안 검수 도구를 만드나?** `rgb.png`+`seg.png` 오버레이를 격자로 붙여 PNG 한 장으로
   뽑는 스크립트면 충분하다(~20줄). CVAT까지는 지금 규모에 과하다.
4. **`best.pt` 보관 위치** — `object_detection` share? 별도 릴리스? 팀 공유 드라이브?

---

## 6. DoD (이게 다 되면 "완료"라고 쓴다)

- [ ] 클래스별 데이터셋이 `dataset/`에 있고 `data.yaml`의 `names`가 실제 물체와 일치
- [ ] 라벨 육안 검수 1회 통과 (틀린 프레임 제거 후 장수 기록)
- [ ] `yolo segment train` 완료, `best.pt` 경로 기록
- [ ] `yolo_seg_node`가 새 가중치로 기동, `/yolo_seg/classes`에 새 클래스 이름이 뜨는 것 확인
- [ ] `grasp_bridge_node -p target_classes:=<새이름>`으로 `/grasp/best` 발행 확인
- [ ] 실패 사례 1개 이상을 [[ws/cobot2/context/constraints]]에 기록
- [ ] **발표용 자료를 같은 세션에서 회수** — 같은 씬의 COCO / 파인튜닝 `/yolo_seg/overlay` 두 장,
      `rgb.png`+`seg.png` 쌍 몇 장(사람 라벨링 0회를 보여주는 그림).
      근거·목록은 [[ws/cobot2/plans/2026-08-10-presentation]] §3-2.
      **실기 세션이 끝나면 다시 뽑으려고 세션을 또 잡아야 한다.**
