<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    GPU 확보 후 착수할 항목별 순서·인터페이스·검증 기준
-->

# Plan: GPU 확보 후 착수할 스프린트 후보 (GPU 없이는 시작 불가)

**작성일:** 2026-08-03
**전제:** `M0609_perception_motion_sprint_plan.md` §6-2에서 보류된 항목들. 이 랩탑은 Intel UHD 내장뿐, NVIDIA GPU 없음(`constraints.md` 확인 완료). **이 문서의 어떤 명령도 GPU 머신을 확보하기 전엔 실행하지 않는다** — 지금 할 일은 GPU가 생겼을 때 바로 시작할 수 있도록 순서·인터페이스·검증 기준만 정해두는 것.
> 📁 문서 지도: [[ws/cobot2/README]] · 상태: **⏸ 보류 — GPU 머신 확보 전엔 어떤 명령도 실행하지 않는다.**
> CPU로 되는 구간은 [[ws/cobot2/plans/2026-08-01-pc-role-split]], 현재 스택 기준선은 [[ws/cobot2/review_moveit]].

**현재 상태:** 카메라 파이프라인(D435i/C270)과 MoveIt2 Octomap 충돌 회피 확인 완료 → GPU-불필요 구간(§6-1)은 이미 실기 검증된 스택 위에 있음. 이 문서는 그 위에 얹을 GPU 구간만 다룬다.

---

## 0. Day0 — GPU 머신이 생기면 제일 먼저 할 것

- `nvidia-smi`, `docker info | grep -i runtime` 로 nvidia-docker 런타임 확인
- `GraspGenX` 저장소 실재 확인 (`NVlabs/GraspGen`은 확인됨, `GraspGenX`는 미확인 — §6-2 원문 그대로. URL 하나 열어보는 것으로 끝)
- 위 둘 중 하나라도 실패하면 이 계획 전체를 그 자리에서 재조정 (대체안은 각 항목 "실패 시" 참고)

---

## 1. 항목별 계획

### 1-1. FoundationPose (6D pose 추정)

- **선행조건:** GPU + CUDA/TensorRT, `isaac_ros_pose_estimation` release-3.2가 Humble에서 빌드되는지
- **작업:** `M0609_perception_motion_sprint_plan.md` Day4 P0 블록 그대로 재사용 (이미 작성돼 있음, 새로 쓸 것 없음)
- **인터페이스 선계약(GPU 없이 지금 정할 수 있는 것):** 출력 토픽/메시지 타입만 먼저 고정해두면, §6-1의 "색상/AprilTag CPU 대체재"를 그대로 갈아끼울 수 있음
  - 예: `geometry_msgs/PoseStamped` on `/perception/object_pose` (frame_id=`base_link`)
- **DoD:** 물체 1종 6D pose 오차 <5mm(위치)/사전정의 각도 오차(회전), 실측 대비
- **실패 시:** release-3.2에 패키지 없으면 그 시점 최신 Humble 호환 태그로 대체, 그래도 없으면 CPU 대체재(§6-1)를 그대로 본채택

### 1-2. GraspGenX (RG2 그립 생성)

- **선행조건:** GraspGenX 저장소 실재(Day0에 확인), FoundationPose가 준 pose/메시 입력
- **작업:** Day4 P0 블록 재사용. `integrate_gripper.py`/`run_graspgenx.py`는 문서 자체가 스크립트명 미확인이라 적어둔 것 — 저장소 확인 후 실제 CLI로 교체
- **DoD:** 실행 가능(충돌 없는) 그립 후보 1개 이상
- **실패 시:** `NVlabs/GraspGen`의 Robotiq 2F-140 체크포인트를 RG2 스트로크(110mm)로 오프셋 보정, 그래도 안 되면 물체 1종 한정 그립 지점 하드코딩

### 1-3. nvblox 3D 재구성

- **우선순위:** 낮음. 충돌 회피는 이미 Octomap이 실기 검증된 상태로 담당 중 — nvblox는 시각화 전용
- **작업:** 확보된 GPU에서 §Day1 P0 블록(release-3.2) 그대로. Octomap 대체가 아니라 병행 실행만
- **DoD:** RViz에서 mesh/voxel 재구성 실시간 확인 (충돌 회피 판단에는 관여 안 시킴)

### 1-4. cuTAMP / cuRobo / cuMotion (GPU 모션 플래닝)

- **선행조건 재검토 기준:** §6-1의 CPU OMPL 스택(narrow-passage 샘플러 튜닝 포함)이 실측으로 병목이라고 확인된 뒤에만 착수. GPU가 생겨도 자동 채택 대상 아님
- **판단 근거를 만드는 법:** Day3 플래너 튜닝 로그(성공률/평균 계획시간)를 먼저 쌓아두고, 그 수치가 실사용에 부족할 때만 이 항목을 꺼낸다
- **DoD 없음 — 착수 조건 미충족 시 계획하지 않음** (ponytail: 병목이 확인되지 않은 최적화는 스코프에서 제외, 필요해지면 그때 계획 작성)

### 1-5. VLM 기반 자연어 지시 → 서브골 생성

- **선행조건:** 로컬 GPU 불필요(API 호출로 가능), 단 TAMP-lite 골격(§6-1의 PDDLStream)이 먼저 서야 붙일 대상이 생김
- **순서:** §6-1 PDDLStream 완료 → 이 항목. 지금은 순서만 기록, 별도 계획 불필요

---

## 2. 이 문서에서 하지 않은 것

- 항목별 상세 커맨드 재작성 — Day4 블록이 이미 있고 GPU 확보 전엔 실행할 수 없으므로 중복 작성 안 함
- cuTAMP/cuRobo 계획 — 착수 조건(OMPL 병목 확인) 미충족, 조건 충족 시 별도 문서 작성

## 3. 확인 요청

GPU 머신 확보 시점/경로(사내 서버, 클라우드 GPU, 신규 구매 등)가 정해지면 §0 Day0 체크를 그 환경에서 먼저 돌려야 이 계획의 나머지가 유효해짐.

---
확신도: 추론(근거 있으나 미확인) — 이 문서 자체는 계획서라 실행 검증 대상 없음
내가 채워넣은 가정: (1) cuTAMP류는 OMPL 병목 확인 전까지 보류 (2) VLM 항목은 PDDLStream 완료 후 순서 (3) nvblox는 GPU 확보해도 우선순위 낮음으로 유지
확인 요청: GPU 머신 확보 경로(사내 서버/클라우드/구매)가 정해졌나요?
