<!-- meta
updated: 2026-08-09 (작성)
status:  live (일부 검증됨 — 나머지는 `rokey` GPU 머신 필요)
owns:    seg_source/run_bridge 기본값 전환(2026-08-08~09)의 잔여 검증 항목
-->

# graspx yolo 기본값 전환 — 잔여 검증 (CPU 개인PC에서 못 끝낸 부분)

> 관련: [[ws/cobot2/plans/2026-08-09-yolo-seg-finetune]](같은 전제 — `seg_source=yolo`가 기본이라는
> 사실 위에 얹힌 계획), [[ws/cobot2/state]]
> 이 세션(`kimkh-17U70N-GA70K`, 개인PC·GPU 없음, `nvidia-smi` 없음·`lspci`엔 Intel UHD만)
> 확인 결과다. **`hostname` 먼저 확인함** — CLAUDE.md 4절 규칙.

---

## 0. 이번 세션에서 바꾼 것

사용자 요청("target_classes로 물체 하나씩 지정해 yolo만 돌릴 예정")에 따라 기본값 2곳을 전환:

| 파일 | 값 | 이전 → 이후 |
|---|---|---|
| `capture_graspgenx_scene.py:91` | `DEFAULTS['seg_source']` | `geometric` → `yolo` |
| `graspx.launch.py:41` | `ARGS['seg_source']` | `geometric` → `yolo` |
| `graspx.launch.py:43` | `ARGS['run_bridge']` | `true` → `false` (컨테이너에서 무심코 같이 뜨는 사고 방지) |

`colcon build --symlink-install --packages-select graspgenx_perception` — **PASS** (이번 세션 실행).
`ros2 launch graspgenx_perception graspx.launch.py --show-args` — **PASS**, 새 기본값 반영 확인.

## 1. "CPU 환경에서도 테스트되나?" — 답: **부분적으로만.** 코드 경로 자체는 되지만 파이프라인 전체는 안 된다

### 1-1. 검증됨 (이 머신, 이번 세션에 실행)

- **`yolo_seg_node`의 `device='cpu'` 경로 자체는 CPU에서 동작한다.**
  ultralytics가 이 호스트 시스템 파이썬에 이미 있다(`8.4.76`, `torch 2.7.1+cu126`,
  `torch.cuda.is_available() == False` — 셋 다 이번 세션 `python3 -c` 실행으로 확인).
  임시로 구한 `yolo11n-seg.pt`(COCO 80종, 이 repo 정본 아님 — 아래 1-2 참고)로
  `output/00/rgb.png`를 `device='cpu'`로 직접 추론: **0.31초, box 9개, mask (9,480,848)** —
  예외 없이 끝남.
  → **코드가 GPU를 하드코딩하지 않았다는 것**은 증명됐다. `device:=cpu` 런치 인자가 죽는 경로는 아니다.

### 1-2. 이 머신에서 막힌 것 (코드 문제 아님 — 환경에 없는 것들)

- ⛔ **가중치가 이 워크스페이스에 없다.** `object_detection` 패키지 share에 `.pt` 0개
  (`find src/object_detection` 확인 — `.gitignore`의 `*.pt` 때문에 커밋 안 됨, README 규칙대로).
  방금 쓴 가중치는 이 repo 정본이 아니라 예전 세션이 스크래치패드에 받아둔 것이다.
  → **rokey에도 실제로 놓여 있는지 확인 필요** (`yolo_seg_node.py:51-52`
  `DEFAULT_WEIGHT_PKG='object_detection'`, `DEFAULT_WEIGHT_NAME='yolo11n-seg.pt'`).
- ⛔ **`od_kimkh` 컨테이너가 이 머신에 없다** (`docker images`/`docker ps -a` 확인 —
  `ros:humble`, `dsr_emulator`, `portainer`만 있다). `scripts/graspx_container.sh`가 가정하는
  실행 환경 자체가 이 머신엔 없다 — GPU 유무와 별개로 컨테이너 경로는 여기서 원천 불가.
- ⛔ **`grasp_bridge_node` → `graspgen_worker.py`(GraspGenX)는 CPU 경로가 아예 없다.**
  `grep cuda|device|cpu`가 0건 — device 선택 코드 자체가 없다는 뜻은 GraspGenX 쪽에
  하드코딩된 CUDA 의존이라는 뜻이다(2026-08-05 실기 검증 당시부터 GPU 전제, README §2 "8GB VRAM 기준"
  주석도 그 근거). **`run_bridge:=true`로 뭘 하든 이 머신에선 애초에 안 뜬다.**
  → seg_source 기본값 전환과 무관하게, grasp 계산 자체는 항상 GPU 필요.
- ⛔ **카메라가 없다.** D435i는 `rokey`에 물려 있다. `image_topic` 구독 자체를 테스트할 방법이 없다
  (bag 파일도 이 워크스페이스엔 없음 — `find *.db3` 0건, 이전 세션 확인 사항 재확인).

## 2. `rokey`(GPU)에서 마저 확인해야 할 것

1. **컨테이너 안 정본 가중치 확인** —
   `docker exec od_kimkh ls -l $(ros2 pkg prefix object_detection)/share/object_detection/*.pt`
   같은 명령으로 실제 `.pt`가 있는지, 있으면 몇 종 클래스인지(COCO 80 그대로인지).
2. **새 기본값으로 컨테이너 인자 없이 뜨는지** —
   `scripts/graspx_container.sh` 인자 없이 실행 → `run_yolo=true, run_bridge=false`가
   기본이므로 컨테이너 안엔 `yolo_seg_node`만 뜨는지 확인 (`docker exec od_kimkh pgrep -af ros2`).
3. **호스트 쪽 새 명령줄** — `graspx.launch.py:9-11`에 적어둔 대로
   `run_yolo:=false run_bridge:=true target_classes:=apple`로 실행해 `grasp_bridge_node`가
   실제로 뜨는지, `/grasp/compute` 서비스 콜까지 도는지.
4. **`target_classes` 필터가 `seg_source=yolo` 기본값에서 실제로 먹는지** — 물체 하나 놓고
   `target_classes:=apple`과 `target_classes:=cup`으로 각각 호출해 다른 라벨이 잡히는지 대조.
5. **`device:=cpu` vs `device:=0` 지연시간 비교(선택)** — 1-1에서 이 머신은 0.31초/프레임이었다.
   RTX 4060에서 GPU 대비 몇 배 차이 나는지는 궁금하면 참고용으로만 재보면 된다 — 운용은 GPU 기본.

## 3. 결론

- **런치 파일·기본값 변경 자체는 이 세션에서 빌드·인자 파싱까지 다 검증됨.** 여기서 더 할 게 없다.
- **실제 추론·grasp 계산·컨테이너 기동은 전부 `rokey` 전용.** 이 개인PC로는 코드가 CPU를
  거부하지 않는다는 것 이상은 증명 못 한다 — GraspGenX 워커가 GPU 필수라서 파이프라인
  절반(세그멘테이션 이후 전부)은 애초에 CPU 환경으로 시험 불가능한 설계다.

---
확신도: 검증됨 (1절의 모든 항목은 이번 세션 도구 출력 기준). 2절은 미실행 — 다음 `rokey` 세션 체크리스트다.
내가 채워넣은 가정: (1) "CPU에서도 테스트되나"는 방금 바꾼 seg_source/run_bridge 기본값 전환을 뜻한다고
해석 (2) rokey의 `od_kimkh`에 정본 가중치가 있다고 가정하고 확인 항목으로만 남김 — 실측 아님
(3) GraspGenX CPU 미지원은 소스에 device 분기 코드가 0건이라는 데서 추론한 것이지, GraspGenX
자체 문서를 읽어 확정한 것은 아니다
확인 요청: 다음 `rokey` 세션에서 2절 순서대로 그대로 확인하면 되나? (O/X)
