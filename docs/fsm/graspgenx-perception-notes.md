<!-- meta
updated: 2026-08-09
status:  live — src/graspgenx_perception/README.md 에서 이관(2026-08-09, 3-README 통합 작업).
         날짜별 실기 검증·버그 발견·설계 검토 기록이다. 안정된 레퍼런스(토픽·파라미터·
         실행법·현재 상태)는 src/PACKAGES.md "graspgenx_perception" 절로 옮겼다 — 거기가
         지금 참인 값의 단일 출처다. 여기는 원문을 그대로 보존한 이력이다.
owns:    graspgenx_perception 패키지의 날짜별 실기 검증·버그·설계 검토 기록 (원문 그대로 보존)
-->

# graspgenx_perception — 실기 검증·설계 검토 로그 (이력)

> 레퍼런스(토픽·파라미터·실행법·현재 검증 상태)는 `src/PACKAGES.md`
> "graspgenx_perception" 절이 단일 출처다. GraspGenX **알고리즘** 자체(출력 규약, 폭 계산)의
> 단일 출처는 `md/detect_graspx.md`다. 여기는 이 패키지의 날짜별 실기 디버깅·검토 과정을
> 원문 그대로 보존한 것이다.

---


(구 `yolo_seg` — GraspGenX 파이프라인에 결합하며 패키지명을 바꿨다. 노드명·토픽(`/yolo_seg/*`)은
그대로다.)

YOLO 인스턴스 세그멘테이션을 ROS 토픽에 붙인다. 컬러 이미지를 구독해 **인스턴스 라벨맵**과
**이진 마스크**를 발행한다.

원본 실험 스크립트(`yoloseg.py`)에서 두 가지를 바꿨다:

- **pyrealsense2 로 카메라를 직접 열지 않는다.** RealSense 는 한 프로세스만 잡을 수 있고
  이 워크스페이스는 `realsense2_camera` 가 이미 물고 있다(graspx 가 정렬 depth 를 쓴다).
  직접 열면 둘 중 하나가 죽으므로 컬러 **토픽을 구독**한다.
- **`show=True` 대신 overlay 토픽.** GUI 창은 컨테이너 X11 에 묶이고 헤드리스에서 죽는다.
  `publish_overlay:=true` 로 켜는 이미지 토픽으로 뺐다.

## 실행 환경 — 컨테이너 전용이다 (2026-08-07 재확인)

**이 노드는 호스트에서 돌지 않는다.** 호스트 시스템 파이썬에 `ultralytics`/`torch` 가 없기
때문이다. 넣지도 말 것 — torch 가 numpy 를 끌어올려 apt `cv_bridge` 를 깬다
(`~/.claude/CLAUDE.md` §3).

2026-08-07 이 PC를 직접 측정한 상태다. **README 이전 버전(2026-08-06)의 서술은 세 항목이
뒤집혔으므로 그대로 믿지 말 것** — 그날은 GPU도 가중치도 컨테이너도 없는 상태였다.

| 확인 | 2026-08-06 (옛 README) | **2026-08-07 실측** |
|---|---|---|
| 호스트 GPU | 없음 | **RTX 4060 Laptop** (driver 595.84, CUDA 13.2) |
| `od_kimkh` 컨테이너 | 없음 | **있음** (`object_detection_backup_20260806:latest`) |
| 그 컨테이너 GPU 패스스루 | 미설정 | **설정됨** — 컨테이너 안 `torch.cuda.is_available()` **True** |
| 컨테이너 파이썬 | — | torch 2.13.0+cu130 / ultralytics 8.4.113 / numpy 1.26.4 |
| 호스트 `~/.local` 오염 | torch·ultralytics·anyio 있었음 | **정리됨** — `pymodbus` 하나뿐. numpy 는 apt 1.21.5 |
| 가중치 `yolo11n-seg.pt` | 없음 | **있음** (`src/object_detection/resource/`, 6.2MB) |
| 로봇·카메라 | 미연결 | **연결됨** — `/camera/camera` 848×480, `dsr01` 컨트롤러 기동 중 |

~~`~/.local` 이 정리된 덕분에 **`pytest` 를 그냥 돌려도 된다.**~~
🔴 **2026-08-08 되돌아갔다 — `-p no:anyio` 를 다시 붙여야 한다.**
`~/M0609_VLA_system` 의 `pip install --user -r requirements.txt` 가 `~/.local` 에
`anyio 4.13`·`torch 2.7.1`·`opencv-python 4.10`·`ultralytics 8.4.76`·`numpy 1.24.4` 를
다시 깔았다. 증상은 `ModuleNotFoundError: No module named '_pytest.scope'`(apt pytest 6.x
와 anyio 4.x 의 충돌). 우회하면 **24개 PASS**(2026-08-08 실측).

```bash
python3 -m pytest -p no:anyio src/graspgenx_perception/test/test_yolo_seg.py
```

`cv_bridge` 왕복은 아직 정상이다(segfault 없음, 같은 날 실측). 자세한 것은
[[ws/cobot2/plans/2026-08-08-vla-integration]] §2.

## 빠른 실행

⚠️ **`ROS_DOMAIN_ID` 가 호스트와 컨테이너에서 같아야 한다.** 이 ws 의 규약은 **93** 이다
(`src/pick_fsm/README.md` §2 실행 절이 단일 출처). 컨테이너 이미지에는 이미 `ROS_DOMAIN_ID=93`
이 `Config.Env` 로 박혀 있으므로 **컨테이너에서는 아무것도 export 하지 않아도 맞는다.**

틀리기 쉬운 쪽은 **호스트**다. 호스트 셸은 기본이 도메인 0 이라 bringup·카메라를 `export
ROS_DOMAIN_ID=93` 없이 띄우면 컨테이너와 갈라진다. 2026-08-07 이 세션에서 실제로 그 상태였고,
증상은 "컨테이너에서 `ros2 topic list` 에 카메라 토픽이 **0개**" 였다.

```bash
docker start od_kimkh && docker exec -it od_kimkh bash
# --- 컨테이너 안 (도메인은 이미 93) ---
source /opt/ros/humble/setup.bash && source /home/kimkh/cobot2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
ros2 run graspgenx_perception yolo_seg_node --ros-args -p publish_overlay:=true -p device:=0
```

한 줄로(대화형 셸 없이) 띄울 때는 **래퍼 스크립트를 쓴다.** 기존 인스턴스 정리 · pty 부착 ·
종료 시 정리를 한 번에 한다. 인자는 `graspx.launch.py` 로 그대로 넘어간다:

```bash
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true classes:='[46,47,49]'
```

**표준 2터미널 구성 (2026-08-08~, YOLO 가 주 파이프라인)** — 컨테이너가 "무엇이 보이나",
호스트가 "무엇을 잡나"를 맡는다. `classes` 는 넓게, `target_classes` 는 좁게 둔다:

```bash
# [컨테이너] 탐지 — person(0) 을 넣지 않는다. yolo 경로엔 self-filter 가 없다
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true \
  classes:='[39,41,44,46,47,49,64]'

# [호스트] 파지 계산 — 집을 것만 지정
export ROS_DOMAIN_ID=93
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false seg_source:=yolo \
  target_classes:=apple
```

> ✅ **2026-08-08 실사용 확인.** 이 한 줄로 누적돼 있던 **10개 → 1개**가 됐다
> (`ros2 topic info /yolo_seg/mask` 의 Publisher count 10 → **1**, GPU 프로세스 1개 360MB,
> swap 2.0Gi 소진 → 581Mi). 락 파일 `/tmp/yolo_seg_node-<mask_topic>-<uid>.lock` 이 생기고
> 프로세스에 pty 가 붙는다(`ps` 의 `TT` 가 `?` 가 아니라 `pts/N`).
> **아직 미검증**: 그 상태에서 Ctrl-C 가 실제로 전달되는지는 눌러 보지 않았다 —
> 그래서 래퍼는 `-t` 외에 INT/TERM/HUP 트랩 정리를 보험으로 함께 건다.

> 🔴 **`docker exec` 를 직접 쓰지 말 것 — Ctrl-C 가 컨테이너 안까지 가지 않는다.**
> `docker exec` 에는 `--sig-proxy` 가 **없다**(`docker exec --help`, docker 29.7.0 확인).
> `-t` 를 안 붙이면 컨테이너 안에 제어 터미널이 없어서(`ps` 의 `TT` 가 `?`) 호스트 Ctrl-C 는
> 호스트 쪽 docker 클라이언트만 죽이고, `ros2 launch` 는 containerd-shim 밑에 **살아서**
> 남는다. **재실행 1회 = 인스턴스 +1** 이다. 2026-08-08 에 이 방식으로 `yolo_seg_node` 가
> **10개**까지 쌓였다 — RAM 12GB, VRAM 2.6GB, CPU 8코어, `/yolo_seg/mask` 의 publisher 10개.
> (숫자는 그때의 스냅샷이다. 고정값이 아니라 "재실행마다 +1"이 사실이다.)
>
> ~~종료 후 `<defunct>` 좀비로 남는다~~ 는 **오진이었다.** 남는 것은 `Z` 가 아니라 CPU 를
> 90% 쓰며 계속 도는 `Sl` 프로세스다. "좀비니까 무해하다"로 읽혀서 더 위험했다.
>
> **단, 상태는 두 가지다 (2026-08-08 추가).** 위는 *정리되지 않은* 경우다. 래퍼나 `pkill` 로
> **정리한 뒤에는 진짜 좀비(`Z`)로 남고, 그건 영원히 안 사라진다** — 컨테이너 PID 1 이
> `sleep infinity` 라 `wait()` 를 호출하지 않아 자식을 수거하지 못한다(실측: 좀비 10개,
> 부모 PID 1 = `sleep infinity`). 좀비는 RAM·GPU 를 쓰지 않으므로(같은 시각 GPU compute
> apps 0개, VRAM 65/8188 MiB) 자원 문제는 아니다. **문제는 오독이다** — `pgrep -af
> yolo_seg_node` 가 10줄을 뱉으니 "돌고 있다"고 읽게 된다. 실제로 2026-08-08 에 이걸로
> "토픽이 안 온다"를 한참 헤맸다. **`ps -eo pid,stat,cmd` 로 `STAT` 를 같이 볼 것.**
> `Z` 뿐이면 아무도 안 돌고 있는 것이다. 정리는 `docker restart od_kimkh`.
>
> 대화형 셸(위 `docker exec -it od_kimkh bash`)은 pty 가 붙으므로 Ctrl-C 가 정상 동작한다.
> 문제는 **`-t` 없는 한 줄 실행**이다.

쌓인 것을 손으로 정리하려면(컨테이너 안 PID 기준):

```bash
docker exec od_kimkh pgrep -af yolo_seg_node        # 확인
docker exec od_kimkh pkill -f graspgenx_perception  # 전부 종료
```

오버레이를 보려면 **호스트** 터미널에서:

```bash
export ROS_DOMAIN_ID=93
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
ros2 run rqt_image_view rqt_image_view
#   토픽 드롭다운에서 /yolo_seg/overlay 를 고르고 transport 를 compressed 로 둔다
```

> ~~도메인 93 에서 컨테이너 → 호스트 방향 데이터가 지금 안 흐른다.~~
> **2026-08-07 21:15 재측정에서 이 방향이 정상으로 돌아왔다** (`/yolo_seg/labels` 호스트 수신
> 25.6 Hz). 아래 "컨테이너 → 호스트 — 재측정" 절 참고.

## 데이터가 안 올 때 — 위에서부터

노드가 떠 있는데 아무것도 안 나오면 이 순서로 본다. 노드는 5초마다
`5초간 <토픽> 를 한 장도 못 받았다` 경고를 찍으므로 **먼저 노드 로그를 본다.**

| # | 확인 | 명령 | 정상 |
|---|---|---|---|
| **1** | **도메인이 양쪽에서 같은가** | 호스트·컨테이너 각각 `echo $ROS_DOMAIN_ID` | **둘 다 93.** 다르면 **토픽이 아예 안 보인다** |
| 2 | 입력이 들어오는가 | 노드 로그에 watchdog 경고가 없는가 | 경고 없음 |
| 3 | 오버레이가 켜져 있는가 | `-p publish_overlay:=true` 를 줬는가 | 안 주면 `/yolo_seg/overlay` **토픽 자체가 없다** |
| 4 | 양쪽에 프로파일이 걸렸는가 | `echo $FASTRTPS_DEFAULT_PROFILES_FILE` | 빈 값이면 **토픽은 보이는데 데이터가 0** |
| 5 | 데이터가 오는가 | `ros2 topic hz /yolo_seg/labels` | 카메라 fps 와 같은 값 |
| **6** | **인스턴스가 하나뿐인가** | `ros2 topic info /yolo_seg/mask` | **Publisher count: 1.** 2 이상이면 프레임마다 다른 인스턴스의 마스크가 섞여 온다 |

**노드가 뜨자마자 죽고 `'/yolo_seg/mask' 에 발행하는 인스턴스가 이미 PID N 로 돌고 있다` 가
찍히면** 중복 방지 락(`acquire_singleton`)이 막은 것이다. 버그가 아니라 설계된 실패다 —
메시지에 적힌 `kill -INT N` 으로 기존 것을 끝내거나 `scripts/graspx_container.sh` 로 다시
띄운다. 락 파일은 `/tmp/yolo_seg_node-<mask_topic>-<uid>.lock` 이고, 키가 **노드 이름이 아니라
`mask_topic`** 이라 카메라 2대를 서로 다른 토픽으로 돌리는 구성은 정상적으로 공존한다.

**1번과 4번은 증상이 다르다.** 이걸 구분하면 진단이 빨라진다 (2026-08-07 A/B 실측. 그날
호스트 스택이 도메인 0 에 떠 있었으므로 "맞음"이 0 이었다 — 규약대로면 93 이다):

| 도메인 일치 | 프로파일 | `ros2 topic list` 의 camera 토픽 | `ros2 topic hz` |
|---|---|---|---|
| 불일치 | 있음 | **0개** | — |
| 불일치 | 없음 | **0개** | — |
| **일치** | 없음 | 47개 | **데이터 안 옴** |
| **일치** | **있음** | 47개 | **14.06 Hz** ✅ |

즉 **도메인이 탐색을, 프로파일이 데이터를 결정한다.**

`ROS_DOMAIN_ID=93` 은 컨테이너 이미지의 `Config.Env` 에 박혀 있어 `docker exec` 마다 상속된다
(`.bashrc` 에는 없다). 이 값이 ws 규약과 같으므로 **컨테이너 쪽은 그대로 두고, 호스트 셸에서
`export ROS_DOMAIN_ID=93` 을 빠뜨리지 않는 것**이 맞는 운용이다.

`FASTRTPS_DEFAULT_PROFILES_FILE` 이 필요한 이유는 `fastdds_udp_only.xml` 주석에 있다 —
FastDDS 공유메모리가 컨테이너 경계를 못 넘는다.

## ✅ 컨테이너 → 호스트 — 재측정 (2026-08-07 21:15)

**같은 날 저녁 재측정에서 이 방향이 정상이다.** 아래 "🔴 오전 측정" 은 기록으로 남긴다 —
원인을 특정하지 못한 채 증상이 사라졌으므로 **재발할 수 있다고 보고, 매번 실측으로 확인한다.**

컨테이너에서 `yolo_seg_node` 를 띄우고 `/yolo_seg/labels`(1280×720 mono8, 921KB/프레임)를
`yolo_seg_node` 와 **같은 QoS**(BEST_EFFORT, depth=1)로 세 지점에서 동시 수신:

| 수신 위치 | 프로파일 | 수신율 |
|---|---|---|
| 컨테이너 내부 | 있음 | 24.9 Hz |
| **호스트** | **있음** | **25.6 Hz** |
| 호스트 | 없음 | 13.8 Hz (절반 — 프로파일을 걸 것) |

⚠️ **측정 도구를 조심할 것.** `ros2 topic hz` 는 기본이 RELIABLE 구독이라 BEST_EFFORT
퍼블리셔(카메라·이 노드)와 **QoS 불일치로 0건이 나온다** — 전송 장애처럼 보인다.
Humble 의 `ros2 topic hz` 에는 `--qos-reliability` 플래그가 **없다**. 이번 세션에서 실제로
이걸 전송 실패로 오진할 뻔했다. rclpy 로 BEST_EFFORT 구독을 짜서 재는 게 확실하다.

### 🔴 오전 측정 (2026-08-07, 원인 미특정 — 기록)

당시엔 **`yolo_seg_node` 를 컨테이너에서 띄우면 호스트의 어떤 소비자도 `/yolo_seg/*` 를 못
받았다.** 측정한 것 (20바이트 `std_msgs/String` 프로브까지 내려가서 확인):

| 방향 | 결과 |
|---|---|
| **호스트 → 컨테이너** | **5.000 Hz — 정상** |
| **컨테이너 → 호스트** | **0건** |

- 도메인 **0 / 77 / 93 셋 다** 같은 결과다 — 도메인 경합이 아니다.
- FastDDS 프로파일 **있으나 없으나** 같다 — SHM/UDP 선택 문제가 아니다.
- 메시지 크기 무관 — 53KB(`overlay/compressed`)도 407KB(`labels`)도 20B(String)도 전부 0건.
- **탐색은 양방향으로 된다.** 호스트에서 `ros2 topic info -v /yolo_seg/labels` 가 퍼블리셔
  GID·QoS 까지 다 보여준다. 데이터만 안 온다.
- **같은 날 오전에는 이 방향이 14.095 Hz 로 됐다** (도메인 0, 호스트 스택도 0). 그 뒤 호스트
  스택이 93 으로 재기동됐고 컨테이너도 `docker stop`/`start` 를 거쳤다. 그 사이에 무엇이
  바뀌었는지는 특정하지 못했다.

재현:

```bash
docker exec od_kimkh bash -c 'source /opt/ros/humble/setup.bash; \
  timeout 25 ros2 topic pub -r 5 /probe std_msgs/String "{data: hi}"' &
export ROS_DOMAIN_ID=93 && ros2 topic hz /probe      # 0건
```

원인 미특정이므로 **추측으로 고치지 말 것**(`~/.claude/CLAUDE.md` §7). 전용 `/debug` 세션이
필요하다. 유력 가설 순서: (1) `net=host` 컨테이너와 호스트가 participant 포트를 나눠 갖는
과정에서 컨테이너 퍼블리셔가 도달 불가한 unicast locator 를 광고, (2) `docker stop` 이 남긴
root 소유 `/dev/shm/fastrtps_*` 잔재(현재 5개), (3) 호스트 스택 재기동 스크립트가 바꾼 환경변수.

**우회**: `seg_source:=geometric` 은 호스트 안에서만 도는 경로라 이 문제와 무관하다.
지금 pick_fsm 파이프라인의 기본값이 `geometric` 이므로 **현재 파이프라인은 이 버그에 걸리지
않는다.**

## 컨테이너 운용에서 확인된 것 (2026-08-07 저녁 실측)

**1. 컨테이너 쪽 `FASTRTPS_DEFAULT_PROFILES_FILE` 은 필수다 — 호스트 쪽은 아니다.**
호스트 카메라(`realsense2_camera_node`)는 지금 이 변수 **없이** 떠 있는데(`/proc/<pid>/environ`
확인), 컨테이너에서 변수를 걸면 정상 수신된다. A/B (yolo_seg_node 와 같은 BEST_EFFORT/depth=1
구독으로 10초씩):

| 컨테이너 프로파일 | `/camera/camera/color/image_raw` 수신 |
|---|---|
| 없음 | **0 Hz** (watchdog 경고 3회/18초) |
| 있음 | **26.6 Hz** (참고: 호스트에서 24.6 Hz) |

**2. `yolo_seg_node` 를 돌릴 때마다 인스턴스가 하나씩 쌓인다** — 실행 횟수와 1:1 로 증가한다.

> 🔴 **2026-08-08 정정.** 이 절은 원래 "좀비(`Z <defunct>`)가 쌓인다 / 기능에는 영향이 없다 /
> 정리는 `docker restart` 뿐"이라고 적혀 있었다. **두 상태를 하나로 뭉갠 것이 문제였다.**
>
> | | 살아 있는 동안 | 죽인 뒤 |
> |---|---|---|
> | 상태 | **`Sl`/`Rl` — CPU 90% 를 쓰며 돈다** | `Z <defunct>` |
> | 영향 | **`/yolo_seg/mask` publisher 가 인스턴스 수만큼.** 소비자가 프레임마다 다른 인스턴스의 마스크를 받는다. RAM 12GB · VRAM 2.6GB · swap 2GB 전량 소진 | PID 만 소모. 기능 영향 없음 |
> | 원인 | **`docker exec` 에 `--sig-proxy` 가 없어 Ctrl-C 가 컨테이너 안까지 안 간다** (위 "빠른 실행" 🔴 박스) | PID 1 이 `sleep infinity` 라 자식을 reap 하지 않는다 |
> | 정리 | `docker exec od_kimkh pkill -f graspgenx_perception` | `docker restart od_kimkh` |
>
> **옛 서술의 reap 메커니즘은 맞았다** — 2026-08-08 에 누적된 10개를 `pkill` 했더니 부모가
> PID 1 인 `Z <defunct>` 가 정확히 10개 생겼다(실측). 틀린 것은 **"쌓이는 동안의 상태와
> 영향"** 이다. 아직 죽지 않은 프로세스를 좀비로 적어 두는 바람에 "좀비니까 무해하다"로
> 읽혔고, CPU 90% 를 쓰며 토픽을 오염시키는 실제 문제가 **문서에 가려졌다.**
>
> 예방은 `scripts/graspx_container.sh` 로 띄우는 것, 마지막 방어선은 노드 자신의 flock 이다.
> 죽인 뒤 남는 좀비는 무해하지만 신경 쓰이면 `docker restart od_kimkh`(도는 노드도 같이 죽는다).

**3. 프로파일을 걸었는데도 프레임이 0인 실행이 4회 중 1회 있었다** (첫 실행, 35초 내내
watchdog 경고). 같은 명령의 이후 3회는 전부 정상이었다. **원인 미특정** — 그래서 실행할 때마다
노드 로그의 watchdog 경고 유무를 먼저 본다.

**4. 카메라 해상도가 848×480 이 아니라 1280×720 이다** (2026-08-07 21:12 실측:
`color`/`aligned_depth_to_color` 둘 다 1280×720, rgb8). 이 문서의 속도·대역폭 수치는
848×480 시절 측정값이므로 **지금 값이 아니다** — 프레임당 raw 는 1.16MB 가 아니라 2.76MB 다.

## 이 PC에서 지금 테스트 가능한가

**2026-08-07 이 세션에서 직접 빌드·실행해 확인했다.** 실기 모션 명령은 하나도 실행하지 않았다
(이 노드는 카메라를 구독하고 마스크를 발행할 뿐 로봇을 움직이지 않는다).

| 하고 싶은 것 | 지금 이 PC에서 | 근거 |
|---|---|---|
| `colcon build --packages-select graspgenx_perception` | **가능** | 실행함 — **PASS** (0.87s) |
| 순수 함수 유닛테스트 | **가능** | `pytest` 그냥 실행 → **10 passed**. 우회 플래그 불필요 |
| 호스트에서 `yolo_seg_node` 실행 | **불가** | 호스트에 `ultralytics`/`torch` 없음 (`_load_model()` 에서 ImportError) |
| 컨테이너에서 GPU 추론 | **가능 — 확인함** | `od_kimkh`, `torch.cuda.is_available()` True, RTX 4060 |
| **실제 카메라 → GPU 추론 → 토픽 발행** | **가능 — 확인함** | 848×480 라이브 입력, watchdog 경고 0건, 아래 수신율 참고 |
| CPU 추론(`device:=cpu`) | **가능** | 컨테이너 내 실측 47.8 ms/frame |
| `/grasp/compute` 등 graspx 연동 | **이번에 재확인 안 함** | 이전 세션 값만 있다 — "검증 결과" 표 참고 |

### 실측 성능 (2026-08-07, 라이브 카메라 848×480)

추론 20회 median, warmup 3회 제외:

| device | median | min / max |
|---|---|---|
| `0` (RTX 4060 Laptop) | **5.5 ms** | 5.2 / 6.0 |
| `cpu` (컨테이너) | **47.8 ms** | 46.3 / 72.8 |

수신율 — 컨테이너 내부와 호스트에서 **같은 시간창에** 동시 측정:

| 측정 위치 | `labels` | `overlay/compressed` |
|---|---|---|
| 컨테이너 내부 | 14.075 Hz | 14.091 Hz |
| 호스트 | 14.095 Hz | 14.036 Hz |

같은 창에서 카메라 원본이 12.6~14.3 Hz 였다. **경계를 넘으며 잃는 프레임이 없다.**
오버레이는 약 53 KB/프레임(543 KB/s).

> 측정 함정: 이전 실행의 노드가 안 죽은 채로 새로 띄우면 퍼블리셔가 둘이 되어 `hz` 가 **두 배**로
> 나온다(28~30 Hz). 재실행 전 `pkill -f yolo_seg_node` 로 확인할 것 — 이번 세션에서 실제로 한 번
> 속았고, 그 값을 오버레이 손실로 오해할 뻔했다.

## 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| sub | `/camera/camera/color/image_raw` | `sensor_msgs/Image` (**rgb8**) | BEST_EFFORT, **depth=1** |
| pub | `/yolo_seg/labels` | `sensor_msgs/Image` (mono8) | 인스턴스 라벨맵. `obj_1`→101, `obj_2`→102 … |
| pub | `/yolo_seg/classes` | `std_msgs/String` (JSON) | 라벨값→클래스 이름. 라벨맵과 같은 stamp |
| pub | `/yolo_seg/mask` | `sensor_msgs/Image` (mono8) | 전경 이진 마스크 0/255 |
| pub | `/yolo_seg/overlay/compressed` | `sensor_msgs/CompressedImage` (jpeg) | `publish_overlay:=true` 일 때. **기본** |
| pub | `/yolo_seg/overlay` | `sensor_msgs/Image` (bgr8) | `overlay_compressed:=false` 일 때만 |

입력 토픽의 **와이어 인코딩은 `rgb8` 이다**(2026-08-07 실측 — 옛 README 는 `bgr8` 로 잘못 적혀
있었다). 노드는 `imgmsg_to_cv2(desired_encoding='bgr8')` 로 받으므로 cv_bridge 가 변환해 준다.
직접 `imgmsg_to_cv2(msg)` 로 받아 쓰는 코드를 새로 짜면 **채널이 뒤집힌다.**

**오버레이가 왜 JPEG 이 기본인가**: 848×480 bgr8 = 1.16MB 다. UDP 전용 경로로는 이 크기를
15Hz 로 못 보낸다 — 컨테이너 안에서조차 3.75Hz 까지 떨어졌다(실측). 같은 화면이 JPEG q80 이면
**53KB**(22배)라 카메라 fps 그대로 무손실로 나간다. rqt 로 볼 때 raw 를 고르면 그림이 안 뜨거나
끊긴다.

라벨 규약(`LABEL_OBJ_BASE=100`, `MAX_OBJECTS=155`)은 `graspgenx_perception/capture_graspgenx_scene.py`
와 맞췄다. GraspGenX 로더가 `obj_` 접두어 라벨만 보기 때문이다. 상한이 갈리면 조용히
어긋나므로 테스트(`test_max_objects_matches_capture_script`)가 두 파일을 대조한다.

**겹침 처리**: 입력이 신뢰도 내림차순이라 **역순으로 칠해 고신뢰가 최상위**에 온다.
겹침으로 픽셀이 0개가 되거나 `min_pixels` 미만으로 깎인 인스턴스는 버리고 남은 것에
101부터 **연속으로** 다시 매긴다 — 라벨이 비면(`101,103,…`) 소비자의 "obj_N = N번째 물체"
가정이 깨진다. `build_label_map` 이 `kept` 를 같이 돌려주는 것도 이 재번호 매기기 때문이다:
번호를 다시 매기고 나면 라벨에서 원래 인스턴스(=클래스)로 되돌아갈 길이 없다.

⚠️ **단, 이 연속성은 발행 시점의 규약일 뿐 하류의 불변식이 아니다.** 브리지의
`target_classes` 는 대상 외 라벨을 지워 일부러 구멍을 낸다(`101…108` → `107` 만).
GraspGenX 로더는 `label_map` 을 순회할 뿐 연속성을 가정하지 않으므로(`scene_loaders.py:92`,
2026-08-08 원문 확인) 무해하다. **필터 뒤에 "일관성 회복"이라며 재번호를 매기면 안 된다** —
클래스맵의 라벨 키가 통째로 어긋난다.

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `''` | 비우면 `object_detection` share 의 `resource/yolo11n-seg.pt` |
| `image_topic` | `/camera/camera/color/image_raw` | 구독할 컬러 토픽. depth=1 이라 추론이 느리면 묵은 프레임 대신 최신만 본다 |
| `mask_topic` / `label_topic` / `overlay_topic` | `/yolo_seg/{mask,labels,overlay}` | 발행 토픽 |
| `class_topic` | `/yolo_seg/classes` | 라벨값→클래스 이름 매핑(JSON). 아래 "클래스맵" 절 |
| `publish_overlay` | `false` | 오버레이 발행 여부 |
| `conf` | `0.25` | 신뢰도 임계 |
| `device` | `'0'` | `'0'`=첫 GPU, `'cpu'` |
| `classes` | `[]` | COCO 클래스 인덱스 필터. 비우면 전체. 예: `-p classes:="[1,16]"` |
| `max_objects` | `155` | 라벨맵이 uint8 이라 `100+156` 은 0 으로 랩어라운드한다. 더 크게 줘도 155로 잘린다 |
| `min_pixels` | `0` | 이보다 작은 인스턴스는 버린다(겹침으로 깎인 것 포함) |
| `overlay_compressed` | `true` | `false` 면 raw Image 로 발행 (대역폭 22배) |
| `overlay_jpeg_quality` | `80` | JPEG 품질 |

숫자·문자열 파라미터는 `dynamic_typing=True` 로 선언했다. launch/CLI 값은 YAML 로 파싱돼
`device:=0` 이 STRING 선언에 INTEGER 로 들어오고 `InvalidParameterTypeException` 이 난다.

`classes` 도 같은 이유다. 빈 리스트를 그냥 넘기면 rclpy 가 타입을 `BYTE_ARRAY` 로 추론해
정수 목록을 못 넣는다. `capture_graspgenx_scene.py:111` 이 같은 우회를 쓴다.

## 실행 명령 — 기하 vs YOLO 성능 비교 (2026-08-07 최신)

두 경로를 **번갈아 띄워** 같은 장면에서 비교한다. 바뀌는 것은 세그멘테이션뿐이고
GraspGenX 워커·선택 정책·서비스 호출은 완전히 같다.

### 공통 전제 (한 번만, 호스트에서)

```bash
export ROS_DOMAIN_ID=93                      # 빠뜨리면 컨테이너와 토픽이 아예 안 보인다
source /opt/ros/humble/setup.bash && source ~/cobot2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true   # 1280x720
# 로봇 bringup 은 실기 모션이라 사람이 직접 (base_link <- camera_color_optical_frame TF 필요)
```

### A. 기하 세그 (`geometric`) — 호스트 한 대

신경망 0개. `run_yolo:=false` 로 YOLO 노드를 아예 안 띄운다.

```bash
export ROS_DOMAIN_ID=93
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false
# 다른 터미널 (같은 도메인)
ros2 service call /grasp/compute std_srvs/srv/Trigger
```

### B. YOLO 세그 (`yolo`) — 컨테이너 + 호스트

**터미널 1 — 컨테이너**(GPU 추론. 프로파일 없으면 프레임 0장이다):

```bash
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true classes:='[46,47]'
```

> 이 자리에 `docker exec` 를 직접 쓰면 Ctrl-C 가 안 먹어 인스턴스가 누적된다.
> 위 "빠른 실행" 의 🔴 박스 참고.

**터미널 2 — 호스트**(브리지만. `seg_source:=yolo` 로 라벨맵을 쓰게 한다):

```bash
export ROS_DOMAIN_ID=93
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false seg_source:=yolo
# 다른 터미널
ros2 service call /grasp/compute std_srvs/srv/Trigger
```

**확인**: 컨테이너 노드 로그에 `한 장도 못 받았다` 경고가 없어야 한다. 오버레이를 보려면
호스트에서 `FASTRTPS_DEFAULT_PROFILES_FILE` 을 걸고 `rqt_image_view` →
`/yolo_seg/overlay/compressed`.

### 무엇을 비교하나

`/grasp/compute` 응답과 노드 로그에 비교에 필요한 것이 전부 찍힌다:

| 볼 것 | 어디에 |
|---|---|
| 채택된 물체 수·픽셀 수 | 브리지 로그의 세그 진단 (`obj_N: NNNN px`) |
| 세그 소요시간 | 기하 38.8 ms(CPU) vs YOLO 5.5 ms(GPU) — 848×480 시절 값, 1280×720 에선 재측정 필요 |
| grasp 후보 수·통과율 | `선택 단계:` 로그 (`점수 -> 도달 -> 접근축` 단계별 개수) |
| 최종 점수·손끝 좌표 | 서비스 응답 message |
| 눈으로 대조 | 저장된 씬의 `seg.png` / `rgb.png` (아래 "라이브 경로" 절의 저장 위치) |

씬이 호출마다 타임스탬프 디렉토리로 남으므로 **두 경로의 `seg.png` 를 나중에 나란히 열 수 있다.**

**2026-08-07 21:29 이 명령들로 실제 비교한 결과**(같은 장면, 위 A/B 를 그대로 실행.
씬은 `data/graspgenx_scene/cmp_geo`, `cmp_yolo` 에 남겨 뒀다):

| | `cmp_geo` (기하) | `cmp_yolo` (YOLO, `classes:="[46,47]"`) |
|---|---|---|
| 박스 안 픽셀 | 299,386 | 299,766 |
| 채택 물체 | **3개** (덩어리 7개 중) | **1개** |
| 크기 | 4182 / 5501 / 477 px | 4953 px |
| seg 라벨값 | `0, 2, 101, 102, 103` | `0, 101` |
| `label_map` | `ground/table/obj_1..3` | `obj_1` 만 |

읽는 법: 기하는 **박스 안이면 뭐든** 잡아 로봇 팔로 보이는 덩어리(표면중심 z=+0.25 m,
테이블 위 33 cm)까지 `obj_1` 로 넣었다. YOLO 는 `classes` 로 걸러 1개만 냈지만 그 대신
**테이블 라벨이 없다** — `segment_from_labels()` 는 `obj_` 만 만든다. GraspGenX 로더가
`obj_` 접두어만 보고 씬 점군엔 유효 depth 가 전부 들어가므로 이 차이는 결과에 영향이 없다.

### 단독 실행 (씬 한 장만 뜨고 싶을 때)

```bash
ros2 run graspgenx_perception capture_graspgenx_scene --ros-args -p scene:=cmp_geo
ros2 run graspgenx_perception capture_graspgenx_scene --ros-args \
  -p scene:=cmp_yolo -p seg_source:=yolo
```

> **2026-08-07 파일 위치를 통일했다.** 이전에는 소스가 워크스페이스 `scripts/` 에 있고
> `setup.py` 가 `scripts=[...]` 로 바깥 경로를 심었다 — 한 기능의 파일이 두 디렉토리에
> 흩어져 편집·grep 이 번거로웠다. 지금은 전부 패키지 안에 있다.
>
> | 이전 | 지금 |
> |---|---|
> | `graspgenx_perception/capture_graspgenx_scene.py` | `graspgenx_perception/capture_graspgenx_scene.py` |
> | `graspgenx_perception/grasp_bridge_node.py` | `graspgenx_perception/grasp_bridge_node.py` |
> | `scripts/graspgen_worker.py` | `graspgenx_perception/graspgen_worker.py` |
> | `scripts/test_capture_graspgenx_scene.py` | `test/manual_capture_scene.py` |
> | `scripts/test_grasp_bridge.py` | `test/manual_grasp_bridge.py` |
> | `scripts/test_scene_roundtrip.py` | `test/manual_scene_roundtrip.py` |
>
> **실행 파일 이름에서 `.py` 가 빠졌다** (`scripts=[...]` → `console_scripts`).
> 옛 이름으로 `ros2 run` 하면 실행 파일을 못 찾는다.
>
> 테스트 3종을 `test_*` 가 아니라 `manual_*` 로 바꾼 이유: 셋 다 pytest 함수가 없는
> **스크립트형**이라(최상위 `assert`, `sys.argv`, `rclpy.init()`, `import graspgenx`)
> `test/` 에 `test_` 이름으로 두면 `colcon test` 가 이들을 실행하다 깨진다. 이 패키지에
> 이미 있던 `test/manual_roundtrip.py` 와 같은 규칙이다.
>
> `graspgen_worker.py` 는 패키지 안에 있지만 **`console_scripts` 진입점이 아니다** —
> rclpy 가 아니라 GraspGenX venv 에서 `uv run python <경로>` 로 도는 별도 프로세스다.
> `grasp_bridge_node` 의 `worker_script` 파라미터가 형제 파일로 자동 해석한다.
>
> `--symlink-install` 로 빌드하면 `build/.../graspgenx_perception` 이 `src/` 를 가리키는
> 심볼릭 링크라 **파이썬 파일 편집은 재빌드 없이 즉시 반영된다**(2026-08-07 확인).

**두 노드는 같은 머신에서 못 돈다.** `yolo_seg_node` 는 ultralytics 때문에 **컨테이너 전용**
이고, `grasp_bridge_node` 는 GraspGenX 워커를 `uv` 로 띄우는데 **컨테이너에 uv 가 없다**.
그래서 위 "실행 명령" 절처럼 `run_yolo` / `run_bridge` 로 반씩 나눠 띄운다. **한 머신에서
둘 다 true 로 두면 안 된다** — 기본값이 둘 다 `true` 라서 인자 없이 띄우면 있는 쪽만 살고
없는 쪽은 즉시 죽는다.

카메라와 로봇 bringup 은 이 런치에 넣지 않았다 — bringup 은 실기 모션이라 사람이 직접
실행해야 하고, 카메라는 다른 파이프라인과 공유한다.

| 런치 인자 | 기본값 | 설명 |
|---|---|---|
| `seg_source` | `geometric` | `geometric` 또는 `yolo`. **브리지에만** 간다 |
| `run_yolo` / `run_bridge` | `true` | 어느 쪽을 띄울지 |
| `image_topic` | `/camera/camera/color/image_raw` | |
| `publish_overlay` | `true` | |
| `device` / `conf` / `min_pixels` | `0` / **`0.1`** / `300` | 런치의 `conf` 기본은 노드 기본(0.25)보다 낮다 — 의도한 값이다. 낮출수록 검출은 늘고 오검출도 는다 |
| `classes` | `'[]'` | **탐지할 물체 지정** — COCO 인덱스 목록. 아래 절 참고 |

## 탐지할 물체를 바꾸려면 — `classes` 파라미터 (banana 등)

**어디에 넣나: `yolo_seg_node` 의 `classes` 파라미터.** 브리지도 캡처 노드도 아니다.
값은 이름이 아니라 **COCO 클래스 인덱스의 정수 목록**이다. 비우면 80종 전부.

세 가지 넣는 법 (전부 같은 파라미터):

```bash
# 1) 런치 인자 (컨테이너에서 YOLO 를 띄우는 정식 경로)
ros2 launch graspgenx_perception graspx.launch.py run_bridge:=false classes:="[46,47]"

# 2) 노드 단독 실행
ros2 run graspgenx_perception yolo_seg_node --ros-args -p classes:="[46,47]" -p device:=0

# 3) 컨테이너 한 줄 — 래퍼를 쓴다 (docker exec 직접 호출은 Ctrl-C 가 안 먹는다)
scripts/graspx_container.sh run_bridge:=false device:=0 classes:='[46,47]'
```

⚠️ **`ros2 param set` 으로는 안 바뀐다.** 노드가 `__init__` 에서 한 번만 읽어
`self.classes` 에 담는다(`yolo_seg_node.py:122-123`). 바꾸려면 **노드를 다시 띄운다.**

자주 쓰는 인덱스 (2026-08-07 `yolo11n-seg.pt` 의 `model.names` 로 직접 확인):

| 물체 | idx | 물체 | idx | 물체 | idx |
|---|---|---|---|---|---|
| banana | **46** | cup | 41 | book | 73 |
| apple | **47** | bottle | 39 | vase | 75 |
| orange | 49 | bowl | 45 | scissors | 76 |
| sports ball | 32 | knife / fork / spoon | 43 / 42 / 44 | teddy bear | 77 |
| person | 0 | cell phone | 67 | dining table | 60 |

전체 목록은 컨테이너에서:

```bash
docker exec od_kimkh python3 -c "from ultralytics import YOLO; \
  print(YOLO('/home/kimkh/cobot2_ws/install/object_detection/share/object_detection/resource/yolo11n-seg.pt').names)"
```

## 클래스맵 — `/yolo_seg/classes` 와 `target_classes`

`classes` 를 넓히면 탐지가 늘지만, **`/yolo_seg/labels` 는 정수 라벨(101,102,…)뿐이라
"obj_2 가 사과였다"를 담지 못한다.** 그래서 `yolo_seg_node` 가 매 프레임 클래스맵을
`std_msgs/String`(JSON)으로 같이 낸다. 라벨맵과 **같은 `header.stamp`** 를 `stamp_ns` 로
싣는다 — 소비자가 여러 프레임을 모아 그중 하나를 고르므로, 최신값 하나만 들면 짝이 어긋난다.

```json
{"stamp_ns": 1754640000123456789, "frame_id": "camera_color_optical_frame",
 "objects": [{"label": 107, "class": "apple", "cls_id": 47, "conf": 0.237}]}
```

`grasp_bridge_node` 의 **`target_classes`** 가 이걸 써서 대상을 좁힌다. 콤마 구분
문자열이다(리스트가 아닌 이유: rcl YAML 파서의 리스트 타입 함정 — `CLAUDE.md` §4).

```bash
# 탐지는 7종, grasp 연산은 사과만
scripts/graspx_container.sh run_bridge:=false device:=0 classes:='[39,41,44,46,47,49,64]'
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false seg_source:=yolo \
    target_classes:=apple

# 런타임 변경이 먹는다 (compute() 마다 다시 읽는다 — yolo 쪽 classes 와 다른 점)
ros2 param set /grasp_bridge_node target_classes apple,cup
```

⚠️ **콤마 뒤에 공백을 넣지 말 것.** `target_classes:=apple, banana` 라고 쓰면 셸이 `banana` 를
별개 인자로 쪼개서 `malformed launch argument 'banana'` 로 죽는다. 공백을 쓰려면 통째로
따옴표: `target_classes:='apple, banana'`.

**두 파라미터의 역할이 다르다**: `classes`(yolo) = 무엇을 **탐지**할지, 넓게. 
`target_classes`(bridge) = 무엇을 **잡을지**, 좁게. 브리지는 대상 외 라벨을 **워커에
넘기기 전에** 0 으로 지우므로 GraspGenX 연산 자체가 줄어든다 — `target`(obj_N) 은 이미
계산이 끝난 결과에서 고르는 것이라 시간이 줄지 않는다.

클래스맵을 한 프레임도 못 받으면 브리지는 **실패로 끝낸다.** 조용히 전부 연산하면
"지정한 물체만"이 거짓말이 되기 때문이다.

**`classes` 는 필터지 학습이 아니다.** 이 가중치는 COCO 80종만 안다 — 이 프로젝트의 공구 5종
(drill/hammer/pliers/screwdriver/wrench)은 **어떤 인덱스로도 안 잡힌다.** 테이블 위 공구를
찍으면 `apple`/`cup`/`person` 같은 걸로 억지 매핑된다(2026-08-07 라이브 실측).
공구를 잡으려면 seg 데이터셋 재학습이 필요하고, 그 전까지는 `geometric` 이 정답이다.

**같이 손대게 되는 파라미터 둘**: `conf`(신뢰도 임계, 낮추면 더 잡지만 오검출↑),
`min_pixels`(이보다 작은 인스턴스는 버린다 — 겹침으로 깎인 것 포함).
`seg_source:=geometric` 경로는 depth 만 보므로 **`classes` 와 무관하다.**
| `out_dir` | `''` | 씬 4파일 저장 경로. 비우면 `<repo>/data/graspgenx_scene` (2026-08-07부터 항상 영구 저장, 임시 디렉토리 아님 — 아래 "라이브 경로" 절) |

## pick_fsm 과의 연결 — 지금 작동하는가

전체 사슬. **머신이 셋으로 갈린다**(컨테이너 / 호스트), 도메인은 **전부 93**이어야 한다.

```
카메라 /camera/camera/{color,aligned_depth_to_color}/image_raw   [호스트]
   │                                    │
   │ (seg_source=yolo — **기본**)        │ (seg_source=geometric 일 때만)
   ▼                                    │
yolo_seg_node  [컨테이너·GPU]            │
   │ /yolo_seg/labels                   │
   ▼                                    ▼
capture_graspgenx_scene.py ◀────────────┘        [호스트]
   │ 씬 4파일
   ▼
grasp_bridge_node.py ──uv──▶ GraspGenX 워커       [호스트]
   │ /grasp/compute (Trigger) · /grasp/best (PoseStamped, base_link, GraspGenX 원시 grasp 프레임)
   │   ⚠️ tool0 목표가 아니다 — FSM 이 to_gripper_base() 로 rg2_base_link 목표로 바꾼다
   ▼
task_manager (pick_fsm) ──▶ MoveIt ──▶ 로봇
```

### ⛔ 기본값 그대로 띄우면 연결이 안 된다

`pick_fsm` 의 `grasp_source` 기본값이 **`compute_grasp`** 인데
(`config/pick_fsm.yaml:66`, `launch/pick_fsm.launch.py:43`), 그 경로가 부르는
**`/grasp/compute_grasp` (`pick_fsm_msgs/ComputeGrasp`) 서버는 이 워크스페이스 어디에도
구현이 없다.** `grasp_bridge_node.py:141` 이 만드는 건 `/grasp/compute` (`std_srvs/Trigger`)
하나뿐이다. `pick_fsm/README.md` §3 도 이 계약을 "**아직 없음** — 정본 계약"으로 적어 두었다.

→ **`grasp_source:=legacy_trigger` 를 명시해야 한다.**

```bash
ros2 launch pick_fsm pick_fsm.launch.py grasp_source:=legacy_trigger
```

`legacy_trigger` 는 폭 정보를 못 받으므로 `default_width_m`(0.06 m, UNVERIFIED)로 잡는다.

### 세 경로의 현재 상태

| 경로 | 인식 소스 | 지금 |
|---|---|---|
| `seg_source=geometric` + `grasp_source=legacy_trigger` | depth (호스트 전용) | **유일하게 도는 조합.** 이 패키지의 `yolo_seg_node` 는 **아예 안 쓰인다** |
| `seg_source=yolo` | `/yolo_seg/labels` (컨테이너) | **전송은 뚫렸다**(2026-08-07 21:15, 호스트 25.6 Hz). 남은 막힘은 **COCO 클래스 불일치** 하나 |
| `grasp_source=compute_grasp` | — | **서버 없음.** 기본값이라 그대로 띄우면 여기서 걸린다 |

즉 **`graspgenx_perception` 의 `yolo_seg_node` 는 현재 pick_fsm 파이프라인에 실질적으로
연결돼 있지 않다.** 기본 경로가 depth 기반 기하 세그라서 라벨맵을 아무도 구독하지 않는다.
이 패키지에서 pick_fsm 이 실제로 쓰는 것은 `setup.py` 가 심어 둔
`capture_graspgenx_scene.py` / `grasp_bridge_node.py` 두 실행 파일이다.

> **⚠️ 2026-08-09 정정 — 위 두 문단과 표는 `seg_source` 기본값이 `geometric` 이던
> 2026-08-07 기준이다.** 2026-08-08 에 기본값이 **`yolo`** 로 바뀌었고
> (`capture_graspgenx_scene.py:91`), 그래서 **지금은 반대다**: `grasp_bridge_node` 를
> 인자 없이 띄우면 `/yolo_seg/labels` 를 **반드시** 구독하며, 컨테이너에서
> `yolo_seg_node` 를 같이 띄우지 않으면 `/grasp/compute` 가
> `seg_source=yolo 인데 라벨맵을 못 받았다`(`capture_graspgenx_scene.py:329`)로 실패한다.
> `grasp_source:=legacy_trigger` 가 필요하다는 지적은 그대로 유효하다
> (`/grasp/compute_grasp` 서버는 여전히 없다 — 2026-08-09 재확인).
>
> 이 기본값 변경이 `pick_fsm/README.md` §2 기동 순서에 반영돼 있지 않아 4번만 띄우게
> 되어 있었다 → 2026-08-09 에 3.5 단계로 보강했다.

### 확인한 전제 (2026-08-07, 도메인 93)

| 항목 | 상태 |
|---|---|
| `color` 848×480 / `aligned_depth_to_color` 848×480 | **일치** — `segment_from_labels` 의 shape 검사 통과 조건 |
| TF `base_link → camera_color_optical_frame` | **있음** (`camera_calib_tf`, xyz `[1.237, -0.237, 0.784]`) |
| 호스트에 `uv` | **있음** (`~/.local/bin/uv`) — 브리지가 GraspGenX 워커를 이걸로 띄운다 |
| `pick_fsm` 안전 기본값 | ⚠️ **바뀜(2026-08-09)** — `dry_run` 제거. 남은 건 `require_approval:=true` 뿐이고 **실기 모션이 실제로 나간다** |

## 기하 세그 vs YOLO-seg

`capture_graspgenx_scene.py` 의 `seg_source` 파라미터로 고른다. 라벨 규약(101,102,…)이
같아 변환이 없다.

| | 기하 (`geometric`) | YOLO (`yolo`) |
|---|---|---|
| 입력 | **depth** | **RGB** |
| 방식 | base 프레임 작업공간 박스 + 테이블면 높이 + `connectedComponents` | 학습된 클래스의 인스턴스 마스크 |
| 속도(848×480) | 38.8 ms (CPU, 이전 세션) | **5.5 ms** (RTX 4060 Laptop) / 47.8 ms (CPU) — 2026-08-07 실측 |
| 클래스 제한 | **없음** — 박스 안에 있으면 뭐든 잡는다 | 학습한 것만 |
| 붙어 있는 물체 | **하나로 뭉친다** | 분리한다 |
| 로봇 팔 | **물체로 잡힌다** (self-filter 없음) | 클래스에 없으면 무시된다 |
| 실기 씬 10 | `obj_1`~`obj_4` 채택 | `person`/`cell phone`/`sink` — **오검출** |
| 라이브 씬 (2026-08-07) | — | `apple`/`cup`/`person` — **여전히 COCO 클래스** |
| **실기 `/grasp/compute`** | **성공** (score 0.703, 46개) | **0개 통과** (충돌 필터 전멸) |

### 2026-08-08: 주 파이프라인을 **YOLO 로 바꾼다**

이전 결론은 "지금도 기하가 정답"이었다. 근거는 **공구 5종을 잡는 것이 목표**라는 전제였다 —
COCO 가 공구를 모르니 YOLO 는 쓸 수 없다는 논리다. 목표가 바뀌면 그 논리도 바뀐다.

**지금 목표는 "어떤 물체를 집을지 고르는 것"이다.** 그러면 두 경로의 우열이 뒤집힌다:

| | 기하 (`geometric`) | YOLO (`yolo`) |
|---|---|---|
| 물체를 **고를 수 있나** | ❌ **불가능** — 라벨이 `obj_1,obj_2`뿐이고 그게 뭔지 모른다 | ✅ `target_classes` |
| 붙어 있는 물체 | 하나로 뭉친다 | 분리한다 |
| 로봇 팔 self-filter | `obj_max_h` 로 자름 | ⚠️ **없다** — `person` 으로 잡힌다 |

**기하 경로는 물체 선정이 원리적으로 불가능하다.** depth 덩어리에는 정체가 없다. 사람이
`obj_3` 을 눈으로 보고 고르는 것 외에 방법이 없고, `obj_N` 은 프레임마다 바뀐다.
LLM/GUI 가 "사과를 집어"라고 말하는 순간 기하 경로에는 그 말을 받을 자리가 없다.

**그래서 COCO 80종이라는 한계를 받아들이고 YOLO 를 주 경로로 쓴다.** 잡을 물체를 COCO 안에서
고른다(사과·컵·병·바나나·가위…). 공구 5종은 seg 재학습 전까지 **대상에서 뺀다** — 억지로
`apple` 로 잡히는 것을 쓰느니 못 잡는 게 낫다.

`geometric` 은 지우지 않는다. **폴백**이다: 컨테이너/GPU 가 없을 때, COCO 밖 물체를 일단
집어야 할 때, YOLO 오검출을 의심할 때 같은 씬을 두 경로로 떠서 대조한다.

⚠️ **바꾸면서 같이 생기는 문제 하나**: 기하 경로의 self-filter(`obj_max_h`)가 yolo 경로엔
없다. COCO 0 = `person` 이라 로봇 팔·사람이 `obj_N` 으로 들어온다. 지금 막는 수단은
`classes` 에서 0 을 빼는 것뿐이다 (`classes:='[39,41,44,46,47,49]'` 처럼 **person 을 넣지 않는다**).

속도는 부차적이다 — 어느 쪽이든 병목은 GraspGenX 추론이라 세그 38.8ms 는 문제가 아니다.

## 가중치

`yolo11n-seg.pt` 는 `object_detection/resource/` 에 있어야 하고 **`.gitignore` 의 `*.pt` 로
커밋되지 않는다.**

> ⚠️ **어느 머신에서 보고 있는지부터 확인할 것.** 이 ws 는 두 PC 를 쓰고 hostname·계정이
> 같아서 **`nvidia-smi` 유무로만 구분된다**(`md/state.md` "두 PC 체제").
> **개인PC(CPU, `nvidia-smi` 없음)에서는 가중치도 컨테이너도 없는 게 정상이다** —
> 2026-08-08 개인PC 실측: `find`(src·build·install) 0건, `docker ps -a` 에 `od_kimkh` 없음.
> 그건 **그 머신의 사실일 뿐 GPU PC 와 무관하다.** 아래 서술과 이 절 이후의 컨테이너
> 이름·경로는 **GPU PC 기준**이다.

다른 PC 에서 `git pull` 만 하면 파일이 없어 노드가 뜨자마자 죽는다:

```bash
docker exec -it od_kimkh bash -lc \
  'cd /home/kimkh/cobot2_ws/src/object_detection/resource && python3 -c "from ultralytics import YOLO; YOLO(\"yolo11n-seg.pt\")"'
```

호스트에는 ultralytics 가 없으므로 **호스트에서 받는 방법은 없다.** 컨테이너에서 받아 바인드
마운트된 워크스페이스에 두는 게 유일한 경로다(`/home/kimkh/cobot2_ws` 가 컨테이너에 그대로
마운트돼 있다).

노드는 시작 시 두 번 막는다:

- 파일이 없으면 `FileNotFoundError`. ultralytics 는 basename 이 공식 에셋명이면 없는 경로를
  받아도 **조용히 네트워크에서 받아오므로**, 존재 확인을 노드가 먼저 한다.
- `model.task != 'segment'` 면 `RuntimeError`. 이 워크스페이스의 `yolov8n_tools_0122.pt` 는
  `task: detect` 라 **마스크를 못 낸다** — 여기 쓸 수 없다.

## 검증 결과

**2026-08-07 이 세션에서 재확인한 것:**

| 항목 | 상태 |
|---|---|
| `colcon build --symlink-install --packages-select graspgenx_perception` | **PASS** (0.87s) |
| `pytest src/graspgenx_perception/test/test_yolo_seg.py` (호스트, 우회 없음) | **PASS** 10개 |
| 컨테이너 GPU 가용성 (`torch.cuda.is_available()`) | **True** — RTX 4060 Laptop, torch 2.13.0+cu130 |
| 컨테이너에서 기본 가중치 자동 해석 + 로드 (`classes=80`) | **검증됨** (노드 로그) |
| **실기 카메라 → GPU 추론 → `labels`/`mask`/`overlay` 발행** | **검증됨** — watchdog 경고 0, ERROR 0 |
| 호스트 수신율 `labels` 14.095Hz / `overlay` 14.036Hz | **검증됨** — 무손실 |
| 컨테이너 수신율 `labels` 14.075Hz / `overlay` 14.091Hz | **검증됨** |
| 오버레이 대역폭 543 KB/s (≈53KB/프레임) | **검증됨** |
| 추론 속도 GPU 5.5ms / CPU 47.8ms (848×480 median×20) | **검증됨** |
| 도메인 일치/불일치 × 프로파일 유무 4조합 A/B | **검증됨** — 위 표 |
| 입력 와이어 인코딩이 `rgb8` | **검증됨** |
| 도메인 93(ws 규약)에서 컨테이너가 카메라 47토픽 수신 · 추론 정상 | **검증됨** — 컨테이너 내부 10.3Hz, watchdog 0 |
| **컨테이너 → 호스트 데이터 전송** | **🔴 실패** — 0건. 도메인 0/77/93, 프로파일 유무, 20B~407KB 전부. 위 "미해결" |
| 호스트 → 컨테이너 데이터 전송 | **정상** — 5.000Hz |
| `color`/`aligned_depth_to_color` 해상도 일치 (848×480) | **검증됨** |
| TF `base_link → camera_color_optical_frame` 존재 | **검증됨** |
| `/grasp/compute_grasp` 서버 부재 (pick_fsm 기본값이 이걸 부른다) | **확인됨** — 소스 전수 grep, 구현 없음 |

**클래스맵 / `target_classes` (2026-08-08 추가):**

| 항목 | 상태 |
|---|---|
| `colcon build --symlink-install --packages-select graspgenx_perception` | **PASS** |
| 순수 함수 테스트 24개 (`test_yolo_seg`, `test_best_labels`) | **PASS** |
| **거르고 나서 고른다** — 순서를 뒤집으면 대상이 깜빡인 컷이 뽑혀 grasp 0개 | cross-review 2026-08-08 지적, 수정 + 회귀 테스트(`test_filter_before_select_beats_select_before_filter`) |
| `class_payload()` — 저장된 실제 씬 `rgb.png`(1280×720) 로 컨테이너에서 실행 | **검증됨** — 8 인스턴스 → 라벨 101~108, `{107: apple, 106: cup, 105: dining table, …}` |
| `res.boxes.cls` 와 `res.masks` 인덱스 정렬 | **검증됨** — ultralytics 8.4.113, 마스크 2개 ↔ `boxes.cls` 길이 2 |
| `filter_labels_by_class()` — 위 클래스맵으로 `apple` 만 남기기 | **검증됨** — `[101…108]` → `[0, 107]` (3,330 px), 원본 배열 불변 |
| GraspGenX 로더가 라벨 구멍(101 없이 107만)을 견디는가 | **확인됨** — `scene_loaders.py:92` 가 `label_map` 순회, 연속성 미가정 |
| 라이브 파이프라인에서 `/yolo_seg/classes` 실제 발행·수신 | ⚠️ **미검증** — 카메라+컨테이너 기동 필요 |
| `target_classes` 로 grasp 연산 시간이 실제로 줄어드는지 | ⚠️ **미검증** — 워커 실행 필요 |

**이전 세션 값 — 이번에 재확인하지 않았다:**

| 항목 | 상태 |
|---|---|
| 컨테이너 통합 — 194×259 입력 → `labels` 194×259, `mask` `[0,255]` | 검증됨 (이전) |
| `classes:="[1,16]"` 필터 — 4개 검출이 2개(`[0,101,102]`)로 | 검증됨 (이전) |
| 입력 없을 때 5초 watchdog 경고 · SIGTERM 시 스택트레이스 없이 종료 | 검증됨 (이전) |
| `graspx.launch.py run_bridge:=false` / `run_yolo:=false` | 검증됨 (개명 전 `yolo_seg` 기준) |
| `seg_source=yolo` 경로 (`segment_from_labels`) | 검증됨 (이전) |
| 기존 graspx 테스트 2종 회귀 (`test_capture_graspgenx_scene`, `test_grasp_bridge`) | PASS (이전) |
| **`/grasp/compute` 실기 호출** (`geometric`) | 성공 — obj_1 score=0.703, 후보 46개 |
| **`/grasp/compute` 실기 호출** (`yolo`) | 실패 — 후보는 나오나 충돌 필터 0/29·0/28 통과 |

`retina_masks=True` 가 없으면 `masks.data` 가 letterbox 된 모델 해상도로 나온다
(194×259 입력 → 480×640 마스크, ultralytics 8.4.113 실측). 그 상태로 라벨맵에 인덱싱하면
`IndexError` 다 — `build_label_map()` 이 먼저 잡아 `retina_masks` 를 지목하는 메시지를 낸다.

### 🔴→✅ 라이브 경로(`/grasp/compute`)가 판단 근거를 안 남기던 문제 — 2026-08-07 수정

**증상(발견 당시)**: `capture_graspgenx_scene` 단독 실행은 `out_dir`가 비어 있으면
`<repo>/data/graspgenx_scene/<scene>/`에 영구 저장하는데, `grasp_bridge_node.compute()`는
같은 `out_dir=''` 기본값에서 `tempfile.TemporaryDirectory()`를 쓰고 워커 호출 직후 `finally`
블록에서 **즉시 지웠다.** `/grasp/compute`를 실기로 불러도 GraspGenX가 뭘 보고 판단했는지
(rgb.png/seg.png/meta_data.json)를 나중에 열어볼 방법이 없었다 — 이 워크스페이스 어디에도
`data/graspgenx_scene/`가 존재하지 않았던 이유이기도 하다(`find` 전수조사 0건).

**수정**: `grasp_bridge_node.compute()`에서 임시 디렉토리 분기를 없애고 항상
`capture_graspgenx_scene.default_out_dir()`(비었으면 이 값, 지정하면 그 경로) 아래에
영구 저장한다. `scene` 파라미터를 안 바꾸면(기본값 `00`) 호출마다 타임스탬프
(`YYYYmmdd_HHMMSS`) 하위 디렉토리를 새로 만들어 이전 호출 기록을 덮어쓰지 않는다.
고정된 씬 이름이 필요하면(재현 테스트 등) `scene` 파라미터를 명시하면 그 이름을 그대로 쓴다.

**수정 중에 딸려나온 두 번째 버그(환경 감지)**: `default_out_dir()`가 `os.path.abspath(__file__)`
로 "패키지 루트"를 계산했는데, 이 워크스페이스의 기본 빌드 방식(`--symlink-install`)에서는
`install/setup.bash`가 PYTHONPATH에 `build/graspgenx_perception/graspgenx_perception/`를
먼저 얹고, 그 안의 각 파일은 `src/`를 가리키는 **심볼릭 링크**다. `abspath`는 링크를 풀지
않으므로 계산된 경로가 `build/graspgenx_perception/data/graspgenx_scene`가 됐다 — `colcon
build`/`rm -rf build`로 지워지는 산출물 디렉토리다. `python3 -c` 로 직접 import 해
`__file__`이 `build/...`로 잡히는 것과, `realpath`로 풀면 `src/graspgenx_perception/...`가
되는 것을 확인하고 `abspath` → `realpath`로 고쳤다(같은 파일, `default_out_dir()`).

```bash
# 재확인 (2026-08-07, colcon build PASS 후):
python3 -c "from graspgenx_perception.capture_graspgenx_scene import default_out_dir; print(default_out_dir())"
# -> /home/kimkh/cobot2_ws/src/graspgenx_perception/data/graspgenx_scene  (수정 전엔 build/ 밑)
```

**2026-08-07 21:12 실캡처로 재확인**: `ros2 run graspgenx_perception capture_graspgenx_scene
-p scene:=zzprobe` → `src/graspgenx_perception/data/graspgenx_scene/zzprobe/` 에 파일 4개
(depth.npy 3.6MB / rgb.png 1.1MB / seg.png / meta_data.json)가 **남았다**. 프로세스 종료
후에도 그대로다 — `main()` 의 `finally` 는 `destroy_node()`/`shutdown()` 만 하고 파일을 건드리지
않는다. `grasp_bridge_node.compute()` 도 같은 `write_scene()`/`default_out_dir()` 를 쓰고,
삭제는 **쓰기 실패 시 `shutil.rmtree`** 한 곳뿐이다.

⚠️ 그 `rmtree` 에 남은 구멍: `scene` 을 고정 이름으로 주면(예: `scene:=73`) 같은 디렉토리에
다시 쓰는데, 이때 **중간에 실패하면 이전에 성공했던 씬까지 통째로 지운다.** 기본값(타임스탬프)
경로는 디렉토리가 매번 새것이라 해당 없다. 고정 이름을 쓸 거면 이걸 알고 쓴다.

🔴→✅ **`.gitignore` 가 새 저장 경로를 못 막고 있었다** (2026-08-07 21:32 발견·수정).
규칙이 `data/graspgenx_scene/` 였는데, 슬래시가 중간에 있는 패턴은 `.gitignore` 위치(repo
루트)에 **고정**된다 — `realpath` 수정으로 저장 위치가 `src/graspgenx_perception/data/...`
로 옮겨간 순간 규칙 밖으로 나갔고, 실제로 `git status` 에 씬 8개 파일(8.9MB)이 커밋 대기로
떴다. `**/data/graspgenx_scene/` 로 고쳐 세 경로(루트·`src/`·`build/`) 전부 막히는 것을
`git check-ignore -v` 로 확인했다.
옛 씬 7개(`00,001,0012,01,10,11,73`)는 **워크스페이스 루트** `data/graspgenx_scene/` 에 있다 —
소스가 `scripts/` 에 있던 시절의 경로다. 지금 코드가 쓰는 곳은
`src/graspgenx_perception/data/graspgenx_scene/` 이므로 **옛 씬을 찾을 때 두 군데를 본다.**
`build/graspgenx_perception/data/graspgenx_scene/00`(08-07 10:27)은 `realpath` 수정 전에
떨어진 것이라 다음 `rm -rf build` 때 사라진다.
회귀 확인: `pytest test_yolo_seg.py`(10개) + `manual_grasp_bridge.py` + `pick_fsm`
`pytest`(26개) 전부 PASS(2026-08-07).

**cross-review 로 추가 발견/수정된 것 (2026-08-07)**: (1) 씬 디렉토리명을 초 단위
타임스탬프로 잡아 빠른 재시도가 충돌·덮어쓰기 할 수 있었다 — `%f`(마이크로초)를 붙여 수정.
(2) `write_scene()` 이 파일 4개 중 일부만 쓰다 실패하면 반쪽짜리 씬이 영구히 남는다 —
실패 시 `shutil.rmtree` 로 통째로 지우도록 수정. 정리 정책(오래된 씬 자동 삭제)과
`scene='00'`(기본값과 같은 문자열이라 "명시"해도 구분 불가) 은 낮은 우선순위로 보고
그대로 뒀다 — 필요해지면 추가.

### yolo 세그 — 최신 한 프레임 대신 최근 n 장 중 탐지 최선을 쓴다 (2026-08-07)

**요청 배경**: grasp 연산(GPU 워커, 수 초~수십 초)에 비하면 카메라 프레임 몇 장을 더 보는
시간은 무시할 만하다 — 탐지 정확도를 그 여유로 사도 되는지 확인하고 반영.

`SceneCapture` 가 `/yolo_seg/labels` 최신 한 장(`self.yolo_labels`)만 쓰던 것을, depth 처럼
최근 프레임을 버퍼(`self.yolo_labels_history`, 상한은 depth 와 같은 `MAX_DEPTH_BUFFER`)에
쌓아두고 `best_labels()`로 그중 물체 픽셀(라벨 > 100)이 가장 많은 프레임을 골라 쓰도록 바꿨다.
`capture_graspgenx_scene.run()`은 depth 를 모으는 `frames`(기본 10)장 동안 쌓인 라벨을,
`grasp_bridge_node.compute()`는 호출마다 지운 뒤 새로 쌓인 라벨 전부를 후보로 본다.

- ponytail: 라벨맵은 픽셀별 정수 클래스ID라 depth처럼 중앙값을 낼 수 없다(서로 다른 프레임을
  섞으면 의미 없는 값) — "픽셀 수 최대인 프레임 통째로 채택"이 가장 싼 대리 지표다.
- geometric 경로(기본값)는 원래 depth 만 쓰므로 이 변경과 무관하다.
- 회귀: `pytest test_best_labels.py`(3개, 신규) 통과 확인(2026-08-07).
- ⚠️ **미검증**: 실제 카메라로 "흔들린 프레임 하나 때문에 탐지가 비었다가 다른 프레임에서
  살아나는" 상황을 재현해서 개선을 실측하지는 않았다 — 논리상 개선이지 관측한 적은 없다.

## YOLO 를 주 파이프라인으로 쓸 때 남은 것

배선은 끝났다(`seg_source:=yolo` + `classes` + `target_classes`).

1. ~~**클래스 불일치.**~~ → **한계로 확정하고 목표를 좁혔다** (위 "2026-08-08" 절).
   `yolo11n-seg.pt` 는 COCO 80종이라 공구 5종(drill/hammer/pliers/screwdriver/wrench)이
   없다. 실기 캡처 `data/graspgenx_scene/10/rgb.png` 에서는 `['person','cell phone','sink']`,
   2026-08-07 라이브 카메라에서는 `['apple','cup','person']` 로 오검출했다. **공구는 대상에서
   뺀다.** 공구가 필요해지면 그때 seg 데이터셋 재학습이고, 그건 `classes`/`target_classes`
   배선을 하나도 안 바꾼다 — 가중치만 갈아끼우면 된다(`model_path` 파라미터).

2. ~~컨테이너 → 호스트 전송이 막혀 있다.~~ **2026-08-07 21:15 재측정에서 뚫렸다** — 라벨맵이
   호스트에 25.6 Hz 로 도달하고, 라벨맵(1280×720)과 정렬 depth(1280×720)의 해상도도 같아
   `segment_from_labels()` 의 shape 검사를 통과한다. 남은 것은 클래스 불일치뿐이다.
   (원인을 특정하지 못한 채 증상이 사라진 것이므로 재발 가능성은 남아 있다.)

> 옛 README 는 "`seg_source=yolo` 는 작업공간 박스를 안 본다"고 적었는데 **사실이 아니다.**
> `segment_from_labels()` 는 기하 경로와 **같은** `workspace_mask()` 박스와 반경 크롭
> (`obj_radius_m`)을 적용한다(`capture_graspgenx_scene.py:290-308`). 역할 분담은
> "YOLO 가 어느 물체인지, 기하가 닿을 수 있는 곳인지"다. 박스가 없으면 COCO 의 `dining table`
> 같은 라벨이 화면 대부분을 덮어 GraspGenX 가 죽는다 — 2026-08-06 에 67,879 px 라벨로 41.7GB
> 할당을 시도한 사고가 그래서 코드에 박스가 들어간 이유다.

## TensorRT `.engine` 으로 바꿔야 하나 — **아니다** (2026-08-08 실측 근거)

결론부터: **바꿔도 되지만 실익이 없다.** 파이프라인이 추론에 묶여 있지 않다.

| 구간 | 실측 | 근거 |
|---|---|---|
| YOLO 추론 (PyTorch, RTX 4060 Laptop) | **6.7 ms/frame (149 fps)** | 1280×720 실제 씬, 30회 평균, `torch.cuda.synchronize()` 포함 |
| 카메라 공급 | **~26 Hz (38.5 ms/frame)** | `/camera/camera/color/image_raw` 실측 |
| GraspGenX 워커 | **수십 초** | 이 파이프라인의 진짜 병목 |

**추론이 카메라보다 이미 5.7배 빠르다.** engine 으로 6.7ms → 3ms 가 돼도 파이프라인은
1 프레임도 더 처리하지 못한다. 카메라 바운드다.

**VRAM 도 문제가 아니다**: YOLO 상주는 `alloc 50 MiB / reserved 120 MiB` (8188 MiB 중 1.5%).
GPU 경합의 실체는 **GraspGenX 워커**와 **cuMotion/nvblox** 다 — GPU 가 하나뿐이라
(`md/context/constraints.md:314`) 팀원이 cuMotion 을 돌리면 그쪽에서 모자란다. YOLO 를
engine 으로 바꿔 아낄 수 있는 건 최대 120 MiB 다.

**바꿀 때 치르는 비용 (전부 확인함):**

1. **`tensorrt`·`onnx` 가 컨테이너에 없다** (2026-08-08 확인). 설치 휠이 1~2 GB다.
2. **`.engine` 은 이식이 안 된다.** GPU 아키텍처 + 드라이버 + TensorRT 버전에 묶인다.
   `.pt` 는 다른 PC 로 복사되지만 `.engine` 은 그 PC 에서 다시 빌드해야 한다 — 팀 공유
   랩탑에서 `git pull` 로 굴러가던 방식이 깨진다.
3. **변환 자체가 GPU 를 수 분간 크게 점유한다.** cuMotion/nvblox 가 도는 중이면 하면 안 된다.
4. **FP16 이 기본이라 마스크 경계가 미세하게 달라진다.** 지금 `conf` 를 0.1 로 **낮게** 쓰고
   있어 경계 검출이 뒤바뀔 수 있고, 그러면 `min_pixels=300`·`obj_radius_m=0.05` 같은
   실측 튜닝값의 근거가 함께 흔들린다.
5. **`retina_masks=True` 가 engine 에서도 되는지 별도 확인이 필요하다.** 이 코드는 그걸
   강하게 의존한다 — 안 되면 `build_label_map` 이 shape mismatch 로 예외를 던진다
   (그건 조용히 틀리는 것보다는 낫지만, 파이프라인은 선다).

**언제 재검토하나**: 카메라를 60 fps 이상으로 올리거나, 한 프레임에 여러 모델을 태우거나,
YOLO 를 GraspGenX 와 **동시에** 돌려 VRAM 이 실제로 모자랄 때. 지금은 셋 다 아니다.

## 다음 방향 — "어떤 물체를 집을지" 고르기 (설계, 미구현)

> 이 절은 **방향만** 적는다. 코드는 아직 없다. 결정을 하고 나서 짠다.

### 지금 어디까지 왔나

| 단계 | 질문 | 수단 | 상태 |
|---|---|---|---|
| 1 | 무엇이 **보이나** | `/yolo_seg/classes` | ✅ 구현됨 |
| 2 | 무슨 **종류**를 집나 | `target_classes=apple` | ✅ 구현됨 |
| 3 | **어느 개체**를 집나 | — | ❌ **여기가 다음** |
| 4 | 누가 **고르나** (LLM/GUI) | — | ❌ 3 이 먼저 |

사과가 2개 놓이면 3번에서 막힌다. `target_classes=apple` 은 "사과 종류"까지만 좁히고,
둘 중 점수 높은 쪽이 그냥 뽑힌다.

### 막고 있는 것은 구조다 — 고를 **틈**이 없다

`/grasp/compute` 는 **캡처 → 세그 → 워커 → 선택**을 한 호출에 끝낸다. 워커가 수십 초를
쓰고 나서야 결과가 나오는데, 그 사이에 사람이든 LLM이든 개입할 지점이 없다. `target` 과
`target_classes` 가 둘 다 "부르기 **전에** 정해두는 값"인 것도 그래서다.

### 제안: 서비스를 둘로 쪼갠다

```
/grasp/scene    (빠름, 수백 ms — 세그까지만)  →  후보 목록 발행
        ↓  사람 / GUI / LLM 이 고른다
/grasp/compute  (느림, 수십 초 — 워커)        →  고른 것 하나만 연산
```

`/grasp/scene` 이 내는 후보 하나 = `{scene_id, obj_N, class, conf, base XYZ(표면중심), px}`.
**`scene_id` 를 핸들에 붙이는 것이 핵심이다** — `obj_N` 은 프레임마다 바뀌므로 그것만으로는
지목이 성립하지 않는다. 씬은 이미 호출마다 타임스탬프 디렉토리로 **영구 저장**되므로
(`data/graspgenx_scene/<scene_id>/`), 지목이 오면 그 씬의 depth/seg 를 **다시 캡처하지 않고
그대로** 워커에 넘긴다. 그러면 "고르는 사이에 장면이 변했다"는 문제가 원리적으로 사라진다.
저장을 진단용으로 만들어 둔 것이 그대로 핸들 저장소가 된다.

### VLA 통합 — id 가 아니라 **좌표**로 잇는다

`M0609_VLA_system` 은 **고정 Webcam + homography** 로 장면을 보고, graspgenx 는
**eye-to-hand D435i**(작업대 옆 고정, 팔에 붙어 있지 않다)로 본다. 카메라가 **둘**이므로
**`apple_17` 같은 id 는 경계를 넘지 못한다.** LLM 이 지목한 것을 graspgenx 후보와 잇는 유일한
실용적 키는 **base 프레임 XY 근사 + 클래스 일치**다.

> 🔴 이 절은 2026-08-08 까지 D435i 를 "손목 RealSense" 로 적고 있었다 — **오기다.**
> 이 ws 의 D435i 는 처음부터 eye-to-hand 고정이고(`md/state.md` 캘리브 절, `T_cam2base.npy`),
> pick_fsm·graspgenx 가 쓰는 카메라는 **이 한 대뿐**이다.
>
> 카메라가 고정이라는 사실은 선택 로직 설계에 직접 영향을 준다: **화면 클릭의 픽셀 좌표가
> 팔이 움직여도 계속 유효하다.** eye-in-hand 였다면 클릭한 픽셀은 다음 모션 한 번으로
> 무효가 되어 즉시 base 좌표로 바꿔 보관해야 했다. 지금은 그 제약이 없다 — 픽셀을 그대로
> 써도 되고, 무효화되는 조건은 "누가 카메라를 건드렸다"(= 캘리브가 깨진 상황이라 어차피
> 파이프라인 전체가 못 쓴다) 뿐이다.

```
Webcam + LLM        →  "무엇을"  (의도·정체·대화 맥락)   → class + base XY (±수 cm)
고정 D435i          →  "어디를"  (정밀 6D 파지)          → 그 XY 근처의 obj_N
```

이 역할 분담은 VLA README 가 이미 스스로 내린 결론과 같다 — homography 좌표는 "올바른 물체로
팔을 보내기엔 충분하지만 손가락을 닫기엔 부족하다". 그 부족분을 메우는 것이 graspgenx다.
**매칭에는 homography 정확도로 충분하고, 파지에는 RealSense 좌표를 쓴다.**

### 결정해야 할 것 (이 순서로)

1. **선정 주체**: 사람(CLI/rqt) 먼저인가, 바로 LLM 인가. 사람 경로가 없으면 LLM 이 틀렸을 때
   무엇이 틀렸는지 분리할 수 없다 — 사람 경로를 먼저 만드는 쪽을 권한다.
2. **동점 처리 정책**: 지목이 없을 때(`target_classes` 만) 무엇을 고르나. 점수 최고(현재) /
   가장 가까운 / 가장 왼쪽. 지금은 점수 최고인데 이게 의도인지 우연인지 코드에 안 적혀 있다.
3. **씬 유효기간**: 고른 씬이 몇 초까지 유효한가. 무한이면 물체가 치워진 뒤에도 잡으러 간다.
   `pick_fsm` 에 `max_scene_age_s` 에 해당하는 값이 필요하다.
4. **person 제외를 어디서**: `classes` 에서 0 을 빼는 것(탐지 자체를 안 함)과 `target_classes`
   에 안 넣는 것(탐지는 하되 안 잡음)은 다르다. 후자는 사람이 씬에 **장애물로** 남는다.

## 수동 통합 확인

```bash
ros2 run graspgenx_perception yolo_seg_node --ros-args -p image_topic:=/yolo_seg_probe/image &
python3 src/graspgenx_perception/test/manual_roundtrip.py corecode/OD_Tutorial/YOLO_SIMPLE/sample2.jpg
```

프로브는 기본적으로 `/yolo_seg_probe/image` 로 쏜다. 실제 카메라 토픽에 주입하면
`realsense2_camera` 를 쓰는 다른 소비자에게도 합성 프레임이 간다.

grasp 결과가 Doosan `move_line` 커맨드로 어떻게 번역되는지만 눈으로 보려면 (**로봇을
움직이지 않는다 — 문자열만 출력한다**):

```bash
python3 src/graspgenx_perception/test/manual_grasp_to_movel.py
```
