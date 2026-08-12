<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    D435i rosbag 녹화·재생 명령 전부
-->

# D435i rosbag — 녹화 완료. 이제 쓰는 문서다 (2026-08-04 갱신)

**결론: 필요한 bag은 이미 다 찍혔다. 4개 전부 검증 통과했다.**
2026-08-03 21시대 재캘리브 후 녹화된 `rosbag/bag_0803calibed/` 4개는 요구 토픽 11개를
전부 담았고 `base_link→camera_link`도 내용으로 확인됐다(2026-08-04 실측, §2).

따라서 이 문서를 **다시 열 일은 셋뿐이다**:
- bag을 돌린다 → **§3 재생 절차**
- bag으로 뭘 개발할지 → **§4**
- 장면 2개(빈 테이블 / 장애물 여러 개)를 마저 찍는다 → **§6**

§7(토픽 선정 근거)은 참조용이다. 토픽 목록을 바꿀 일이 없으면 안 읽어도 된다.
2026-08-03 이전 판에 있던 "못 쓰는 bag 4.8GB"(`rosbag_modified`) 사건은 §8에 요약만 남겼다.
그 bag들은 삭제됐다.

---

## 1. 지금 있는 bag

`rosbag/bag_0803calibed/` — 총 1.9 GB (zstd, FILE 모드). `.gitignore:28`의 `rosbag/`로 제외됨.

| bag | 크기 | 길이 | 내용 (프레임 확인) | 로봇 팔 | 주 용도 |
|---|---|---|---|---|---|
| `d435i_0803_2140_obstacle1` | 181 MB | 24.9 s | 정적. 테이블에 노트북 파우치 1개 | 정지 | 장애물 1개, 중력벡터 검증 |
| `d435i_0803_2141_hand` | 547 MB | 72.5 s | 사람이 파우치를 들고 들어와 컨베이어에 놓음 | 정지 | 동적 장애물·Octomap decay |
| `d435i_0803_2143_robot_moving` | 848 MB | 115.3 s | **로봇이 실제로 움직인다** (link_1 104°, link_3 91°, link_5 90°) | **동작** | **self-filter 검증 — 유일** |
| `d435i_0803_2149_apple` | 356 MB | 53.0 s | 사과 + 비닐포장 물체를 손으로 배치 | 정지 | YOLO·파지점 |

**`2143_robot_moving`이 가장 비싼 자산이다.** `/tf`가 실제로 변하는 유일한 bag이라 Octomap
self-filter를 이것 없이는 검증할 수 없다. 나머지 셋은 팔이 고정이라 self-filter가 자명하게 통과한다.

장면 이름은 2026-08-04에 폴더명에 붙였다(녹화 시엔 없었다 — 그래서 이 표를 만드느라 프레임을
다 뽑아야 했다). 폴더 안의 `.db3.zstd` 파일명은 그대로이고 `metadata.yaml`의 경로가 폴더 상대라
rename 후에도 `ros2 bag info`가 정상 동작한다(실측). **이름을 또 바꾸면 이 표도 같이 고친다.**

## 2. 검증 결과 (2026-08-04 실측 — 4개 bag 전부)

`zstd -dc` → `sqlite3` → `rclpy.serialization.deserialize_message`로 직접 열어 확인했다.

- **요구 토픽 11개 전부 존재. 누락 0.** (`ros2 bag record`는 없는 토픽을 조용히 건너뛴다 — 그 사고는 없었다)
- **`/tf_static` 18개 변환, 4개 bag 동일.** `world→base_link→camera_link` 체인 완결.
  `tool0`, `rg2_base_link`까지 들어있다. **개수가 아니라 내용으로 확인했다.**
- 해상도 **848x480** (color / aligned depth / depth 전부). 지시대로.
- `aligned_depth_to_color/image_raw` **14.5~14.8 Hz** → 드랍 1~3%. 정상.
- IMU: gyro **198 Hz**, accel **100 Hz**. 4개 다 있음.
- 압축 zstd / FILE 모드.

### 이 bag들의 캘리브 값 (npy와 일치 확인)

```
base_link -> camera_link
  t = (1.063222, 1.166012, 0.586973)
  q = (-0.146496, -0.025093, 0.886536, -0.438136)
```
현재 `src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy`와 **완전히 일치**한다
(npy는 mm 단위: 1063.222 / 1166.012 / 586.973).

⚠️ **이전 판 문서에 "검증됨"으로 적혀 있던 `(1.148174, 0.640096, 0.677658)`은 폐기된 값이다.**
y가 0.53 m 다르다. 08-03 21시대 재캘리브로 npy가 바뀌었고, 그때 만들어진 게 이 4개 bag이다.
**옛 bag(`d435i_0803_1640` 등)과 새 bag은 서로 다른 캘리브를 담고 있으니 섞어 쓰지 않는다.**
옛 bag은 삭제됐으므로 지금은 문제가 없지만, 백업에서 되살릴 일이 있으면 이 항목을 먼저 본다.

### 미해결: `depth/image_rect_raw`가 24~28 Hz

프로파일은 15 fps인데 4개 bag 전부 depth raw가 24~28 Hz로 나온다(`depth/camera_info`는 29 Hz).
08-03의 `1640` bag에서 43 Hz가 나왔던 것과 같은 현상이고 **원인은 여전히 규명 안 됐다.**
컬러·정렬 depth는 15로 맞으므로 **프레임 드랍 판정은 `aligned_depth_to_color`로 한다.**

## 3. 재생 절차

**새 bag(`bag_0803calibed/`)은 TF 보충이 필요 없다.** `base_link→camera_link`가 bag 안에 있고
그 값이 현재 npy와 같다. `camera.launch.py driver:=false`를 띄울 이유가 없다.

```bash
# 터미널 1 — 소비 노드(Octomap, 인지 등)를 먼저. use_sim_time:=true 필수
#   ros2 launch ... use_sim_time:=true

# 터미널 2 — 재생
source /opt/ros/humble/setup.bash && \
  ros2 bag play rosbag/bag_0803calibed/d435i_0803_2143_robot_moving --clock -l
```

주의:
- `--clock` ↔ 소비 노드 `use_sim_time:=true`는 **한 짝이다.** 안 맞추면 TF가
  "extrapolation into the future"로 계속 터진다.
- **실기 카메라 드라이버가 떠 있으면 먼저 끈다.** 같은 토픽에 두 소스가 조용히 섞인다.
- `-l`(loop) 없이는 재생이 끝나면 latched TF도 사라진다. 파라미터를 만지며 반복할 땐 `-l`.
- compressed 컬러를 raw로 받는 노드에는 `image_transport republish`가 필요하다.

### 디스크 — 재생하면 `.db3`가 폴더에 남는다

`ros2 bag play`는 `.db3`를 bag 폴더 안에 풀고 **지우지 않는다.**
실측(2026-08-04): `2140_obstacle1`은 189 MB → **870 MB (4.6배)**. 4개 다 풀면 ~9 GB인데 `/` 여유가 16 GB다.
**하나씩 풀고 쓰고 지운다.**

### 포인트클라우드는 재생 시 만든다

녹화에서 뺐다(§7). 필요하면:
```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz \
  -p use_sim_time:=true          # ⚠️ 미검증 (bag 입력으로 실행해본 적 없음)
```

### 스크립트로 파싱할 때

`rosbag2_py.SequentialReader`는 **압축 bag을 못 연다** (`sqlite3` 플러그인이 `.db3.zstd`를
"not a database"로 거절 — 2026-08-03 실측). `ros2 bag play`/`info`만 알아서 푼다.
스크립트가 필요하면 이렇게 한다 (2026-08-04에 §2 검증을 이 방식으로 했다):

```bash
zstd -dq -o /tmp/w.db3 <bag>/<bag>_0.db3.zstd
```
```python
import sqlite3
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
c = sqlite3.connect("/tmp/w.db3")
tid = {n: i for i, n in c.execute("select id,name from topics")}
for (d,) in c.execute("select data from messages where topic_id=?", (tid["/tf_static"],)):
    for t in deserialize_message(bytes(d), TFMessage).transforms:
        print(t.header.frame_id, "->", t.child_frame_id)
```
**부분 압축 해제(`head -c`)는 안 된다** — sqlite가 "database disk image is malformed"로 거절한다.

## 4. 이 bag으로 할 개발 (순서대로)

녹화의 목적은 데이터 수집이 아니라 **실기 없이 반복 가능한 입력을 확보하는 것**이다.
넷 다 실기 점유 없이, 사람 승인 없이 랩탑에서 돈다.

**1) Octomap self-filter 검증 — `2143_robot_moving`** ([[ws/cobot2/plans/2026-08-03-octomap-integration]])
`/tf`가 실제로 변하는 유일한 bag. 재생하면서 로봇 팔 자체가 장애물로 잡히는지 본다.
나머지 3개로는 이 검증이 성립하지 않는다.

**2) Octomap 파라미터 튜닝 — `2140_obstacle1` / `2141_hand`**
`resolution`, `max_range`, `point_subsample`을 바꿔가며 같은 장면을 반복 입력한다.
실기에선 매번 물체를 똑같이 놓을 수 없어 비교가 성립하지 않는다. bag이면 성립한다.
`2141_hand`(손 진입)은 동적 장애물 잔상(decay) 튜닝용.

**3) hand-eye 검증 — `2140_obstacle1`**
정지 장면이라 accel이 중력 벡터를 준다. `T_cam2base.npy`로 `base_link`에 옮겨 `-Z`가 나오는지
본다. 안 나오면 캘리브 회전이 틀린 것이다. **체커보드와 완전히 독립한 측정**이라
"캘리브가 틀렸나 코드가 틀렸나"를 가른다. 한계: 중력축 둘레 **yaw는 관측 불가**(3 DOF 중 2개만).
⚠️ 원리는 확실하나 이 ws에서 실행해본 적 없다. D435i accel 바이어스가 실용 정밀도를 내는지 미검증.

**4) YOLO 데이터 — `2149_apple`(사과·비닐물체) + `2141_hand`(파우치)**
정렬 depth가 같이 있어 박스 중심의 3D 좌표를 정답으로 붙일 수 있다 — 검출 정확도와 파지점
정확도를 따로 잰다. ⚠️ **정적 장면이라 15 Hz 전 프레임은 거의 중복이다.** 1 Hz로 서브샘플링하면
`2149_apple` ~53장, `2141_hand` ~72장.

**5) pick&place 회귀 테스트 — `2149_apple`**
인지 파이프라인(YOLO → 3D 좌표 → 목표 pose)만 bag 입력으로 돌려 결과 좌표를 고정 기대값과
비교. 모션은 빼므로 로봇도 승인도 필요 없다.

## 5. 다시 찍어야 하는 것은 장면 2개뿐

| 장면 | 왜 필요한가 | 상태 |
|---|---|---|
| 빈 테이블 | Octomap 바닥 제거·노이즈 기준선 | ❌ 없음 (`2140_obstacle1`도 파우치가 있다) |
| 장애물 여러 개 | Octomap 해상도·플래너 회피 | ❌ 없음 |
| 장애물 1개 | 검출 최소 케이스 | ✅ `2140_obstacle1` |
| 사람 손 진입 | 동적 장애물·decay | ✅ `2141_hand` |
| 로봇 동작 중 | self-filter | ✅ `2143_robot_moving` |

60초씩 두 개만 더 찍으면 §4의 용도가 전부 커버된다. 절차는 §6.

## 6. 재녹화 절차 (장면 2개용)

### 6-0. 카메라를 옮겼는가?

- **안 옮겼다** → 현재 npy가 유효하다. 그대로 진행. bag의 `/tf_static`에 박히므로 보충 불필요.
- **옮겼다** → npy가 거짓이 된다. 그대로 녹화하면 **bag에 가짜 캘리브가 박혀** 캘리브가 아예
  없는 것보다 나쁘다. **재캘리브 먼저**, npy 교체 후 녹화. 그리고 **§2의 캘리브 값을 갱신한다** —
  안 그러면 새 bag과 이 4개가 서로 다른 캘리브를 담게 되고 문서가 그걸 모른다.

카메라 고정 상태를 마스킹테이프로 표시하고 사진을 찍어둔다.
`realsense-viewer`가 떠 있으면 **닫는다** — USB를 독점해 ROS 노드를 죽이는데 증상이
"TF 프레임 없음"으로 나와 오진을 부른다.

**세 터미널의 `ROS_DOMAIN_ID`가 같아야 한다.** `.bashrc`의 `rdm` alias는 93인데 런치들은
도메인을 안 줘서 0에서 뜬다(`md/context/constraints.md`). 하나만 `rdm`을 치면
**`ros2 bag record`가 토픽을 하나도 못 보고 빈 bag을 만든다 — 에러 없이.**
녹화 터미널에서 `echo $ROS_DOMAIN_ID`로 확인한다.

### 6-1. 터미널 1 — 로봇 bringup (`/tf`용, 로봇을 안 움직여도 띄운다)

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```
⚠️ 미검증 (출처는 `camera.launch.py` docstring)

없으면 로봇 링크 TF가 안 들어가 **Octomap self-filter를 못 한다**(팔이 장애물로 잡힌다).

### 6-2. 터미널 2 — 카메라 (`reals` 말고 인자를 준다)

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 launch m0609_rg2_bringup camera.launch.py \
    depth_profile:=848x480x15 color_profile:=848x480x15
```

- **공식 `rs_align_depth_launch.py`를 쓰지 않는다.** 그 런치엔 `camera_calib_tf`가 없어
  `base_link→camera_link`가 bag에 안 들어간다 — §8에서 4.8 GB를 버린 원인이다.
- **`reals`(인자 없음)도 쓰지 않는다.** 기본 424x240x15는 Octomap 실시간 운용용 저해상도다.
- **848x480x15인 이유**: 해상도는 YOLO·depth 품질에 직결되지만 **fps는 정적 장면에서 가치가 없다.**
  30 fps 대비 대역폭 절반이라 드랍 위험이 준다. color를 depth와 맞추는 건 `align_depth`가
  depth를 **컬러 해상도로 리샘플**하기 때문(color가 1280x720이면 aligned depth만 55 MB/s).

`pointcloud.enable`이 `True`로 하드코딩돼 있지만 **끄지 않는다** — 포인트클라우드는 호스트에서
depth로부터 계산하는 파생물이라 USB 대역폭을 안 쓰고, CPU는 이미 병목이 아니라고 측정됐다
(`md/context/constraints.md`, 2026-08-01: load 0.5~0.6, realsense 노드 18.8%).
측정으로 배제된 원인을 잡겠다고 런치 인자를 늘리지 않는다. **§6-4의 15 Hz 확인으로 판단한다.**

### 6-3. 터미널 3 — 녹화

```bash
ros2 bag record --compression-mode file --compression-format zstd \
  -o d435i_$(date +%m%d_%H%M)_empty \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  /camera/camera/extrinsics/depth_to_color \
  /camera/camera/gyro/sample \
  /camera/camera/accel/sample \
  /tf /tf_static
```

**장면 이름(`_empty`, `_obstacle3`)을 `-o`에 직접 넣는다.** 기존 4개가 이름이 없어서
§1 표를 만드느라 프레임을 다 뽑아봐야 했다. 장면당 60초.

`--compression-mode file`인 이유: `message` 모드는 메시지마다 압축해 녹화 중 CPU를 계속 먹는다.
이 랩탑은 i7-10510U 15 W에 `ros2_control_node`가 상시 200%대라 **녹화 중 CPU를 쓰면 드랍된다.**

**녹화 중 하면 안 되는 것**: 카메라→base 임시 static TF를 띄우는 것. `/tf_static`에 가짜
캘리브가 박히면 나중에 진짜 값과 충돌하고, bag만 봐서는 어느 쪽이 진짜인지 알 수 없다.

### 6-4. 녹화 직후 검증 (실기 떠나기 전에)

```bash
ros2 bag info <bag>
```
1. **토픽 11개**가 다 잡혔는가
2. `aligned_depth_to_color/image_raw` Count ÷ Duration ≈ **15**인가
   (**`depth/image_rect_raw`로 재지 말 것** — §2의 미해결 항목)
3. `/tf` count > 0 (bringup이 떠 있었는지)
4. **`base_link→camera_link`가 실제로 들어갔는가 — 개수로 판정하지 않는다.**
   §3의 파이썬 조각으로 내용을 본다. 안 나오면 **그 자리에서 다시 찍는다.**

## 7. 토픽 선정 근거 (참조용 — 목록을 바꿀 때만 읽는다)

| 토픽 | 없으면 못 하는 것 |
|---|---|
| `depth/image_rect_raw` | depth 원본. 포인트클라우드 재생성, depth 필터 오프라인 튜닝 |
| `depth/camera_info` | 위 depth를 3D로 못 푼다 (fx·fy·cx·cy). 절대 따로 빼지 않는다 |
| `aligned_depth_to_color/image_raw` | **픽셀(u,v) → 3D 매핑.** YOLO 박스 중심 깊이 — pick&place의 핵심 |
| `aligned_depth_to_color/camera_info` | 위의 intrinsic. **color 프로파일을 따른다** |
| `color/image_raw/compressed` | 시각 확인, YOLO 입출력, 라벨링. raw는 30 fps 848x480만 37 MB/s라 뺐다 |
| `color/camera_info` | compressed 컬러를 3D와 못 엮는다 |
| `extrinsics/depth_to_color` | 정렬을 직접 재계산할 때만. 1회 latched라 용량 0 → 그냥 담는다 |
| `gyro/sample` | 마운트 충격 포렌식 (§7-1) |
| `accel/sample` | 정지 시 중력 벡터 → hand-eye 독립 검증 (§4-3) |
| `/tf_static` | `camera_link` ↔ optical frame + 로봇 트리 연결. **없으면 로봇 좌표계에 안 붙는다** |
| `/tf` | 로봇 관절이 움직이는 장면. 용량이 작아 항상 담는다 |

**의도적으로 뺀 것**
- `depth/color/points` — depth image + camera_info로 재생성되는 파생물인데 ~390 MB/s. §3에서 만든다.
  > ⚠️ **이 390 MB/s 는 고해상도 프로파일 기준이다.** 2026-08-09 실측(`424x240x15`)에서는
  > **20.0 MB/s** 였다 — 40배 넘게 벌어지므로 **프로파일을 안 밝히고 이 숫자를 인용하지 말 것.**
  > 클라우드는 비조밀(유효 depth 만 실림)이라 씬에 따라서도 흔들린다.
  > 실측표는 [[ws/cobot2/context/constraints]] "카메라 토픽 실측 대역폭"이 단일 출처다.
  > (녹화에서 빼는 결정 자체는 어느 프로파일에서도 그대로 유효하다.)
- `color/image_raw`(raw) — compressed와 중복.
- `*/theora`, `*/compressedDepth` — 재생 시 디코드 실패가 잦다.
- `*/metadata` — 재생 파이프라인이 안 쓴다.
- `infra1`/`infra2` — depth 알고리즘 자체를 뜯을 게 아니면 불필요.

### 7-1. IMU를 담는 이유 (eye-to-hand인데도)

VIO/SLAM은 여기서 무의미하다 — 카메라가 안 움직인다. 그래도 담는 이유는 둘.
1. **중력 벡터로 hand-eye 독립 검증** (§4-3).
2. **마운트 충격 포렌식.** eye-to-hand의 가장 흔한 조용한 실패는 "누가 카메라를 건드려 캘리브가
   무효화됐는데 아무도 모름"이다. 200 Hz gyro가 있으면 충격 시점을 짚어 이후 프레임을 버릴 수 있다.

**비용/재수집 비대칭이 결정적이다.** 200 Hz IMU는 수십 KB/s로 영상 50 MB/s 옆에서 반올림
오차인데, 안 담으면 실기를 다시 잡아야 한다. 평소라면 YAGNI로 자를 항목이다.

**IMU가 켜져 있는 이유**: `rs_launch.py:63-64`는 `enable_gyro`/`enable_accel`을 명시적으로 `false`로
**끈다.** 이 ws의 `camera.launch.py`는 그 인자를 안 주므로 **노드 자체 기본값(켬)**이 살아난다.
`unite_imu_method` 기본이 `0`이라 gyro/accel이 따로 나온다. **공식 런치의 기본값을 드라이버의
기본값으로 착각하지 말 것.**

### 7-2. 네임스페이스 `/camera/camera/...`는 런치와 무관하다

`realsense2_camera_node` **자체의 기본값이 name=`camera`, namespace=`camera`**다. 공식 `rs_launch.py`가
같은 값을 명시할 뿐 이름을 만들어내는 게 아니다. **어느 런치로 띄워도 토픽 경로는 같다**(2026-08-03 실측).

## 8. 폐기된 bag — 왜 4.8 GB를 버렸나 (요약)

2026-08-03에 `rosbag_modified/` 3개(4.8 GB)를 공식 `rs_align_depth_launch.py`로 찍었다.
그 런치엔 `camera_calib_tf`가 없어 `base_link→camera_link`가 안 들어갔고
(+`enable_gyro/accel=false`라 IMU도 빠졌다), 포인트클라우드가 로봇 트리에 못 붙는
**고아 프레임**이 됐다. 159 MB짜리 `reals` bag이 4.8 GB짜리보다 쓸모가 많았다 —
**용량이 데이터 가치가 아니다.** 원인은 사본이 갈라진 게 아니라 **세 문서가 입을 모아 틀린 런치를
지시**하고 있었던 것이다. 그래서 녹화·재생 명령의 출처를 이 문서 하나로 모았다.

**교훈 두 개가 §6-4에 규칙으로 살아있다**: (1) 토픽 11개 확인, (2) `base_link→camera_link`는
개수가 아니라 내용으로 확인. 폐기된 bag들은 삭제됐다.
