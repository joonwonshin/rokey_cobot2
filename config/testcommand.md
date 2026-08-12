<!-- meta
updated: 2026-08-08
status:  live
owns:    실행 명령어(호스트/컨테이너 구분) · 노드 지도 · 단계별 검증 명령
         2026-08-08: md/context/test_grap_plan.md(경로 B)를 여기로 합쳤다. 그 파일은
         비교용으로 남아 있으나 갱신하지 않는다 — 값의 정본은 이 문서다
-->

# 실행 명령서 — 로봇 + MoveIt 위의 두 경로

**T1~T3(로봇·카메라·MoveIt)를 공유하고 거기서 갈라진다.**

| | 무엇 | 어디서 | 목적 |
|---|---|---|---|
| **경로 A** | cuMotion + nvblox | 컨테이너(GPU) | 동적 장애물 회피. 로봇은 계획만 |
| **경로 B** | GraspGenX + pick_fsm | 호스트(GPU) | 실제로 집는다 |

> 왜 이렇게 하는지·함정의 근거는 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]]과
> [[ws/cobot2/context/constraints]]가 단일 출처다. 여기엔 **치는 것**만 둔다.
> 경로 A는 2026-08-06 실기 전 구간 관통 확인(OMPL 10/10, cuMotion 10/10).

---

## ⚡ 명령어만 — 복붙용

**공통 T1~T3** (모든 터미널에서 `rdm` = `export ROS_DOMAIN_ID=93` 먼저)

🔴 **호스트↔컨테이너(T3~T7) 터미널을 쓰는 날은 아래도 모든 호스트 터미널에 같이 건다.**
`graspx_container.sh`(od_kimkh 컨테이너)는 `docker exec -e`로 이미 2026-08-06부터 이걸 박아서
쓰고 있었다 — `isaac_ros_dev-x86_64-container`(T3~T7) 쪽만 반영이 안 돼 있었다(2026-08-11 확인).
안 걸면 `ros2 topic list`엔 컨테이너 쪽 토픽이 다 보이는데 **데이터가 0건**으로 온다(§4 T4 각주 참고).
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
```

```bash
# T1 카메라 — 이미 떠 있으면 띄우지 않는다 (ros2 node list | grep camera)
#   🔴 alias(`reals`)를 쓰지 말고 아래처럼 인자를 직접 친다. **alias 정의가 머신마다 다르다**
#      (개인PC `.bashrc:174` 는 인자 없음 → 424x240x15. GPU PC 는 다를 수 있다)
#      내 머신 것 확인: alias reals
ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=424x240x15 color_profile:=424x240x15
#   ✅ 2026-08-09 실기 확정: 이 값은 런치 기본값과 같다 — 사실 인자를 안 줘도 된다.
#      ~~480x320x15~~ 는 **D435i 가 지원하지 않는 프로파일**이라 폐기했다(rs-enumerate-devices 실측).
#      Color: 320x180 320x240 424x240 640x360 640x480 848x480 960x540 1280x720 1920x1080
#      Depth: 256x144 424x240 480x270 640x360 640x480 848x100 848x480 1280x720
#      ⚠️ depth 의 `480x270` 과 헷갈리지 말 것 — `480x320` 은 어느 쪽에도 없다.
#   ⚠️ 캘리브 데이터 수집 때만 예외로 1280x720 (424x240 으로 찍으면 코너가 안 잡혀 불합격난다)

# T2 로봇 (실기) — rviz:=false 필수, moveit이 자기 RViz를 띄운다
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false

# T3 MoveIt — standalone:=false 필수. cumotion:=true는 경로 A(컨테이너)에서만
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
```

**경로 A — cuMotion + nvblox** (T4~T7, 전부 컨테이너. 셸마다 §3의 source 4줄 먼저)

```bash
# T4 robot_segmenter — 빼면 로봇 자기 몸이 장애물이 된다
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p distance_threshold:=0.15 \
  -p depth_image_topics:="[/camera/camera/aligned_depth_to_color/image_raw]" \
  -p depth_camera_infos:="[/camera/camera/aligned_depth_to_color/camera_info]" \
  -p robot_mask_publish_topics:="[/cumotion/camera_1/robot_mask]" \
  -p world_depth_publish_topics:="[/cumotion/camera_1/world_depth]"

# T5 nvblox — esdf_mode:=3d 없으면 cuMotion 첫 요청에 FATAL
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link -p use_lidar:=false -p num_cameras:=1 -p esdf_mode:=3d \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info

# T6 cuMotion 플래너 — read_esdf_world:=False면 장애물을 못 보는데 계획은 성공한다
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True \
  -p publish_curobo_world_as_voxels:=True

# 검증 — 로봇 안 움직인다
python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 10
```

**경로 B — GraspGenX + pick_fsm** (T4~T6, 호스트)

```bash
# T4 grasp 브리지 — GPU 워커를 자식으로 띄운다. 첫 실행은 모델 로드로 수십 초
ros2 run graspgenx_perception grasp_bridge_node --ros-args \
  -p out_dir:=$(pwd)/data/graspgenx_scene -p scene:=01
#   out_dir 비우면 임시 디렉토리에 썼다가 지운다. scene 같으면 덮어쓴다

# T5 인식만 단독 확인 — 로봇 안 움직인다 ⭐
ros2 service call /grasp/compute std_srvs/srv/Trigger {}
ros2 topic echo /grasp/best_tcp --once    # 손끝 좌표를 자로 잰 물체 위치와 대조

# T6 FSM — 여기서부터 실제로 움직인다
ros2 launch pick_fsm pick_fsm.launch.py \
  grasp_source:=legacy_trigger voice:=false target:=apple dry_run:=false

# 조작 (다른 터미널)
ros2 topic echo /pick/state &
ros2 service call /pick/start   std_srvs/srv/Trigger {}   # → WAIT_APPROVAL 에서 멈춘다
ros2 service call /pick/approve std_srvs/srv/Trigger {}   # ✋ 여기서 로봇이 움직인다
ros2 service call /safety/stop  std_srvs/srv/Trigger {}   # 즉시 정지
```

**T0 사전 점검** (경로 B 원문에 있던 것 — 경로 A에도 유효하다)

```bash
nvidia-smi                                 # GPU PC인지 판별. 없으면 개인PC다
ping -c1 192.168.1.100                     # 로봇
ping -c1 192.168.1.1                       # RG2 Modbus
rdm && ros2 node list                      # 남의 계정 move_group이 있는지
```

---

## 🔴 합치면서 드러난 파라미터 불일치 — 사람이 정해야 한다

두 문서가 **같은 명령을 다른 파라미터로** 적고 있었다. 아래는 launch 파일 소스로 확인한
사실이고, **어느 쪽이 맞는지는 실기에서 정한다**(이 판정은 CPU PC에서 못 한다).
원문 비교가 필요하면 `md/context/test_grap_plan.md`가 그대로 남아 있다.

| 명령 | 경로 A (이 문서 원본) | 경로 B (`test_grap_plan`) | 실제 차이 |
|---|---|---|---|
| `bringup.launch.py` | `rviz:=false` | `model:=m0609` (rviz 미지정) | 🔴 **B가 위험하다.** `rviz` 기본값이 `true`라 B는 bringup RViz를 띄우고 moveit RViz와 2개가 된다 — launch 파일 26~28행 주석이 "moveit과 함께 쓸 땐 false"라고 직접 적어놨다. `model`은 **선언된 인자가 아니라 조용히 무시된다**(실측 확인: 미선언 인자는 에러 없이 무시). bringup이 `model='m0609'`를 하드코딩(46행)하므로 결과는 같지만 **아무 일도 안 하는 인자**다 |
| `camera.launch.py` | `848x480x15` (alias `reals` 경유로 표기) | 인자 없음 | ✅ **해결(2026-08-09 실기).** ~~2026-08-08 결정: `480x320x15` 로 통일~~ 은 **그 값이 D435i 미지원이라 폐기됐다** → **`424x240x15`(= 런치 기본값)로 통일**(위 T1). 🔴 **alias 를 경유하지 말고 인자를 직접 쓴다** — `reals` 의 정의가 **머신마다 다르다**(개인PC `.bashrc:174` 에서는 인자가 없어 기본 `424x240x15` 로 뜬다. GPU PC 의 `.bashrc` 는 이 문서 표기대로 848 일 수 있으나 **확인 못 했다**). 그래서 "같은 명령을 쳤는데 해상도가 다르다"가 성립한다. 참고로 GraspGenX 실측 기록(README:34 → :258)은 848×480 → **1280×720** 로 바뀌는데, 그건 `alias realsense`(다른 런치, color `1280x720x30`)로 띄웠고 `aligned_depth_to_color` 가 **color 를 따라가기** 때문이다(`constraints.md:25`) |
| `moveit.launch.py` | `octomap:=true cumotion:=true` | `standalone:=false` 만 | `octomap` 기본값은 `true`라 같다. **`cumotion`은 기본값 `false`** (51행) → 경로 B에는 cuMotion 파이프라인이 **안 올라온다.** `pick_fsm ... planning_pipeline:=isaac_ros_cumotion`을 쓰려면 T3를 `cumotion:=true`로 띄워야 한다. 단 그 인자는 **Isaac ROS 컨테이너에서만** 켤 수 있다(48행 주석) |

**정하고 나면 이 표를 지우고 위 "명령어만" 블록에 반영한다.**

---

## 0. 파이프라인 한 장 (경로 A)

```
호스트                                              컨테이너 (Isaac ROS 3.2)
─────────────────────────────────────────────────────────────────────────────────
T1  camera.launch.py
      └ /camera/camera/aligned_depth_to_color/image_raw ──┐
      └ camera_calib_tf (base_link→camera_link)           │
                                                          ▼
T2  bringup.launch.py (실기 로봇)              T4  robot_segmenter_node
      └ /dsr01/* 컨트롤러                            (로봇 몸을 depth에서 지움)
      └ /joint_states (12관절 + velocity)               │ /cumotion/camera_1/world_depth
                                    │                    ▼
                                    │            T5  nvblox_node (esdf_mode:=3d)
                                    │                    │ get_esdf_and_gradient (서비스)
                                    │                    ▼
                                    └──────────▶ T6  cumotion_planner_node
                                                         │ /cumotion/move_group (액션)
                                                         ▼
                                                 T7  move_group (cumotion:=true)
                                                         └ RViz 드롭다운 OMPL ↔ cuMotion
```

**장애물이 두 경로로 들어간다. 헷갈리지 말 것:**

| 플래너 | 장애물 출처 | self-filter |
|---|---|---|
| **OMPL** | MoveIt octomap (`/camera/camera/depth/color/points`) | `sensors_3d.yaml`의 padding |
| **cuMotion** | nvblox ESDF (서비스로 pull) | **`robot_segmenter_node`** |

🔴 **cuMotion은 octomap을 아예 안 본다.** 그래서 `robot_segmenter` + nvblox가 빠지면
"계획은 성공하는데 장애물을 통과"한다. 실패가 아니라 **성공처럼 보이는 실패**다.

---

## 1. 호스트 — T1: 카메라

```bash
rdm                                    # ROS_DOMAIN_ID=93
ros2 node list | grep camera           # ⚠️ 먼저 확인. 이미 있으면 띄우지 않는다
reals
# = ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=848x480x15 color_profile:=848x480x15
```

- ⚠️ `realsense-viewer`가 떠 있으면 먼저 닫는다 (USB 독점 → 노드가 죽는데 증상은 "TF 없음"으로 나온다)
- ⚠️ **드라이버를 두 번 띄우면 depth가 반토막 난다.** `ros2 node list`에 `/camera/camera`가
  2개면 그것이다 (2026-08-06 실측: 15 → 5.6 Hz)

**검증**

```bash
ros2 node list | grep -c "camera/camera"                         # 1
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw    # 실측 9.65 Hz
ros2 run tf2_ros tf2_echo base_link camera_link                  # [1.237, -0.223, 0.784]
```

## 1.5 호스트 — graspgenx 고해상도 vs T4/T5 저해상도 분리 (2026-08-11 도입, 미검증)

**전제가 다를 때만 필요하다**: graspgenx가 1280x720 depth를 요구하고, 동시에 T4
(robot_segmenter)/nvblox 반응속도를 지금(424x240 기준 세그멘터 3.7 Hz, `config/testcommand.md:257`
구 번호)보다 떨어뜨리고 싶지 않을 때. RealSense 드라이버는 depth를 한 해상도로만 내므로
카메라를 고해상도로 한 번 열고, T4 직전에서 다운샘플해 나눠 먹인다.

```bash
# T1 — 🔴 depth_profile을 1280x720으로 직접 올리지 말 것(2026-08-11 실기 확인).
#   depth_profile:=1280x720x15/x30 둘 다 "Frames didn't arrived within 5 seconds"로 죽는다
#   (USB 대역폭 초과로 추정 — depth+IR×2+color+motion을 전부 1280x720/848x480로 동시 요청하면 못 견딤).
#   해결: depth_module은 네이티브 848x480(가벼움)에 두고, color_profile만 1280x720으로 올린다.
#   align_depth.enable=true라 aligned_depth_to_color는 **color 해상도를 따라간다**
#   (camera.launch.py:62 주석) — 실측: aligned_depth_to_color가 실제로 1280x720, ~19~29 Hz로 안정.
#   ⚠️ 단 이건 848x480 원본을 1280x720 격자로 리샘플한 것이다 — 색/마스크와의 픽셀 대응은
#   1280x720이 맞지만, depth 자체의 공간 분해능은 848x480 수준이 상한이다(업샘플이 디테일을
#   만들어내지 않는다). graspgenx가 원하는 게 "YOLO 마스크와 픽셀 단위로 맞는 depth"라면 이걸로
#   충분하고, "센서가 실제로 848x480보다 세밀하게 찍는 것"을 기대한 거라면 기대와 다르다.
ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=848x480x30 color_profile:=1280x720x30

# T1.5 depth 다운샘플 — 호스트, GPU 불필요. INTER_NEAREST + K 스케일
ros2 run m0609_rg2_bringup depth_downsample_node.py --ros-args \
  -p target_width:=424 -p target_height:=240
```

**검증** (2026-08-11 실기로 아래 값 확인됨)

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw   # 실측 19~29 Hz (color 1280x720)
ros2 topic hz /cumotion/depth_1/image                            # 실측 15~26 Hz — 다운샘플이 병목 안 만든다
ros2 topic echo /cumotion/depth_1/image --field width --once     # 424
ros2 topic echo /cumotion/depth_1/camera_info --field k --once   # fx,fy,cx,cy가 원본의 424/1280, 240/720배 — 실측 일치
```

- graspgenx는 그대로 `/camera/camera/aligned_depth_to_color/image_raw`(1280x720)를 구독한다 — 수정 불필요.
- 아래 T4 명령의 `depth_image_topics`/`depth_camera_infos`를 `/cumotion/depth_1/image`,
  `/cumotion/depth_1/camera_info`로 바꿔야 한다(§4에 반영됨).
- 이 경로를 안 쓰면(카메라를 계속 424x240 하나로만 굴리면) 이 절은 건너뛰고 기존 §4 원본
  토픽을 그대로 쓴다.
- octomap(OMPL 경로) 쪽은 다운샘플 노드가 안 건드린다 — `/camera/camera/depth/color/points`가
  1280x720 기준으로 커지므로 `m0609_rg2_moveit/config/sensors_3d.yaml`의 `point_subsample`을
  올려야 한다(그 파일 주석 참고, 배율은 실측 필요).
- **미검증**: T4(robot_segmenter) 3.7 Hz가 다운샘플된 입력으로 실제로 유지/개선되는지, 이 조합에서
  graspgenx/yolo가 실제로 잘 동작하는지는 컨테이너·grasp 파이프라인까지 붙여서 재야 한다
  (이번 검증은 호스트 T1~T1.5까지만; T4~T7은 컨테이너라 사용자 터미널에서 확인 필요).

## 2. 호스트 — T2: 실기 로봇

```bash
rdm && br
# = ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false
```

**검증** — `/joint_states`가 cuMotion의 전제조건이다.

```bash
ros2 topic info /joint_states          # Publisher count: 1  ← 2면 옛 launch가 살아있는 것
ros2 topic echo /joint_states --once   # name 12개 / position 12개 / velocity 12개
```

🔴 **velocity가 비어 있으면 cuMotion 계획이 전부 실패한다.** `publish_default_velocities: True`가
`bringup.launch.py`에 들어가 있어야 한다(커밋됨).

> `[OnRobot Modbus]: Connection failed!`는 그리퍼 통신 실패다. 계획에는 영향 없다
> (`rg2_finger_joint`는 XRDF에서 lock). 그리퍼를 실제로 여닫으려면 이걸 먼저 고쳐야 한다.

## 3. 호스트 — T3: 컨테이너 기동

```bash
rdm                                    # ⚠️ 먼저. run_dev.sh가 -e ROS_DOMAIN_ID로 넘긴다
cd ~/cobot2_ws/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

`rdm` 없이 열면 컨테이너가 도메인 0이 되어 로봇을 못 본다.

### 컨테이너에 들어가면 **맨 처음 한 번**

```bash
bash /workspaces/cobot2_ws/scripts/container_setup.sh
```

🔴 **`run_dev.sh`는 컨테이너를 재사용하지 않고 새로 만든다.** pip 설치(warp 1.5.0, numpy 1.26.4)가
매번 날아간다. 안 하면 `AttributeError: module 'warp' has no attribute 'torch'`로 죽는다.

### 컨테이너 **셸마다** (T4~T7 전부)

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/cobot2_ws/fastdds_udp_only.xml
```

🔴 **컨테이너 쪽에도 반드시 건다 — 호스트에만 걸면 여전히 안 온다** (양쪽 다 필요, `fastdds_udp_only.xml`
파일 안 주석 그대로). `graspx_container.sh`가 `docker exec -e`로 이미 하던 것과 같은 조치다.

⚠️ **`RMW_IMPLEMENTATION`은 설정하지 않는다.** cyclonedds로 바꾸면 T7의 컨트롤러 spawner가
호스트 `controller_manager` **서비스**를 못 불러 멈춘다(교차 벤더는 토픽만 된다).

## 4. 컨테이너 — T4: robot_segmenter (로봇을 depth에서 지움)

```bash
cd /workspaces/isaac_ros-dev
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p distance_threshold:=0.15 \
  -p depth_qos:=SENSOR_DATA \
  -p depth_info_qos:=SENSOR_DATA \
  -p depth_image_topics:="[/camera/camera/aligned_depth_to_color/image_raw]" \
  -p depth_camera_infos:="[/camera/camera/aligned_depth_to_color/camera_info]" \
  -p robot_mask_publish_topics:="[/cumotion/camera_1/robot_mask]" \
  -p world_depth_publish_topics:="[/cumotion/camera_1/world_depth]"
```

> §1.5(고해상도 카메라 + 다운샘플 분리)를 쓰는 중이면 `depth_image_topics`/`depth_camera_infos`를
> `/cumotion/depth_1/image` / `/cumotion/depth_1/camera_info`로 바꿔서 띄운다 — 원본 그대로 물리면
> T4가 다시 1280x720을 받아 반응속도 이득이 없어진다.

🔴 **이걸 빼면 cuMotion이 로봇 자기 몸을 장애물로 보고 계획이 전부 실패한다**
(`INVALID_START_STATE_WORLD_COLLISION`). nvblox는 MoveIt의 self-filter를 안 거친다.

🔴 **`depth_qos`/`depth_info_qos`를 안 주면 world_depth가 아예 발행되지 않는다 (2026-08-11 실측).**
`isaac_ros_common/qos.py`의 `add_qos_parameter(self, 'DEFAULT', 'depth_qos')` 기본값은
`QoSProfile(depth=10)` = **RELIABLE**인데, realsense 드라이버(및 depth_downsample_node.py)는
`BEST_EFFORT`(sensor_data QoS)로 발행한다. 안 맞추면 로그에
`New publisher discovered... offering incompatible QoS. No messages will be received from it.`가
뜨고 depth 구독 자체가 죽는다 — 에러 없이 조용히 아무것도 안 들어온다. 위 명령에 이미 반영함.

🔴 **realsense depth 프레임의 `header.stamp`가 로봇(`/joint_states`)의 시스템 클록보다
일관되게 약 1.0~1.06초 뒤처진다 (2026-08-11 실측, `depth_qos` 수정 후에도 재현).**
`robot_segmenter_node`는 depth와 `/joint_states`를 `ApproximateTimeSynchronizer(slop=0.1)`로
동기화하는데, 이 오프셋은 slop의 10배가 넘어 **동기화 콜백이 영원히 안 불린다** —
`/cumotion/camera_1/world_depth`, `/cumotion/camera_1/robot_mask`,
`/cumotion/robot_segmenter/robot_spheres` 셋 다 발행 자체가 없다(에러 로그도 없음, 조용히 멈춤).
  - **§1.5(다운샘플) 문제가 아니다** — 원본 `/camera/camera/aligned_depth_to_color/image_raw`도
    같은 ~1.0s 오프셋을 보인다(직접 대조 확인). 즉 원본 해상도로 T4를 띄워도 이 상태면 똑같이 막힌다.
  - 원인 후보(미확인): realsense2_camera_node가 하드웨어 타임스탬프를 쓰고 있어서 시스템 클록과
    안 맞을 가능성 (`use_ros_time_first_pkt` 등 관련 파라미터 확인 필요) — **UNVERIFIED, 다음에 확인**
  - 문서에 있던 "world_depth 3.7 Hz" 실측치가 어떤 조건에서 나온 값인지도 같이 재확인이 필요하다
    (이 오프셋이 그때도 있었다면 애초에 그 3.7 Hz는 안 나왔어야 한다).
  - 재현 확인법: `ros2 topic echo <depth_topic> --field header.stamp --once` vs
    `ros2 topic echo /joint_states --field header.stamp --once`를 거의 동시에 찍어 sec.nanosec 차를 본다.

🔴 **2026-08-11 근본 원인 확정 — 위 "클록 오프셋"은 가짜였다.** `ros2 topic echo --once`를
순차 호출(subprocess 기동마다 ~1s)해서 잰 게 원인이었다 — 한 프로세스가 depth+joint_states를
동시에 구독하는 `message_filters` 프로브로 다시 재니 실제 diff는 ±0.03~0.09s로 slop=0.1
안에 들어온다(정상). **진짜 원인은 컨테이너가 호스트 토픽을 아예 못 받는 것이었다**:
  - `ros2 topic list`/`ros2 topic info -v`는 컨테이너에서 정상 — 퍼블리셔·QoS까지 다 보인다.
  - 그런데 `ros2 topic hz <아무 호스트 토픽>`을 컨테이너 안에서 돌리면 **0건**이다
    (`/joint_states`, `/camera/.../image_raw`, `/camera/.../camera_info` 전부 재현).
  - 원인: **`FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/cobot2_ws/fastdds_udp_only.xml`을
    호스트·컨테이너 양쪽 셸에 안 걸었다.** 이 파일은 2026-08-06에 이미 같은 증상으로
    만들어져 있었는데(파일 안 주석에 "토픽은 보이는데 안 열린다" 그대로 적혀 있다) T4 실행
    커맨드에 반영이 안 돼 있었다. 걸고 나면 `/joint_states`는 즉시 10Hz로 안정 수신됨(재확인).
  - ⚠️ **이걸 걸어도 카메라 스트림(image/camera_info)은 아직 간헐적으로 0건이 난다** —
    같은 명령을 반복하면 어떤 때는 14-15Hz로 멀쩡히 들어오고 어떤 때는 통째로 안 온다.
    `/joint_states`(작은 메시지, 10Hz)는 매번 안정적이었다 — **크기 큰 메시지에서만 재현**되는
    경향으로 봐서 `fastdds_udp_only.xml` 자체 주석에 있는 `net.core.rmem_max`(커널 소켓
    버퍼 상한, 이 랩탑 기본 212992) 부족 쪽이 유력하지만 **미검증**. sysctl은 랩탑 전역 설정이라
    팀 합의 없이 안 건드렸다.
  - **T4/T5를 다시 테스트할 때는 반드시 아래를 호스트·컨테이너 양쪽 셸에 다 걸고 시작한다**:
    ```bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml   # 호스트
    export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/cobot2_ws/fastdds_udp_only.xml   # 컨테이너
    ```
    이번 세션에서는 위 카메라 스트림 간헐 유실 때문에 `world_depth` 실제 발행까지는
    확인 못 했다(`is_subscribed()` 게이트는 통과 확인됨 — `robot_segmenter.py:241` 참고,
    mask/world_depth 퍼블리셔 중 하나라도 구독자가 있어야 `on_timer`가 진행된다).

✅ **2026-08-11 재검증 — 카메라·컨테이너를 처음부터 다시 깨끗하게(참여자 난립 없이) 띄우고
FASTRTPS 프로파일을 처음부터 걸고 시작하니 간헐 유실 없이 한 번에 됐다.** 앞서 겪은 "카메라
스트림만 간헐적으로 0건" 현상은 소켓 버퍼 문제가 아니라 **같은 세션에서 급하게 반복한
docker exec/ros2 프로세스 난립(디버깅 중 여러 개 겹쳐 뜬 것)이 원인이었을 가능성이 높다** —
정리 안 하면 이 증상 재현되는지도 다음에 봐야 한다.
  - **`world_depth`가 원본 1280x720 그대로 실제 발행됨을 확인.** 실측 rate **9.2~10.0 Hz**,
    프레임당 Node Time ~20ms(Computation ~15.7ms) — 카메라 depth 프레임레이트(~9.65Hz)를
    거의 그대로 따라간다.
  - 🔴 **문서에 있던 "world_depth 3.7 Hz" 병목 수치는 이 세션 조건에서 재현되지 않았다.**
    GPU 세그멘테이션 자체는 프레임당 15.7ms로 여유가 크다(100ms 예산의 15%) — 이전 3.7Hz는
    이 자리의 FASTRTPS 미설정으로 인한 **DDS 패킷 유실/재전송**이 원인이었을 가능성이 크다.
    (미확정 — 이 문서의 T4 명령이 그때도 FASTRTPS 없이 돌았는지는 기록에 없음)
  - **결론: 원본 해상도로도 T4가 카메라 프레임레이트를 거의 무손실로 따라간다.** §1.5의
    "424x240 다운샘플로 T4 반응속도를 올린다"는 애초 가설은 **이 조건에서는 불필요할 수
    있다** — 다운샘플 자체는 여전히 유효한 도구지만(GPU 부하가 다른 이유로 커지면 손잡이로
    남겨둔다), 지금 병목이었던 건 해상도가 아니라 순전히 FASTRTPS 설정 누락이었다.

✅ **2026-08-11 T4→T5 관통 확인.** T4를 원본 1280x720으로 띄운 채 T5(§5 명령 그대로,
FASTRTPS 프로파일만 추가)를 이어 붙이니:
  - `/cumotion/camera_1/world_depth`(T4→T5 입력) 구독 시작 직후 rate가 5.7Hz → 8.4Hz로
    수렴(구독 시작 램프업으로 보임, 안정화 후 더 지켜볼 것)
  - `/nvblox_node/mesh` 정상 발행, rate ~8.5Hz로 수렴
  - `FATAL`/크래시 없음, `/nvblox_node/get_esdf_and_gradient` 서비스 정상 등록,
    `global_frame` 파라미터 `base_link` 확인
  - T6(cuMotion 플래너)·T7(MoveIt)은 로봇 계획/실행이 걸리는 영역이라 이번엔 진행하지 않음

**검증**: `ros2 topic hz /cumotion/camera_1/world_depth` → 실측 **3.7 Hz**
⚠️ 여기가 파이프라인 병목이다(카메라 9.65 → 3.7 Hz, 최대 공백 3.1초).

## 5. 컨테이너 — T5: nvblox

```bash
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link \
  -p use_lidar:=false \
  -p num_cameras:=1 \
  -p esdf_mode:=3d \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info
```

- 🔴 **`esdf_mode:=3d` 없으면 cuMotion의 첫 요청에 nvblox가 FATAL로 죽는다.**
  기본값이 `2d`다. cuMotion 로그에는 계획 실패만 남으므로 **`pgrep -f nvblox_node`로 확인할 것**
- **depth 입력만** 세그멘터 출력으로 바꾼다. `camera_info`·color는 원본 그대로
- 세그멘터를 나중에 끼웠다면 **nvblox를 재시작한다** — 기존 지도의 로봇은 안 지워진다
- `nvblox_realsense.yaml`은 얹지 않는다 (`map_clearing_frame_id`가 우리 TF와 안 맞는다)

**검증**

```bash
ros2 param get /nvblox_node global_frame       # base_link
ros2 service list | grep esdf                  # /nvblox_node/get_esdf_and_gradient
pgrep -f nvblox_node                           # 살아 있어야 한다
```

## 6. 컨테이너 — T6: cuMotion 플래너

```bash
cd /workspaces/isaac_ros-dev
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True \
  -p publish_curobo_world_as_voxels:=True
```

- `robot:=`은 **파일명만** 준다(경로 아님). `isaac_ros_cumotion_robot_description/xrdf/`에서 찾는다
- 워밍업에 5~10초 걸린다. `cuMotion is ready for planning queries!`가 나와야 준비 완료
- 🔴 **`read_esdf_world:=False`로 띄우면 장애물을 못 본다.** 계획은 성공한다 — 그래서 위험하다

**검증**: `ros2 action list | grep cumotion` → `/cumotion/move_group`

## 7. 컨테이너 — T7: move_group (+ RViz)

```bash
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
#   RViz를 따로 띄울 거면 rviz:=false
```

**검증** — 로그에 이 세 줄이 다 나와야 한다.

```
Loading planning pipeline 'ompl'                 → Using planning interface 'OMPL'
Loading planning pipeline 'isaac_ros_cumotion'   → Using planning interface 'Generate minimum-jerk ... cuMotion'
Configured and activated dsr_moveit_controller   ← 이게 있어야 Execute가 된다
```

```bash
ros2 topic hz /moveit/filtered_cloud    # 실측 2.3 Hz — OMPL octomap용 self-filter 결과
```

> `Controller already loaded, skipping load_controller` → `Failed to configure controller`는
> **옛 move_group이 이미 spawn해 둔 것**이다. 컨트롤러는 이미 active이므로 Execute는 된다.

> ⚠️ **`ros2 node list`에 `/move_group`이 2개로 보이는 것은 정상이다.** MoveIt이 내부적으로
> 같은 이름의 노드를 하나 더 만든다(궤적 실행 관리자). **중복 실행이 아니다.**
> 진짜로 판정하려면 이 둘을 본다 — 둘 다 1이어야 한다:
> ```bash
> ps -eo cmd | grep -c "moveit_ros_move_group/move_group"   # 1
> ros2 action list | grep -cE "^/move_action$"              # 1
> ```

---

## 8. RViz에서 볼 것

| 보고 싶은 것 | Display | Topic |
|---|---|---|
| **cuMotion이 쥔 장애물** | Marker | `/curobo/voxels` |
| nvblox 지도 | NvbloxMesh / PointCloud2 | `/nvblox_node/mesh`, `/nvblox_node/color_layer` |
| OMPL octomap 입력 | PointCloud2 | `/moveit/filtered_cloud` |
| octomap 결과 | PointCloud2 | `/octomap_point_cloud_centers` |

- 🔴 **`/nvblox_node/static_esdf_pointcloud`는 `esdf_mode:=3d`에서 발행되지 않는다** (2d 슬라이스 전용).
  RViz에서 비어 보이는 게 정상이다 — `mesh`/`color_layer`를 쓴다
- 🔴 **`/curobo/voxels`는 계획을 한 번 돌려야 나온다.** 구독자가 있을 때만, 계획 요청 처리 중에만
  발행한다. 대기 중 `topic hz`로 판정하지 말 것
- `octomap_rviz_plugins` 미설치라 `/octomap_binary`는 못 본다

---

## 9. 검증 — 계획 시간 재기 (로봇 안 움직임)

```bash
python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 10
```

`plan_only=True` 고정이라 **로봇은 움직이지 않는다.** RViz 드롭다운을 사람이 번갈아 누르는 대신
`pipeline_id`만 바꿔 같은 목표를 N회 계획한다.

**2026-08-06 실기 실측** (로봇+카메라+nvblox 전부 살아 있는 상태, 관절목표, 각 10회):

| | server 중앙값 | wall 중앙값 | 성공 |
|---|---|---|---|
| OMPL | 42.4 ms | 106.0 ms | 10/10 |
| cuMotion | 110.6 ms | 204.1 ms | **10/10** |

cuMotion이 쥔 장애물 복셀: **27,646개** (`/curobo/voxels`)

> ⚠️ 이 숫자로 "cuMotion이 느리다"고 결론내지 말 것 — 관절공간 목표는 OMPL(RRTConnect)에
> 가장 유리한 조건이다. 판단 근거는 장애물이 궤적을 실제로 막는 씬에서의 비교다.

---

## 10. 종료 — GPU를 다음 사람에게 넘기는 절차

🔴 **이 랩탑은 세 계정(`joonwon`·`kimkh`·`rokey`)이 동시 로그인해 같은 GPU와 같은 ROS 도메인(93)을
쓴다.** 다른 계정도 자기 Isaac ROS 컨테이너(`cumotion-joonwon` 등)를 띄운다.
→ **`ps`에 `user`를 넣지 않으면 남의 프로세스를 내 것으로 착각한다.** 2026-08-06에 실제로 헤맸다.

```bash
# ① 누가 GPU를 쥐고 있나
nvidia-smi --query-compute-apps=pid,used_memory --format=csv

# ② 그 PID가 내 것인지 확인 — 남의 것이면 절대 kill하지 않는다
ps -o pid,user,cmd -p <pid>
```

**종료는 올린 순서의 반대로**: T7 move_group → T6 플래너 → T5 nvblox → T4 세그멘터 →
T2 bringup → T1 카메라. 각 터미널에서 Ctrl+C.

```bash
# ③ 정말 죽었는지 확인 — 오늘 실패의 절반이 "죽은 줄 알았던 노드"였다
ps -eo pid,user,cmd | grep -E "move_group|nvblox|cumotion|segmenter|realsense2_camera_node" | grep -v grep
# 내 것이 남아 있으면 PID로: kill <pid>   (안 죽으면 kill -9)
```

⚠️ **`pkill -f`를 쓰지 말 것.** `docker exec bash -c`에서 **자기 명령줄에도 매칭돼 자기 셸을 먼저
죽인다** — 뒤 명령이 조용히 실행되지 않는데 출력은 깨끗해서 "정리됨"으로 오독한다.
공유 랩탑에서는 남의 프로세스까지 걸린다.

```bash
# ④ 반납 확인
nvidia-smi --query-gpu=memory.used --format=csv,noheader   # 아무도 안 쓰면 ~33 MiB
```

**컨테이너는 지우지도 stop하지도 않는다.** 안의 노드만 내리면 GPU는 반납된다.
`run_dev.sh`를 다시 돌리면 컨테이너가 **새로 만들어져** `container_setup.sh`를 또 돌려야 한다.

**VRAM 실측 (2026-08-06, full-up)** — 8 GB의 31%라 셋 동시 실행에 여유가 있다:

| 노드 | VRAM |
|---|---|
| `cumotion_planner_node` | 1,508 MiB |
| `robot_segmenter_node` | 660 MiB |
| `nvblox_node` | 334 MiB |
| 합계 | **약 2.5 GB / 8 GB** |

---

## 11. 증상 → 원인 빠른 표

| 증상 | 원인 | 조치 |
|---|---|---|
| `module 'warp' has no attribute 'torch'` | 컨테이너 재생성으로 pip 설치 유실 | `container_setup.sh` |
| `import cv2` → `numpy.core.multiarray failed` | 이미지 numpy 2.2.6 vs apt cv2 | `container_setup.sh` |
| cuMotion 계획 전부 실패, 로그엔 `Calling ESDF service`만 | **nvblox가 죽었다** (`esdf_mode` 2d) | `-p esdf_mode:=3d` |
| `INVALID_START_STATE_WORLD_COLLISION` | 로봇이 nvblox 지도에 들어감 | `robot_segmenter_node` + nvblox 재시작 |
| `INVALID_START_STATE_SELF_COLLISION` | XRDF 구 과대추정 | XRDF `self_collision.ignore` (해결됨) |
| cuMotion만 실패, velocity 오류 | `/joint_states`에 velocity 없음 | `publish_default_velocities: True` (해결됨) |
| 계획이 **산발적으로** 실패 | 옛 노드가 안 죽고 중복 발행 | `ros2 topic info` / `ros2 action list`로 개수 확인 |
| 계획은 성공하는데 장애물을 통과 | `read_esdf_world:=False` | `True` + nvblox |
| depth가 절반 이하 | RealSense 드라이버 2개 | `ros2 node list \| grep camera` |

---

## 12. 아직 안 된 것

- **장애물이 궤적을 실제로 바꾸는지 미검증.** 계획 성공과 복셀 적재까지만 확인했다.
  손을 작업공간에 넣고 같은 목표로 OMPL/cuMotion 각각 계획해 궤적이 달라지는지 봐야 한다
- **depth 9.65 Hz** — 요청은 15 Hz다. 원인 미특정
- **세그멘터 3.7 Hz** — 파이프라인 병목. 최대 공백 3.1초는 사람 팔 반응에 부족할 수 있다
- **그리퍼 Modbus 연결 실패** — 여닫기 불가
- **XRDF `link_4 ↔ rg2_base_link` 자기충돌 검사를 꺼 뒀다** — 실기 모션 전 재검토 필수
