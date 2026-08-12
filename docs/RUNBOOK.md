# 실행 순서 안내 (RUNBOOK)

---

## 참고용 개인 단축 명령(alias)

아래는 개인적으로 사용하시는 셸 별칭(alias)으로 보입니다. 실제 동작은 각자의 `.bashrc`/`.zshrc`에 정의된 내용을 확인해 주세요.

`rdm` · `sob` · `rdme` · `cdco` · `si`

---

## 1. 빌드 (호스트)

🔴 코드나 yaml 파일을 새로 받으셨다면 먼저 빌드해 주세요. **yaml 파일만 수정하신 경우에도 다시 빌드가 필요합니다.** (`ament_python` 패키지는 share 경로가 `build/`를 참조하기 때문에 `src` 수정 사항이 자동으로 반영되지 않습니다 — 팀 컨벤션 문서 §4)

```bash
colcon build --symlink-install --packages-select pick_fsm pick_fsm_msgs voice_processing graspgenx_perception
source install/setup.bash
```

## 2. 브링업

| 명령 | 설명 |
| --- | --- |
| `br` | 로봇 브링업 실행 (개인 alias) |
| `reals1280` | RealSense 카메라를 1280 해상도로 실행 (개인 alias) |

## 3. 도커 컨테이너 실행 (터미널 3~6)

컨테이너 안에서 총 4개의 터미널로 각각 cumotion segmenter · nvblox · cumotion planner · cumotion moveit을 실행합니다.

**컨테이너 진입**

```bash
cd ~/cobot2_ws_new/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws_new:/workspaces/cobot2_ws_new"
```

**컨테이너의 첫 번째 터미널에서 실행**

```bash
bash /workspaces/cobot2_ws_new/scripts/container_setup.sh
```

**이후 터미널 진입**

```bash
docker exec -i -t -u admin
```

**컨테이너의 각 터미널에서 공통으로 실행** (컨테이너는 재빌드가 필요 없습니다 — `install_container`에는 `pick_fsm`·`voice_processing`·`graspgenx_perception`이 포함되어 있지 않고 `cumotion`·`dsr`·`moveit`·`bringup`·`onrobot`만 있어 이번 변경과는 무관합니다.)

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws_new/install_container/setup.bash
export ROS_DOMAIN_ID=93
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/cobot2_ws_new/fastdds_udp_only.xml   # ← 이 설정도 꼭 필요합니다
```

**터미널 3 — cumotion segmenter**

```bash
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/cumotion_segmenter.yaml
```

**터미널 4 — nvblox**

```bash
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/nvblox_realtime.yaml \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info
```

**터미널 5 — cumotion planner**

```bash
cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/cumotion_planner.yaml
```

**터미널 6 — move_group + RViz**

🔴 반드시 컨테이너 안에서 실행해 주세요. (호스트 쪽 터미널 3, 즉 같은 런치 파일을 쓰는 쪽은 꺼져 있어야 합니다.) 실행 위치(컨테이너)와 `cumotion:=true` 옵션만 다르고 나머지는 동일합니다.

```bash
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
```

## 4. 호스트 터미널 (인식 · FSM · 브릿지)

**터미널 7 — YOLO-seg**

```bash
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true
```

**터미널 8 — GraspGenX**

```bash
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true
```

**터미널 9 — Pick FSM**

`voice` 옵션은 기본값이 `true`입니다. rqt로만 실행하실 경우 `voice:=false`로 설정해 주세요. `planning_pipeline`은 `cumotion` 대신 `ompl`도 사용하실 수 있습니다.

🔴 목적지(place) 지정 없이 pick만 실행하면 `WAIT_PLACE_TARGET` 상태에서 물체를 든 채 **무기한** 대기합니다 (2026-08-12 변경). "테이블에 놔"(`set_place`) 또는 "그냥 거기 놔"(`release_now`)라고 말씀해 주셔야 종료됩니다. 종료 전에는 `/pick/stow`를 호출해 주세요 — 놓을 자리로 먼저 이동한 뒤 내려놓고 홈으로 복귀합니다 (자세한 내용은 맨 아래 참고).

```bash
ros2 launch pick_fsm pick_fsm.launch.py voice:=true planning_pipeline:=isaac_ros_cumotion
```

**터미널 10 — VLA 연결 브릿지**

🔴 `pixel_policy`의 기본값이 2026-08-12부터 `select`로 바뀌었습니다 (이전 기본값은 `warn`). "이거 집어줘"처럼 VLA가 특정 개체를 픽셀 좌표로 지목해 보내면, 이제 FSM이 그 픽셀 정보를 실제로 사용합니다. `warn`으로 설정하면 픽셀 정보를 무시하고 클래스만으로 판단하므로, 같은 종류의 물체가 여러 개 있을 때 지목과 무관하게 점수가 높은 쪽을 집게 됩니다 (증상은 "가끔 엉뚱한 걸 집는다"로 나타납니다). 이전 동작으로 되돌리시려면 `pixel_policy:=warn`으로 설정해 주세요.

```bash
ros2 launch voice_processing vla_command.launch.py auto_start:=true
```

## 5. VLA 터미널 (별도 워크스페이스)

🔴 `--packages-select` 옵션은 붙이지 말아 주세요. `build.sh`는 인자(`$@`)를 사용하지 않으므로 해당 옵션은 조용히 무시됩니다. 이후 누군가 `build.sh`가 인자를 받도록 수정하게 되면, `vla_interfaces`가 빌드되지 않아 `agent_node`에서 `ImportError: cannot import name 'MissionState'` 오류로 종료될 수 있습니다. (2026-08-12에 `MissionState`/`MissionCommand` 메시지 2개가 새로 추가되었습니다.)

```bash
cd ~/M0609_VLA_system_new
./scripts/build.sh
source scripts/env.sh
ros2 run vla_system vla_gui
```

## 참고: 수동 이동 테스트 명령

```bash
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [-47, 612, 174, 91, 158, 176], vel: [30.0, 20.0], acc: [30.0, 40.0], time: 0.0, radius: 0.0, ref: 0, mode: 0, blend_type: 0, sync_type: 0}"
```

---

## 🆕 새로 추가된 기능 (2026-08-12)

사용하지 않으셔도 기존 흐름은 그대로 동작합니다. 아래는 모두 `std_srvs/srv/Trigger` 타입입니다. 먼저 source해 두시면 편리합니다.

```bash
source /opt/ros/humble/setup.bash && source ~/cobot2_ws_new/install/setup.bash
```

| 서비스 | 설명 |
| --- | --- |
| `/pick/pause` | ✋ 멈춤 — 되돌릴 수 있습니다. 어떤 상태에서든 적용되며, 그리퍼는 물체를 문 상태로 정지합니다. 시간이 지나도 자동으로 재개·내려놓기·홈 복귀되지 않습니다. |
| `/pick/resume` | 멈춘 지점부터 이어서 진행합니다. 물체를 들고 있었다면 재인식 없이 `PLACE`로, 들고 있지 않았다면 `PERCEIVE`부터 시작합니다. |
| `/pick/stow` | 🔴 종료 전 정리용입니다. 물체를 들고 있다면 놓을 자리로 먼저 이동한 뒤 내려놓고 홈으로 복귀합니다. ("그리퍼를 열고 홈으로 복귀"를 그대로 수행하면 현재 위치(약 30cm 상공)에서 물체가 떨어집니다.) 물체를 든 채로 `Ctrl-C`로 종료하면 그리퍼는 문 상태로 남는데, 이는 물체를 떨어뜨리는 것보다 안전하도록 의도한 동작입니다. |
| `/pick/release_now` | 물체를 든 채 대기 중일 때 "그냥 거기 놔"에 해당합니다. 이동 없이 현재 위치에서 바로 그리퍼를 엽니다. `WAIT_PLACE_TARGET` 또는 `PAUSED` 상태에서만 동작합니다. |

기존 서비스(변경 없음): `/pick/start` · `/pick/abort` · `/pick/reset` · `/pick/home` · `/pick/retry_place`

⚠️ `abort`는 여전히 파괴적인 동작입니다 — `SAFE_STOP` 상태로 전환되며 `/pick/reset`과 `HOME` 복귀가 필요합니다. 잠깐 멈추시려는 것이라면 `abort` 대신 `pause`를 사용해 주세요.

VLA(JSON)에서도 동일한 동작을 사용하실 수 있습니다 — `/vla/pick_command`의 `cmd` 값으로 다음을 전달하면 됩니다: `pause` · `resume` · `stow` · `release_now` · `set_place` · `pick` · `abort` · `reset` · `home` · `start`
