# ─────────────────────────────────────────────────────────────
# 2026-08-12 갱신 — 명령은 안 바뀌었다. 바뀐 건 아래 3개뿐.
#   ① 들고 대기하면 이제 **안 내려놓는다** (wait_place_timeout_sec: 100.0 → 0.0)
#      예전엔 100초 뒤 basket 에 알아서 놨다. 되돌리려면 pick_fsm.yaml 에 양수만 넣으면 된다
#   ② GUI 정지 버튼이 2개 — ⏸ 멈춰(되돌림) / 🔴 비상정지(리셋 필요). 말로 "멈춰"는 ①쪽
#   ③ vla 빌드에서 --packages-select 를 뺐다 (아래 vla터미널 주석 참고)
#   새 서비스 4개는 맨 아래 "새로 생긴 것" 절.
# ─────────────────────────────────────────────────────────────

rdm
sob
rdme
cdco
si

# 🔴 코드/yaml 을 받았으면 먼저 빌드한다. yaml 만 고쳐도 재빌드해야 한다
#    (ament_python 은 share 가 build/ 를 가리켜서 src 수정이 안 넘어간다 — 팀 컨벤션 문서 §4)
colcon build --symlink-install --packages-select pick_fsm pick_fsm_msgs voice_processing graspgenx_perception
source install/setup.bash

br #브링업
reals1280 #realsense 1280



#도커 실행 명령어 - 총 4개 (cumotion seg , nvblox , cumotion planner , cumotion moveit)
cd ~/cobot2_ws_new/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws_new:/workspaces/cobot2_ws_new"

#도커 첫 터미널에서 수행
bash /workspaces/cobot2_ws_new/scripts/container_setup.sh

docker exec -i -t -u admin

#도커 각 터미널에 입력
# (컨테이너는 재빌드 불필요 — install_container 에 pick_fsm/voice_processing/graspgenx 가
#  아예 없다. cumotion·dsr·moveit·bringup·onrobot 뿐이라 이번 변경과 무관하다)
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws_new/install_container/setup.bash
export ROS_DOMAIN_ID=93
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/cobot2_ws_new/fastdds_udp_only.xml   # ← 이것도 필수


#터미널 3
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/cumotion_segmenter.yaml

#터미널 4
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/nvblox_realtime.yaml \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info

#터미널 5
cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  --params-file /workspaces/cobot2_ws_new/config/cumotion_planner.yaml

# [터미널 3C] move_group + RViz — ★ 컨테이너 안에서 띄운다. 호스트 터미널 3은 꺼져 있어야 한다
#   3절 터미널 3과 같은 런치다. 실행 위치(컨테이너)와 cumotion:=true 만 다르다
#터미널 6
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true



#터미널 7, yolo seg
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true

#터미널 8, graspgenx
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true

#터미널 9, fsm (voice true가 기본값, rqt로만 실행 시 voice:=false / cumotion 대신P ompl도 가능)
# ⚠️ place 없이 pick 하면 WAIT_PLACE_TARGET 에서 **무기한** 들고 기다린다(2026-08-12).
#    "테이블에 놔"(set_place) 또는 "그냥 거기 놔"(release_now) 를 말해야 끝난다.
#    끄기 전엔 /pick/stow 를 부르면 놓을 자리로 먼저 가서 놓고 홈으로 간다(맨 아래 참고).
ros2 launch pick_fsm pick_fsm.launch.py voice:=true planning_pipeline:=isaac_ros_cumotion

#터미널 10, vla쪽과 연결 위한 브릿지)
# 🔴 pixel_policy 기본값이 2026-08-12 부터 select 다 (예전 warn).
#    "이거 집어줘"처럼 VLA 가 **어느 개체인지** 지목해 보낸 pixel 을 FSM 이 실제로 쓴다.
#    warn 이면 그 pixel 을 버리고 클래스만으로 골라서, 사과가 둘일 때 지목과 무관하게
#    점수 높은 쪽을 집는다(증상이 "가끔 엉뚱한 걸 집는다"로만 나온다).
#    되돌리려면: pixel_policy:=warn
ros2 launch voice_processing vla_command.launch.py auto_start:=true
///


vla터미널
cd ~/M0609_VLA_system_new
# 🔴 --packages-select 를 붙이지 마라. build.sh 는 $@ 를 안 쓰므로 그 인자는 조용히 무시되고,
#    누가 build.sh 가 인자를 받게 "고치는" 순간 vla_interfaces 가 안 빌드돼서
#    agent_node 가 ImportError: cannot import name 'MissionState' 로 죽는다.
#    (2026-08-12에 MissionState/MissionCommand 두 msg 가 새로 생겼다)
./scripts/build.sh
source scripts/env.sh
ros2 run vla_system vla_gui


ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [-47, 612, 174, 91, 158, 176], vel: [30.0, 20.0], acc: [30.0, 40.0], time: 0.0, radius: 0.0, ref: 0, mode: 0, blend_type: 0, sync_type: 0}"


# ─────────────────────────────────────────────────────────────
# 새로 생긴 것 (2026-08-12) — 안 써도 기존 흐름은 그대로 돈다
# ─────────────────────────────────────────────────────────────
# 전부 std_srvs/srv/Trigger. 먼저 source 해두면 편하다:
#   source /opt/ros/humble/setup.bash && source ~/cobot2_ws_new/install/setup.bash

# ✋ 멈춰 — 되돌릴 수 있다. 어떤 상태에서든 먹는다. 그리퍼는 그대로 문 채 선다.
#    시간이 지나도 아무 일도 안 일어난다(자동 재개·자동 내려놓기·자동 홈 전부 없음).
ros2 service call /pick/pause std_srvs/srv/Trigger {}

# 계속해 — 멈춘 지점 기준으로 복귀. 보유 중이면 재인식 없이 PLACE, 비보유면 PERCEIVE 부터.
ros2 service call /pick/resume std_srvs/srv/Trigger {}

# 🔴 끄기 전 정리. 들고 있으면 **놓을 자리로 먼저 가서** 놓고 → 홈 → IDLE.
#    ("그리퍼 열고 홈 복귀"를 글자대로 하면 지금 있는 자리에 떨어뜨린다. 30cm 상공이면 낙하)
#    물체 든 채 Ctrl-C 하면 그리퍼는 문 채로 남는다 — 일부러 그렇게 뒀다(떨어뜨리는 것보다 안전).
ros2 service call /pick/stow std_srvs/srv/Trigger {}

# 들고 대기 중 "그냥 거기 놔" — 이동 없이 그 자리에서 연다. WAIT_PLACE_TARGET/PAUSED 에서만.
ros2 service call /pick/release_now std_srvs/srv/Trigger {}

# 기존 것 (변경 없음): /pick/start /pick/abort /pick/reset /pick/home /pick/retry_place
# ⚠️ abort 는 여전히 파괴적이다 — SAFE_STOP 으로 떨어져 /pick/reset + HOME 왕복이 필요하다.
#    잠깐 세우려는 것이면 abort 말고 pause 를 쓴다.

# VLA(JSON) 로도 같은 것들이 된다 — /vla/pick_command 에 cmd 값으로:
#   pause · resume · stow · release_now · set_place · pick · abort · reset · home · start



  