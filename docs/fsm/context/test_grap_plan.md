<!-- meta
updated: 2026-08-08
status:  superseded — 비교용 보존. 갱신하지 않는다
owns:    없음. 값의 정본은 config/testcommand.md 다
-->

> 🔄 **2026-08-08: 이 문서는 `config/testcommand.md` 로 합쳐졌다** (거기 "경로 B").
> **원문 그대로 남겨둔 이유**: 합치는 과정에서 두 문서가 같은 명령을 **다른 파라미터로**
> 적고 있던 게 3건 드러났고(`bringup` 의 `rviz`/`model`, `camera` 의 해상도,
> `moveit` 의 `cumotion`), 어느 쪽이 맞는지는 실기에서만 정할 수 있다.
> 사용자가 직접 대조할 수 있도록 손대지 않았다 — 대조표는 `config/testcommand.md`
> "합치면서 드러난 파라미터 불일치" 절.
>
> ⚠️ **여기 명령을 그대로 쓰지 말 것.** 아래 T1 은 `rviz:=false` 가 빠져 있어 MoveIt 과
> 함께 띄우면 RViz 가 2개가 된다.

---

# ── T0. 사전 점검 (여기서 막히면 아래로 안 갑니다) ──────────────
nvidia-smi                                    # RTX 4060 나와야 함
ping -c1 192.168.1.100                        # 로봇
ping -c1 192.168.1.1                          # RG2 Modbus ← 여기가 지금 막힌 곳
export ROS_DOMAIN_ID=93 && ros2 node list     # 남의 계정 move_group이 있는지 확인

# ── T1. 로봇 ────────────────────────────────────────────────
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
#   검증: ros2 topic info /joint_states   → Publisher count: 1  (2면 옛 launch가 살아있음)

# ── T2. MoveIt (bringup 위에 얹으므로 standalone:=false) ─────
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false

# ── T3. 카메라 + 캘리브 TF ──────────────────────────────────
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch m0609_rg2_bringup camera.launch.py
#   검증: ros2 run tf2_ros tf2_echo base_link camera_link

# ── T4. grasp 브리지 (GPU 워커를 자식으로 띄운다) ───────────
export ROS_DOMAIN_ID=93
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run graspgenx_perception grasp_bridge_node --ros-args \
  -p out_dir:=$(pwd)/data/graspgenx_scene \
  -p scene:=01
#   ⚠️ 첫 실행은 모델 로드로 수십 초. "worker ready" 류 로그를 기다린다
#   out_dir 를 비우면(기본값) 씬은 임시 디렉토리에 썼다가 요청 끝나면 지워진다
#   (grasp_bridge_node.py:236-241). 여기 지정한 out_dir/scene(기본 "00") 아래
#   depth.npy/rgb.png/seg.png/meta_data.json 4파일이 남는다(data/ 는 .gitignore 처리됨).
#   씬마다 남기려면 매 호출 전에 -p scene:=01 처럼 바꿔 부른다 — 같은 scene 이면 덮어쓴다.

# ── T5. 인식만 먼저 단독으로 확인 (로봇 안 움직임) ⭐ ────────
export ROS_DOMAIN_ID=93 && source install/setup.bash
ros2 service call /grasp/compute std_srvs/srv/Trigger {}
#   → T4 로그에 "라벨별 후보 개수 → 점수/도달/접근축 통과 → 손끝≈(x,y,z)"가 찍힌다
ros2 topic echo /grasp/best_tcp --once     # 손끝 좌표를 자로 잰 물체 위치와 대조

# ── T6. FSM — 실제로 움직임 ─────────────────────────────────
export ROS_DOMAIN_ID=93 && source install/setup.bash
ros2 launch pick_fsm pick_fsm.launch.py \
  grasp_source:=legacy_trigger voice:=false target:=apple
#   🔴 dry_run 인자는 2026-08-09 제거됐다. 붙여도 경고 없이 무시되고 로봇은 움직인다

# 조작 (다른 터미널)
ros2 topic echo /pick/state &                              # 상태 감시
ros2 service call /pick/start   std_srvs/srv/Trigger {}    # 시작
#   → PERCEIVE → SCENE_PREP → PLAN → WAIT_APPROVAL 에서 멈춘다
ros2 service call /pick/approve std_srvs/srv/Trigger {}    # ✋ 여기서 로봇이 움직인다
ros2 service call /safety/stop  std_srvs/srv/Trigger {}    # 즉시 정지
