다른 터미널에서 접속 (attach)

방법 1 — run_dev.sh 재실행 (권장)

bash
cd ~/cobot2_ws/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh

이미 컨테이너가 실행 중이면 새로 빌드하지 않고 바로 새 bash 세션을 열어줍니다. 여러 터미널에서 동시에 같은 컨테이너 안에 각각 셸을 띄우고 싶을 때(예: 한쪽은 ros2 launch, 다른 쪽은 ros2 topic echo) 이 방법이 제일 편합니다.

방법 2 — docker exec 직접 사용
bash
sudo docker exec -it isaac_ros_dev-x86_64-container bash

컨테이너 이름을 알고 있으니 이렇게 바로 붙어도 됩니다. run_dev.sh가 내부적으로 하는 것도 결국 이 방식입니다.

끄기 / 켜기

중지 (컨테이너는 유지, 프로세스만 정지)
bash
sudo docker stop isaac_ros_dev-x86_64-container

재시작 (멈춘 컨테이너를 다시 켬 — 내부 상태/설치한 패키지 유지됨)
bash
sudo docker start isaac_ros_dev-x86_64-container
# 켠 뒤 셸 붙이기
sudo docker exec -it isaac_ros_dev-x86_64-container bash

sha256sum -c isaac_ros_dev-x86_64.tar.gz.sha256   # 무결성 확인
gunzip -c isaac_ros_dev-x86_64.tar.gz | docker load  # 또는: docker load -i <(pigz -dc isaac_ros_dev-x86_64.tar.gz)



export ROS_DOMAIN_ID=93
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link \
  -p use_lidar:=false \
  -p num_cameras:=1 \
  -p static_mapper.esdf_slice_min_height:=-0.3 \
  -p static_mapper.esdf_slice_max_height:=0.5 \
  -p static_mapper.esdf_slice_height:=0.0 \
  -r camera_0/depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info


pip3 install 'warp-lang==1.5.0'   # 새 컨테이너라 매번 유실됨 — 먼저 재설치
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True


yolo segment predict model=src/object_detection/resource/yolo11n-seg.pt source=0 show=True
