# 터미널 1 — 컨테이너 (프로파일 없으면 프레임 0장)
docker start od_kimkh
docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml   od_kimkh bash -lc 'source /opt/ros/humble/setup.bash
    source /home/kimkh/cobot2_ws/install/setup.bash
    ros2 launch graspgenx_perception graspx.launch.py \ 
      run_bridge:=false device:=0 publish_overlay:=true classes:="[39,41,44,46,47,49,64]"'


docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml   od_kimkh bash -lc 'source /opt/ros/humble/setup.bash
    source /home/kimkh/cobot2_ws/install/setup.bash
    ros2 launch graspgenx_perception graspx.launch.py scripts/graspx_container.sh scripts/graspx_container.sh run_bridge:=false device:=0 classes:='[39,40,41,42,43,44,45,46,47,49,64,73,76]'



# 터미널 2 — 호스트
export ROS_DOMAIN_ID=93
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false seg_source:=yolo



root@rokey:/home/kimkh/cobot2_ws# python3 -c "from ultralytics import YOLO; print(YOLO('yolov8n-seg.pt').names)"
{39: 'bottle'
41: 'cup'
44: 'spoon'
46: 'banana
47: 'apple'
49: 'orange'
64: 'mouse'
67: 'cell phone'
73: 'book', 74: 'clock'
76: 'scissors'
79: 'toothbrush'}


