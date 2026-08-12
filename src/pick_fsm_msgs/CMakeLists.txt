cmake_minimum_required(VERSION 3.8)
project(pick_fsm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeGrasp.srv"
  # 🔴 서버 없음 — 정의만 (2026-08-12, 손목 seam). 파일 상단 주석 참고.
  "srv/AcquireTarget.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
