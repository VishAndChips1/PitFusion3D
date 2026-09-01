cmake_minimum_required(VERSION 3.8)
project(landfill_camera_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/platform_motion_controller.py
    scripts/livox_pattern_filter.py
    scripts/calibration_publisher.py
    scripts/sensor_fusion_node.py
    scripts/registration_merge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
