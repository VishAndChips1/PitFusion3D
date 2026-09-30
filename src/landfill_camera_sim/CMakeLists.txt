cmake_minimum_required(VERSION 3.8)
project(landfill_camera_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/cloud_io.py
    scripts/platform_motion_controller.py
    scripts/scene_traffic_controller.py
    scripts/livox_pattern_filter.py
    scripts/calibration_publisher.py
    scripts/sensor_fusion_node.py
    scripts/registration_merge_node.py
    scripts/fusion_web_streamer.py
  DESTINATION lib/${PROJECT_NAME}
)

# fusion_web_streamer.py resolves this relative to its own file location
# (lib/${PROJECT_NAME}/web_assets), not share/ -- keep this DESTINATION
# matching that if it ever changes. Bundling three.js/OrbitControls here
# instead of loading them from a CDN means the viewer page works with no
# internet access at all, which matters on a robot deployed behind its
# own standalone WiFi hotspot (see pitfusion3d_bringup).
install(
  DIRECTORY scripts/web_assets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
