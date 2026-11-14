source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=$(find ~/ros2_ws/src -name fastdds_large_buffers.xml | head -1)
export PYTHONPATH=$HOME/ros2_ws/src/landfill_camera_sim/scripts:$PYTHONPATH
