"""Bring up the physical sensor rig: Livox Avia + RPi RGB camera + UVC thermal camera.

This replaces landfill_camera_sim's Gazebo scenario launches
(landfill_camera_sim.launch.py / scan_test_sim.launch.py) for real
hardware -- it publishes the same topics/frames those simulated sensors
did, so sensor_fusion.launch.py and web_stream.launch.py from
landfill_camera_sim run against it unmodified.

    ros2 launch pitfusion3d_bringup real_sensors.launch.py
    ros2 launch landfill_camera_sim sensor_fusion.launch.py
    ros2 launch landfill_camera_sim web_stream.launch.py

The Livox driver is launched here (rather than reusing
livox_ros2_avia's own launch file) purely to set its frame_id to what
sensor_fusion_node.py expects by default (camera_platform/lidar_frame)
-- the underlying node/config (livox_ros2_avia_node,
config/livox_lidar_config.json) is untouched.

Its raw `livox/lidar` output has its timestamp epoch corrected by
livox_restamp_node before reaching /landfill/livox/points -- see that
node's docstring: the driver stamps clouds with the lidar's own internal
clock (timesync is disabled in livox_lidar_config.json), which never
lines up with the camera nodes' host-clock stamps inside
sensor_fusion_node's sync slop otherwise.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Redundant with pitfusion3d_full.launch.py's copy of this when
    # launched through it (SetEnvironmentVariable is idempotent) --
    # kept here too so `ros2 launch pitfusion3d_bringup
    # real_sensors.launch.py` alone still avoids each OpenCV/numpy-using
    # node spawning its own core-count-sized thread pool. See that
    # launch file for the full rationale.
    limit_threads = [
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('MKL_NUM_THREADS', '1'),
        SetEnvironmentVariable('NUMEXPR_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENCV_NUM_THREADS', '1'),
    ]

    # Every node here runs on this one machine, so DDS discovery has no
    # business depending on wlan0's subnet (the environment default,
    # ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, makes it do exactly that --
    # see pitfusion3d_full.launch.py for the full incident this fixes).
    localhost_only_discovery = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros2_avia'), 'config', 'livox_lidar_config.json'
    )
    # livox_ros2_avia only installs launch/ + package assets by default;
    # config is looked up relative to the install share dir the same way
    # its own launch file does. Fall back to the source-tree path if the
    # share-dir config isn't present (symlink-install workspaces).
    if not os.path.isfile(livox_config_path):
        livox_config_path = os.path.join(
            get_package_share_directory('livox_ros2_avia'), '..', '..', '..', '..',
            'src', 'livox_ros2_avia', 'livox_ros2_avia', 'config', 'livox_lidar_config.json',
        )

    livox_driver = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,          # PointCloud2 (matches sensor_fusion_node's XYZI reader)
            'multi_topic': 0,
            'data_src': 0,             # 0 = live lidar, not an .lvx file
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'camera_platform/lidar_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_config_path,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    livox_restamp = Node(
        package='pitfusion3d_bringup',
        executable='livox_restamp_node',
        name='livox_restamp_node',
        output='screen',
        parameters=[{
            'input_topic': 'livox/lidar',
            'output_topic': '/landfill/livox/points',
        }],
    )

    rgb_camera = Node(
        package='pitfusion3d_bringup',
        executable='rgb_camera_node',
        name='rgb_camera_node',
        output='screen',
        parameters=[{
            'topic': '/landfill/rgb/image_raw',
            'frame_id': 'camera_platform/rgb_optical_frame',
            'width': 1280,
            'height': 720,
            'framerate': 15,
        }],
    )

    thermal_camera = Node(
        package='pitfusion3d_bringup',
        executable='thermal_camera_node',
        name='thermal_camera_node',
        output='screen',
        parameters=[{
            # 'device' deliberately omitted -- falls back to
            # thermal_camera_node's own by-id default, which survives
            # /dev/videoN renumbering across reboots (see that node's
            # docstring).
            'topic': '/landfill/thermal/image_raw',
            'frame_id': 'camera_platform/thermal_optical_frame',
            'width': 640,
            'height': 512,
            'framerate': 30,  # hardware only offers 30/60fps -- see thermal_camera_node.py
        }],
    )

    calibration_publisher = Node(
        package='landfill_camera_sim',
        executable='calibration_publisher.py',
        name='calibration_publisher',
        output='screen',
        parameters=[{
            'image_frame_id': 'camera_platform/camera_link',
            'rgb_camera_info_topic': '/landfill/rgb/camera_info',
            'thermal_camera_info_topic': '/landfill/thermal/camera_info',
            # This data is loaded once from YAML at startup and never
            # changes -- republishing it at the 30Hz default is pure
            # waste. 2Hz is still instant for any RViz/rqt display that
            # comes up expecting it soon, without a timer firing 15x more
            # often than the data can possibly differ.
            'publish_rate': 2.0,
        }],
    )

    return LaunchDescription([
        localhost_only_discovery,
        *limit_threads,
        livox_driver,
        livox_restamp,
        rgb_camera,
        thermal_camera,
        calibration_publisher,
    ])
