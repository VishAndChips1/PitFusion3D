"""Bring up the physical sensors: Livox Avia, Raspberry Pi RGB camera and UVC thermal camera.

Publishes the topics and frames that sensor_fusion.launch.py and
web_stream.launch.py (landfill_camera_sim) expect:

    ros2 launch pitfusion3d_bringup real_sensors.launch.py
    ros2 launch landfill_camera_sim sensor_fusion.launch.py
    ros2 launch landfill_camera_sim web_stream.launch.py

The Livox driver's output is re-stamped into the host clock by
livox_restamp_node before it reaches /landfill/livox/points.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Describe the sensor nodes, with the camera nodes started a little after the LiDAR."""
    # One BLAS/OpenMP/OpenCV thread per process (the nodes share 4 cores).
    limit_threads = [
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('MKL_NUM_THREADS', '1'),
        SetEnvironmentVariable('NUMEXPR_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENCV_NUM_THREADS', '1'),
    ]

    # All nodes run on this machine: keep DDS discovery on localhost, independent of the network.
    localhost_only_discovery = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros2_avia'), 'config', 'livox_lidar_config.json'
    )
    # Fall back to the source tree if the config is not in the install share directory.
    if not os.path.isfile(livox_config_path):
        livox_config_path = os.path.join(
            get_package_share_directory('livox_ros2_avia'), '..', '..', '..', '..',
            'src', 'livox_ros2_avia', 'livox_ros2_avia', 'config', 'livox_lidar_config.json',
        )

    livox_driver = Node(
        package='livox_ros2_avia',
        executable='livox_ros2_avia_node',
        name='livox_lidar_publisher',
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[{
            'topic': '/landfill/rgb/image_raw',
            'frame_id': 'camera_platform/rgb_optical_frame',
            'width': 1280,
            'height': 720,
            'framerate': 10,  # the LiDAR runs at 10 Hz, so faster frames are never fused
        }],
    )

    thermal_camera = Node(
        package='pitfusion3d_bringup',
        executable='thermal_camera_node',
        name='thermal_camera_node',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[{
            # 'device' is omitted on purpose: the node defaults to its stable /dev/v4l/by-id path.
            'topic': '/landfill/thermal/image_raw',
            'frame_id': 'camera_platform/thermal_optical_frame',
            'width': 640,
            'height': 512,
            'framerate': 30,  # the camera only offers 30 or 60 fps
        }],
    )

    calibration_publisher = Node(
        package='landfill_camera_sim',
        executable='calibration_publisher.py',
        name='calibration_publisher',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[{
            'image_frame_id': 'camera_platform/camera_link',
            'rgb_camera_info_topic': '/landfill/rgb/camera_info',
            'thermal_camera_info_topic': '/landfill/thermal/camera_info',
            # Static data from YAML: a slow republish rate is enough.
            'publish_rate': 2.0,
        }],
    )

    # Staggered start: spreads the start-up CPU burst of each node over a few seconds.
    rgb_camera_delayed = TimerAction(period=1.0, actions=[rgb_camera])
    thermal_camera_delayed = TimerAction(period=2.0, actions=[thermal_camera])

    return LaunchDescription([
        localhost_only_discovery,
        *limit_threads,
        livox_driver,
        livox_restamp,
        rgb_camera_delayed,
        thermal_camera_delayed,
        calibration_publisher,
    ])
