"""Launch the fusion pipeline: colour the LiDAR cloud (sensor_fusion_node) and merge a rolling window (registration_merge_node).

    ros2 launch landfill_camera_sim sensor_fusion.launch.py

Override the *_topic arguments to fuse a different set of topics.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'landfill_camera_sim'


def generate_launch_description():
    """Declare the launch arguments and the two fusion nodes."""
    return LaunchDescription([
        DeclareLaunchArgument('lidar_points_topic', default_value='/landfill/livox/points'),
        DeclareLaunchArgument('rgb_image_topic', default_value='/landfill/rgb/image_raw'),
        DeclareLaunchArgument('thermal_image_topic', default_value='/landfill/thermal/image_raw'),
        DeclareLaunchArgument('fused_points_topic', default_value='/landfill/fusion/points_rgbt'),
        DeclareLaunchArgument('merged_points_topic', default_value='/landfill/fusion/points_merged'),
        DeclareLaunchArgument('thermal_min_temp', default_value='253.15'),
        DeclareLaunchArgument('thermal_resolution', default_value='0.01'),
        DeclareLaunchArgument('sync_slop', default_value='0.15'),
        DeclareLaunchArgument(
            'window_duration',
            default_value='1.0',
            description='Length, in seconds, of the scan window merged by registration_merge_node.',
        ),
        DeclareLaunchArgument(
            'output_voxel_size',
            default_value='0.01',
            description='Voxel size to decimate the merged cloud by; 0 keeps every point from every scan.',
        ),

        Node(
            package=PACKAGE_NAME,
            executable='sensor_fusion_node.py',
            name='sensor_fusion_node',
            respawn=True,
            respawn_delay=2.0,
            output='screen',
            parameters=[{
                'lidar_points_topic': LaunchConfiguration('lidar_points_topic'),
                'rgb_image_topic': LaunchConfiguration('rgb_image_topic'),
                'thermal_image_topic': LaunchConfiguration('thermal_image_topic'),
                'output_topic': LaunchConfiguration('fused_points_topic'),
                'thermal_min_temp': LaunchConfiguration('thermal_min_temp'),
                'thermal_resolution': LaunchConfiguration('thermal_resolution'),
                'sync_slop': LaunchConfiguration('sync_slop'),
            }],
        ),
        Node(
            package=PACKAGE_NAME,
            executable='registration_merge_node.py',
            name='registration_merge_node',
            respawn=True,
            respawn_delay=2.0,
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('fused_points_topic'),
                'output_topic': LaunchConfiguration('merged_points_topic'),
                'window_duration': LaunchConfiguration('window_duration'),
                'output_voxel_size': LaunchConfiguration('output_voxel_size'),
            }],
        ),
    ])
