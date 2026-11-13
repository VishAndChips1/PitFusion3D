"""Launch the web viewer server (fusion_web_streamer) for the fused point clouds.

Open http://<this-machine>:8080/ in any browser on the network.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'landfill_camera_sim'


def generate_launch_description():
    """Declare the launch arguments and the streamer node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'merged_topic',
            default_value='/landfill/fusion/points_merged',
            description='Dense windowed-merge PointCloud2 (x,y,z,rgb,thermal) topic.',
        ),
        DeclareLaunchArgument(
            'live_topic',
            default_value='/landfill/fusion/points_rgbt',
            description='Faster per-scan fused PointCloud2 (x,y,z,rgb,thermal) topic.',
        ),
        DeclareLaunchArgument('http_port', default_value='8080'),
        # Point caps per served cloud (see fusion_web_streamer.py).
        DeclareLaunchArgument('max_points_merged', default_value='20000'),
        DeclareLaunchArgument('max_points_live', default_value='5000'),

        Node(
            package=PACKAGE_NAME,
            executable='fusion_web_streamer.py',
            name='fusion_web_streamer',
            respawn=True,
            respawn_delay=2.0,
            output='screen',
            parameters=[{
                'merged_topic': LaunchConfiguration('merged_topic'),
                'live_topic': LaunchConfiguration('live_topic'),
                'http_port': LaunchConfiguration('http_port'),
                'max_points_merged': LaunchConfiguration('max_points_merged'),
                'max_points_live': LaunchConfiguration('max_points_live'),
            }],
        ),
    ])
