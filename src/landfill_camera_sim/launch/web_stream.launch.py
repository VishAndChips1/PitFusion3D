from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Run alongside sensor_fusion.launch.py to view the fused cloud from any
# browser on the network:
#
#   ros2 launch landfill_camera_sim landfill_camera_sim.launch.py
#   ros2 launch landfill_camera_sim sensor_fusion.launch.py
#   ros2 launch landfill_camera_sim web_stream.launch.py
#
# then open http://<this-machine-IP>:8080/ -- see the package README for
# the WSL2 note on reaching that from another device on the WiFi network.
#
# The page serves both the dense ~1Hz windowed-merge cloud (default view)
# and the faster ~10Hz per-scan fused cloud, with a one-click toggle
# between them -- see fusion_web_streamer.py.
PACKAGE_NAME = 'landfill_camera_sim'


def generate_launch_description():
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
        # 35000/10000 were the original values; measured real-WiFi power
        # testing (tools/power_monitoring/burst_monitor.py) found
        # 35000pts peaking at 6.29A on the CPU core rail -- over this
        # project's hard 5A ceiling (set to avoid tripping a USB-C
        # battery bank's overcurrent protection, a full unrecoverable
        # power loss, not a graceful shutdown). These are the
        # launch-time-overridable defaults; fusion_web_streamer.py's own
        # declare_parameter defaults (20000/5000) were updated to match
        # at the same time, so both sources of truth now agree -- see
        # that file's comment for the full measured reasoning, including
        # why this needs re-validating with real WiFi viewing (not
        # loopback) and a thermally-settled Pi if it's ever raised again.
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
