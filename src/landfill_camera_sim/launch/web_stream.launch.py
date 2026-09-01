from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Run alongside sensor_fusion.launch.py to view the fused/merged cloud
# from any browser on the network:
#
#   ros2 launch landfill_camera_sim landfill_camera_sim.launch.py
#   ros2 launch landfill_camera_sim sensor_fusion.launch.py
#   ros2 launch landfill_camera_sim web_stream.launch.py
#
# then open http://<this-machine-IP>:8080/ -- see the package README for
# the WSL2 note on reaching that from another device on the WiFi network.
PACKAGE_NAME = 'landfill_camera_sim'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/landfill/fusion/points_merged',
            description='PointCloud2 (x,y,z,rgb,thermal) topic to stream.',
        ),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('max_points', default_value='20000'),

        Node(
            package=PACKAGE_NAME,
            executable='fusion_web_streamer.py',
            name='fusion_web_streamer',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'http_port': LaunchConfiguration('http_port'),
                'max_points': LaunchConfiguration('max_points'),
            }],
        ),
    ])
