"""Launch the whole real-hardware PitFusion3D pipeline.

  Livox Avia + RPi RGB camera + thermal camera
    -> sensor_fusion_node (colorize + thermal-tag each lidar point)
    -> registration_merge_node (windowed ICP merge)
    -> fusion_web_streamer (http://<this-machine>:8080/)

    ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py

The thermal field is the camera's relative 0-255 value, not Kelvin (see thermal_camera_node.py).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Describe the full pipeline: sensors, then fusion/merge, then the web viewer server."""
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

    # Fast DDS profile with large socket buffers, so multi-MB images are not dropped on loopback.
    dds_large_buffers = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        PathJoinSubstitution([
            FindPackageShare('pitfusion3d_bringup'), 'config', 'fastdds_large_buffers.xml',
        ]),
    )

    real_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('pitfusion3d_bringup'), 'launch', 'real_sensors.launch.py']
            )
        ),
    )

    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('landfill_camera_sim'), 'launch', 'sensor_fusion.launch.py']
            )
        ),
        launch_arguments={
            'thermal_min_temp': '0.0',
            'thermal_resolution': '0.00390625',  # 1/256: recovers the raw 0-255 Y value
            # Shorter merge window than the 1.0 s default: faster refresh, fewer scans per ICP window.
            'window_duration': '0.5',
        }.items(),
    )

    web_stream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('landfill_camera_sim'), 'launch', 'web_stream.launch.py']
            )
        ),
    )

    # Staggered start spreads the start-up CPU/current burst over several seconds.
    sensor_fusion_delayed = TimerAction(period=3.0, actions=[sensor_fusion])
    web_stream_delayed = TimerAction(period=6.0, actions=[web_stream])

    return LaunchDescription([
        localhost_only_discovery,
        dds_large_buffers,
        *limit_threads,
        real_sensors,
        sensor_fusion_delayed,
        web_stream_delayed,
    ])
