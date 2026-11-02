"""One-shot launch for the whole real-hardware PitFusion3D pipeline:

  Livox Avia + RPi RGB camera + thermal camera
    -> sensor_fusion_node (colorize + thermal-tag each lidar point)
    -> registration_merge_node (windowed ICP merge)
    -> fusion_web_streamer (http://<this-machine>:8080/)

    ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py

thermal_min_temp/thermal_resolution are set so the fused cloud's
'thermal' field comes back as the thermal camera's raw 0-255 AGC value
(see thermal_camera_node.py's docstring) -- treat it as a relative
hot/cold signal, not literal Kelvin, until a radiometric thermal source
is available.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Every node here that imports numpy/OpenCV/scipy otherwise starts its
    # own BLAS/OpenMP/OpenCV thread pool sized to the core count (4 on
    # this Pi) *independently* -- with 8 such processes that's severe
    # oversubscription, measured as high CPU% with declining throughput
    # under load rather than the expected linear cost. Force each library
    # to a single thread per process instead; each node's own per-message
    # work here is too small for intra-op parallelism to pay for its own
    # coordination overhead anyway.
    limit_threads = [
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('MKL_NUM_THREADS', '1'),
        SetEnvironmentVariable('NUMEXPR_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENCV_NUM_THREADS', '1'),
    ]

    # Every node in this pipeline runs on this one machine -- there is no
    # other host to discover. The environment ships with
    # ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, which makes DDS discovery (and
    # the multicast it uses) depend on wlan0's current subnet. That subnet
    # changes completely when wlan0 flips from a normal WiFi client
    # (192.168.1.0/24) to the PitFusion3D-AP hotspot (10.42.0.0/24) --
    # confirmed live: topics kept publishing individually, but the web
    # streamer's subscriptions went silent (data in, nothing out) right
    # after that switch, even after a full pipeline restart on the new
    # network. Pinning discovery to localhost makes every node find every
    # other node over 127.0.0.1 regardless of what wlan0 is doing --
    # client, AP, or unplugged.
    localhost_only_discovery = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    # Root-cause fix for large topics (RGB images ~2.64MB/frame, also
    # relevant to thermal frames and point clouds) measuring a fraction
    # of their genuine publish rate on the subscriber side. Confirmed via
    # direct per-stage timing instrumentation that rgb_camera_node
    # publishes a healthy, correct 15Hz internally; an external
    # subscriber on the same machine was still only seeing ~4-6Hz. Root
    # cause: this Pi's kernel socket buffer ceiling
    # (net.core.rmem_max/wmem_max) measured at only ~208KB via sysctl --
    # far too small to hold the dozens of UDP fragments a multi-MB
    # message needs, so under any burst from the rest of this pipeline
    # sharing the same loopback DDS transport, fragments get silently
    # dropped before reassembly (BEST_EFFORT doesn't retry). This profile
    # requests much larger buffers from FastDDS; it is necessary but not
    # sufficient on its own -- the kernel ceiling also needs raising (see
    # config/fastdds_large_buffers.xml's own header comment for the
    # sysctl command), since a request larger than the kernel allows is
    # silently capped back down.
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
            # Halved from the 1.0s default: roughly doubles the merged
            # cloud's refresh rate and roughly halves the number of scans
            # registration_merge_node has to ICP-register per window,
            # which also cuts the per-cycle compute-time variance that
            # was showing up as jitter in the publish cadence.
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

    # All 8 node processes used to start at once: every one of them
    # imports numpy/OpenCV/rclpy and stands up a DDS participant in the
    # same ~1-2s window, which measured as a brief ~8.9A spike on the
    # VDD_CORE rail alone during a cold launch -- on a weak supply (e.g.
    # the Anker 737's native 5V/3A rail, before the PD board) this is
    # enough on its own, with no viewer/streaming load at all, to trip
    # under-voltage and crash the Pi (confirmed: measured EXT5V sag to
    # 4.72V and a real reboot during one such test). Staggering node
    # startup spreads that import/init burst over several seconds instead
    # of concentrating it in one spike -- same total work, lower peak.
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
