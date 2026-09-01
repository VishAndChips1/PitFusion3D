import os
import math
from pathlib import Path
from xml.sax.saxutils import escape as xml_escape

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = 'landfill_camera_sim'


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _as_float_list(value, length, name):
    if value is None:
        raise ValueError(f'Missing required list: {name}')
    if len(value) != length:
        raise ValueError(f'{name} must contain {length} values')
    return [float(item) for item in value]


def _camera_info_topic(image_topic):
    normalized = _normalize_topic(image_topic).rstrip('/')
    parent = normalized.rsplit('/', 1)[0]
    return f'{parent}/camera_info' if parent else '/camera_info'


def _normalize_topic(topic):
    topic = str(topic).strip()
    if not topic:
        return topic
    return topic if topic.startswith('/') else f'/{topic}'


def _positive_float(value, name):
    value = float(value)
    if value <= 0.0:
        raise ValueError(f'{name} must be greater than zero')
    return value


def _non_negative_float(value, name):
    value = float(value)
    if value < 0.0:
        raise ValueError(f'{name} must be zero or greater')
    return value


def _positive_int(value, name):
    value = int(value)
    if value <= 0:
        raise ValueError(f'{name} must be greater than zero')
    return value


def _angle_limits_from_degrees(fov_degrees, name):
    fov = _positive_float(fov_degrees, name)
    half_angle = math.radians(fov) / 2.0
    return -half_angle, half_angle


def _read_yaml(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def _render_template(template_path, output_path, replacements):
    with open(template_path, 'r', encoding='utf-8') as template_file:
        text = template_file.read()
    for key, value in replacements.items():
        text = text.replace(f'__{key}__', str(value))
    with open(output_path, 'w', encoding='utf-8') as output_file:
        output_file.write(text)


def _xml(value):
    return xml_escape(str(value), {'"': '&quot;', "'": '&apos;'})


def _setup(context):
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    config_arg = LaunchConfiguration('config_file').perform(context)
    config_path = Path(os.path.expanduser(config_arg))
    if not config_path.is_absolute():
        config_path = Path.cwd() / config_path

    config = _read_yaml(config_path)

    world_cfg = config.get('world', {})
    sensors_cfg = config.get('sensors', {})
    rgb_cfg = sensors_cfg.get('rgb', {})
    thermal_cfg = sensors_cfg.get('thermal', {})
    lidar_cfg = sensors_cfg.get('lidar', {})
    rgb_image_cfg = rgb_cfg.get('image', {})
    thermal_image_cfg = thermal_cfg.get('image', {})
    clip_cfg = sensors_cfg.get('clip', {})
    pose_cfg = sensors_cfg.get('pose', {})
    atmosphere_cfg = world_cfg.get('atmosphere', {})
    topic_cfg = config.get('topics', {})
    input_topics = topic_cfg.get('input', {})
    output_topics = topic_cfg.get('output', {})
    platform_cfg = config.get('platform', {})
    motion_cfg = platform_cfg.get('motion', {})
    rviz_cfg = config.get('rviz', {})

    world_name = str(world_cfg.get('name', 'landfill_camera_world')).strip()
    rgb_camera_name = str(rgb_cfg.get('name', 'landfill_rgb_camera')).strip()
    thermal_camera_name = str(thermal_cfg.get('name', 'landfill_thermal_camera')).strip()
    lidar_name = str(lidar_cfg.get('name', 'landfill_livox_avia_lidar')).strip()
    frame_id = str(sensors_cfg.get('frame_id', 'camera_platform/camera_link')).strip()

    xyz = _as_float_list(pose_cfg.get('xyz', [-8.0, -6.0, 3.2]), 3, 'sensors.pose.xyz')
    rpy = _as_float_list(pose_cfg.get('rpy', [0.0, 0.28, 0.64]), 3, 'sensors.pose.rpy')
    horizontal_fov = _positive_float(sensors_cfg.get('horizontal_fov', 1.047), 'sensors.horizontal_fov')
    update_rate = _positive_float(sensors_cfg.get('update_rate', 30.0), 'sensors.update_rate')
    rgb_width = _positive_int(rgb_image_cfg.get('width', 1280), 'sensors.rgb.image.width')
    rgb_height = _positive_int(rgb_image_cfg.get('height', 720), 'sensors.rgb.image.height')
    rgb_image_format = str(rgb_image_cfg.get('format', 'R8G8B8')).strip()
    thermal_width = _positive_int(
        thermal_image_cfg.get('width', rgb_width),
        'sensors.thermal.image.width',
    )
    thermal_height = _positive_int(
        thermal_image_cfg.get('height', rgb_height),
        'sensors.thermal.image.height',
    )
    thermal_image_format = str(thermal_image_cfg.get('format', 'L16')).strip()
    near_clip = _positive_float(clip_cfg.get('near', 0.05), 'sensors.clip.near')
    far_clip = _positive_float(clip_cfg.get('far', 120.0), 'sensors.clip.far')
    if near_clip >= far_clip:
        raise ValueError('sensors.clip.near must be less than sensors.clip.far')
    thermal_temperature_cfg = thermal_cfg.get('temperature', {})
    thermal_min_temp = float(thermal_temperature_cfg.get('min', 253.15))
    thermal_max_temp = float(thermal_temperature_cfg.get('max', 673.15))
    thermal_resolution = _positive_float(
        thermal_temperature_cfg.get('resolution', 0.01),
        'sensors.thermal.temperature.resolution',
    )
    if thermal_min_temp >= thermal_max_temp:
        raise ValueError('sensors.thermal.temperature.min must be less than max')
    lidar_update_rate = _positive_float(
        lidar_cfg.get('update_rate', update_rate),
        'sensors.lidar.update_rate',
    )
    lidar_horizontal_min, lidar_horizontal_max = _angle_limits_from_degrees(
        lidar_cfg.get('horizontal_fov_deg', 70.4),
        'sensors.lidar.horizontal_fov_deg',
    )
    lidar_vertical_min, lidar_vertical_max = _angle_limits_from_degrees(
        lidar_cfg.get('vertical_fov_deg', 77.2),
        'sensors.lidar.vertical_fov_deg',
    )
    lidar_horizontal_samples = _positive_int(
        lidar_cfg.get('horizontal_samples', 200),
        'sensors.lidar.horizontal_samples',
    )
    lidar_vertical_samples = _positive_int(
        lidar_cfg.get('vertical_samples', 120),
        'sensors.lidar.vertical_samples',
    )
    lidar_downsample = _positive_int(
        lidar_cfg.get('downsample', 1),
        'sensors.lidar.downsample',
    )
    lidar_points_per_scan = _positive_int(
        lidar_cfg.get('points_per_scan', lidar_horizontal_samples * lidar_vertical_samples),
        'sensors.lidar.points_per_scan',
    )
    lidar_pattern_cfg = lidar_cfg.get('pattern_node', {})
    lidar_pattern_enabled = _as_bool(lidar_pattern_cfg.get('enabled', True))
    lidar_pattern_csv = str(lidar_pattern_cfg.get('scan_pattern_csv', '')).strip()
    lidar_range_cfg = lidar_cfg.get('range', {})
    lidar_min_range = _positive_float(lidar_range_cfg.get('min', 0.1), 'sensors.lidar.range.min')
    lidar_max_range = _positive_float(lidar_range_cfg.get('max', 200.0), 'sensors.lidar.range.max')
    lidar_range_resolution = _positive_float(
        lidar_range_cfg.get('resolution', 0.01),
        'sensors.lidar.range.resolution',
    )
    if lidar_min_range >= lidar_max_range:
        raise ValueError('sensors.lidar.range.min must be less than max')
    atmosphere_temperature = float(atmosphere_cfg.get('temperature', 300.0))
    atmosphere_temperature_gradient = float(atmosphere_cfg.get('temperature_gradient', 0.1))

    gz_rgb_image_topic = _normalize_topic(
        input_topics.get('rgb_image', input_topics.get('image', '/landfill/rgb/image_raw'))
    )
    gz_rgb_camera_info_topic = _normalize_topic(
        input_topics.get('rgb_camera_info', input_topics.get('camera_info')) or
        _camera_info_topic(gz_rgb_image_topic)
    )
    gz_thermal_image_topic = _normalize_topic(
        input_topics.get('thermal_image', '/landfill/thermal/image_raw')
    )
    gz_thermal_camera_info_topic = _normalize_topic(
        input_topics.get('thermal_camera_info') or _camera_info_topic(gz_thermal_image_topic)
    )
    gz_lidar_scan_topic = _normalize_topic(
        input_topics.get('lidar_scan', '/landfill/livox/scan')
    )
    gz_clock_topic = _normalize_topic(
        input_topics.get('clock') or f'/world/{world_name}/clock'
    )
    ros_rgb_image_topic = _normalize_topic(
        output_topics.get('rgb_image', output_topics.get('image', gz_rgb_image_topic))
    )
    ros_rgb_camera_info_topic = _normalize_topic(
        output_topics.get('rgb_camera_info', output_topics.get('camera_info')) or
        _camera_info_topic(ros_rgb_image_topic)
    )
    ros_thermal_image_topic = _normalize_topic(
        output_topics.get('thermal_image', gz_thermal_image_topic)
    )
    ros_thermal_camera_info_topic = _normalize_topic(
        output_topics.get('thermal_camera_info') or _camera_info_topic(ros_thermal_image_topic)
    )
    ros_lidar_scan_topic = _normalize_topic(
        output_topics.get('lidar_scan', gz_lidar_scan_topic)
    )
    ros_lidar_points_topic = _normalize_topic(
        output_topics.get('lidar_points', '/landfill/livox/points')
    )
    ros_clock_topic = _normalize_topic(output_topics.get('clock', '/clock'))
    platform_ros_cmd_topic = _normalize_topic(
        motion_cfg.get('ros_cmd_topic', '/landfill/platform/cmd_vel')
    )
    platform_gz_cmd_topic = _normalize_topic(
        motion_cfg.get('gz_cmd_topic', '/model/camera_platform/cmd_vel')
    )
    fixed_frame = str(rviz_cfg.get('fixed_frame', frame_id)).strip()

    motion_enabled = _as_bool(motion_cfg.get('enabled', True))
    motion_axis = _as_float_list(motion_cfg.get('axis', [0.0, 1.0, 0.0]), 3, 'platform.motion.axis')
    motion_travel_distance = _non_negative_float(
        motion_cfg.get('travel_distance', 8.0),
        'platform.motion.travel_distance',
    )
    motion_linear_speed = _non_negative_float(
        motion_cfg.get('linear_speed', 0.35),
        'platform.motion.linear_speed',
    )
    motion_angular_speed = float(motion_cfg.get('angular_speed', 0.18))
    motion_update_rate = _positive_float(
        motion_cfg.get('update_rate', 20.0),
        'platform.motion.update_rate',
    )

    generated_dir = Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / PACKAGE_NAME
    generated_dir.mkdir(parents=True, exist_ok=True)
    world_path = generated_dir / 'generated_landfill_sensors_world.sdf'
    bridge_path = generated_dir / 'generated_landfill_sensors_bridge.yaml'
    rviz_path = generated_dir / 'generated_landfill_sensors.rviz'
    gui_config_path = package_share / 'config' / 'landfill_gui.config'

    deck_height = max(0.35, xyz[2] - 0.7)
    leg_height = max(0.25, deck_height)
    rail_height = deck_height + 0.55
    mast_height = max(0.3, xyz[2] - deck_height)

    _render_template(
        package_share / 'worlds' / 'landfill_camera_world.sdf.in',
        world_path,
        {
            'WORLD_NAME': _xml(world_name),
            'ATMOSPHERE_TEMPERATURE': f'{atmosphere_temperature:.6f}',
            'ATMOSPHERE_TEMPERATURE_GRADIENT': f'{atmosphere_temperature_gradient:.6f}',
            'RGB_CAMERA_NAME': _xml(rgb_camera_name),
            'THERMAL_CAMERA_NAME': _xml(thermal_camera_name),
            'LIDAR_NAME': _xml(lidar_name),
            'CAMERA_FRAME_ID': _xml(frame_id),
            'CAMERA_X': f'{xyz[0]:.6f}',
            'CAMERA_Y': f'{xyz[1]:.6f}',
            'CAMERA_Z': f'{xyz[2]:.6f}',
            'CAMERA_ROLL': f'{rpy[0]:.6f}',
            'CAMERA_PITCH': f'{rpy[1]:.6f}',
            'CAMERA_YAW': f'{rpy[2]:.6f}',
            'CAMERA_HORIZONTAL_FOV': f'{horizontal_fov:.6f}',
            'CAMERA_UPDATE_RATE': f'{update_rate:.6f}',
            'RGB_IMAGE_WIDTH': str(rgb_width),
            'RGB_IMAGE_HEIGHT': str(rgb_height),
            'RGB_IMAGE_FORMAT': _xml(rgb_image_format),
            'THERMAL_IMAGE_WIDTH': str(thermal_width),
            'THERMAL_IMAGE_HEIGHT': str(thermal_height),
            'THERMAL_IMAGE_FORMAT': _xml(thermal_image_format),
            'CAMERA_NEAR_CLIP': f'{near_clip:.6f}',
            'CAMERA_FAR_CLIP': f'{far_clip:.6f}',
            'CAMERA_VISUALIZE': 'true' if _as_bool(sensors_cfg.get('visualize', True)) else 'false',
            'GZ_RGB_IMAGE_TOPIC': _xml(gz_rgb_image_topic),
            'GZ_THERMAL_IMAGE_TOPIC': _xml(gz_thermal_image_topic),
            'GZ_LIDAR_SCAN_TOPIC': _xml(gz_lidar_scan_topic),
            'THERMAL_MIN_TEMP': f'{thermal_min_temp:.6f}',
            'THERMAL_MAX_TEMP': f'{thermal_max_temp:.6f}',
            'THERMAL_RESOLUTION': f'{thermal_resolution:.6f}',
            'LIDAR_UPDATE_RATE': f'{lidar_update_rate:.6f}',
            'LIDAR_VISUALIZE': 'true' if _as_bool(lidar_cfg.get('visualize', True)) else 'false',
            'LIDAR_HORIZONTAL_SAMPLES': str(lidar_horizontal_samples),
            'LIDAR_VERTICAL_SAMPLES': str(lidar_vertical_samples),
            'LIDAR_DOWNSAMPLE': str(lidar_downsample),
            'LIDAR_HORIZONTAL_MIN_ANGLE': f'{lidar_horizontal_min:.6f}',
            'LIDAR_HORIZONTAL_MAX_ANGLE': f'{lidar_horizontal_max:.6f}',
            'LIDAR_VERTICAL_MIN_ANGLE': f'{lidar_vertical_min:.6f}',
            'LIDAR_VERTICAL_MAX_ANGLE': f'{lidar_vertical_max:.6f}',
            'LIDAR_MIN_RANGE': f'{lidar_min_range:.6f}',
            'LIDAR_MAX_RANGE': f'{lidar_max_range:.6f}',
            'LIDAR_RANGE_RESOLUTION': f'{lidar_range_resolution:.6f}',
            'PLATFORM_GZ_CMD_TOPIC': _xml(platform_gz_cmd_topic),
            'PLATFORM_X': f'{xyz[0]:.6f}',
            'PLATFORM_Y': f'{xyz[1]:.6f}',
            'DECK_HEIGHT': f'{deck_height:.6f}',
            'LEG_HALF_HEIGHT': f'{leg_height / 2.0:.6f}',
            'LEG_HEIGHT': f'{leg_height:.6f}',
            'RAIL_HEIGHT': f'{rail_height:.6f}',
            'MAST_HALF_HEIGHT': f'{deck_height + mast_height / 2.0:.6f}',
            'MAST_HEIGHT': f'{mast_height:.6f}',
        },
    )

    bridge_config = [
        {
            'ros_topic_name': ros_rgb_image_topic,
            'gz_topic_name': gz_rgb_image_topic,
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'gz.msgs.Image',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
            'lazy': False,
            'frame_id': frame_id,
        },
        {
            'ros_topic_name': ros_thermal_image_topic,
            'gz_topic_name': gz_thermal_image_topic,
            'ros_type_name': 'sensor_msgs/msg/Image',
            'gz_type_name': 'gz.msgs.Image',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
            'lazy': False,
            'frame_id': frame_id,
        },
        {
            'ros_topic_name': ros_lidar_scan_topic,
            'gz_topic_name': gz_lidar_scan_topic,
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'SENSOR_DATA',
            'lazy': False,
            'frame_id': frame_id,
        },
        {
            'ros_topic_name': ros_clock_topic,
            'gz_topic_name': gz_clock_topic,
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
            'qos_profile': 'CLOCK',
            'lazy': False,
        },
        {
            'ros_topic_name': platform_ros_cmd_topic,
            'gz_topic_name': platform_gz_cmd_topic,
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ',
            'lazy': False,
        },
    ]
    with open(bridge_path, 'w', encoding='utf-8') as bridge_file:
        yaml.safe_dump(bridge_config, bridge_file, sort_keys=False)

    _render_template(
        package_share / 'rviz' / 'landfill_camera.rviz.in',
        rviz_path,
        {
            'ROS_RGB_IMAGE_TOPIC': ros_rgb_image_topic,
            'ROS_THERMAL_IMAGE_TOPIC': ros_thermal_image_topic,
            'ROS_LIDAR_SCAN_TOPIC': ros_lidar_scan_topic,
            'ROS_LIDAR_POINTS_TOPIC': ros_lidar_points_topic,
            'FIXED_FRAME': fixed_frame,
        },
    )

    gz_args = ['-r', '--gui-config', str(gui_config_path), str(world_path)]
    if not _as_bool(LaunchConfiguration('gz_gui').perform(context)):
        gz_args = ['-s', '-r', '--headless-rendering', str(world_path)]
    if _as_bool(LaunchConfiguration('verbose').perform(context)):
        gz_args.insert(0, '-v 4')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items(),
    )

    # RGB/thermal camera_info are intentionally not bridged from Gazebo:
    # calibration_publisher.py is the single source of truth for intrinsics,
    # publishing them on these same ROS topics from config/calibration/intrinsics.yaml.
    bridge_arguments = [
        f'{gz_rgb_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_thermal_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_lidar_scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'{gz_clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'{platform_gz_cmd_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ]
    bridge_remappings = [
        (gz_rgb_image_topic, ros_rgb_image_topic),
        (gz_thermal_image_topic, ros_thermal_image_topic),
        (gz_lidar_scan_topic, ros_lidar_scan_topic),
        (gz_clock_topic, ros_clock_topic),
        (platform_gz_cmd_topic, platform_ros_cmd_topic),
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='landfill_sensors_bridge',
        output='screen',
        arguments=bridge_arguments + [
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level').perform(context),
        ],
        remappings=bridge_remappings,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='landfill_sensors_rviz',
        output='screen',
        arguments=['-d', str(rviz_path)],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    actions = [gz_sim, bridge]

    actions.append(Node(
        package=PACKAGE_NAME,
        executable='calibration_publisher.py',
        name='calibration_publisher',
        output='screen',
        parameters=[{
            'image_frame_id': frame_id,
            'rgb_camera_info_topic': ros_rgb_camera_info_topic,
            'thermal_camera_info_topic': ros_thermal_camera_info_topic,
            'publish_rate': update_rate,
        }],
    ))

    if motion_enabled:
        actions.append(Node(
            package=PACKAGE_NAME,
            executable='platform_motion_controller.py',
            name='platform_motion_controller',
            output='screen',
            parameters=[{
                'cmd_topic': platform_ros_cmd_topic,
                'axis': motion_axis,
                'travel_distance': motion_travel_distance,
                'linear_speed': motion_linear_speed,
                'angular_speed': motion_angular_speed,
                'update_rate': motion_update_rate,
            }],
        ))

    if lidar_pattern_enabled:
        pattern_node_parameters = {
            'input_scan_topic': ros_lidar_scan_topic,
            'output_points_topic': ros_lidar_points_topic,
            'frame_id': frame_id,
            'horizontal_samples': lidar_horizontal_samples,
            'vertical_samples': lidar_vertical_samples,
            'points_per_scan': lidar_points_per_scan,
            'downsample': lidar_downsample,
            'horizontal_min_angle': lidar_horizontal_min,
            'horizontal_max_angle': lidar_horizontal_max,
            'vertical_min_angle': lidar_vertical_min,
            'vertical_max_angle': lidar_vertical_max,
            'range_min': lidar_min_range,
            'range_max': lidar_max_range,
        }
        if lidar_pattern_csv:
            pattern_node_parameters['scan_pattern_csv'] = lidar_pattern_csv
        actions.append(Node(
            package=PACKAGE_NAME,
            executable='livox_pattern_filter.py',
            name='livox_pattern_filter',
            output='screen',
            parameters=[pattern_node_parameters],
        ))

    actions.append(rviz)
    return actions


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    default_config = str(Path(package_share) / 'config' / 'sensors_sim.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML file containing sensor parameters and Gazebo/ROS topic names.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz with the generated camera view configuration.',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Open Gazebo GUI. Set false for server-only headless rendering.',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Pass verbose logging to Gazebo.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for ros_gz_bridge.',
        ),
        OpaqueFunction(function=_setup),
    ])
