#!/usr/bin/env python3

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from tf2_ros import StaticTransformBroadcaster


def _package_path(relative):
    return get_package_share_directory('landfill_camera_sim') + '/' + relative


def load_extrinsics(path):
    """Read config/calibration/extrinsics.yaml.

    Returns (reference_frame, [(child_frame, (x, y, z), (roll, pitch, yaw)), ...]).
    """
    with open(path, 'r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}
    reference_frame = str(data.get('reference_frame', 'camera_platform/camera_link'))
    entries = []
    for child_frame, transform in (data.get('transforms') or {}).items():
        translation = transform.get('translation', {})
        rotation = transform.get('rotation_rpy', {})
        xyz = (
            float(translation.get('x', 0.0)),
            float(translation.get('y', 0.0)),
            float(translation.get('z', 0.0)),
        )
        rpy = (
            float(rotation.get('roll', 0.0)),
            float(rotation.get('pitch', 0.0)),
            float(rotation.get('yaw', 0.0)),
        )
        entries.append((str(child_frame), xyz, rpy))
    return reference_frame, entries


def load_camera_intrinsics(path, camera_key):
    """Read one camera's block (rgb_camera/thermal_camera) from intrinsics.yaml."""
    with open(path, 'r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}
    camera = data.get(camera_key)
    if camera is None:
        raise KeyError(f'{camera_key} missing from {path}')
    width = int(camera['image_width'])
    height = int(camera['image_height'])
    matrix = camera['camera_matrix']
    fx = float(matrix['fx'])
    fy = float(matrix['fy'])
    cx = float(matrix['cx'])
    cy = float(matrix['cy'])
    distortion_model = str(camera.get('distortion_model', 'plumb_bob'))
    d = [float(v) for v in camera.get('distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])]
    r = [float(v) for v in camera.get(
        'rectification_matrix', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )]
    return {
        'width': width,
        'height': height,
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'distortion_model': distortion_model,
        'D': d,
        'R': r,
    }


def make_camera_info(intrinsics, frame_id):
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = intrinsics['width']
    info.height = intrinsics['height']
    info.distortion_model = intrinsics['distortion_model']
    info.d = intrinsics['D']
    fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = intrinsics['R']
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class CalibrationPublisher(Node):
    def __init__(self):
        super().__init__('calibration_publisher')

        self.declare_parameter('extrinsics_path', _package_path('config/calibration/extrinsics.yaml'))
        self.declare_parameter('intrinsics_path', _package_path('config/calibration/intrinsics.yaml'))
        self.declare_parameter('image_frame_id', 'camera_platform/camera_link')
        self.declare_parameter('rgb_camera_info_topic', '/landfill/rgb/camera_info')
        self.declare_parameter('thermal_camera_info_topic', '/landfill/thermal/camera_info')
        self.declare_parameter('publish_rate', 30.0)

        extrinsics_path = str(self.get_parameter('extrinsics_path').value)
        intrinsics_path = str(self.get_parameter('intrinsics_path').value)
        image_frame_id = str(self.get_parameter('image_frame_id').value)
        rgb_topic = str(self.get_parameter('rgb_camera_info_topic').value)
        thermal_topic = str(self.get_parameter('thermal_camera_info_topic').value)
        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        reference_frame, extrinsic_entries = load_extrinsics(extrinsics_path)
        rgb_intrinsics = load_camera_intrinsics(intrinsics_path, 'rgb_camera')
        thermal_intrinsics = load_camera_intrinsics(intrinsics_path, 'thermal_camera')

        self._broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for child_frame, (x, y, z), (roll, pitch, yaw) in extrinsic_entries:
            transforms.append(self._build_transform(reference_frame, child_frame, x, y, z, roll, pitch, yaw))
        self._broadcaster.sendTransform(transforms)

        self._rgb_info = make_camera_info(rgb_intrinsics, image_frame_id)
        self._thermal_info = make_camera_info(thermal_intrinsics, image_frame_id)
        self._rgb_pub = self.create_publisher(CameraInfo, rgb_topic, 10)
        self._thermal_pub = self.create_publisher(CameraInfo, thermal_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            'Publishing %d static extrinsic transform(s) from %s (%s) and calibrated '
            'CameraInfo on %s, %s (from %s)'
            % (len(transforms), reference_frame, extrinsics_path, rgb_topic, thermal_topic, intrinsics_path)
        )

    @staticmethod
    def _build_transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        self._rgb_info.header.stamp = now
        self._thermal_info.header.stamp = now
        self._rgb_pub.publish(self._rgb_info)
        self._thermal_pub.publish(self._thermal_info)


def main():
    rclpy.init()
    node = CalibrationPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
