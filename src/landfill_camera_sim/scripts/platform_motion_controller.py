#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PlatformMotionController(Node):
    def __init__(self):
        super().__init__('platform_motion_controller')

        self.declare_parameter('cmd_topic', '/landfill/platform/cmd_vel')
        self.declare_parameter('axis', [0.0, 1.0, 0.0])
        self.declare_parameter('travel_distance', 8.0)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.18)
        self.declare_parameter('update_rate', 20.0)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        axis = [float(value) for value in self.get_parameter('axis').value]
        axis_norm = math.sqrt(sum(value * value for value in axis))
        if axis_norm <= 1e-9:
            self.get_logger().warn('platform.motion.axis is zero; using [0, 1, 0]')
            axis = [0.0, 1.0, 0.0]
            axis_norm = 1.0
        self.axis = [value / axis_norm for value in axis]

        self.travel_distance = max(0.0, float(self.get_parameter('travel_distance').value))
        self.linear_speed = max(0.0, float(self.get_parameter('linear_speed').value))
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.update_rate = max(1.0, float(self.get_parameter('update_rate').value))

        self.direction = 1.0
        self.distance_in_leg = 0.0
        self.last_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self._publish_command)

        self.get_logger().info(
            'Moving camera platform on %s: speed %.3f m/s, travel %.3f m, yaw %.3f rad/s'
            % (self.cmd_topic, self.linear_speed, self.travel_distance, self.angular_speed)
        )

    def _publish_command(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now

        if self.travel_distance > 0.0 and self.linear_speed > 0.0:
            self.distance_in_leg += self.linear_speed * dt
            if self.distance_in_leg >= self.travel_distance:
                self.distance_in_leg = 0.0
                self.direction *= -1.0

        linear_speed = self.linear_speed if self.travel_distance > 0.0 else 0.0

        msg = Twist()
        msg.linear.x = self.axis[0] * linear_speed * self.direction
        msg.linear.y = self.axis[1] * linear_speed * self.direction
        msg.linear.z = self.axis[2] * linear_speed * self.direction
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PlatformMotionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            stop_msg = Twist()
            node.publisher.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
