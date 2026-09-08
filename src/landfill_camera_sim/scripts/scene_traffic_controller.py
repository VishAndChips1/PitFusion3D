#!/usr/bin/env python3
"""Drive the scene's background vehicles around looping routes.

The props are kinematic models carrying gz-sim's VelocityControl plugin, so
this node sets their velocity directly rather than pushing them with forces.
TrajectoryFollower was tried first and is a poor fit here: it applies a
constant force, and with only Coulomb friction to oppose it there is no
terminal velocity, so the props accelerated past their waypoints and left
the site.

Each vehicle follows its route by turning in place until it is pointed at
the next waypoint, then driving straight to it -- enough for background
traffic that the cameras and lidar can see.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# name, route (x, y), cruise speed m/s, turn rate rad/s, spawn yaw rad.
# The spawn yaw must match the model's <pose> in the world template: this
# node dead-reckons from the velocity it commands, so starting from the
# wrong heading makes every subsequent leg veer off the route.
DEFAULT_ROUTES = [
    ('haul_truck', [(18.0, 14.0), (-2.0, 16.0), (-13.0, 7.0), (-6.0, -1.0),
                    (11.0, 2.0), (20.0, 7.0)], 2.2, 0.45, 3.14159),
    ('wheel_loader', [(6.0, -6.0), (12.0, -2.0), (14.0, 4.0), (8.0, 6.0),
                      (4.0, 0.0)], 1.3, 0.40, 1.2),
    ('service_pickup', [(-16.0, -10.0), (-2.0, -12.0), (10.0, -10.0),
                        (2.0, -4.0), (-12.0, -4.0)], 3.0, 0.55, 0.5),
]


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Vehicle:
    """Dead-reckoned pose plus a turn-then-drive route follower.

    gz-sim's VelocityControl takes body-frame velocity and publishes no
    odometry back, so the commanded velocity is integrated here to know
    where the prop has got to.
    """

    def __init__(self, name, route, speed, turn_rate, publisher, yaw=0.0):
        self.name = name
        self.route = route
        self.speed = speed
        self.turn_rate = turn_rate
        self.publisher = publisher
        self.x, self.y = route[0]
        self.yaw = yaw
        self.target = 1 % len(route)

    def step(self, dt, arrival_radius=1.5):
        goal_x, goal_y = self.route[self.target]
        dx, dy = goal_x - self.x, goal_y - self.y
        distance = math.hypot(dx, dy)
        if distance < arrival_radius:
            self.target = (self.target + 1) % len(self.route)
            goal_x, goal_y = self.route[self.target]
            dx, dy = goal_x - self.x, goal_y - self.y
            distance = math.hypot(dx, dy)

        heading_error = _wrap_to_pi(math.atan2(dy, dx) - self.yaw)

        command = Twist()
        if abs(heading_error) > 0.20:
            # Turn in place until roughly lined up, then drive.
            command.angular.z = math.copysign(self.turn_rate, heading_error)
        else:
            command.linear.x = min(self.speed, distance)
            command.angular.z = 2.0 * heading_error

        self.yaw = _wrap_to_pi(self.yaw + command.angular.z * dt)
        self.x += command.linear.x * math.cos(self.yaw) * dt
        self.y += command.linear.x * math.sin(self.yaw) * dt
        self.publisher.publish(command)


class SceneTrafficController(Node):
    def __init__(self):
        super().__init__('scene_traffic_controller')
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('topic_template', '/model/{name}/cmd_vel')
        self.declare_parameter('vehicles', [row[0] for row in DEFAULT_ROUTES])

        update_rate = float(self.get_parameter('update_rate').value)
        template = str(self.get_parameter('topic_template').value)
        enabled = list(self.get_parameter('vehicles').value)

        self.vehicles = []
        for name, route, speed, turn_rate, spawn_yaw in DEFAULT_ROUTES:
            if name not in enabled:
                continue
            topic = template.format(name=name)
            publisher = self.create_publisher(Twist, topic, 10)
            self.vehicles.append(
                Vehicle(name, route, speed, turn_rate, publisher, spawn_yaw))

        self._dt = 1.0 / update_rate
        self.create_timer(self._dt, self._on_timer)
        self.get_logger().info(
            'Driving %d background vehicles: %s'
            % (len(self.vehicles), ', '.join(v.name for v in self.vehicles))
        )

    def _on_timer(self):
        for vehicle in self.vehicles:
            vehicle.step(self._dt)


def main():
    rclpy.init()
    node = SceneTrafficController()
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
