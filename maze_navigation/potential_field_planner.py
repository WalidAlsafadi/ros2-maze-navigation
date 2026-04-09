#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')

        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.12)
        self.declare_parameter('d_obs', 0.65)
        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('front_clearance_distance', 0.40)
        self.declare_parameter('front_slow_linear_cap', 0.05)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.d_obs = float(self.get_parameter('d_obs').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.front_clearance_distance = float(self.get_parameter('front_clearance_distance').value)
        self.front_slow_linear_cap = float(self.get_parameter('front_slow_linear_cap').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        self.scan_angles = None
        self.goal_reached = False

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Planner initialized. Goal=({self.goal_x}, {self.goal_y}), mode=Potential field'
        )

    def euler_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)

        invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges <= 0.0)
        fallback = msg.range_max if msg.range_max > 0.0 else 10.0
        ranges[invalid] = fallback

        self.scan_data = ranges

        if self.scan_angles is None:
            self.scan_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def publish_cmd(self, linear_x, angular_z):
        if not rclpy.ok():
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

    def front_distance(self):
        indices = np.where((self.scan_angles >= -0.30) & (self.scan_angles <= 0.30))[0]
        if len(indices) == 0:
            return 10.0
        return float(np.min(self.scan_data[indices]))

    def run_potential_field(self):
        dx_goal = self.goal_x - self.current_x
        dy_goal = self.goal_y - self.current_y
        goal_dist = math.hypot(dx_goal, dy_goal)

        f_att_x = self.k_att * dx_goal
        f_att_y = self.k_att * dy_goal

        f_rep_x = 0.0
        f_rep_y = 0.0

        for d, angle in zip(self.scan_data, self.scan_angles):
            if d <= self.d_obs:
                d = max(d, 0.05)
                mag = self.k_rep * ((1.0 / d) - (1.0 / self.d_obs)) / (d * d)

                obstacle_angle_world = self.current_yaw + angle
                f_rep_x += -mag * math.cos(obstacle_angle_world)
                f_rep_y += -mag * math.sin(obstacle_angle_world)

        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        desired_heading = math.atan2(f_total_y, f_total_x)
        heading_error = self.normalize_angle(desired_heading - self.current_yaw)

        angular_z = 1.5 * heading_error
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        linear_gain = 0.12
        heading_scale = max(0.0, math.cos(heading_error))
        linear_x = min(self.max_linear_vel, linear_gain * goal_dist) * heading_scale

        if self.front_distance() < self.front_clearance_distance:
            linear_x = min(linear_x, self.front_slow_linear_cap)

        self.publish_cmd(linear_x, angular_z)

    def control_loop(self):
        if self.scan_data is None or self.scan_angles is None:
            return

        goal_dist = self.distance_to_goal()

        if goal_dist <= self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    f'Goal reached. Final pose=({self.current_x:.2f}, {self.current_y:.2f}), '
                    f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}), dist={goal_dist:.3f}'
                )
            self.stop_robot()
            return

        self.run_potential_field()


def main(args=None):
    rclpy.init(args=args)
    planner = PotentialFieldPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            planner.stop_robot()
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()