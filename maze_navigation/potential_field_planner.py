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

        # Parameters
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.25)
        self.declare_parameter('d_obs', 1.0)
        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 1.5)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.d_obs = float(self.get_parameter('d_obs').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        self.scan_angles = None
        self.goal_reached = False

        # Publisher / Subscribers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Planner initialized. Goal: ({self.goal_x}, {self.goal_y})'
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

        # Replace invalid readings with a large safe distance
        invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges <= 0.0)
        ranges[invalid] = msg.range_max if msg.range_max > 0.0 else 10.0

        self.scan_data = ranges

        if self.scan_angles is None:
            self.scan_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def publish_cmd(self, linear_x, angular_z):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):
        if self.scan_data is None or self.scan_angles is None:
            return

        # Distance to goal
        dx_goal = self.goal_x - self.current_x
        dy_goal = self.goal_y - self.current_y
        goal_dist = math.hypot(dx_goal, dy_goal)

        # Stop when near goal
        if goal_dist <= 0.2:
            if not self.goal_reached:
                self.get_logger().info(
                    f'Goal reached. Stopping robot. Final pose=({self.current_x:.2f}, {self.current_y:.2f}), '
                    f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}), dist={goal_dist:.3f}'
                )
                self.goal_reached = True
            self.stop_robot()
            return

        # 1) Attractive force
        f_att_x = self.k_att * dx_goal
        f_att_y = self.k_att * dy_goal

        # 2) Repulsive force
        f_rep_x = 0.0
        f_rep_y = 0.0

        for d, angle in zip(self.scan_data, self.scan_angles):
            if d <= self.d_obs:
                d = max(d, 0.05)  # avoid division by zero / huge explosion
                mag = self.k_rep * ((1.0 / d) - (1.0 / self.d_obs)) / (d * d)

                obstacle_angle_world = self.current_yaw + angle

                # Repulsive force points away from obstacle
                f_rep_x += -mag * math.cos(obstacle_angle_world)
                f_rep_y += -mag * math.sin(obstacle_angle_world)

        # 3) Total force
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        desired_heading = math.atan2(f_total_y, f_total_x)
        heading_error = self.normalize_angle(desired_heading - self.current_yaw)

        # 4) Convert force direction to velocity commands
        angular_k = 1.5
        angular_z = angular_k * heading_error
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        # Move slower when turning a lot
        linear_gain = 0.12
        heading_scale = max(0.0, math.cos(heading_error))
        linear_x = min(self.max_linear_vel, linear_gain * goal_dist) * heading_scale

        # Small safety slowdown if obstacle very close in front
        front_indices = np.where(np.abs(self.scan_angles) < 0.35)[0]
        if len(front_indices) > 0:
            min_front = float(np.min(self.scan_data[front_indices]))
            if min_front < 0.35:
                linear_x = min(linear_x, 0.03)

        self.publish_cmd(linear_x, angular_z)


def main(args=None):
    rclpy.init(args=args)
    planner = PotentialFieldPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.stop_robot()
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()