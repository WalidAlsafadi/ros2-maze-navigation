#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PotentialFieldBonusPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_bonus_planner')

        self.declare_parameter('goal_x', 10.5)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('front_clearance_distance', 0.40)
        self.declare_parameter('front_blocked_distance', 0.24)
        self.declare_parameter('front_slow_linear_cap', 0.05)

        self.declare_parameter('wall_follow_side', 'left')
        self.declare_parameter('wall_target_distance', 0.38)
        self.declare_parameter('wall_follow_linear_vel', 0.07)
        self.declare_parameter('wall_kp', 1.6)
        self.declare_parameter('goal_heading_kp', 1.5)
        self.declare_parameter('mline_tolerance', 0.18)
        self.declare_parameter('leave_goal_improvement', 0.18)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.front_clearance_distance = float(self.get_parameter('front_clearance_distance').value)
        self.front_blocked_distance = float(self.get_parameter('front_blocked_distance').value)
        self.front_slow_linear_cap = float(self.get_parameter('front_slow_linear_cap').value)

        self.wall_follow_side = str(self.get_parameter('wall_follow_side').value).strip().lower()
        if self.wall_follow_side not in ('left', 'right'):
            self.wall_follow_side = 'left'

        self.wall_target_distance = float(self.get_parameter('wall_target_distance').value)
        self.wall_follow_linear_vel = float(self.get_parameter('wall_follow_linear_vel').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)
        self.goal_heading_kp = float(self.get_parameter('goal_heading_kp').value)
        self.mline_tolerance = float(self.get_parameter('mline_tolerance').value)
        self.leave_goal_improvement = float(self.get_parameter('leave_goal_improvement').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        self.scan_angles = None
        self.goal_reached = False

        self.mode = 'GO_TO_GOAL'
        self.start_x = None
        self.start_y = None
        self.hit_goal_dist = None

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Bonus planner initialized. Goal=({self.goal_x}, {self.goal_y}), mode=Bug2-style'
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

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

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

    def get_sector_min_distance(self, angle_min, angle_max):
        indices = np.where((self.scan_angles >= angle_min) & (self.scan_angles <= angle_max))[0]
        if len(indices) == 0:
            return 10.0
        return float(np.min(self.scan_data[indices]))

    def get_sector_mean_distance(self, angle_min, angle_max):
        indices = np.where((self.scan_angles >= angle_min) & (self.scan_angles <= angle_max))[0]
        if len(indices) == 0:
            return 10.0
        return float(np.mean(self.scan_data[indices]))

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

    def goal_heading_error(self):
        desired_heading = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        return self.normalize_angle(desired_heading - self.current_yaw)

    def front_distance(self):
        return self.get_sector_min_distance(-0.30, 0.30)

    def on_m_line(self):
        if self.start_x is None or self.start_y is None:
            return False

        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        denom = math.hypot(dx, dy)

        if denom < 1e-6:
            return True

        numer = abs(
            dy * self.current_x
            - dx * self.current_y
            + self.goal_x * self.start_y
            - self.goal_y * self.start_x
        )
        distance_to_line = numer / denom
        return distance_to_line <= self.mline_tolerance

    def run_go_to_goal(self):
        front = self.front_distance()
        heading_error = self.goal_heading_error()
        goal_dist = self.distance_to_goal()

        if front < self.front_blocked_distance:
            self.mode = 'FOLLOW_WALL'
            self.hit_goal_dist = goal_dist
            self.get_logger().warn(
                f'Obstacle hit. Switching to FOLLOW_WALL. '
                f'hit_goal_dist={self.hit_goal_dist:.2f}, side={self.wall_follow_side}'
            )
            self.run_follow_wall()
            return

        angular_z = self.goal_heading_kp * heading_error
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        heading_scale = max(0.0, math.cos(heading_error))
        linear_x = self.max_linear_vel * heading_scale

        if front < self.front_clearance_distance:
            linear_x = min(linear_x, self.front_slow_linear_cap)

        self.publish_cmd(linear_x, angular_z)

    def should_leave_wall(self):
        if self.hit_goal_dist is None:
            return False

        front = self.front_distance()
        goal_dist = self.distance_to_goal()
        heading_error = abs(self.goal_heading_error())

        return (
            self.on_m_line()
            and goal_dist < (self.hit_goal_dist - self.leave_goal_improvement)
            and front > self.front_clearance_distance
            and heading_error < 0.6
        )

    def run_follow_wall(self):
        front = self.front_distance()

        if self.should_leave_wall():
            self.mode = 'GO_TO_GOAL'
            self.get_logger().info(
                f'Leave point found. Switching to GO_TO_GOAL. '
                f'goal_dist={self.distance_to_goal():.2f}'
            )
            self.run_go_to_goal()
            return

        linear_x = self.wall_follow_linear_vel
        angular_z = 0.0

        if self.wall_follow_side == 'left':
            side_dist = self.get_sector_mean_distance(1.10, 1.45)
            front_side_dist = self.get_sector_min_distance(0.35, 1.00)

            if front < self.front_blocked_distance:
                self.publish_cmd(0.0, -self.max_angular_vel)
                return

            if side_dist > self.wall_target_distance + 0.18:
                angular_z = 0.55
                linear_x = 0.04
            elif side_dist < self.wall_target_distance - 0.10:
                angular_z = -0.45
                linear_x = 0.04
            elif front_side_dist < self.front_clearance_distance:
                angular_z = -0.25
            else:
                side_error = self.wall_target_distance - side_dist
                angular_z = -self.wall_kp * side_error

        else:
            side_dist = self.get_sector_mean_distance(-1.45, -1.10)
            front_side_dist = self.get_sector_min_distance(-1.00, -0.35)

            if front < self.front_blocked_distance:
                self.publish_cmd(0.0, self.max_angular_vel)
                return

            if side_dist > self.wall_target_distance + 0.18:
                angular_z = -0.55
                linear_x = 0.04
            elif side_dist < self.wall_target_distance - 0.10:
                angular_z = 0.45
                linear_x = 0.04
            elif front_side_dist < self.front_clearance_distance:
                angular_z = 0.25
            else:
                side_error = self.wall_target_distance - side_dist
                angular_z = self.wall_kp * side_error

        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))
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

        if self.mode == 'GO_TO_GOAL':
            self.run_go_to_goal()
        else:
            self.run_follow_wall()


def main(args=None):
    rclpy.init(args=args)
    planner = PotentialFieldBonusPlanner()

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