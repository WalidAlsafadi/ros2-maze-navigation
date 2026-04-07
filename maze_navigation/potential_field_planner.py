#!/usr/bin/env python3

import math
import random
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')

        # Core parameters
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.25)
        self.declare_parameter('d_obs', 1.0)
        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 1.5)

        # Bonus mode parameters
        self.declare_parameter('bonus_mode', False)
        self.declare_parameter('stuck_distance_threshold', 0.03)
        self.declare_parameter('stuck_cycles_threshold', 35)
        self.declare_parameter('escape_cycles', 18)
        self.declare_parameter('escape_linear_vel', 0.03)
        self.declare_parameter('escape_angular_vel', 1.0)
        self.declare_parameter('front_slowdown_distance', 0.35)
        self.declare_parameter('front_slow_linear_cap', 0.03)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.d_obs = float(self.get_parameter('d_obs').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)

        self.bonus_mode = bool(self.get_parameter('bonus_mode').value)
        self.stuck_distance_threshold = float(self.get_parameter('stuck_distance_threshold').value)
        self.stuck_cycles_threshold = int(self.get_parameter('stuck_cycles_threshold').value)
        self.escape_cycles = int(self.get_parameter('escape_cycles').value)
        self.escape_linear_vel = float(self.get_parameter('escape_linear_vel').value)
        self.escape_angular_vel = float(self.get_parameter('escape_angular_vel').value)
        self.front_slowdown_distance = float(self.get_parameter('front_slowdown_distance').value)
        self.front_slow_linear_cap = float(self.get_parameter('front_slow_linear_cap').value)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        self.scan_angles = None
        self.goal_reached = False

        # Bonus / stuck detection state
        self.prev_x = None
        self.prev_y = None
        self.stuck_counter = 0

        self.escape_mode = False
        self.escape_phase = 'rotate'   # 'rotate' or 'forward'
        self.escape_counter = 0
        self.escape_turn_dir = 1.0

        # ROS interfaces
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        random.seed(42)

        self.get_logger().info(
            f'Planner initialized. Goal: ({self.goal_x}, {self.goal_y}), bonus_mode={self.bonus_mode}'
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
        ranges[invalid] = msg.range_max if msg.range_max > 0.0 else 10.0

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

    def update_stuck_detector(self, goal_dist):
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            return

        moved_dist = math.hypot(self.current_x - self.prev_x, self.current_y - self.prev_y)

        if goal_dist > 0.3 and moved_dist < self.stuck_distance_threshold:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        self.prev_x = self.current_x
        self.prev_y = self.current_y

        if self.stuck_counter >= self.stuck_cycles_threshold and not self.escape_mode:
            self.escape_mode = True
            self.escape_phase = 'rotate'
            self.escape_counter = self.escape_cycles
            self.escape_turn_dir = random.choice([-1.0, 1.0])
            self.stuck_counter = 0

            direction_text = 'left' if self.escape_turn_dir > 0 else 'right'
            self.get_logger().warn(
                f'Stuck detected. Escape mode ON. Phase=rotate, turning {direction_text}.'
            )

    def run_escape_behavior(self):
        front_indices = np.where(np.abs(self.scan_angles) < 0.35)[0]
        min_front = float(np.min(self.scan_data[front_indices])) if len(front_indices) > 0 else 10.0

        # Phase 1: rotate until there is enough open space ahead
        if self.escape_phase == 'rotate':
            self.publish_cmd(0.0, self.escape_turn_dir * self.escape_angular_vel)

            # Once front is open enough, commit forward motion
            if min_front > 0.6:
                self.escape_phase = 'forward'
                self.escape_counter = self.escape_cycles
                self.get_logger().info('Escape phase switched to FORWARD.')
            return

        # Phase 2: move forward for a short burst
        if self.escape_phase == 'forward':
            linear_x = self.escape_linear_vel

            # If something is too close, stop forward escape and rotate again
            if min_front < 0.22:
                self.escape_phase = 'rotate'
                self.get_logger().info('Escape forward blocked. Switching back to ROTATE.')
                self.publish_cmd(0.0, self.escape_turn_dir * self.escape_angular_vel)
                return

            self.publish_cmd(linear_x, 0.0)

            self.escape_counter -= 1
            if self.escape_counter <= 0:
                self.escape_mode = False
                self.escape_phase = 'rotate'
                self.get_logger().info('Escape mode finished. Returning to normal potential field navigation.')

    def control_loop(self):
        if self.scan_data is None or self.scan_angles is None:
            return

        dx_goal = self.goal_x - self.current_x
        dy_goal = self.goal_y - self.current_y
        goal_dist = math.hypot(dx_goal, dy_goal)

        if goal_dist <= 0.2:
            if not self.goal_reached:
                self.get_logger().info(
                    f'Goal reached. Stopping robot. Final pose=({self.current_x:.2f}, {self.current_y:.2f}), '
                    f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}), dist={goal_dist:.3f}'
                )
                self.goal_reached = True
            self.stop_robot()
            return

        if self.bonus_mode:
            self.update_stuck_detector(goal_dist)
            if self.escape_mode:
                self.run_escape_behavior()
                return

        # Attractive force
        f_att_x = self.k_att * dx_goal
        f_att_y = self.k_att * dy_goal

        # Repulsive force
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

        angular_k = 1.5
        angular_z = angular_k * heading_error
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))

        linear_gain = 0.12
        heading_scale = max(0.0, math.cos(heading_error))
        linear_x = min(self.max_linear_vel, linear_gain * goal_dist) * heading_scale

        front_indices = np.where(np.abs(self.scan_angles) < 0.35)[0]
        if len(front_indices) > 0:
            min_front = float(np.min(self.scan_data[front_indices]))
            if min_front < self.front_slowdown_distance:
                linear_x = min(linear_x, self.front_slow_linear_cap)


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