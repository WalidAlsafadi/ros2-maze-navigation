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

        self.declare_parameter('goal_x', 11.4)
        self.declare_parameter('goal_y', 11.4)
        self.declare_parameter('spawn_x', 0.6)
        self.declare_parameter('spawn_y', 0.6)

        self.declare_parameter('max_linear_vel', 0.18)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('goal_tolerance', 0.20)

        self.declare_parameter('front_clearance_distance', 0.40)
        self.declare_parameter('front_blocked_distance', 0.24)
        self.declare_parameter('front_slow_linear_cap', 0.05)

        self.declare_parameter('wall_follow_side', 'right')
        self.declare_parameter('wall_target_distance', 0.38)
        self.declare_parameter('wall_follow_linear_vel', 0.07)
        self.declare_parameter('wall_kp', 1.6)

        self.declare_parameter('goal_heading_kp', 1.5)
        self.declare_parameter('mline_tolerance', 0.18)
        self.declare_parameter('leave_goal_improvement', 0.18)

        self.declare_parameter('goal_clear_heading_error', 0.35)
        self.declare_parameter('goal_clear_sector_half_width', 0.18)
        self.declare_parameter('goal_clearance_cap', 0.75)
        self.declare_parameter('min_wall_follow_steps', 18)

        # New watchdog parameters
        self.declare_parameter('wall_progress_check_steps', 180)
        self.declare_parameter('min_goal_progress', 0.45)
        self.declare_parameter('min_pose_progress', 0.20)

        self.goal_x_world = float(self.get_parameter('goal_x').value)
        self.goal_y_world = float(self.get_parameter('goal_y').value)
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.front_clearance_distance = float(self.get_parameter('front_clearance_distance').value)
        self.front_blocked_distance = float(self.get_parameter('front_blocked_distance').value)
        self.front_slow_linear_cap = float(self.get_parameter('front_slow_linear_cap').value)

        self.default_wall_follow_side = str(self.get_parameter('wall_follow_side').value).strip().lower()
        if self.default_wall_follow_side not in ('left', 'right'):
            self.default_wall_follow_side = 'left'
        self.wall_follow_side = self.default_wall_follow_side

        self.wall_target_distance = float(self.get_parameter('wall_target_distance').value)
        self.wall_follow_linear_vel = float(self.get_parameter('wall_follow_linear_vel').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)

        self.goal_heading_kp = float(self.get_parameter('goal_heading_kp').value)
        self.mline_tolerance = float(self.get_parameter('mline_tolerance').value)
        self.leave_goal_improvement = float(self.get_parameter('leave_goal_improvement').value)

        self.goal_clear_heading_error = float(self.get_parameter('goal_clear_heading_error').value)
        self.goal_clear_sector_half_width = float(self.get_parameter('goal_clear_sector_half_width').value)
        self.goal_clearance_cap = float(self.get_parameter('goal_clearance_cap').value)
        self.min_wall_follow_steps = int(self.get_parameter('min_wall_follow_steps').value)

        self.wall_progress_check_steps = int(self.get_parameter('wall_progress_check_steps').value)
        self.min_goal_progress = float(self.get_parameter('min_goal_progress').value)
        self.min_pose_progress = float(self.get_parameter('min_pose_progress').value)

        self.goal_x_odom = None
        self.goal_y_odom = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.scan_data = None
        self.scan_angles = None

        self.goal_reached = False
        self.mode = 'GO_TO_GOAL'

        self.hit_goal_dist = None
        self.wall_follow_steps = 0

        # New wall-follow progress tracking
        self.wall_start_x = None
        self.wall_start_y = None
        self.wall_start_goal_dist = None
        self.best_wall_goal_dist = None

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Bonus planner initialized. Goal(world)=({self.goal_x_world}, {self.goal_y_world}), mode=Bug2-style'
        )

    def choose_wall_follow_side(self):
        left_side = self.get_sector_mean_distance(1.10, 1.45)
        right_side = self.get_sector_mean_distance(-1.45, -1.10)

        left_front = self.get_sector_min_distance(0.20, 1.00)
        right_front = self.get_sector_min_distance(-1.00, -0.20)

        left_open = min(left_side, left_front)
        right_open = min(right_side, right_front)

        # If left side is more open, turn left around the obstacle,
        # which means keep the wall on the RIGHT.
        if left_open > right_open + 0.05:
            return 'right'

        # If right side is more open, turn right around the obstacle,
        # which means keep the wall on the LEFT.
        if right_open > left_open + 0.05:
            return 'left'

        return self.default_wall_follow_side

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
            self.start_yaw = self.current_yaw

            dx_world = self.goal_x_world - self.spawn_x
            dy_world = self.goal_y_world - self.spawn_y

            c = math.cos(self.start_yaw)
            s = math.sin(self.start_yaw)

            dx_odom = c * dx_world + s * dy_world
            dy_odom = -s * dx_world + c * dy_world

            self.goal_x_odom = self.current_x + dx_odom
            self.goal_y_odom = self.current_y + dy_odom

            self.get_logger().info(
                f'Initial yaw={self.start_yaw:.2f} rad | '
                f'world_delta=({dx_world:.2f}, {dy_world:.2f}) -> '
                f'odom_delta=({dx_odom:.2f}, {dy_odom:.2f}) | '
                f'goal_odom=({self.goal_x_odom:.2f}, {self.goal_y_odom:.2f})'
            )

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
        if self.scan_data is None or self.scan_angles is None:
            return 10.0

        indices = np.where((self.scan_angles >= angle_min) & (self.scan_angles <= angle_max))[0]
        if len(indices) == 0:
            return 10.0
        return float(np.min(self.scan_data[indices]))

    def get_sector_mean_distance(self, angle_min, angle_max):
        if self.scan_data is None or self.scan_angles is None:
            return 10.0

        indices = np.where((self.scan_angles >= angle_min) & (self.scan_angles <= angle_max))[0]
        if len(indices) == 0:
            return 10.0
        return float(np.mean(self.scan_data[indices]))

    def distance_to_goal(self):
        if self.goal_x_odom is None or self.goal_y_odom is None:
            return float('inf')

        return math.hypot(
            self.goal_x_odom - self.current_x,
            self.goal_y_odom - self.current_y
        )

    def goal_heading_error(self):
        if self.goal_x_odom is None or self.goal_y_odom is None:
            return 0.0

        desired_heading = math.atan2(
            self.goal_y_odom - self.current_y,
            self.goal_x_odom - self.current_x
        )
        return self.normalize_angle(desired_heading - self.current_yaw)

    def front_distance(self):
        return self.get_sector_min_distance(-0.30, 0.30)

    def on_m_line(self):
        if self.start_x is None or self.start_y is None:
            return False
        if self.goal_x_odom is None or self.goal_y_odom is None:
            return False

        dx = self.goal_x_odom - self.start_x
        dy = self.goal_y_odom - self.start_y
        denom = math.hypot(dx, dy)

        if denom < 1e-6:
            return True

        numer = abs(
            dy * self.current_x
            - dx * self.current_y
            + self.goal_x_odom * self.start_y
            - self.goal_y_odom * self.start_x
        )
        distance_to_line = numer / denom
        return distance_to_line <= self.mline_tolerance

    def goal_direction_clear(self):
        goal_dist = self.distance_to_goal()
        heading_error = self.goal_heading_error()

        # Near the final chamber, be much more permissive:
        # do not require the robot to already face the goal well,
        # and do not require large side clearance from the wall.
        if goal_dist < 2.5:
            sector_half_width = 0.35
            clearance_along_goal = self.get_sector_min_distance(
                heading_error - sector_half_width,
                heading_error + sector_half_width
            )

            # Only require that the path toward the goal is clear
            # approximately up to the goal distance.
            return clearance_along_goal > max(0.35, goal_dist - 0.10)

        if abs(heading_error) > self.goal_clear_heading_error:
            return False

        clearance_along_goal = self.get_sector_min_distance(
            heading_error - self.goal_clear_sector_half_width,
            heading_error + self.goal_clear_sector_half_width
        )

        required_clearance = min(goal_dist, self.goal_clearance_cap)
        if clearance_along_goal <= required_clearance:
            return False

        if self.wall_follow_side == 'left':
            side_dist = self.get_sector_mean_distance(1.10, 1.45)
        else:
            side_dist = self.get_sector_mean_distance(-1.45, -1.10)

        if side_dist < (self.wall_target_distance + 0.08):
            return False

        return True

    def start_wall_follow_episode(self, goal_dist):
        self.wall_follow_steps = 0
        self.hit_goal_dist = goal_dist
        self.wall_start_x = self.current_x
        self.wall_start_y = self.current_y
        self.wall_start_goal_dist = goal_dist
        self.best_wall_goal_dist = goal_dist

    def reset_wall_follow_progress_window(self, goal_dist=None):
        if goal_dist is None:
            goal_dist = self.distance_to_goal()

        self.wall_follow_steps = 0
        self.wall_start_x = self.current_x
        self.wall_start_y = self.current_y
        self.wall_start_goal_dist = goal_dist

    def maybe_flip_wall_side(self):
        if self.wall_follow_steps < self.wall_progress_check_steps:
            return False
        if self.wall_start_goal_dist is None or self.wall_start_x is None or self.wall_start_y is None:
            return False

        goal_progress = self.wall_start_goal_dist - self.distance_to_goal()
        pose_progress = math.hypot(
            self.current_x - self.wall_start_x,
            self.current_y - self.wall_start_y
        )

        # Only flip if we are clearly doing badly.
        clearly_stuck = (
            goal_progress < -0.05
            or (goal_progress < 0.03 and pose_progress < 0.10)
        )

        if not clearly_stuck:
            return False

        old_side = self.wall_follow_side
        self.wall_follow_side = 'right' if old_side == 'left' else 'left'

        new_goal_dist = self.distance_to_goal()
        self.reset_wall_follow_progress_window(new_goal_dist)

        self.get_logger().warn(
            f'Low wall-follow progress. Flipping side {old_side}->{self.wall_follow_side}. '
            f'goal_progress={goal_progress:.2f}, pose_progress={pose_progress:.2f}, '
            f'goal_dist={new_goal_dist:.2f}'
        )
        return True

    def run_go_to_goal(self):
        front = self.front_distance()
        heading_error = self.goal_heading_error()
        goal_dist = self.distance_to_goal()

        if front < self.front_blocked_distance:
            self.mode = 'FOLLOW_WALL'
            self.wall_follow_side = self.choose_wall_follow_side()
            self.start_wall_follow_episode(goal_dist)

            self.get_logger().warn(
                f'Obstacle hit. Switching to FOLLOW_WALL. '
                f'hit_goal_dist={self.hit_goal_dist:.2f}, side={self.wall_follow_side}'
            )
            self.run_follow_wall()
            return

        angular_z = self.goal_heading_kp * heading_error
        
        heading_scale = max(0.0, math.cos(heading_error))
        linear_x = self.max_linear_vel * heading_scale

        if front < self.front_clearance_distance:
            linear_x = min(linear_x, self.front_slow_linear_cap)

        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))
        self.publish_cmd(linear_x, angular_z)

    def should_leave_wall(self):
        if self.hit_goal_dist is None:
            return False

        front = self.front_distance()
        goal_dist = self.distance_to_goal()
        heading_error = abs(self.goal_heading_error())

        # Late maze / debug-spawn region:
        # do not force m-line, and do not require a nearly perfect heading.
        # If the robot has a reasonably open front and is not getting farther
        # than its best recent wall-follow distance, let GO_TO_GOAL try.
        if goal_dist < 7.0:
            best_dist = self.best_wall_goal_dist if self.best_wall_goal_dist is not None else goal_dist
            return (
                front > (self.front_blocked_distance + 0.08)
                and heading_error < 1.20
                and goal_dist <= (best_dist + 0.10)
            )

        # Earlier in the maze, keep the stricter Bug2-style condition.
        return (
            self.on_m_line()
            and goal_dist < (self.hit_goal_dist - self.leave_goal_improvement)
            and front > self.front_clearance_distance
            and heading_error < 0.50
        )

    def should_leave_wall_opportunistic(self):
        if self.best_wall_goal_dist is None:
            return False

        goal_dist = self.distance_to_goal()
        heading_error = self.goal_heading_error()
        front = self.front_distance()

        if goal_dist > (self.best_wall_goal_dist + 0.05):
            return False

        if goal_dist > 3.0:
            return False

        if abs(heading_error) > 0.60:
            return False

        if front <= (self.front_clearance_distance + 0.05):
            return False

        clearance_along_goal = self.get_sector_min_distance(
            heading_error - 0.20,
            heading_error + 0.20
        )

        required_clearance = min(goal_dist, 1.00)
        return clearance_along_goal > required_clearance

    def run_follow_wall(self):
        front = self.front_distance()
        self.wall_follow_steps += 1

        goal_dist = self.distance_to_goal()
        if self.best_wall_goal_dist is None or goal_dist < self.best_wall_goal_dist:
            self.best_wall_goal_dist = goal_dist

        # Near-goal hard override first.
        if (
            goal_dist < 3.5
            and self.wall_follow_steps >= self.min_wall_follow_steps
            and front > (self.front_blocked_distance + 0.08)
        ):
            self.mode = 'GO_TO_GOAL'
            self.wall_follow_steps = 0
            self.get_logger().info(
                f'Near-goal override. Switching to GO_TO_GOAL. '
                f'goal_dist={goal_dist:.2f}, '
                f'front={front:.2f}, '
                f'heading_error={self.goal_heading_error():.2f}'
            )
            self.run_go_to_goal()
            return

        # Then try normal leave.
        if self.should_leave_wall():
            self.mode = 'GO_TO_GOAL'
            self.wall_follow_steps = 0
            self.get_logger().info(
                f'Leave point found. Switching to GO_TO_GOAL. '
                f'goal_dist={goal_dist:.2f}, '
                f'heading_error={self.goal_heading_error():.2f}'
            )
            self.run_go_to_goal()
            return

        # Then try opportunistic leave.
        if (
            self.wall_follow_steps >= self.min_wall_follow_steps
            and self.should_leave_wall_opportunistic()
        ):
            self.mode = 'GO_TO_GOAL'
            self.wall_follow_steps = 0
            self.get_logger().info(
                f'Opportunistic leave. Switching to GO_TO_GOAL. '
                f'goal_dist={goal_dist:.2f}, '
                f'best_wall_goal_dist={self.best_wall_goal_dist:.2f}, '
                f'heading_error={self.goal_heading_error():.2f}'
            )
            self.run_go_to_goal()
            return

        # Only if leave failed, allow late-maze side flipping.
        if goal_dist < 7.0:
            flipped = self.maybe_flip_wall_side()
            if flipped:
                return

        linear_x = self.wall_follow_linear_vel
        angular_z = 0.0

        if self.wall_follow_side == 'left':
            side_dist = self.get_sector_mean_distance(1.10, 1.45)
            front_side_dist = self.get_sector_min_distance(0.35, 1.00)

            if front < self.front_blocked_distance:
                self.publish_cmd(0.03, -0.8 * self.max_angular_vel)
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
                self.publish_cmd(0.03, 0.8 * self.max_angular_vel)
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
        if self.goal_x_odom is None or self.goal_y_odom is None:
            return
        if self.scan_data is None or self.scan_angles is None:
            return

        goal_dist = self.distance_to_goal()

        if goal_dist <= self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    f'Goal reached. Final pose=({self.current_x:.2f}, {self.current_y:.2f}), '
                    f'goal=({self.goal_x_odom:.2f}, {self.goal_y_odom:.2f})'
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