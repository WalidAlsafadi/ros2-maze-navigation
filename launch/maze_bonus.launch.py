import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze_nav = get_package_share_directory('maze_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    robot_sdf = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='complex_maze.world',
        description='World file inside maze_navigation/worlds'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.6')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.6')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='11.4')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='11.4')

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    world_path = PathJoinSubstitution([pkg_maze_nav, 'worlds', world])

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    start_robot_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf,
            '-name', 'turtlebot3_burger',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.01',
        ],
        output='screen'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
        ],
        output='screen'
    )

    start_planner_cmd = Node(
        package='maze_navigation',
        executable='potential_field_bonus_planner',
        name='potential_field_bonus_planner',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,

            'max_linear_vel': 0.20,
            'max_angular_vel': 1.7,
            'goal_tolerance': 0.20,

            'front_clearance_distance': 0.45,
            'front_blocked_distance': 0.28,
            'front_slow_linear_cap': 0.08,

            'wall_follow_side': 'right',
            'wall_target_distance': 0.32,
            'wall_follow_linear_vel': 0.09,
            'wall_kp': 1.05,

            'goal_heading_kp': 1.3,
            'mline_tolerance': 0.08,
            'leave_goal_improvement': 0.18,

            'goal_clear_heading_error': 0.45,
            'goal_clear_sector_half_width': 0.25,
            'goal_clearance_cap': 1.00,
            'min_wall_follow_steps': 8,

            'wall_progress_check_steps': 90,
            'min_goal_progress': 0.45,
            'min_pose_progress': 0.20,
        }]
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        goal_x_arg,
        goal_y_arg,
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gz_sim_cmd,
        TimerAction(period=5.0, actions=[start_robot_spawner_cmd]),
        start_gazebo_ros_bridge_cmd,
        TimerAction(period=9.0, actions=[start_planner_cmd]),
    ])