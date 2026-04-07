import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze_nav = get_package_share_directory('maze_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gui_config = os.path.expanduser('~/ros2_project_ws/src/maze_navigation/config/maze_gui.config')

    world_file = os.path.join(pkg_maze_nav, 'worlds', 'simple_maze.world')
    robot_sdf = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='9.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='9.0')

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'--gui-config {gui_config} -r {world_file}'
        }.items()
    )

    start_robot_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf,
            '-name', 'turtlebot3_burger',
            '-x', '0.5',
            '-y', '0.5',
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
        executable='potential_field_planner',
        name='potential_field_planner',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'k_att': 1.0,
            'k_rep': 0.25,
            'd_obs': 1.0,
            'max_linear_vel': 0.18,
            'max_angular_vel': 1.5,
        }]
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gz_sim_cmd,
        start_robot_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        TimerAction(period=3.0, actions=[start_planner_cmd]),
    ])