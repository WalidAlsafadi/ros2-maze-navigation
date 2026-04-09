# ROS 2 Maze Navigation with TurtleBot3

Autonomous maze navigation project built with **ROS 2 Jazzy**, **Gazebo Harmonic**, and **TurtleBot3 Burger** using the **Potential Field Method** for local navigation and obstacle avoidance.

## Project Summary

This project implements autonomous maze navigation in Gazebo Sim using a TurtleBot3 Burger robot. The robot is spawned in a maze world, receives **LiDAR** and **odometry** data through `ros_gz_bridge`, computes **attractive** and **repulsive** forces, and publishes velocity commands to move safely toward a goal while avoiding collisions.

The base project was developed for the **simple maze** environment and is designed to satisfy the main project requirements:

- robot integration in ROS 2 Jazzy and Gazebo
- Gazebo environment launch and robot spawning
- topic bridging for `/scan`, `/odom`, `/cmd_vel`, and `/clock`
- Potential Field navigation
- invalid LiDAR filtering
- stopping within **0.2 m** of the goal

## Robot and Environment

- **Robot:** TurtleBot3 Burger
- **ROS 2 version:** Jazzy
- **Simulator:** Gazebo Harmonic / `ros_gz`
- **Base world:** `simple_maze.world`
- **Default spawn point:** `(0.5, 0.5)`
- **Default goal point:** `(9.0, 9.0)`

## Repository Contents

This repository contains the custom `maze_navigation` package:

- `launch/maze_sim.launch.py` — launches Gazebo, spawns the robot, starts the ROS-Gazebo bridge, and starts the planner
- `maze_navigation/potential_field_planner.py` — Potential Field planner implementation
- `worlds/simple_maze.world` — base maze world
- `worlds/complex_maze.world` — bonus maze world
- `config/maze_gui.config` — Gazebo GUI configuration

## Prerequisites

Tested on:

- Ubuntu 24.04
- WSL Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

You should already have:

- ROS 2 Jazzy installed
- Gazebo Sim installed
- GUI support working if using WSL

## Required Dependencies

Install the required ROS / Gazebo packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-dynamixel-sdk
```

## Step-by-Step Build and Launch Instructions

### Step 1: Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_project_ws/src
cd ~/ros2_project_ws/src
```

### Step 2: Clone this repository

```bash
git clone https://github.com/WalidAlsafadi/ros2-maze-navigation.git maze_navigation
```

### Step 3: Clone the required TurtleBot3 repositories

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### Step 4: Build the workspace

```bash
cd ~/ros2_project_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### Step 5: Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_project_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### Step 6: Launch the base project

```bash
ros2 launch maze_navigation maze_sim.launch.py
```

This launches:

- Gazebo with the maze world
- TurtleBot3 Burger at the default spawn point
- `ros_gz_bridge` for `/clock`, `/odom`, `/scan`, and `/cmd_vel`
- the `potential_field_planner` node

## Default Base Run

The default launch configuration for the main/base project is:

- `world:=simple_maze.world`
- `spawn_x:=0.5`
- `spawn_y:=0.5`
- `goal_x:=9.0`
- `goal_y:=9.0`
- `bonus_mode:=false`

Equivalent explicit launch command:

```bash
ros2 launch maze_navigation maze_sim.launch.py \
  world:=simple_maze.world \
  spawn_x:=0.5 \
  spawn_y:=0.5 \
  goal_x:=9.0 \
  goal_y:=9.0 \
  bonus_mode:=false
```

## Custom Launch Parameters

You can manually change the world, spawn point, and goal point:

```bash
ros2 launch maze_navigation maze_sim.launch.py \
  world:=simple_maze.world \
  spawn_x:=0.5 \
  spawn_y:=0.5 \
  goal_x:=9.0 \
  goal_y:=9.0 \
  bonus_mode:=false
```

Examples:

### Run the base maze with a different goal

```bash
ros2 launch maze_navigation maze_sim.launch.py goal_x:=8.0 goal_y:=8.5
```

### Spawn at a different position for testing

```bash
ros2 launch maze_navigation maze_sim.launch.py spawn_x:=1.0 spawn_y:=1.0 goal_x:=9.0 goal_y:=9.0
```

## How the Planner Works

The planner uses a Potential Field method:

- **Attractive force:** pulls the robot toward the goal
- **Repulsive force:** pushes the robot away from nearby obstacles detected by LiDAR
- **Combined force:** determines the desired heading and forward motion

The planner subscribes to:

- `/odom`
- `/scan`

The planner publishes:

- `/cmd_vel`

## Requirement Coverage

The base project implementation covers the following required items:

- **Robot selection and integration:** TurtleBot3 Burger integrated into ROS 2 Jazzy workspace
- **Gazebo environment setup:** simple maze world launched in Gazebo Sim
- **Bridge setup:** `/scan`, `/odom`, `/cmd_vel`, and `/clock` bridged using `ros_gz_bridge`
- **Potential Field implementation:** attractive and repulsive force navigation
- **Invalid LiDAR handling:** invalid readings such as `0.0`, `inf`, and `nan` are filtered before force computation
- **Stopping condition:** robot stops when it reaches within **0.2 m** of the goal

## Notes on Base Project Behavior

- Goal completion is evaluated using the robot's odometry position.
- The final stopping position may vary slightly between runs due to simulation timing and the continuous nature of Potential Field control.
- Small visual differences in the final stop position in the Gazebo GUI are acceptable as long as the robot stops within the required goal tolerance.
- The main/base project focuses on the **simple maze**. The complex maze is bonus work and is not required for the base submission.

## Troubleshooting

### `ros_gz_bridge` or `ros_gz_sim` not found

Install:

```bash
sudo apt install ros-jazzy-ros-gz
```

### `dynamixel_sdk` missing during build

Install:

```bash
sudo apt install ros-jazzy-dynamixel-sdk
```

### Gazebo opens but the robot does not move

Check that:

- the workspace built successfully
- you sourced both setup files
- `TURTLEBOT3_MODEL=burger` is exported
- the planner node is running
- the bridge is running correctly

### Simulation feels slow on WSL

This project was tested on WSL Ubuntu 24.04. Gazebo performance may be slower depending on hardware, GPU support, and Windows graphics configuration. This may affect visualization smoothness but does not necessarily affect the correctness of the base navigation logic.

## Future Improvements

- stronger local minima escape methods for complex mazes
- improved visualization and debugging tools
- additional tuning for different maze layouts
- more robust evaluation and logging utilities

## Acknowledgment

This work builds on the starter skeleton provided in **DSAI 4304: Robot Simulation** by **Dr. Marwan Radi**. The provided skeleton served as the initial foundation for the package structure and project setup, while the ROS 2 launch integration, Gazebo bridging, TurtleBot3 setup, Potential Field navigation logic, and testing were completed in this implementation.
