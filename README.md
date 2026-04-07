# ROS 2 Maze Navigation with TurtleBot3

Autonomous maze navigation project built with **ROS 2 Jazzy**, **Gazebo Harmonic**, and **TurtleBot3 Burger** using a **Potential Field** method for obstacle avoidance and goal-directed motion.

## Overview

This project demonstrates a simulated mobile robot navigating a maze using:

- odometry for robot pose estimation
- range-based obstacle avoidance
- potential field navigation for combining attractive and repulsive forces
- Gazebo Sim for simulation

The robot starts near the bottom-left of the maze and navigates toward a target goal while avoiding walls.

## Features

- TurtleBot3 Burger simulation in Gazebo
- obstacle avoidance using range sensor data
- odometry-based goal-directed navigation
- potential field planner in Python
- ROS 2 ↔ Gazebo topic bridging using `ros_gz_bridge`
- goal reaching with stop threshold

## Tech Stack

- ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz`
- Python
- TurtleBot3

## Repository Structure

This repository contains the **custom project package only**:

- `launch/maze_sim.launch.py` — launches Gazebo, spawns the robot, starts the bridge, and runs the planner
- `maze_navigation/potential_field_planner.py` — potential field navigation logic
- `worlds/simple_maze.world` — maze world
- `config/maze_gui.config` — Gazebo GUI config

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

## Install Dependencies

Install the required ROS / Gazebo packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-dynamixel-sdk
```

## Workspace Setup

This repository is intended to live inside a ROS 2 workspace alongside TurtleBot3 packages.

Create a workspace:

```bash
mkdir -p ~/ros2_project_ws/src
cd ~/ros2_project_ws/src
```

Clone this repository into the workspace `src` folder:

```bash
git clone https://github.com/WalidAlsafadi/ros2-maze-navigation.git maze_navigation
```

Clone the required TurtleBot3 repositories:

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## Build

```bash
cd ~/ros2_project_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_project_ws/install/setup.bash
```

## Run

Launch the maze simulation and planner:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_project_ws/install/setup.bash
ros2 launch maze_navigation maze_sim.launch.py
```

## Custom Goal

You can launch with a custom goal:

```bash
ros2 launch maze_navigation maze_sim.launch.py goal_x:=9.0 goal_y:=9.0
```

## How It Works

The planner computes:

- an **attractive force** toward the goal
- a **repulsive force** away from nearby obstacles

These forces are combined into a desired heading and converted into velocity commands using odometry and sensor data.

## Notes

- On WSL, Gazebo may feel somewhat laggy depending on GPU and system resources.
- The Gazebo **Reset Simulation** button may not fully reset the behavior while the planner node is still running; a relaunch is usually the cleanest reset.
- The project was tested primarily through Gazebo simulation and terminal-based debugging.

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

### Gazebo opens but robot does not move

Check that:

- the workspace was built successfully
- you sourced both:
  - `/opt/ros/jazzy/setup.bash`
  - `~/ros2_project_ws/install/setup.bash`

## Future Improvements

- complex maze / bonus version
- local minima escape strategy
- cleaner TF and visualization pipeline
- tuning and performance improvements

## Acknowledgment

This work builds on the starter skeleton provided in **DSAI 4304: Robot Simulation** by **Dr. Marwan Radi**. The provided skeleton served as the initial foundation for the package structure and project setup, while the ROS 2 launch integration, Gazebo bridging, TurtleBot3 setup, potential field navigation logic, and testing were completed in this implementation.
