# ROS2 Jazzy Mobile Robot Simulation

This project simulates a differential-drive mobile robot using ROS 2 (Jazzy Jalisco) with URDF, TF, RViz, and Ignition Gazebo (Harmonic). It models a mobile robot base, publishes its TF tree, and visualizes it in both RViz and Gazebo.


## Features

- URDF robot description with wheels and caster.
- Full TF frame broadcasting using `robot_state_publisher`.
- Visualization in RViz (TF and Robot Model).
- Simulation in Ignition Gazebo Harmonic with ROS-Gazebo bridge.
- Keyboard and joystick teleoperation supported.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Ignition Gazebo Harmonic
- `ros_gz` bridge packages

## Setup

```bash
# Clone the project
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your_repository_url.git>

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
