# Development of a Four-Wheeled Differential Drive Robot for Simulation and Analysis

## Overview
This repository contains the complete implementation of a four-wheeled differential drive mobile robot developed using **ROS 2** and **Gazebo**. The project focuses on robot modeling, simulation, coordinate frame management, sensor integration, and system bring-up for analysis and visualization purposes.

The robot was designed, simulated, and validated entirely in a virtual environment, with emphasis on correct kinematics, TF structure, topic communication, and realistic sensor behavior.

---

## Project Goals and Tasks

### Goals
- Design a four-wheeled differential drive robot model
- Simulate the robot in a physics-based environment
- Enable motion control using velocity commands
- Visualize robot state, TF frames, and sensor data
- Ensure modular and reproducible execution using ROS 2 tools

### Tasks Completed
- Created a URDF/Xacro-based robot description
- Configured differential drive motion control
- Integrated Gazebo simulation support
- Added sensor plugins (camera and lidar)
- Set up TF frames for correct spatial relationships
- Verified ROS 2 node, topic, and TF behavior in RViz
- Prepared the project for containerized execution (Docker-ready)

---

## System Architecture

### Core Components
- **Robot Description**: URDF/Xacro model defining links, joints, and inertial properties
- **Control**: Differential drive control via `/cmd_vel`
- **Simulation**: Gazebo world with physics and sensor plugins
- **Visualization**: RViz for TF frames, robot model, and sensor data
- **Bringup**: Launch files for coordinated startup

---

## Implemented Features

- Differential drive kinematics
- Velocity-based motion control
- Camera sensor plugin
- Lidar sensor plugin
- TF tree with base, wheels, and sensor frames
- ROS 2 topic and node communication
- Gazebo–ROS 2 integration
- Launch-based system orchestration

---

## Repository Structure

```text
ros2_differential_drive_robot/
├── src/
│   ├── my_robot_description/
│   ├── my_robot_bringup/
│   ├── my_robot_gazebo/
│   └── my_robot_control/
├── install/
├── build/
├── log/
├── Dockerfile
└── README.md
```

## Requirements
- Ubuntu 22.04  
- ROS 2 Jazzy  
- Gazebo Harmonic  
- colcon  
- RViz2  
- (Optional) Docker  

---

## Build Instructions

From the root of the ROS 2 workspace:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

After building:
```bash
source install/setup.bash
```

---

## Running the Project

Launch the full system:
```bash
ros2 launch my_robot_bringup my_robot_launch.xml
```

This will:
- Start Gazebo with the robot loaded  
- Enable differential drive control  
- Spawn sensors  
- Publish TF frames  
- Make the robot available for visualization in RViz  

---

## Example Commands

Move the robot:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

Inspect active nodes:
```bash
ros2 node list
```

Inspect topics:
```bash
ros2 topic list
```

---

## Results and Demonstrations

### Simulation Video
Add your simulation video here  
`[wheeled_robot.webm]`

### Screenshots
**Robot in Gazebo**  
`[wheeled_robot.png]`


---

## Validation and Observations
- Robot motion matches expected differential drive behavior  
- TF frames remain consistent during motion  
- Sensor data aligns correctly with optical and base frames  
- No frame drift or control instability observed  
- System launches reproducibly using a single launch file  

---

## Notes
- The project is fully simulation-based  
- All components were implemented and configured manually  
- The setup is compatible with Docker-based execution  
- The workspace can be rebuilt cleanly after system restart  

---

## Potential Extensions
- Autonomous navigation  
- SLAM integration  
- Controller tuning and performance comparison  
- Multi-robot simulation  
- Real hardware deployment  

---

## Author
Developed as an independent robotics simulation project using ROS 2 and Gazebo.

---

## License
This project is intended for academic and educational use.
