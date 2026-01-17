# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble+-blue?logo=ros)](https://docs.ros.org)
![GitHub License](https://img.shields.io/github/license/adityakamath/lekiwi_ros2)
[![Website](https://img.shields.io/badge/Website-kamathrobotics.com-blue)](https://kamathrobotics.com)

> This repository is a work in progress and includes experimental and some(unreviewed, untested) AI-generated content. Proceed with caution.

> ROS 2 software stack for LeKiwi 3-wheel omnidirectional mobile robot.

## Overview

Omnidirectional mobile robot platform built with ROS 2 and ros2_control framework. Features holonomic drive, odometry, teleoperation, and real-time motor diagnostics.

## Packages

### lekiwi_base_control

Robot control with ros2_control framework. Includes omni wheel drive controller, joint state broadcaster, and motor diagnostics node.

- **Launch**: `ros2 launch lekiwi_base_control base_control.launch.py`
- **Docs**: [Getting Started](lekiwi_base_control/docs/getting_started.md) · [Architecture](lekiwi_base_control/docs/architecture.md)

### lekiwi_description

URDF models, xacro files, and STL meshes for robot visualization and kinematics.

### lekiwi_bringup

System integration launch files for complete robot startup (base control + sensors).

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-select lekiwi_base_control lekiwi_description lekiwi_bringup
source install/setup.bash
```

## Usage

```bash
# Launch robot control
ros2 launch lekiwi_base_control base_control.launch.py

# Launch complete system (control + sensors)
ros2 launch lekiwi_bringup base.launch.py
```

## Dependencies

- **ROS 2**: Humble onwards, tested with Kilted
- **ros2_control** framework with standard controllers
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)**: Hardware interface for Feetech STS servos
- **[ldlidar_ros2](https://github.com/ldrobotSensorTeam/ldlidar_ros2)**: LD06 LiDAR driver
- **[camera_ros](https://github.com/ros-drivers/camera_ros)**: USB camera driver

## Structure

```text
lekiwi_ros2/
├── lekiwi_base_control/    # Control, diagnostics, launch files
├── lekiwi_description/     # URDF models and meshes
└── lekiwi_bringup/         # System integration launch files
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
