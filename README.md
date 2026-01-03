# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble+-blue?logo=ros)](https://docs.ros.org)
![GitHub License](https://img.shields.io/github/license/adityakamath/lekiwi_ros2)
[![Website](https://img.shields.io/badge/Website-kamathrobotics.com-blue)](https://kamathrobotics.com)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

> Complete ROS 2 software stack for the LeKiwi 3-wheel omnidirectional mobile robot.

## Overview

LeKiwi is a compact omnidirectional mobile robot platform built with ROS 2 and ros2_control. This repository contains all ROS 2 packages for robot control, sensor integration, and robot description.

### Features

- **Omnidirectional mobility** - Holonomic drive with independent X, Y, and rotational control
- **Standard ROS 2 interfaces** - Built on ros2_control framework with standard controllers
- **Position-based odometry** - Multi-turn encoder feedback for accurate odometry
- **Real-time control** - 50 Hz control loop for responsive operation
- **Emergency stop** - Broadcast stop functionality for safety

### Hardware

- **Platform**: 3-wheel omnidirectional robot (120° spacing)
- **Motors**: Feetech STS3215 servo motors (IDs: 7, 8, 9)
- **Communication**: Serial bus at 1000000 baud via `/dev/ttySERVO`
- **Sensors**: HD webcam, LD06 LiDAR
- **Wheel Layout**: 60°, 180°, 300° from +X axis

## Packages

### lekiwi_base_control

Robot control package providing hardware interface and controllers.

**Key Components:**
- Omnidirectional drive controller (cmd_vel → wheel velocities)
- Joint state broadcaster (motor state publishing)
- Hardware interface (STS3215 servo communication)

**Launch:**
```bash
ros2 launch lekiwi_base_control base_control.launch.py
```

**Documentation:**
- [Getting Started](lekiwi_base_control/docs/getting_started.md)
- [Architecture](lekiwi_base_control/docs/architecture.md)

### lekiwi_description

Robot description package containing URDF models and meshes.

**Contents:**
- URDF xacro files for robot structure
- STL meshes for visualization
- Robot geometry and kinematics

### lekiwi_bringup

System integration and sensor driver package.

**Contents:**
- Sensor launch files (webcam, LiDAR)
- System integration configurations

## Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-select lekiwi_base_control lekiwi_description lekiwi_bringup
```

### Basic Usage

```bash
# Launch the robot control system
ros2 launch lekiwi_base_control base_control.launch.py

# Send velocity commands (0.1 m/s forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}"

# Monitor odometry
ros2 topic echo /odom

# View motor states
ros2 topic echo /joint_states
```

### Omnidirectional Movement

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Strafe right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.1}}"

# Rotate counter-clockwise
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Combined motion (forward + rotate)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.5}}"
```

## Dependencies

- **ROS 2**: Humble, Jazzy, or Kilted
- **ros2_control**: Hardware abstraction framework
- **ros2_controllers**: Standard controller implementations
- **sts_hardware_interface**: Hardware interface for Feetech STS servos

## Repository Structure

```text
lekiwi_ros2/
├── lekiwi_base_control/      # Robot control package
│   ├── config/                # Controller configurations
│   ├── launch/                # Launch files
│   ├── urdf/                  # Robot description with ros2_control
│   └── docs/                  # Documentation
├── lekiwi_description/        # Robot description package
│   ├── urdf/                  # Base URDF files
│   └── meshes/                # STL mesh files
└── lekiwi_bringup/            # System integration package
    └── launch/                # Sensor launch files
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
