# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted-blue?logo=ros)](https://docs.ros.org)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Website](https://img.shields.io/badge/Website-kamathrobotics.com-blue)](https://kamathrobotics.com)

> This repository is a work in progress and includes experimental and AI-generated content. Proceed with caution.

> ROS 2 software stack for LeKiwi 3-wheel omnidirectional mobile robot.

## Overview

Omnidirectional mobile robot platform built with ROS 2 and ros2_control. Features holonomic drive, odometry, teleoperation, and real-time motor diagnostics for controls and safety. Includes LiDAR and camera drivers for perception.

## Installation and Usage

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-select lekiwi_base_control lekiwi_description lekiwi_bringup
source install/setup.bash
ros2 launch lekiwi_bringup base.launch.py
```

## Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: Tested with Kilted, but should work on other ROS 2 distributions
- **[ros2_control](https://control.ros.org/)** framework with standard controllers
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)**: Hardware interface for Feetech STS servos
- **[ldlidar_ros2](https://github.com/adityakamath/ldlidar_ros2)**: LD06 LiDAR driver with bug fixes
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
