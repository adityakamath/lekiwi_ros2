# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted-blue?logo=ros)](https://docs.ros.org)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Website](https://img.shields.io/badge/Website-kamathrobotics.com-blue)](https://kamathrobotics.com)

> This repository is a work in progress and includes experimental and AI-generated content. Proceed with caution.

> ROS 2 software stack for LeKiwi 3-wheel omnidirectional mobile robot.

## Overview

Omnidirectional mobile robot platform built with ROS 2 and ros2_control. Features holonomic drive, odometry, teleoperation, and real-time motor diagnostics for controls and safety. Includes LiDAR and an OAK-D depth camera for perception.

## Installation and Usage

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-up-to lekiwi_bringup
source install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py
```

## Launch Arguments

`lekiwi_bringup lekiwi.launch.py` accepts the following arguments:

| Argument              | Default   | Description                                                                                                     |
|-----------------------|-----------|------------------------------------------------------------------------------------------------------------------|
| `payload`             | `pantilt` | Hardware payload: `""` for base only, `pantilt` for base + pan-tilt + OAK-D                                       |
| `pantilt_config`      | `pt100`   | Pan-tilt mesh variant when `payload:=pantilt`: `pt100` or `pt101`                                                 |
| `control_diagnostics` | `false`   | Launch motor and IMU diagnostics nodes in `lekiwi_control`                                                        |
| `control_mock`        | `""`      | Mock mode override (`true`/`false`); empty means use `urdf_config.yaml` value                                     |
| `fusion_mode`         | `base`    | EKF sensor fusion: `base` (wheel odom + BNO055), `imu` (BNO055 only), `odom` (wheel odom only)                    |
| `slam_mode`           | `map`     | Navigation mode: `map` (build new map), `localize` (slam_toolbox localization), `amcl` (AMCL + nav2_map_server)   |
| `map_name`            | `""`      | Map subdirectory to load for `localize`/`amcl` modes (required for those modes)                                  |
| `pointcloud`          | `false`   | Enable RGBD point cloud output from the OAK-D (uses `oakd_vio_pcl.yaml`)                                          |
| `joy`                 | `false`   | Launch `joy_node` locally; leave `false` when `/joy` is published from a remote device                           |
| `use_sim_time`        | `false`   | Use `/clock` from a simulator instead of system time                                                              |

## Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: Tested with Kilted, but should work on other ROS 2 distributions
- **[ros2_control](https://control.ros.org/)** framework with standard controllers
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)**: Hardware interface for Feetech STS servos
- **[bno055_hardware_interface](https://github.com/adityakamath/bno055_hardware_interface)**: Hardware interface for the BNO055 IMU
- **[ldlidar_ros2](https://github.com/adityakamath/ldlidar_ros2)**: LD06 LiDAR driver with bug fixes
- **[laser_filters](https://github.com/ros-perception/laser_filters)**, **[Nav2](https://docs.nav2.org/)**, **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)**, **[robot_localization](https://github.com/cra-ros-pkg/robot_localization)**: Laser filtering, navigation/SLAM, and EKF sensor fusion (`lekiwi_navigation`)
- **[joy](https://github.com/ros-drivers/joystick_drivers)** / **[joy_teleop](https://index.ros.org/p/joy_teleop/)**: Joystick teleoperation
- **[pantilt100](https://github.com/adityakamath/pantilt100)** (git submodule under `payloads/`): Pan-tilt + OAK-D camera payload — see its [README](payloads/pantilt100/README.md) for its own dependencies (depthai-ros, cloudini, etc.)

## Structure

```text
lekiwi_ros2/
├── lekiwi_control/      # Control, diagnostics, launch files
├── lekiwi_description/  # URDF models and meshes
├── lekiwi_navigation/   # SLAM, localization, EKF sensor fusion, maps
├── lekiwi_bringup/      # System integration launch files
└── payloads/
    └── pantilt100/      # Pan-tilt + OAK-D camera payload (git submodule)
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
