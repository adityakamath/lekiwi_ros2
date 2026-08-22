# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted_%7C_Jazzy-blue?logo=ros)](https://docs.ros.org)
[![CI](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

> ROS 2 software stack for LeKiwi 3-wheel omnidirectional mobile robot and payloads.

## ⚠️ Safety

**This is a real, motorized robot with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button or Foxglove) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes and incomplete safety coverage. Simulation (`sim:=true`) has verified motion control (see [Simulation](#simulation)) but is not validated against real hardware behavior. No warranty, express or implied — see [LICENSE](LICENSE).

## Overview

Omnidirectional mobile robot platform built with ROS 2 and ros2_control. Features holonomic drive, odometry, teleoperation, real-time motor diagnostics, battery monitoring, and spoken status announcements for controls and safety. Includes LiDAR and an OAK-D depth camera for perception.

## Packages

- **lekiwi_bringup** — Top-level launch files that bring up the full system (control, navigation, sensors, payload) based on `payload`/`sim`/`mission` and other arguments.
- **lekiwi_description** — URDF and MJCF robot models, meshes, and visualization launch files.
- **lekiwi_control** — ros2_control hardware interfaces, controller configs, and launch files (real, mock, or MuJoCo).
- **lekiwi_navigation** — SLAM (slam_toolbox), localization (AMCL), Nav2, EKF sensor fusion, and map storage.
- **lekiwi_audio** — Spoken status announcements for e-stop, mode switching, waypoint actions, and battery threshold events.

### Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: CI-tested on Kilted and Jazzy
- **[ros2_control](https://control.ros.org/)** framework with standard controllers
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)** (git submodule under `modules/`): Hardware interface for Feetech STS servos
- **[bno055_hardware_interface](https://github.com/adityakamath/bno055_hardware_interface)** (git submodule under `modules/`): Hardware interface for the BNO055 IMU
- **[ldlidar_ros2](https://github.com/adityakamath/ldlidar_ros2)** (git submodule under `modules/`): LD06 LiDAR driver with bug fixes
- **[ina260_ros2](https://github.com/adityakamath/ina260_ros2)** (git submodule under `modules/`): INA260 battery current/voltage/power monitoring, with threshold-based SetBool event services
- **[laser_filters](https://github.com/ros-perception/laser_filters)**, **[Nav2](https://docs.nav2.org/)**, **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)**, **[robot_localization](https://github.com/cra-ros-pkg/robot_localization)**: Laser filtering, navigation/SLAM, and EKF sensor fusion (`lekiwi_navigation`)
- **[joy](https://github.com/ros-drivers/joystick_drivers)** / **[joy_teleop](https://index.ros.org/p/joy_teleop/)**: Joystick teleoperation
- **[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)** (`sudo apt install ros-kilted-mujoco-ros2-control`): MuJoCo simulation backend, `sim:=true` only
- **[pantilt_ros2](https://github.com/adityakamath/pantilt_ros2)** (git submodule under `payloads/`): Pan-tilt + OAK-D camera payload — see its [README](payloads/pantilt_ros2/README.md) for its own dependencies (depthai-ros, cloudini, etc.)

## Installation and Usage

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-up-to lekiwi_bringup
source install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py
```

## Stable Device Names (udev)

Real hardware (not `sim:=true`) depends on three USB devices showing up at fixed paths rather than whatever `/dev/ttyUSB0`-style name the kernel happens to assign on that boot:

| Symlink              | Device                             | Consumed by                                                       |
|-----------------------|-------------------------------------|----------------------------------------------------------------------|
| `/dev/ttyLIDAR`       | LD06 LiDAR (CP210x USB-UART)        | `lekiwi_bringup/config/laser.yaml` → `port_name`                     |
| `/dev/ttySERVO`       | Feetech STS servo bus               | `lekiwi_control/config/base/urdf_config.yaml` → `serial_port`        |
| reSpeaker Flex ALSA nodes | XVF3800 USB audio               | `lekiwi_audio` selects it by ALSA card name; the rule only grants non-root permission, it doesn't rename the card |

Install once per robot:

```bash
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Reconnect the devices (or reboot) afterward. Without this, `/dev/ttyLIDAR` and `/dev/ttySERVO` won't exist and the launch defaults above will fail to open their serial ports.

## Launch Arguments

The most commonly used arguments for `lekiwi_bringup lekiwi.launch.py` (run with `--show-arguments` for the full list):

| Argument         | Default   | Description                                                    |
|------------------|-----------|------------------------------------------------------------------|
| `payload`        | `pantilt` | `""` for base only, `pantilt` for base + pan-tilt + OAK-D        |
| `pantilt_config` | `pt101`   | Pan-tilt mesh variant: `pt100` or `pt101`                        |
| `mission`        | `""`      | Navigation mode: `map` (SLAM mapping), `slam`/`amcl` (localization, needs `map_name`) |
| `map_name`       | `""`      | Map to load, e.g. `livingroom1`                                  |
| `fusion_mode`    | `base`    | EKF sensor fusion: `base`, `imu`, or `odom`. `odom` required when `imu:=false` |
| `imu`            | `true`    | Physical BNO055 present on the base (honored on real hardware and `sim:=true`/MuJoCo). `false` needs `fusion_mode:=odom` |
| `laser`          | `true`    | Physical LD06 LiDAR present on the base. `false` skips `laser.launch.py`'s include entirely |
| `audio`          | `true`    | Physical reSpeaker mic array present on the base. `false` skips the `lekiwi_audio` launch include entirely |
| `battery_monitor` | `true`   | Physical INA260 current/voltage sensor present on the base. `false` skips `ina260_ros2`'s launch include entirely |
| `pointcloud`     | `false`   | Enable RGBD point cloud output from the OAK-D camera              |
| `sim`            | `false`   | Run in MuJoCo instead of real hardware                            |
| `gui`            | `true`    | [`sim` only] Show the MuJoCo viewer                                |
| `diagnostics`    | `false`   | Launch motor/IMU diagnostics nodes                                |
| `joy`            | `false`   | Launch `joy_node` on this device (set `true` if the joystick is plugged in locally) |

## Joystick Configuration

Teleoperation is configured for a **Steam Deck** used as a generic joystick, not through Steam Input, so button/axis numbers below are specific to that interface.

**Drive** (requires the L1 deadman held):

| Control            | Action                  |
|---------------------|--------------------------|
| L1                  | Deadman                  |
| Left stick          | Forward/back, strafe     |
| Right stick         | Rotate in place          |

**Pan-tilt** (`payload:=pantilt`, shares the L1 deadman): D-pad for pan/tilt.

**Other controls**:

| Control     | Action                                              |
|--------------|-------------------------------------------------------|
| B            | Toggle emergency stop                                  |
| X            | Toggle between teleop and Nav2 control                 |
| R1           | Hold to disable the collision monitor's predictive stop |
| Screenshot   | Save the current SLAM map                              |
| Y / A / Settings | Record / toggle / reset a waypoint patrol           |

All of the above (plus navigation goal outcomes) get spoken feedback via `lekiwi_audio` (disabled when `audio:=false`).

## Costmap Zones

Nav2 no-go and speed-limited zones are supported. Zone masks live under `lekiwi_navigation/maps/<map_name>/filters/` and are saved automatically alongside each map.

## Simulation

`sim:=true` runs against MuJoCo instead of real hardware. Verified working end-to-end for both `payload:=""` and `payload:=pantilt`: wheel and pan-tilt commands drive real simulated physics, `imu_sensor_broadcaster` publishes real IMU data (including gravity), and odometry/TF update correctly — a single `payload:=pantilt` `controller_manager` was confirmed driving wheels and pan-tilt concurrently without interference, matching the shared-bus design (see [pantilt_ros2 README](payloads/pantilt_ros2/README.md#launch-time-bring-up-on-a-shared-bus)). Not validated for fidelity against real hardware behavior — only that the `ros2_control` integration itself works.

The simulated OAK-D camera does not work — same `mujoco_ros2_control_node` GLFW init failure documented in [pantilt_ros2's Simulation section](payloads/pantilt_ros2/README.md#simulation) — so `oakd`/`pointcloud`/`octomap` have no simulated equivalent; `sim:=true` skips them entirely (see [Launch Arguments](#launch-arguments)).

## Structure

```text
lekiwi_ros2/
├── lekiwi_control/      # Control, diagnostics, launch files
├── lekiwi_description/  # URDF models and meshes
├── lekiwi_navigation/   # SLAM, localization, EKF sensor fusion, maps
├── lekiwi_bringup/      # System integration launch files
├── lekiwi_audio/        # Spoken status announcements (e-stop, mode switching, waypoints)
├── modules/
│   ├── sts_hardware_interface/     # Feetech STS servo hardware interface (git submodule)
│   ├── bno055_hardware_interface/  # BNO055 IMU hardware interface (git submodule)
│   ├── ldlidar_ros2/               # LD06 LiDAR driver (git submodule)
│   └── ina260_ros2/                # INA260 battery monitoring (git submodule)
└── payloads/
    └── pantilt_ros2/    # Pan-tilt + OAK-D camera payload (git submodule)
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
