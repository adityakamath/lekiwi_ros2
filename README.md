# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted_%7C_Jazzy-blue?logo=ros)](https://docs.ros.org)
[![CI](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/lekiwi_ros2)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for the LeKiwi 3-wheel omnidirectional mobile robot and its payloads. It provides holonomic drive with odometry, joystick teleoperation, SLAM and Nav2 navigation, motor diagnostics, battery monitoring, spoken status announcements and a MuJoCo simulation. The base has a LiDAR, and optional payloads add more; the pan-tilt payload, for example, brings an OAK-D depth camera.

## ⚠️ Safety

**This is a real, motorized robot with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button or Foxglove) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes and incomplete safety coverage. Simulation (`sim:=true`) has verified motion control (see [Simulation](#simulation)) but is not validated against real hardware behavior. No warranty, express or implied — see [LICENSE](LICENSE).

## Contents

| Package | Purpose |
|---------|---------|
| [`lekiwi_bringup`](lekiwi_bringup/README.md) | Top-level launch files that bring up the whole system, and the udev rules |
| [`lekiwi_control`](lekiwi_control/README.md) | `ros2_control` setup, controllers, joystick teleop, diagnostics and emergency-stop handling |
| [`lekiwi_description`](lekiwi_description/README.md) | URDF models and meshes for the base and each payload |
| [`lekiwi_mujoco`](lekiwi_mujoco/README.md) | MuJoCo models generated from the URDF, and a standalone (no ROS) viewer |
| [`lekiwi_navigation`](lekiwi_navigation/README.md) | Nav2, SLAM (slam_toolbox), localization (AMCL), EKF sensor fusion, maps and waypoint patrol |
| [`lekiwi_audio`](lekiwi_audio/README.md) | Spoken status announcements (e-stop, mode switching, waypoints, battery) |

These are separate repositories, included as git submodules under `modules/` and `payloads/`:

| Repository | Purpose |
|------------|---------|
| [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) | `ros2_control` hardware interface for the Feetech STS servos |
| [`bno055_hardware_interface`](https://github.com/adityakamath/bno055_hardware_interface) | `ros2_control` hardware interface for the BNO055 IMU |
| [`ldlidar_ros2`](https://github.com/adityakamath/ldlidar_ros2) | LD06 LiDAR driver with bug fixes |
| [`ina260_battery_monitor`](https://github.com/adityakamath/ina260_battery_monitor) | INA260 battery voltage, current and power, with threshold events |
| [`mujoco_ros2_plugins`](https://github.com/adityakamath/mujoco_ros2_plugins) | `mujoco_ros2_control` plugins, currently the simulated `/emergency_stop` (`sim:=true` only) |
| [`pantilt_ros2`](https://github.com/adityakamath/pantilt_ros2) (`payloads/`) | Pan-tilt and OAK-D camera payload, with its own MuJoCo model |

## Hardware

| Component | Details |
|-----------|---------|
| Drive | Three Feetech STS3215 servo motors (IDs 7 left, 8 back, 9 right) on one serial bus at 1 Mbaud |
| LiDAR | LD06 |
| IMU | BNO055 |
| Battery monitor | INA260 current and voltage sensor |
| Microphone array | reSpeaker Flex (XVF3800), for the spoken announcements |
| Controller | A Steam Deck used as a generic joystick (see [Joystick](#joystick)) |
| Payload | Optional, for example the [pan-tilt](https://github.com/adityakamath/pantilt_ros2) with an OAK-D S2 camera |

### Stable device names (udev)

Real hardware (not `sim:=true`) needs three USB devices at fixed paths, rather than whichever `/dev/ttyUSB0`-style name the kernel assigns on that boot:

| Symlink | Device | Used by |
|---------|--------|---------|
| `/dev/ttyLIDAR` | LD06 LiDAR (CP210x USB-UART) | `lekiwi_bringup/config/laser.yaml` (`port_name`) |
| `/dev/ttySERVO` | Feetech STS servo bus | `lekiwi_control/config/urdf_config.yaml` (`serial_port`) |
| reSpeaker Flex ALSA nodes | XVF3800 USB audio | `lekiwi_audio` selects it by ALSA card name; the rule only grants non-root permission and does not rename the card |

Install the rules once per robot, then reconnect the devices or reboot:

```bash
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Without them, `/dev/ttyLIDAR` and `/dev/ttySERVO` do not exist and the default serial ports fail to open.

## Installation

Requires [ROS 2](https://docs.ros.org/en/kilted/) (CI-tested on Kilted and Jazzy) with:

- [`ros2_control`](https://control.ros.org/) and its standard controllers
- [Nav2](https://docs.nav2.org/), [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), [robot_localization](https://github.com/cra-ros-pkg/robot_localization) and [laser_filters](https://github.com/ros-perception/laser_filters)
- [`joy`](https://github.com/ros-drivers/joystick_drivers) and [`joy_teleop`](https://index.ros.org/p/joy_teleop/)

Clone with the submodules and build:

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/lekiwi_ros2.git
cd ~/ros2_ws
colcon build --packages-up-to lekiwi_bringup
source install/setup.bash
```

The pan-tilt payload has its own dependencies (`depthai-ros`, `cloudini` and others); see its [README](payloads/pantilt_ros2/README.md).

### Simulation (optional)

`sim:=true` needs the MuJoCo packages:

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins
pip install -r lekiwi_mujoco/requirements.txt    # into the interpreter that ROS launch uses
```

See the [`lekiwi_mujoco` README](lekiwi_mujoco/README.md) for the full list.

## Running

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py                                   # real robot with the pan-tilt payload
ros2 launch lekiwi_bringup lekiwi.launch.py payload:=""                       # base only
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=map                      # SLAM mapping
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=amcl map_name:=livingroom1   # localize on a saved map
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home     # MuJoCo simulation
```

### Launch arguments

The most common arguments for `lekiwi.launch.py` (`--show-arguments` lists them all):

| Argument | Default | Description |
|----------|---------|-------------|
| `payload` | `pantilt` | Payload to mount: `""` for the base alone, or a payload name (see [Payloads](#payloads)) |
| `mission` | `""` | Navigation mode: `map` (SLAM mapping), `slam` or `amcl` (localization, needs `map_name`) |
| `map_name` | `""` | Map to load, e.g. `livingroom1` |
| `wp_loops` | `0` | Waypoint patrol passes per start: `0` loops forever, `N>0` runs `N` passes |
| `fusion_mode` | `base` | EKF sensor fusion: `base`, `imu` or `odom`; `odom` is required when `imu:=false` |
| `imu` | `true` | A physical BNO055 is present (honored on hardware and in simulation); `false` needs `fusion_mode:=odom` |
| `laser` | `true` | A physical LD06 is present; `false` skips the LiDAR launch |
| `audio` | `true` | A physical reSpeaker is present; `false` skips `lekiwi_audio` |
| `battery_monitor` | `true` | A physical INA260 is present; `false` skips the battery monitor |
| `sim` | `false` | Run in MuJoCo instead of on hardware |
| `mujoco_scene` | `flat` | `sim` only: `flat`, `arena`, `home` (furnished apartment with pushable props), `maze`, `none` or a scene file |
| `mujoco_gui` | `false` | `sim` only: show the MuJoCo viewer (headless otherwise, no display or GPU needed) |
| `diagnostics` | `false` | Launch the motor and IMU diagnostics nodes |
| `joy` | `false` | Launch `joy_node` on this device (set `true` if the joystick is plugged in locally) |

Payload-specific arguments (`payload:=pantilt`): `pantilt_config` (`pt101` default, or `pt100`) selects the mesh variant, and `pointcloud:=true` enables the OAK-D point cloud.

### Joystick

Teleoperation is set up for a **Steam Deck** used as a generic joystick, not through Steam Input, so the button and axis numbers are specific to that interface.

| Control | Action |
|---------|--------|
| L1 (hold) | Deadman: drive and payload commands are only sent while it is held |
| Left stick | Forward, back and strafe |
| Right stick | Rotate in place |
| D-pad | Pan and tilt, with `payload:=pantilt` |
| B | Toggle emergency stop |
| X | Toggle between teleop and Nav2 control |
| R1 (hold) | Disable the collision monitor's predictive stop |
| Screenshot | Save the current SLAM map |
| Y / A / Settings | Record / toggle / reset a waypoint patrol |

All of these, plus navigation goal outcomes, are announced by `lekiwi_audio` (unless `audio:=false`). To test the patrol announcements without the infinite loop, launch with `wp_loops:=1`.

## Configuration

| File | What it sets |
|------|--------------|
| [`lekiwi_control/config/urdf_config.yaml`](lekiwi_control/config/urdf_config.yaml) | Servo serial port, motor IDs, mock mode, servo speed profile |
| [`lekiwi_control/config/control.yaml`](lekiwi_control/config/control.yaml) | Drive controller and controller manager |
| [`lekiwi_control/config/base_teleop.yaml`](lekiwi_control/config/base_teleop.yaml) | Joystick buttons and axes, and the base's speed limits |
| [`lekiwi_bringup/config/laser.yaml`](lekiwi_bringup/config/laser.yaml) | LiDAR serial port and settings |
| [`lekiwi_navigation/config/nav2/`](lekiwi_navigation/config/nav2/) | Nav2, AMCL, slam_toolbox, map saver and waypoint patrol settings |
| [`lekiwi_navigation/config/robot_localization/`](lekiwi_navigation/config/robot_localization/) | EKF sensor fusion for each `fusion_mode` |
| [`lekiwi_audio/config/phrases.yaml`](lekiwi_audio/config/phrases.yaml) | The spoken phrases |

The base's speed limits are the joystick axis scales in `base_teleop.yaml`; the drive controller enforces none, so `nav2.yaml`'s `velocity_smoother` must match them.

### Costmap zones

Nav2 no-go and speed-limited zones are supported. The zone masks live under `lekiwi_navigation/maps/<map_name>/filters/` and are saved automatically with each map.

## Simulation

`sim:=true` runs the same launch files against MuJoCo instead of the hardware, headless by default (`mujoco_gui:=true` opens the viewer and needs a display). The model is generated at launch from the URDF and controller configuration by [`lekiwi_mujoco`](lekiwi_mujoco/README.md), which mounts the payload's own model. The simulated hardware covers the wheels, IMU, LiDAR (`/scan`), the emergency stop, and the payload's joints and sensors (with the pan-tilt, its camera images). The battery monitor and audio are not simulated and are skipped.

Motion control is verified end to end: a single `controller_manager` drives the wheels and the payload together, the IMU publishes real data including gravity, and odometry and TF update correctly. The simulation has not been compared against real hardware. To watch it from another machine, run `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`.

## Payloads

The base runs on its own (`payload:=""`). Optional payloads, each in its own repository under `payloads/`, mount on top of it; `payload:=<name>` selects one. Currently that is `pantilt` ([pantilt_ros2](https://github.com/adityakamath/pantilt_ros2): pan-tilt + OAK-D camera), which is also the reference to copy. Following the same layout, users can add their own payloads, such as the SO-101 arm.

How a payload is wired in (`<name>` is the `payload` value):

1. **Registered** in `_VALID_PAYLOADS` in `lekiwi_bringup/launch/lekiwi.launch.py`, which forwards `payload` to the control launch (which also starts teleop) and the laser launch.
2. **Description:** the payload repository provides a URDF module with a fixed mount joint; `lekiwi_description/urdf/base_<name>/` combines it with the base.
3. **Control and teleop:** the payload's own control package supplies its controller (`pantilt_controller.yaml`, which `lekiwi_control` gives to the spawner with `--param-file`) and its servo profile, so nothing is copied into lekiwi and one `controller_manager` still drives the base and the payload. `lekiwi_control` keeps only what depends on the host, the joystick layout in `config/<name>_teleop.yaml`, loaded on top of `base_teleop.yaml`.
4. **Sensors:** a payload that blocks part of the LiDAR's view adds `lekiwi_bringup/config/<name>_laser_filter.yaml`, which `laser.launch.py` applies automatically.
5. **Simulation:** the payload's MuJoCo package builds its model from the robot's URDF (for `pantilt`, `pt_mujoco`'s `build_payload_spec`), and `lekiwi_mujoco` attaches it at the URDF's mount joint.

A few launch-file branches still test for `pantilt` explicitly (in `lekiwi.launch.py` and `control.launch.py`), so adding a payload means extending them as well as providing the pieces above.

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
