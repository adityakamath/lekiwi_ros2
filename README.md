# LeKiwi ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted_%7C_Jazzy-blue?logo=ros)](https://docs.ros.org)
[![CI](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/lekiwi_ros2/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for the LeKiwi 3-wheel omnidirectional mobile robot. It provides holonomic drive with odometry, joystick teleoperation, SLAM and Nav2 navigation, motor diagnostics, battery monitoring, spoken status announcements and a MuJoCo simulation.

## ⚠️ Safety

**This is a real, motorized robot with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button or Foxglove) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes. Simulation (`sim:=true`) is not validated against real hardware. No warranty, express or implied — see [LICENSE](LICENSE).

## Robot configurations

| Configuration | Top plate | Camera | Launch | Status |
|---------------|-----------|--------|--------|--------|
| **Base with tray (default)** | Container plate (a tray) | Static Gemini 2 on the front bracket | `lekiwi.launch.py` (default) | Maintained |
| Payload | Payload plate | What the payload brings | `payload:=<name>` | SO-101 arm in progress |
| Pan-tilt payload | Payload plate | Gemini 2 on the pan-tilt | `payload:=pantilt` | Legacy, no longer maintained |

## How it fits together

```text
joystick (/joy) -> joy_teleop -- /cmd_vel_teleop ---------.
                                                           v
Nav2 (planner, MPPI, velocity smoother) -- /cmd_vel_smoothed -> twist_switch_node
                                                                      | /cmd_vel_presafety
                                                                      v
                                                              collision_monitor
                                                                      | /base_controller/cmd_vel
                                                                      v
  omni_wheel_drive_controller -> ros2_control -> sts_hardware_interface -> 3 STS3215 wheel servos

wheels -- /base_controller/odom ------.
BNO055 -- /imu_sensor_broadcaster/imu -+-> ekf_node -> odom->base_footprint TF, /odometry/filtered -> Nav2
LD06 LiDAR -- /scan -> slam_toolbox or AMCL (map->odom), Nav2 costmaps, collision_monitor
Gemini 2 -- /gemini2/* (color, depth, /gemini2/scan, IMU)
```

| Term | Meaning |
|------|---------|
| Twist switch | `twist_switch_node` picks the velocity source: joystick (teleop, the start-up state) or Nav2; the **X** button toggles it |
| Collision monitor | Nav2's `collision_monitor`, placed after the switch: it limits both teleop and Nav2 commands; hold **R1** to disable its predictive stop |
| Deadman | Hold **L1** for joystick drive commands to be sent |
| `mission` | Navigation mode: `map` (build a map), `slam` or `amcl` (localize on a saved map, `map_name`) |
| `fusion_mode` | Which sensors the EKF fuses: `base` (wheel odometry and IMU), `imu` or `odom` |
| Tray / payload plate | The two interchangeable top plates (see [Robot configurations](#robot-configurations)) |

## Contents

| Package | Purpose |
|---------|---------|
| [`lekiwi_bringup`](lekiwi_bringup/README.md) | Top-level launch files that bring up the whole system, and the udev rules |
| [`lekiwi_control`](lekiwi_control/README.md) | `ros2_control` setup, controllers, joystick teleop, diagnostics and emergency-stop handling |
| [`lekiwi_description`](lekiwi_description/README.md) | URDF models and meshes for the base and each payload |
| [`lekiwi_mujoco`](lekiwi_mujoco/README.md) | MuJoCo models generated from the URDF, and a standalone (no ROS) viewer |
| [`lekiwi_navigation`](lekiwi_navigation/README.md) | Nav2, SLAM (slam_toolbox), localization (AMCL), EKF sensor fusion, maps and waypoint patrol |
| [`lekiwi_audio`](lekiwi_audio/README.md) | Spoken status announcements (e-stop, mode switching, waypoints, battery) |

Settings live in each package's `config/` folder; the package READMEs list them. These are separate repositories, included as git submodules under `modules/` and `payloads/`:

| Repository | Purpose |
|------------|---------|
| [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) | `ros2_control` interface for the Feetech STS servos |
| [`bno055_hardware_interface`](https://github.com/adityakamath/bno055_hardware_interface) | `ros2_control` interface for the BNO055 IMU |
| [`ldlidar_ros2`](https://github.com/adityakamath/ldlidar_ros2) | LD06 LiDAR driver with bug fixes |
| [`ina260_battery_monitor`](https://github.com/adityakamath/ina260_battery_monitor) | INA260 battery monitor, with threshold events |
| [`mujoco_ros2_plugins`](https://github.com/adityakamath/mujoco_ros2_plugins) | `mujoco_ros2_control` plugins: the simulated `/emergency_stop` service (`sim:=true` only) |
| [`pantilt_ros2`](https://github.com/adityakamath/pantilt_ros2) (`payloads/`, optional, no longer maintained) | Pan-tilt with a Gemini 2 camera and its own MuJoCo model; see [Pan-tilt payload (legacy)](#pan-tilt-payload-legacy) |

## Hardware

| Component | Details |
|-----------|---------|
| Drive | Three Feetech STS3215 servo motors (IDs 7 left, 8 back, 9 right) on one serial bus at 1 Mbaud |
| LiDAR | LD06 (the same device as the LD19; the driver's product name is `LDLiDAR_LD19`) |
| IMU | BNO055 |
| Battery monitor | INA260 current and voltage sensor |
| Microphone array | reSpeaker Flex (XVF3800), for the announcements |

### udev rules

The real robot needs stable device names for the LiDAR, the servo bus and the reSpeaker (details in [`lekiwi_bringup`](lekiwi_bringup/README.md#stable-device-names-udev)). Install the rules once per robot, then reconnect the devices or reboot:

```bash
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Installation

Requires ROS 2 Kilted or Jazzy (`ros2_control`, Nav2, slam_toolbox, robot_localization, laser_filters, `joy` and `joy_teleop` are installed by `rosdep` below). The simulation with `sim:=true` and the camera driver steps are for Kilted.

Clone with only the submodules the default configuration needs (leave `pantilt_ros2` uninitialized), install dependencies and build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/lekiwi_ros2.git
cd lekiwi_ros2
git submodule update --init modules/sts_hardware_interface modules/bno055_hardware_interface \
  modules/ldlidar_ros2 modules/ina260_battery_monitor
git submodule update --init modules/mujoco_ros2_plugins    # only for sim:=true
cd ~/ros2_ws
rosdep install --from-paths src/lekiwi_ros2 --ignore-src -r -y --rosdistro $ROS_DISTRO
colcon build --symlink-install --packages-up-to lekiwi_bringup
source install/setup.bash
```

- `rosdep` needs `sudo rosdep init && rosdep update` once. With `-r` it continues past two messages: `ament_python` has no rosdep rule, and `orbbec_camera` is unresolved until you build the driver (below).
- `--symlink-install` links the installed URDF, meshes and configs to the source tree, so edits take effect without a rebuild. Without it they are copies that go stale after an edit.
- **Real camera:** build the Orbbec Gemini 2 driver, see [`lekiwi_bringup`](lekiwi_bringup/README.md#gemini-2-driver-real-robot-only). Apply the two patches given there to the clone, or a workspace-wide `colcon build` fails on it.
- **Simulation (`sim:=true`):** `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins` and `pip install -r lekiwi_mujoco/requirements.txt`; see [`lekiwi_mujoco`](lekiwi_mujoco/README.md#requirements).

## Running

Try it in simulation first (no hardware, no camera driver):

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true      # add mujoco_gui:=true for the viewer, mujoco_scene:=home for a furnished apartment
```

Then check it is up and drive it, from a second terminal (`source install/setup.bash` first). The command publishes the same message the joystick sends, and has to repeat because the controller stops after 0.5 s without a command:

```bash
ros2 topic hz /scan                                         # LiDAR; also try /odometry/filtered and /imu_sensor_broadcaster/imu
ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}"
ros2 topic echo /odometry/filtered --field pose.pose.position   # x increases while it drives
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"   # stop all motors (false releases)
```

On the real robot, install the udev rules above, check the wiring and ports in the [`lekiwi_control` configuration](lekiwi_control/README.md#configuration), and keep the emergency-stop button (**B**) within reach.

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py                                   # real robot: base with tray and Gemini 2
ros2 launch lekiwi_bringup lekiwi.launch.py enable_camera:=false              # same, without the camera driver
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=map                      # SLAM mapping
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=amcl map_name:=livingroom1   # localize on a saved map
```

### Launch arguments

The most common arguments for `lekiwi.launch.py` (`--show-arguments` lists them all):

| Argument | Default | Description |
|----------|---------|-------------|
| `payload` | `""` | Payload to mount: `""` for the base with tray and Gemini 2, or a payload name (see [Payloads](#payloads)) |
| `mission` | `""` | Navigation mode: `map` (SLAM mapping), `slam` or `amcl` (localization, needs `map_name`). Empty maps from scratch, or uses AMCL if `map_name` is set |
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

Camera arguments: `enable_camera:=false` skips the real or simulated camera, `camera_fps` sets its rate (15 Hz default, simulation capped at 5 Hz) and `pointcloud:=true` adds a colored point cloud (off by default). See [`lekiwi_bringup`](lekiwi_bringup/README.md#camera).

### Joystick

Teleoperation uses a **Steam Deck** as a generic joystick (not through Steam Input). The button and axis numbers below are for that interface.

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
| Right joystick button (8) | Submit the edited Nav2 target via `/nav2_send_goal` (Nav2 mode only) |
| Y / A / Settings | Record / toggle / reset a waypoint patrol |

`lekiwi_audio` announces these (unless `audio:=false`).

`joy_teleop` reads `/joy`. Start `joy_node` on this machine with `joy:=true`, or run `ros2 run joy joy_node` on the machine the joystick is plugged into (same `ROS_DOMAIN_ID`). For another gamepad, run `ros2 topic echo /joy`, note which axis or button each control changes, and edit the numbers in [`base_teleop.yaml`](lekiwi_control/config/base_teleop.yaml).

## Simulation

`sim:=true` runs the same launch files against MuJoCo instead of the hardware, headless by default. [`lekiwi_mujoco`](lekiwi_mujoco/README.md) generates the model from the URDF; it covers the wheels, IMU, LiDAR, emergency stop and the Gemini 2. The battery monitor and audio are not simulated.

## Payloads

A payload replaces the tray: the base swaps its top plate for `payload_plate` and the payload mounts on it. Each payload lives in its own repository under `payloads/`; `payload:=<name>` selects it. The SO-101 arm is in progress. To add your own, see [using it on another robot](lekiwi_description/README.md#using-it-on-another-robot).

### Pan-tilt payload (legacy)

The pan-tilt runs the wheels in velocity mode and the pan and tilt servos in position mode on one servo bus with [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface). It is no longer maintained and stays at the commit pinned by the submodule. To use it:

1. `git submodule update --init payloads/pantilt_ros2`
2. Follow the [pan-tilt installation instructions](payloads/pantilt_ros2/README.md#installation).
3. `colcon build --packages-up-to lekiwi_bringup pt_description pt_control pt_mujoco pt_bringup`
4. `ros2 launch lekiwi_bringup lekiwi.launch.py payload:=pantilt` (the D-pad pans and tilts)

## Development

| To do this | Look here |
|------------|-----------|
| Change the launch or add a launch argument | `lekiwi_bringup/launch/lekiwi.launch.py` (forwards to `lekiwi_control/launch/control.launch.py` and `lekiwi_navigation/launch/navigation.launch.py`) |
| Change the robot model | `lekiwi_description/urdf/base/` (`base.urdf.xacro`, `base.module.xacro`, `base.common.xacro`) |
| Change joystick buttons or speed limits | `lekiwi_control/config/base_teleop.yaml` |
| Add a payload | `_VALID_PAYLOADS` in `lekiwi_bringup/launch/lekiwi.launch.py` and [using it on another robot](lekiwi_description/README.md#using-it-on-another-robot) |

After editing the URDF, regenerate the pre-built files:

```bash
python3 lekiwi_description/test/test_urdf_xacro.py --write       # urdf/base/base.urdf
cd lekiwi_mujoco && python3 -m lekiwi_mujoco.build_mujoco_models  # MuJoCo models
```

Run the tests from a built and sourced workspace, from the workspace root, after `pip install -r requirements-dev.txt` (lint with `ruff check .`), `pip install -r src/lekiwi_ros2/lekiwi_mujoco/requirements.txt` and `pip install -r src/lekiwi_ros2/lekiwi_audio/requirements-build.txt`:

```bash
pytest src/lekiwi_ros2/{lekiwi_control,lekiwi_navigation,lekiwi_description,lekiwi_mujoco,lekiwi_audio}/test \
  src/lekiwi_ros2/lekiwi_bringup/test/test_laser_filter.py
```

Each package README has its own test notes.

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
