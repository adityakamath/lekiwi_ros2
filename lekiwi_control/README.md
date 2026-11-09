# LeKiwi Control

The `ros2_control` setup for LeKiwi: the wheel and IMU controllers, joystick teleop, the small nodes that make the joystick buttons and Nav2 share the drive safely, and the launch files that bring it all up on real hardware or in the MuJoCo simulation.

## Contents

| Path | Purpose |
|------|---------|
| `launch/control.launch.py` | The control stack: `robot_state_publisher`, the controller manager, the controllers, teleop and the support nodes; simulation extras with `ros2_control_hardware_type:=mujoco` |
| `launch/teleop.launch.py` | Joystick teleop (`joy_teleop`) on its own |
| `config/urdf_config.yaml` | Servo serial port, motor IDs, mock mode, servo speed profile |
| `config/control.yaml` | Controller manager (50 Hz), the drive controller and its wheel geometry |
| `config/base_teleop.yaml`, `pantilt_teleop.yaml` | Joystick buttons and axes: the base, and the legacy pan-tilt added on top |
| `config/twist_switch.yaml`, `toggles.yaml`, `collision_toggle.yaml` | The support nodes below |
| `config/bno055_diagnostics.yaml` | IMU diagnostics |
| `lekiwi_control/` | The support nodes: `twist_switch_node`, `bool_toggle_node`, `collision_toggle_node`, run together as `control_support_node` |

## Requirements

- ROS 2 Kilted with `ros2_control`, `ros2_controllers` (`omni_wheel_drive_controller`, `joint_state_broadcaster`, `imu_sensor_broadcaster`), `joy` and `joy_teleop`.
- The hardware interfaces [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) and [`bno055_hardware_interface`](https://github.com/adityakamath/bno055_hardware_interface), from `modules/`.
- `lekiwi_description` for the robot model. The base with its Gemini 2 needs nothing else. With a payload, its packages too (for the legacy pan-tilt, `pt_control`, `pt_description` and `pt_mujoco` from [pantilt_ros2](../payloads/pantilt_ros2/README.md), which you build explicitly).
- For `ros2_control_hardware_type:=mujoco`: `lekiwi_mujoco` and the [simulation packages](../lekiwi_mujoco/README.md#requirements).

## Running

```bash
ros2 launch lekiwi_control control.launch.py                                   # base with tray and Gemini 2, real hardware
ros2 launch lekiwi_control control.launch.py payload:=pantilt                  # with the legacy pan-tilt
ros2 launch lekiwi_control control.launch.py use_mock:=true                    # no hardware, simulated motors
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco   # MuJoCo (normally started through lekiwi_bringup with sim:=true)
ros2 launch lekiwi_control teleop.launch.py                                    # joystick teleop alone
```

`lekiwi_bringup` normally starts this for you. Control alone publishes `odom -> base_footprint` from wheel odometry; `lekiwi_bringup` turns it off (`enable_odom_tf:=false`); the EKF publishes it then. The `joy` node is only started with `joy:=true`; otherwise run `ros2 run joy joy_node` on whichever machine has the joystick.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `payload` | `""` | `""` for the base with its static Gemini 2, or `pantilt` (legacy) |
| `enable_camera` | `true` | Start simulated camera plugin and processing nodes; geometry stays in URDF/MJCF |
| `pointcloud` | `false` | Publish one colored simulation cloud (off by default) |
| `camera_fps` | `15` | Shared frame rate for either real camera; simulation caps rendering at 5 Hz |
| `sts_serial_port` | `""` | Servo serial port; empty uses `urdf_config.yaml` |
| `use_mock` | `""` | `true` or `false`; empty uses `urdf_config.yaml` |
| `diagnostics` | `false` | Also start the motor and IMU diagnostics nodes |
| `imu` | `true` | A physical BNO055 is present |
| `enable_odom_tf` | `true` | Publish `odom -> base_footprint` from wheel odometry |
| `joy` | `false` | Start `joy_node` on this device |
| `use_sim_time` | `false` | Use `/clock` instead of system time |
| `ros2_control_hardware_type` | `real` | `real` or `mujoco` |
| `mujoco_scene`, `mujoco_model`, `mujoco_headless` | `flat`, `""`, `false` | `mujoco` only: scene (`flat`, `arena`, `home`, `maze`, `none` or a file), a pre-built MJCF (empty generates one at launch), and whether to skip the viewer |

`teleop.launch.py` takes `payload`, `use_sim_time` and `joy`.

## Configuration

**`urdf_config.yaml`** holds the servo serial port (`/dev/ttySERVO`, see the [udev rules](../lekiwi_bringup/README.md#stable-device-names-udev)), baud rate, wheel motor IDs (7, 8, 9), mock mode and the servo speed profile (`internal_max_vel`, `internal_max_acc`, `internal_acc_coeff`, default 254 / 254 / 100). `sts_serial_port` and `use_mock` can also be set on the command line; edit the file for the rest. The pan-tilt reads its own speed profile from `pt_control`.

**`gemini2_imu_broadcaster.yaml`** configures the simulation-only camera IMU broadcaster. It reads sensor interfaces supplied by `mujoco_ros2_control`, separately from the base BNO055 broadcaster. With streaming enabled, it publishes `/gemini2/gyro_accel/sample`.

**Camera topics.** Gemini 2 uses `/gemini2/*` in hardware and simulation. Real and simulated Gemini 2 generate `/gemini2/scan` directly from depth images, including when `pointcloud:=true`; `pointcloud:=true` adds `/gemini2/depth_registered/points`. Compressed RGB is always available in simulation. On real hardware, `pointcloud:=true` also starts Cloudini compression. The real Gemini wrapper requests 640×360 color and 640×400 depth at 15 Hz. See [camera variants](../lekiwi_mujoco/README.md#camera-model) and [driver installation](../lekiwi_bringup/README.md#gemini-2-driver-real-robot-only).

**`control.yaml`** sets the controller manager and the `OmniWheelDriveController`: three wheels at 60°, 180° and 300°, robot radius and wheel radius. The MuJoCo model reads its wheel geometry from here too. The controller enforces no speed limits.

**`base_teleop.yaml`** maps the joystick (a Steam Deck; see the [repository README](../README.md#joystick) for the buttons). Its axis-scale magnitudes are the base's speed limits. Nav2 launch reads this installed configuration and derives MPPI, behavior-server rotation, and velocity-smoother limits automatically. Rebuild after changes when using a copied install, then restart the stack. `pantilt_teleop.yaml` adds the D-pad for the pan-tilt and is loaded on top when `payload:=pantilt`.

Button **8 (right joystick press)** calls `/nav2_send_goal` (`std_srvs/srv/SetBool`, `data: true`), with no L1 modifier. The target node accepts it only in Nav2 mode with an initialized target and no pending or active goal.

## How it works

The launch file expands the URDF from `lekiwi_description` with the values in `urdf_config.yaml`, starts one controller manager for everything, and spawns the controllers: joint states, the IMU broadcaster, the wheel controller, and the pan-tilt controller with `pt_control`'s own configuration file. There is one controller manager whether or not a payload is mounted.

Velocity commands take one path: `joy_teleop` publishes on `/cmd_vel_teleop`, Nav2 on `/cmd_vel_smoothed`, and `twist_switch_node` forwards whichever is selected to `/cmd_vel_presafety`. Nav2's collision monitor sits after it and forwards to the controller, for teleop and for autonomous goals. The support nodes run together in one process:

| Node | What it does |
|------|--------------|
| `twist_switch_node` | Selects between teleop and Nav2 commands through the `/twist_switch` service |
| `bool_toggle_node` | Turns a joystick button press into a toggle: `/emergency_stop`, `/twist_switch` and `/waypoint_follow` flip on each press, and stay in sync when the services are called from elsewhere |
| `collision_toggle_node` | While the R1 button is held, disables the collision monitor's predictive stop; releasing it re-enables it |

In `mujoco` mode the launch file uses `mujoco_ros2_control`'s own control node, which runs the simulation in the same process, generates the model with `lekiwi_mujoco` and adds the simulated LiDAR filter, the camera helpers and the ground-truth pose. See the [`lekiwi_mujoco` README](../lekiwi_mujoco/README.md).

## Using it with another payload

`payload:=<name>` is handled in `control.launch.py`, and a payload's control package supplies its controller and servo profile. Add `config/<name>_teleop.yaml` here, loaded on top of `base_teleop.yaml`. The [description README](../lekiwi_description/README.md#using-it-on-another-robot) lists every step to add a payload; a few launch branches still test for `pantilt` explicitly.

## Tests

```bash
pytest test -q
```

The tests check the configuration files against each other (speed limits, joystick mappings, toggle services), the support nodes, the teleop and payload wiring in the launch files, and that the launch files' arguments are consistent.
