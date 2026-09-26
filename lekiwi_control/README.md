# LeKiwi Control

The `ros2_control` setup for LeKiwi: the wheel and IMU controllers, joystick teleop, the small nodes that make the joystick buttons and Nav2 share the drive safely, and the launch files that bring it all up on real hardware or in the MuJoCo simulation.

## Contents

| Path | Purpose |
|------|---------|
| `launch/control.launch.py` | The control stack: `robot_state_publisher`, the controller manager, the controllers, teleop and the support nodes; simulation extras with `ros2_control_hardware_type:=mujoco` |
| `launch/teleop.launch.py` | Joystick teleop (`joy_teleop`) on its own |
| `config/urdf_config.yaml` | Servo serial port, motor IDs, mock mode, servo speed profile |
| `config/control.yaml` | Controller manager (50 Hz), the drive controller and its wheel geometry |
| `config/base_teleop.yaml`, `pantilt_teleop.yaml` | Joystick buttons and axes: the base, and the pan-tilt added on top |
| `config/twist_switch.yaml`, `toggles.yaml`, `collision_toggle.yaml` | The support nodes below |
| `config/bno055_diagnostics.yaml` | IMU diagnostics |
| `lekiwi_control/` | The support nodes: `twist_switch_node`, `bool_toggle_node`, `collision_toggle_node`, run together as `control_support_node` |

## Requirements

- ROS 2 Kilted with `ros2_control`, `ros2_controllers` (`omni_wheel_drive_controller`, `joint_state_broadcaster`, `imu_sensor_broadcaster`), `joy` and `joy_teleop`.
- The hardware interfaces [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) and [`bno055_hardware_interface`](https://github.com/adityakamath/bno055_hardware_interface), from `modules/`.
- `lekiwi_description` for the robot model. With a payload, its packages too (for the pan-tilt, `pt_control` and `pt_description` from [pantilt_ros2](../payloads/pantilt_ros2/README.md)).
- For `ros2_control_hardware_type:=mujoco`: `lekiwi_mujoco` and the [simulation packages](../lekiwi_mujoco/README.md#requirements).

## Running

```bash
ros2 launch lekiwi_control control.launch.py                                   # base only, real hardware
ros2 launch lekiwi_control control.launch.py payload:=pantilt                  # with the pan-tilt
ros2 launch lekiwi_control control.launch.py use_mock:=true                    # no hardware, simulated motors
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco   # MuJoCo (normally started through lekiwi_bringup with sim:=true)
ros2 launch lekiwi_control teleop.launch.py                                    # joystick teleop alone
```

`lekiwi_bringup` normally starts this for you. Control alone publishes `odom -> base_footprint` from wheel odometry; `lekiwi_bringup` turns that off because the EKF publishes it. The `joy` node is only started with `joy:=true`; otherwise run `ros2 run joy joy_node` on whichever machine has the joystick.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `payload` | `""` | `""` for the base alone, or `pantilt` |
| `pantilt_config` | `pt101` | Pan-tilt mesh variant, `pt100` or `pt101` |
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

**`urdf_config.yaml`** holds the servo serial port (`/dev/ttySERVO`, see the [udev rules](../README.md#stable-device-names-udev)), baud rate, wheel motor IDs (7, 8, 9), mock mode and the servo speed profile (`internal_max_vel`, `internal_max_acc`, `internal_acc_coeff`, default 254 / 254 / 100). `sts_serial_port` and `use_mock` can also be set on the command line; edit the file for the rest. The pan-tilt reads its own speed profile from `pt_control`.

**`control.yaml`** sets the controller manager and the `OmniWheelDriveController`: three wheels at 60°, 180° and 300°, robot radius and wheel radius. The MuJoCo model reads its wheel geometry from here too. The controller enforces no speed limits.

**`base_teleop.yaml`** maps the joystick (a Steam Deck; see the [repository README](../README.md#joystick) for the buttons). Its axis scales, 0.2 m/s, 0.17 m/s and 0.68 rad/s, are the base's speed limits, and `velocity_smoother` in `lekiwi_navigation`'s `nav2.yaml` must match them. `pantilt_teleop.yaml` adds the D-pad for the pan-tilt and is loaded on top when `payload:=pantilt`.

## How it works

The launch file expands the URDF from `lekiwi_description` with the values in `urdf_config.yaml`, starts one controller manager for everything, and spawns the controllers: joint states, the IMU broadcaster, the wheel controller, and the pan-tilt controller with `pt_control`'s own configuration file. There is one controller manager whether or not a payload is mounted, because the wheels and the payload share the servo bus.

Velocity commands take one path: `joy_teleop` publishes on `/cmd_vel_teleop`, Nav2 on `/cmd_vel_smoothed`, and `twist_switch_node` forwards whichever is selected to `/cmd_vel_presafety`. Nav2's collision monitor sits after it, so it protects teleop as well as autonomous goals, and forwards to the controller. The support nodes are small and non-blocking, so they run together in one process:

| Node | What it does |
|------|--------------|
| `twist_switch_node` | Selects between teleop and Nav2 commands through the `/twist_switch` service |
| `bool_toggle_node` | Turns a joystick button press into a toggle: `/emergency_stop`, `/twist_switch` and `/waypoint_follow` flip on each press, and stay in sync when the services are called from elsewhere |
| `collision_toggle_node` | While the R1 button is held, disables the collision monitor's predictive stop; releasing it re-enables it |

In `mujoco` mode the launch file uses `mujoco_ros2_control`'s own control node, which runs the simulation in the same process, generates the model with `lekiwi_mujoco` and adds the simulated LiDAR filter, the camera helpers and the ground-truth pose. See the [`lekiwi_mujoco` README](../lekiwi_mujoco/README.md).

## Using it with another payload

`payload:=<name>` is handled in `control.launch.py`, and a payload's control package supplies its controller and servo profile so nothing is copied here. This package only adds the host-dependent part: `config/<name>_teleop.yaml`, loaded on top of `base_teleop.yaml`. The [repository README](../README.md#payloads) lists every piece of wiring a payload needs; a few launch branches still test for `pantilt` explicitly.

## Tests

```bash
pytest test -q
```

The tests check the configuration files against each other (speed limits, joystick mappings, toggle services), the support nodes, the teleop and payload wiring in the launch files, and that the launch files' arguments are consistent.
