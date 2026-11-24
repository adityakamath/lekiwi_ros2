# LeKiwi Control

The `ros2_control` setup for LeKiwi: the wheel and IMU controllers, joystick teleop, the small nodes that make the joystick buttons and Nav2 share the drive safely, and the launch files that bring it all up on real hardware or in the MuJoCo simulation.

## Contents

| Path | Purpose |
|------|---------|
| `launch/control.launch.py` | The control stack: `robot_state_publisher`, the controller manager, the controllers, teleop and the support nodes; simulation extras with `ros2_control_hardware_type:=mujoco` |
| `launch/teleop.launch.py` | Joystick teleop (`joy_teleop`) on its own |
| `config/base_teleop.yaml` | Joystick mappings and speed scales shared with Nav2 |
| `config/urdf_config.yaml` | Servo serial port, motor IDs, mock mode, servo speed profile |
| `config/control.yaml` | Controller manager (20 Hz), the drive controller and its wheel geometry |
| `config/twist_switch.yaml`, `toggles.yaml`, `collision_toggle.yaml` | The support nodes below |
| `config/bno055_diagnostics.yaml` | IMU diagnostics |
| `lekiwi_control/` | The support nodes: `twist_switch_node`, `bool_toggle_node`, `collision_toggle_node`, run together as `control_support_node` |

## Requirements

- ROS 2 Kilted with `ros2_control`, `ros2_controllers` (`omni_wheel_drive_controller`, `joint_state_broadcaster`, `imu_sensor_broadcaster`), `joy` and `joy_teleop`.
- The hardware interfaces [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) and [`bno055_hardware_interface`](https://github.com/adityakamath/bno055_hardware_interface), from `modules/`.
- For `ros2_control_hardware_type:=mujoco`: `lekiwi_mujoco` and the [simulation packages](../lekiwi_mujoco/README.md#requirements).

## Running

```bash
ros2 launch lekiwi_control control.launch.py                                   # base with tray and Gemini 2, real hardware
ros2 launch lekiwi_control control.launch.py use_mock:=true                    # no hardware, simulated motors
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco   # MuJoCo (normally started through lekiwi_bringup with sim:=true)
ros2 launch lekiwi_control teleop.launch.py                                    # joystick teleop alone
```

`lekiwi_bringup` normally starts this for you. Control alone publishes `odom -> base_footprint` from wheel odometry; `lekiwi_bringup` turns it off (`enable_odom_tf:=false`); the EKF publishes it then. The `joy` node is only started with `joy:=true`; otherwise run `ros2 run joy joy_node` on whichever machine has the joystick.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `hardware_config` | bringup hardware profile | Fitted assemblies and interfaces; passed to Xacro and MuJoCo |
| `enable_camera` | `true` | Start simulated camera plugin and processing nodes; geometry stays in URDF/MJCF |
| `pointcloud` | `false` | Publish one colored simulation cloud (off by default) |
| `sts_serial_port` | `""` | Servo serial port; empty uses `urdf_config.yaml` |
| `use_mock` | `""` | `true` or `false`; empty uses `urdf_config.yaml` |
| `diagnostics` | `false` | Also start the motor and IMU diagnostics nodes |
| `enable_odom_tf` | `true` | Publish `odom -> base_footprint` from wheel odometry |
| `joy` | `false` | Start `joy_node` on this device |
| `use_sim_time` | `false` | Use `/clock` instead of system time |
| `ros2_control_hardware_type` | `real` | `real` or `mujoco` |
| `mujoco_scene`, `mujoco_model`, `mujoco_headless` | `flat`, `""`, `false` | `mujoco` only: scene (`flat`, `arena`, `home`, `maze`, `none` or a file), a pre-built MJCF (empty generates one at launch), and whether to skip the viewer |

## Configuration

Robot composition comes from [`hardware.yaml`](../lekiwi_bringup/config/hardware.yaml).
Control loads that file, expands `lekiwi_description/urdf/base.urdf.xacro` and
starts the wheel controller and joint-state broadcaster. The BNO055 broadcaster
runs when `imu: true`; the simulated Gemini IMU broadcaster requires the camera
and streaming to be enabled. Motor/IMU hardware diagnostics run only on real hardware.

**`urdf_config.yaml`** sets the serial port, wheel IDs (7/8/9), mock mode and
servo profile. Its `internal_control_period` is 10; direct Xacro has a default of 20.
`sts_serial_port` and `use_mock` can override the file at launch.

**`base_teleop.yaml`** maps `/joy` to drive commands and services. Its axis-scale
magnitudes supply the base speed limits to Nav2 and standalone simulation.
Teleop loads this single file. `joy:=true` starts a local `joy_node`; otherwise
run the joystick publisher externally.

**`gemini2_imu_broadcaster.yaml`** configures the simulation-only camera IMU broadcaster. It reads sensor interfaces supplied by `mujoco_ros2_control`, separately from the base BNO055 broadcaster. With streaming enabled, it publishes `/gemini2/gyro_accel/sample`.

**Start-up.** `control.launch.py` starts the controllers one after another: the joint-state broadcaster first, then the others.

**`control.yaml`** sets the controller manager and the `OmniWheelDriveController`: three wheels at 60°, 180° and 300°, robot radius and wheel radius. The MuJoCo model reads its wheel geometry from here too. The controller enforces no speed limits.

Button **8 (right joystick press)** calls `/nav2_send_goal` (`std_srvs/srv/SetBool`, `data: true`), with no L1 modifier. The target node accepts it only in Nav2 mode with an initialized target and no pending or active goal.

## How it works

Velocity commands take one path: `joy_teleop` publishes on `/cmd_vel_teleop`, Nav2 on `/cmd_vel_smoothed`, and `twist_switch_node` forwards whichever is selected to `/cmd_vel_presafety`. Nav2's collision monitor sits after it and forwards to the controller, for teleop and autonomous goals. Control-only runs still need the separately launched collision monitor. The support nodes run together in one process:

| Node | What it does |
|------|--------------|
| `twist_switch_node` | Selects between teleop and Nav2 commands through the `/twist_switch` service |
| `bool_toggle_node` | Turns a joystick button press into a toggle: `/emergency_stop`, `/twist_switch` and `/waypoint_follow` flip on each press, and stay in sync when the services are called from elsewhere. Once a target service is up and has not been called, it sends its `initial_state` once, so the state is readable at start-up (`<service>/_service_event` replays it) |
| `collision_toggle_node` | While the R1 button is held, disables the collision monitor's predictive stop; releasing it re-enables it |

In `mujoco` mode the launch file uses `mujoco_ros2_control`'s own control node, which runs the simulation in the same process, generates the model with `lekiwi_mujoco` and adds the simulated LiDAR filter, the ToF cloud rotation, the camera helpers and the ground-truth pose. See the [`lekiwi_mujoco` README](../lekiwi_mujoco/README.md).

## Tests

```bash
pytest test -q
```

Configure robot composition only in
[`lekiwi_bringup/config/hardware.yaml`](../lekiwi_bringup/config/hardware.yaml).
It selects the fitted assemblies. Launch normally after editing it; no hardware flags are needed. See
[Configure the robot](../lekiwi_description/README.md#configure-the-robot).
