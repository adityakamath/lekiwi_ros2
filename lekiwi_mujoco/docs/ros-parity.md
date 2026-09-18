# ROS/Python parity verification — 2026-09-17

The existing Pixi Kilted environment at `~/Documents/Pixi/ros2_ws` was used without
changing its dependencies. Its Python is 3.12; standalone simulation uses its
separate MuJoCo environment.

Available: rclpy, controller manager, forward_command_controller.
Missing: omni_wheel_drive_controller, mujoco_ros2_control and Python MuJoCo.
The Pixi manifest explicitly documents the MuJoCo vendor's macOS restriction.
Full ROS control/physics/navigation parity cannot be tested in this environment.
A mock hardware test would not establish MuJoCo bridge equivalence.

42 existing ROS node/configuration checks passed under Pixi, using Cyclone DDS,
local discovery, domain 173 and a writable ROS_LOG_DIR. These validate message
conversion, command routing and configuration, not controller trajectories.
Two launch-argument tests were excluded after the initial run reported that this
scratch checkout's lekiwi_control package is not installed in the Pixi overlay.
The user's environment was not modified to install it.

## Remaining semantic differences

| Aspect | Python core | ROS configuration/path |
|---|---|---|
| Base commands | Body velocity or normalized rates; uniform wheel/body saturation | Twist input to omni controller; actual saturation behavior unverified without plugin |
| Acceleration | No base command acceleration ramp | YAML declares x/y/yaw acceleration bounds; runtime enforcement still needs controller test |
| Pan/tilt | Normalized rate integrates a bounded, slew-limited position target | Forward position commands; joystick maps directly to absolute positions; payload control YAML disables velocity limits |
| Stopping | Explicit stop or keyboard release; no command expiry | Base YAML declares 0.5 s timeout; controller expiry behavior unverified |
| Reset/time | Reset clears target history, settles and restarts simulation time | ROS controller buffers, clock and reset integration unverified |

## Next integration test

Use a supported environment containing both missing controller/physics plugins.
Run the same timestamped base commands and pan/tilt position-target sequence,
record commanded and measured joints plus /clock with whatever recording mechanism is added later, and compare
saturation, ramps, timeout, stopping and reset epochs. Decide whether to align
Python with the existing ROS semantics or add an explicit shared rate-command
adapter before changing robot behavior. Do not silently reinterpret ROS position
commands as rates. Hardware motor/contact calibration remains separate.
