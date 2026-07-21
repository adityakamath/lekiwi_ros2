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
| `diagnostics`         | `false`   | Launch motor and IMU diagnostics nodes in `lekiwi_control`                                                        |
| `control_mock`        | `""`      | Mock mode override (`true`/`false`); empty means use `urdf_config.yaml` value                                     |
| `fusion_mode`         | `base`    | EKF sensor fusion: `base` (wheel odom + BNO055), `imu` (BNO055 only), `odom` (wheel odom only)                    |
| `mission`             | `""`      | Navigation mission: empty preserves current behavior (`map_name` empty -> `map`, `map_name` set -> `amcl`); `map` runs slam_toolbox mapping, `slam` runs slam_toolbox localization and requires `map_name`, `amcl` runs AMCL + nav2_map_server and requires `map_name` |
| `map_name`            | `""`      | Map subdirectory to load (e.g. `livingroom1`). With `mission:=slam` loads slam_toolbox localization on the existing map; with `mission:=amcl` loads the static map for AMCL. Also tells `nav2.launch.py` where to load that map's no-go/speed zone masks from, if any - see [Costmap Zones](#costmap-zones) |
| `pointcloud`          | `false`   | Use `oakd_vio_pcl.yaml` (depth aligned to RGB + point cloud) instead of `oakd_vio.yaml` (depth unaligned, no point cloud). Also gates point cloud compression. |
| `octomap`             | `false`   | Run `octomap_server` on the OAK-D point cloud to build a persistent 3D octree (only takes effect when `pointcloud:=true`) |
| `joy`                 | `false`   | Launch `joy_node` locally; leave `false` when `/joy` is published from a remote device                           |
| `use_sim_time`        | `false`   | Use `/clock` from a simulator instead of system time                                                              |

Earlier iterations called `mission` `nav_mode`.

## Joystick Configuration

Teleoperation is configured for a **Steam Deck**, used as a generic joystick (via `joy_node`/`joy_teleop`) rather than through Steam Input — button and axis indices below are specific to how the Deck's controls show up over that interface, not a standard Xbox-style gamepad layout. See the `joy` launch argument above for running `joy_node` locally vs. receiving `/joy` from a remote device.

**Drive** (base, requires the L1 deadman held):

| Control            | Action                                  |
|---------------------|------------------------------------------|
| L1 (button 9)       | Deadman — hold to enable drive commands |
| Left stick (axis 1) | Forward / backward                      |
| Left stick (axis 0) | Strafe left / right                     |
| Right stick (axis 2)| Rotate in place                         |

**Pan-tilt** (`payload:=pantilt` only, shares the same L1 deadman):

| Control          | Action       |
|------------------|--------------|
| D-pad (axis 6)   | Pan          |
| D-pad (axis 7)   | Tilt         |

**Single-button safety/mode controls**:

| Control                | Action                                                                                          |
|-------------------------|--------------------------------------------------------------------------------------------------|
| B (button 1)            | Toggle `/emergency_stop` (via `bool_toggle_node`)                                                |
| X (button 2)            | Toggle base control between teleop and Nav2 (via `bool_toggle_node`, `/twist_switch`)            |
| R1 (button 10)          | Hold to disable `collision_monitor`'s predictive stop (deadman — releasing re-enables it automatically)                          |
| Screenshot (button 4)   | Save the current SLAM map (`/save_map`, also callable from Foxglove)                              |

**Waypoint patrol controls** (`waypoint_recorder_node`):

Exposed as `std_srvs/srv/SetBool` services so the same controls work identically from `joy_teleop` button bindings or Foxglove's "Call Service" panel.

| Control                | Service                          | Action                                                                          |
|-------------------------|------------------------------------|----------------------------------------------------------------------------------|
| Y (button 3)            | `/record_waypoint`               | Record the robot's current pose as the next waypoint (adds a marker on `/waypoint_markers`). Works even while patrolling - queued waypoints join the patrol at the start of the next loop rather than immediately. |
| A (button 0)            | `/waypoint_follow_toggle`         | Toggle (via `bool_toggle_node`): 1st press starts/resumes the patrol; 2nd press pauses it. Waypoints and markers are untouched either way — only Settings clears those. |
| Settings (button 11)    | `/reset_waypoints`               | Cancel the active patrol and clear the recorded waypoints + their markers       |

`/waypoint_follow` (the underlying SetBool the toggle drives) stays directly callable too, e.g. from Foxglove, for explicit start (`true`) / stop (`false`) without the toggle. It's pause/resume, not stop/restart: stopping mid-route and starting again continues toward the same waypoint it was already heading to (via `FollowWaypoints.Goal.goal_index`), not back to the beginning - and is a no-op if it's already patrolling.

Progress is published as `diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics` (status name `waypoint_recorder: patrol`) rather than a visualization - `current_loop`, `from_waypoint`/`to_waypoint`, `total_waypoints`, plus `estimated_time_to_waypoint_sec`/`distance_to_waypoint_m`/`number_of_recoveries` forwarded live from `/navigate_to_pose`'s own feedback, and a derived `estimated_loop_duration_sec`. Level is `STALE` for a deliberate stop, `WARN` while paused waiting to auto-resume (see below).

**Sending the robot a separate navigation goal (RViz "2D Nav Goal", Foxglove's goal-pose panel) while patrolling is a supported detour, not an error.** It preempts the current leg immediately (`stop_on_failure: true` makes this a clean abort rather than nav2_waypoint_follower silently skipping that waypoint), and `waypoint_recorder_node` automatically resumes toward the interrupted waypoint once `/navigate_to_pose` is free again - "go there, then continue the patrol." A pause via `/waypoint_follow false` or the A button cancels this auto-resume too, not just the patrol itself, and also clears the stale plan visualization (`/plan` and a few related topics) that nav2 otherwise leaves on screen after a cancel - it reappears on its own once a new plan is computed (resuming, or a fresh detour).

## Costmap Zones

Nav2's `KeepoutFilter` and `SpeedFilter` are wired up and active by default: no-go zones (the planner routes around them, the controller won't drive into them) and speed-limited zones (the controller slows down inside them). Each filter is backed by its own `map_server` mask instance and a dedicated lifecycle manager, so a bad mask file only disables that filter, not the rest of the nav stack.

Zone masks live inside each map's own folder (`lekiwi_navigation/maps/<map_name>/filters/`), saved automatically by `map_saver_node` alongside the map itself - no separate save step. New maps start with no-op (all-clear) placeholder masks; painting real zone shapes into them is a manual step not yet covered by tooling here.

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
