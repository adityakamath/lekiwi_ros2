# LeKiwi Navigation

Navigation for LeKiwi: sensor fusion, mapping, localization, path planning and patrol. It runs [robot_localization](https://github.com/cra-ros-pkg/robot_localization) for the odometry, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) or AMCL for the map, and [Nav2](https://docs.nav2.org/) for planning and control, plus a service-driven waypoint patrol, a map saver and an editable [Nav2 target](#nav2-target-tracker).

## Contents

| Path | Purpose |
|------|---------|
| `launch/navigation.launch.py` | The entry point: forwards the shared arguments to the three launch files below |
| `launch/ekf.launch.py` | The EKF that publishes `odom -> base_footprint` and `/odometry/filtered` |
| `launch/slam.launch.py` | slam_toolbox (mapping or localization), or a map server with AMCL |
| `launch/nav2.launch.py` | The Nav2 nodes, the waypoint nodes and the zone-filter mask servers |
| `config/robot_localization/` | EKF settings, one file per fusion mode |
| `config/nav2/` | Nav2 (`nav2.yaml`), AMCL, slam_toolbox, map saver, waypoint recorder and Nav2 target settings |
| `lekiwi_navigation/` | `map_saver_node`, `waypoint_recorder_node` and `nav2_target_node`, plus `velocity_limits.py`, which derives the Nav2 speed limits from the teleop scales |
| `maps/` | Saved maps, one folder each (not committed) |

## Requirements

ROS 2 Kilted with `navigation2`, `nav2_bringup`, `slam_toolbox` and `robot_localization`, and Pillow (`python3-pil`) for the map saver. It expects the robot to publish `/scan`, `/base_controller/odom` and the IMU, and to take velocity commands through `lekiwi_control`'s twist switch and collision monitor path.

## Running

`lekiwi_bringup` starts this with the rest of the robot. To run it on its own:

```bash
ros2 launch lekiwi_navigation navigation.launch.py mission:=map                        # map the room with SLAM
ros2 launch lekiwi_navigation navigation.launch.py mission:=amcl map_name:=livingroom  # localize on a saved map
ros2 launch lekiwi_navigation navigation.launch.py mission:=slam map_name:=livingroom  # localize, and keep extending the map
```

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `fusion_mode` | `base` | `base`: wheel odometry and IMU; `imu`: IMU only; `odom`: wheel odometry only (needed without an IMU) |
| `mission` | `""` | `map`: map from scratch; `slam`: localize with slam_toolbox on a saved map; `amcl`: localize with AMCL on a saved map. Empty maps from scratch, or uses AMCL if `map_name` is set |
| `map_name` | `""` | Folder under `maps/`; also loads that map's zone masks |
| `wp_loops` | `0` | Waypoint patrol passes per start: `0` loops forever, `N>0` runs `N` passes |
| `diagnostics` | `false` | Publish the patrol status on `/diagnostics` |
| `use_sim_time` | `false` | Use `/clock` instead of system time |

`nav2.launch.py` also takes `params_file` (default `config/nav2/nav2.yaml`), `autostart` and `log_level`.

## Configuration

| File | What it sets |
|------|--------------|
| `config/nav2/nav2.yaml` | Controller (MPPI), planner (SmacPlanner2D), recoveries, costmaps and inflation, the velocity smoother and the collision monitor |
| `config/nav2/slam_toolbox.yaml`, `amcl.yaml` | Mapping and localization |
| `config/nav2/waypoint_recorder.yaml` | Patrol loops and how often a failing waypoint is retried before it is dropped |
| `config/nav2/map_saver.yaml` | The map saver's timeout |
| `config/nav2/nav2_target.yaml` | The Nav2 target's frames, publish rate, command timeout, 2× motion scales and marker |
| `config/robot_localization/ekf*.yaml` | Which measurements the EKF fuses in each `fusion_mode` |

**Speed limits.** The joystick axis scales in `lekiwi_control/config/base_teleop.yaml` set the speed limits. `nav2.launch.py` reads their absolute values from the installed `lekiwi_control` package and overrides the MPPI, behavior-server rotation and velocity smoother limits after loading `params_file`; this also applies to custom parameter files. Keep acceleration and controller tuning in `nav2.yaml`. After changing the teleop scales, rebuild `lekiwi_control` (if you use a copied install) and restart the stack. Negative scales invert axes; zero or non-finite scales and nonzero offsets are rejected. Launch through `nav2.launch.py` to apply the derived limits.

**Inflation.** The local and global inflation radii are 0.20 m and 0.25 m, and the robot radius is 0.17 m. `inflation_radius` is measured from the obstacle to the robot centre. At start-up Nav2 Kilted's SmacPlanner2D may log an optimization warning about non-circular footprints; this is expected.

### Maps and zones

Saving a map (the `/save_map` service, bound to the Screenshot button) writes a timestamped folder under `maps/` with the map, the SLAM pose graph, the robot's pose for localization start-up, and a `filters/` folder with placeholder masks. Use the folder name as `map_name` (rename it first if you like). No-go and speed-limited zones are the images `filters/keepout_mask.pgm` and `speed_mask.pgm`, which start as no-op placeholders; edit them in an image editor to add zones. Maps are not committed to the repository.

## How it works

The EKF fuses wheel odometry and the IMU into `/odometry/filtered`. With no map, slam_toolbox builds one and publishes `map -> odom`; with a saved map, slam_toolbox (which can keep extending it) or AMCL localizes on it. Nav2 then plans and drives. Its velocity commands go through the safety path described in the [`lekiwi_control` README](../lekiwi_control/README.md#how-it-works): the twist switch, then the collision monitor.

The waypoint patrol is driven by three `SetBool` services, bound to joystick buttons in `lekiwi_control`:

| Service | What it does |
|---------|--------------|
| `/record_waypoint` | Records the robot's current pose as the next waypoint; while patrolling, it joins at the start of the next loop |
| `/waypoint_follow` | `true` starts or resumes the patrol (looping per `wp_loops`), `false` pauses it |
| `/reset_waypoints` | Cancels the patrol and clears all waypoints |

A goal sent from RViz or Foxglove during a patrol is treated as a detour, and the patrol resumes afterwards. A waypoint that fails repeatedly is dropped. With `diagnostics:=true`, the patrol's progress is published on `/diagnostics`.

## Nav2 target tracker

`nav2_target_node` shows and edits a planar target, and sends it to Nav2 as a goal. It starts with `nav2.launch.py` (so with normal navigation and robot bringup), inherits `use_sim_time` and the log level, and is configured in `config/nav2/nav2_target.yaml`. To see it, display `nav2_target_marker` as a Marker in Foxglove/RViz and enable TF axes for `nav2_target`.

**Teleop mode.** The target is attached to the robot (`base_footprint -> nav2_target`, identity) and no map TF is needed.

**Nav2 mode.** On the switch to Nav2 the node converts the target to a map pose once and publishes `map -> nav2_target`. It then edits the target from `/cmd_vel_teleop`: X/Y are translation relative to the target's heading and angular Z rotates it, at up to 2× the base teleop limits (0.4 m/s in X/Y, 1.6 rad/s in yaw). Commands expire after `command_timeout`, and a mode change clears held input. Returning to teleop reattaches the target without changing navigation.

The node follows the mode from `/twist_switch/_service_event`. Until it has seen a successful mode change it shows the target but does not edit (`waiting_for_mode`); it does not call the switch service. If the map TF is missing at the switch, it waits (`waiting_for_map_transform`).

**Topics.**

| Topic | Content |
|-------|---------|
| `nav2_target_marker` | Sphere marker for the target, in one constant colour (`marker_color`) |
| `nav2_target_pose` | The draft pose in its current parent frame (base in teleop, map in Nav2); check `header.frame_id` |
| `nav2_target_status` | `waiting_for_mode`, `teleop_attached`, `waiting_for_map_transform` or `editing` |
| `nav2_target_goal_status` | Goal state: `idle`, `sending`, `active`, `rejected`, `succeeded`, `failed`, `canceled` or `unknown`, with error details when available |
| `nav2_target_submitted_pose` | The last submitted pose; later edits do not change that goal |
| `nav2_target_goal_input` | Input: a one-shot `geometry_msgs/PoseStamped` in `map` that is submitted as a goal and moves the marker there |
| `nav2_target_goal_input_status` | Whether that input was submitted or rejected |

Goals sent directly to `/navigate_to_pose`, including the waypoint patrol, do not move the marker.

### Sending the target to Nav2

In Nav2 mode, edit the target with the joystick (or Foxglove) and submit it with the right joystick button (button 8), or by calling the service:

```bash
ros2 service call /nav2_send_goal std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /nav2_target_goal_status
```

To submit a pose chosen elsewhere, publish its final map pose once. For example, `(x=1.0, y=2.0, yaw=90°)`:

```bash
ros2 topic pub --once /nav2_target_goal_input geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {z: 0.70710678, w: 0.70710678}}}"
```

- A successful service response means the submission started, not that Nav2 accepted or reached the goal. With audio enabled it announces "Sending navigation goal", or "Navigation goal rejected".
- Only one goal from this node can be pending or active at a time. A rejection or a terminal result frees the slot. The request is rejected if mode confirmation or the map pose is missing, the coordinates are not finite, or the action server is unavailable.
- If a transport error leaves the outcome unknown, the slot stays reserved. Inspect or cancel the goal in Nav2, then call `/nav2_send_goal` with `data: false` to clear it. Clearing does not cancel the earlier goal. `data: false` is otherwise a no-op.
- A pose input is a one-shot goal; edit continuously with the target controls and commit with the button.

### Validation and tests

The robot validation checklist and the automated target tests are in [`docs/nav2_target.md`](docs/nav2_target.md).

## Tests

```bash
pytest test -q
```

The tests check the Nav2, EKF and SLAM configuration (including the speed limits against `lekiwi_control`), the launch arguments, and the nodes.
