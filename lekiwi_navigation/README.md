# LeKiwi Navigation

Navigation for LeKiwi: sensor fusion, mapping, localization, path planning and patrol. It runs [robot_localization](https://github.com/cra-ros-pkg/robot_localization) for the odometry, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) or AMCL for the map, and [Nav2](https://docs.nav2.org/) for planning and control, plus a service-driven waypoint patrol and a map saver.

## Contents

| Path | Purpose |
|------|---------|
| `launch/navigation.launch.py` | The entry point: forwards the shared arguments to the three launch files below |
| `launch/ekf.launch.py` | The EKF that publishes `odom -> base_footprint` and `/odometry/filtered` |
| `launch/slam.launch.py` | slam_toolbox (mapping or localization), or a map server with AMCL |
| `launch/nav2.launch.py` | The Nav2 nodes, the waypoint nodes and the zone-filter mask servers |
| `config/robot_localization/` | EKF settings, one file per fusion mode |
| `config/nav2/` | Nav2 (`nav2.yaml`), AMCL, slam_toolbox, map saver and waypoint recorder settings |
| `lekiwi_navigation/` | `map_saver_node` and `waypoint_recorder_node` |
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
| `config/robot_localization/ekf*.yaml` | Which measurements the EKF fuses in each `fusion_mode` |

The velocity smoother's `max_velocity` in `nav2.yaml` must match the joystick axis scales in `lekiwi_control`'s `base_teleop.yaml`, because the drive controller enforces no speed limits itself.

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

## Using it on another robot

The package is written for LeKiwi's frames (`base_footprint`, `odom`, `map`), robot radius and speed limits, so start from `config/nav2/nav2.yaml` and the EKF files, and adjust the radius, speeds and sensor topics for your robot.

## Tests

```bash
pytest test -q
```

The tests check the Nav2, EKF and SLAM configuration (including the speed limits against `lekiwi_control`), the launch arguments, and the two nodes.
