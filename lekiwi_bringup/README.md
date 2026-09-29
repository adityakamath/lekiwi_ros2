# LeKiwi Bringup

The top-level launch files for LeKiwi. One command starts the whole system (control, navigation, sensors, audio, and the payload's camera) on the real robot or in the MuJoCo simulation. This package also holds the LiDAR configuration and the udev rules.

## Contents

| Path | Purpose |
|------|---------|
| `launch/lekiwi.launch.py` | The whole system; includes the other packages' launch files |
| `launch/laser.launch.py` | The LD06 LiDAR driver, with the payload's scan filter if there is one |
| `config/laser.yaml` | LiDAR serial port and settings |
| `config/<payload>_laser_filter.yaml` | Masks the part of the scan a payload blocks (currently `pantilt`) |
| `config/battery_overrides.yaml` | LeKiwi's overrides on top of the battery monitor's defaults |
| `config/diagnostic_aggregator.yaml` | The single top-level health view on `/diagnostics_agg` |
| `udev/99-lekiwi.rules` | Stable device names for the LiDAR and servo bus |

## Requirements

The other LeKiwi packages, built together with `colcon build --packages-up-to lekiwi_bringup`, which also brings in the submodules' packages (`ldlidar_ros2`, `ina260_battery_monitor` and the hardware interfaces), `laser_filters` and `diagnostic_aggregator`. With the pan-tilt payload, `pt_bringup` from [pantilt_ros2](../payloads/pantilt_ros2/README.md). Install the [udev rules](../README.md#stable-device-names-udev) once per robot.

## Running

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py                                       # base and pan-tilt
ros2 launch lekiwi_bringup lekiwi.launch.py payload:=""                           # base only
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=map                          # SLAM mapping
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=amcl map_name:=livingroom1   # localize on a saved map
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home         # MuJoCo simulation
ros2 launch lekiwi_bringup laser.launch.py                                        # LiDAR alone
```

### Launch arguments

The main arguments are in the [repository README](../README.md#launch-arguments). `--show-arguments` lists them all. These are the ones it does not cover:

| Argument | Default | Meaning |
|----------|---------|---------|
| `camera_config` | `gemini2` | Pan-tilt camera geometry: `gemini2` or `oakd_s2`; also selects the generated simulation model |
| `pantilt_config` | `pt101` | Pan-tilt mesh variant, `pt100` or `pt101` |
| `enable_camera` | `true` | Real or simulated camera publishing on/off; camera geometry remains selected |
| `pointcloud` | `false` | One colored cloud for either camera in hardware or simulation; real drivers also start Cloudini compression |
| `octomap` | `false` | OAK-D hardware only; requires `pointcloud:=true` |
| `sts_serial_port`, `use_mock` | `""` | Servo serial port and mock mode; empty uses `lekiwi_control`'s `urdf_config.yaml` |
| `camera_fps` | `15` | Shared RGB/depth rate for either real camera (5, 10, 15, 30 Hz); simulation caps rendering at 5 Hz |
| `mujoco_model` | `""` | `sim` only: a pre-built MJCF; empty generates one at launch |
| `use_sim_time` | `false` | Use `/clock`; forced on with `sim:=true` |

`laser.launch.py` takes `payload` and `custom_filter`, the name of a filter file in `config/` that overrides the payload's.

## Configuration

| File | What it sets |
|------|--------------|
| `config/laser.yaml` | LD06 serial port (`/dev/ttyLIDAR`), baud rate, range and the angle crop |
| `config/pantilt_laser_filter.yaml` | The arc of the scan the pan-tilt blocks, and the filtered scan's QoS |
| `config/battery_overrides.yaml` | Only what differs from the battery monitor's own defaults, such as the state file location; keep it to genuine overrides so it does not drift from upstream |
| `config/diagnostic_aggregator.yaml` | How motor, IMU, patrol and battery diagnostics roll up (with `diagnostics:=true`) |

The rest of the system is configured in the packages that own it; the [repository README](../README.md#configuration) lists the files.

## How it works

`lekiwi.launch.py` includes the other packages' launch files and passes the shared arguments on, so each keeps its own defaults:

| Started | By | Condition |
|---------|----|-----------|
| Control and teleop | `lekiwi_control` | always (MuJoCo model with `sim:=true`) |
| Navigation: EKF, SLAM or AMCL, Nav2, waypoint patrol | `lekiwi_navigation` | always |
| Diagnostic aggregator | this package | `diagnostics:=true` |
| LiDAR | `laser.launch.py` | real hardware and `laser:=true` |
| Audio | `lekiwi_audio` | real hardware and `audio:=true` |
| Battery monitor | `ina260_battery_monitor` | real hardware and `battery_monitor:=true` |
| Gemini 2 or OAK-D camera | `pt_bringup` | real hardware, `payload:=pantilt`, `enable_camera:=true`; only the selected driver is included |

In simulation only the first three run: the simulated LiDAR and camera come from the MuJoCo plugins, and audio and the battery monitor are not simulated. `sim:=true` also forces simulation time and mock hardware. The EKF publishes `odom -> base_footprint`, so the launch file turns off the controller's own.

## Using it with another payload

Add the payload's name to `_VALID_PAYLOADS` in `lekiwi.launch.py`. If it blocks part of the LiDAR's view, add `config/<name>_laser_filter.yaml` and `laser.launch.py` applies it automatically. The [repository README](../README.md#payloads) lists the rest of the wiring; a few launch branches still test for `pantilt` explicitly.

## Tests

```bash
pytest test -q
```

The tests check that the launch arguments are declared and validated, and that the LiDAR filter configuration resolves for each payload.

Camera selection is forwarded unchanged to URDF and MuJoCo generation. Gemini 2 is the default; use `camera_config:=oakd_s2` for the alternative. Gemini 2 uses `/gemini2/*` in hardware and simulation. Simulation slices registered depth into `/gemini2/scan` without building a cloud; `pointcloud:=true` adds `/gemini2/depth_registered/points`; on real hardware it also publishes `/gemini2/depth_registered/points/compressed` through Cloudini. OAK-D S2 retains `/oak/*`; `pointcloud:=true` adds `/oak/rgbd/points` in simulation. Compressed RGB is always available in simulation. The real OAK profile defaults to 15 Hz with VIO off; the real Gemini wrapper requests 640×360 color and 640×400 depth at 15 Hz. See [camera variants](../lekiwi_mujoco/README.md#camera-variants) and [driver installation](../payloads/pantilt_ros2/README.md#installation).

To run LeKiwi with the default Gemini 2 geometry but no real camera streaming:

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py enable_camera:=false
```

`camera_config` defaults to `gemini2`; `camera_config:=oakd_s2` selects the alternative. LeKiwi passes `enable_camera` to the selected camera-only launch in `pt_bringup`, which skips the driver and streaming helpers before resolving camera dependencies when false. Camera geometry remains in the URDF. In simulation, the same flag skips the camera plugin and camera processing while keeping the selected URDF and MJCF geometry.
