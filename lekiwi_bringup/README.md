# LeKiwi Bringup

The top-level launch files for LeKiwi. One command starts the whole system (control, navigation, sensors, audio, and the Gemini 2 camera) on the real robot or in the MuJoCo simulation. This package also holds the LiDAR configuration and the udev rules.

## Contents

| Path | Purpose |
|------|---------|
| `launch/lekiwi.launch.py` | The whole system; includes the other packages' launch files |
| `launch/laser.launch.py` | The LD06 LiDAR driver, with the payload's scan filter if there is one |
| `launch/gemini2.launch.py` | The Orbbec Gemini 2 driver and depth-to-scan for the base (no payload) |
| `config/gemini2_depth_to_scan.yaml` | Settings for the depth-to-scan conversion |
| `patches/orbbec-kilted-*.patch` | Two patches for building the Orbbec driver on Kilted: QoS API and `-Werror` ([build steps](#gemini-2-driver-real-robot-only)) |
| `config/laser.yaml` | LiDAR serial port and settings |
| `config/<payload>_laser_filter.yaml` | Masks the part of the scan a payload blocks (currently `pantilt`) |
| `config/battery_overrides.yaml` | LeKiwi's overrides on top of the battery monitor's defaults |
| `config/diagnostic_aggregator.yaml` | The single top-level health view on `/diagnostics_agg` |
| `udev/99-lekiwi.rules` | Stable device names for the LiDAR and servo bus |

## Requirements

The other LeKiwi packages, built together with `colcon build --packages-up-to lekiwi_bringup`, which also brings in the submodules' packages (`ldlidar_ros2`, `ina260_battery_monitor` and the hardware interfaces), `laser_filters` and `diagnostic_aggregator`. The base with its Gemini 2 needs the [Orbbec driver](#gemini-2-driver-real-robot-only) for the real camera (not for `sim:=true` or `enable_camera:=false`) and nothing from `pantilt_ros2`. With the legacy pan-tilt payload, build `pt_bringup` and its other packages from [pantilt_ros2](../payloads/pantilt_ros2/README.md) too. Install the [udev rules](#stable-device-names-udev) once per robot.

## Stable device names (udev)

Real hardware (not `sim:=true`) uses these USB devices at fixed paths:

| Symlink | Device | Used by |
|---------|--------|---------|
| `/dev/ttyLIDAR` | LD06 LiDAR (CP210x USB-UART) | `lekiwi_bringup/config/laser.yaml` (`port_name`) |
| `/dev/ttySERVO` | Feetech STS servo bus | `lekiwi_control/config/urdf_config.yaml` (`serial_port`) |
| reSpeaker Flex ALSA nodes | XVF3800 USB audio | `lekiwi_audio` selects it by ALSA card name; the rule only grants non-root permission and does not rename the card |

Install `udev/99-lekiwi.rules` once per robot (command in the [repository README](../README.md#udev-rules)), then reconnect the devices or reboot. Without it, `/dev/ttyLIDAR` and `/dev/ttySERVO` do not exist and the default serial ports fail to open.

## Running

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py                                       # base with tray and Gemini 2
ros2 launch lekiwi_bringup lekiwi.launch.py payload:=pantilt                      # legacy pan-tilt payload
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=map                          # SLAM mapping
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=amcl map_name:=livingroom1   # localize on a saved map
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home         # MuJoCo simulation
ros2 launch lekiwi_bringup laser.launch.py                                        # LiDAR alone
```

### Launch arguments

The main arguments are in the [repository README](../README.md#launch-arguments). `--show-arguments` lists them all. These are the ones it does not cover:

| Argument | Default | Meaning |
|----------|---------|---------|
| `enable_camera` | `true` | Real or simulated camera publishing on/off; camera geometry remains selected |
| `pointcloud` | `false` | One colored cloud in hardware or simulation; the real driver also starts Cloudini compression. Off by default |
| `sts_serial_port`, `use_mock` | `""` | Servo serial port and mock mode; empty uses `lekiwi_control`'s `urdf_config.yaml` |
| `camera_fps` | `15` | Shared RGB/depth rate for either real camera (5, 10, 15, 30 Hz); simulation caps rendering at 5 Hz |
| `mujoco_model` | `""` | `sim` only: a pre-built MJCF; empty generates one at launch |
| `use_sim_time` | `false` | Use `/clock`; forced on with `sim:=true` |

`laser.launch.py` takes `payload` and `custom_filter`, the name of a filter file in `config/` that overrides the payload's.

## Configuration

| File | What it sets |
|------|--------------|
| `config/laser.yaml` | LD06 serial port (`/dev/ttyLIDAR`), baud rate, range and the angle crop. `product_name` is `LDLiDAR_LD19` (the LD06 and LD19 are the same device) |
| `config/pantilt_laser_filter.yaml` | The arc of the scan the pan-tilt blocks, and the filtered scan's QoS |
| `config/battery_overrides.yaml` | Only what differs from the battery monitor's own defaults, such as the state file location |
| `config/diagnostic_aggregator.yaml` | How motor, IMU, patrol and battery diagnostics roll up (with `diagnostics:=true`) |

The rest of the system is configured in the packages that own it; see their READMEs.

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
| Gemini 2 camera | `gemini2.launch.py` (this package), or `pt_bringup` with `payload:=pantilt` | real hardware and `enable_camera:=true` |

In simulation only the first three run: the simulated LiDAR and camera come from the MuJoCo plugins, and audio and the battery monitor are not simulated. `sim:=true` also forces simulation time and mock hardware. The launch file turns off the controller's own `odom -> base_footprint`; the EKF publishes it.

## Gemini 2 driver (real robot only)

Build [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) from source (Kilted has no apt package). It is the submodule `modules/OrbbecSDK_ROS2`, pinned to a tested commit. Two patches from this repository are applied to it: `orbbec-kilted-qos.patch` moves an example node to the current `rclcpp::QoS` API, and `orbbec-kilted-werror.patch` stops Kilted's `message_filters` deprecation warnings from failing the build (the package compiles with `-Werror`). Once both are applied, a plain `colcon build` of the workspace builds it. The submodule is configured with `ignore = dirty`, so the applied patches do not show up as changes.

```bash
sudo apt-get update
sudo apt-get install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev libssl-dev \
  ros-kilted-backward-ros ros-kilted-image-transport ros-kilted-image-transport-plugins \
  ros-kilted-image-publisher ros-kilted-camera-info-manager ros-kilted-diagnostic-updater \
  ros-kilted-statistics-msgs ros-kilted-xacro ros-kilted-depthimage-to-laserscan

cd ~/ros2_ws/src/lekiwi_ros2
git submodule update --init modules/OrbbecSDK_ROS2
git -C modules/OrbbecSDK_ROS2 apply ../../lekiwi_bringup/patches/orbbec-kilted-qos.patch \
  ../../lekiwi_bringup/patches/orbbec-kilted-werror.patch
cd ~/ros2_ws
rosdep install --from-paths src/lekiwi_ros2/modules/OrbbecSDK_ROS2 --ignore-src -r -y --rosdistro kilted
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select orbbec_camera_msgs orbbec_description orbbec_camera \
  --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Then install the [Orbbec USB rules](https://github.com/orbbec/OrbbecSDK_ROS2#registration-script-required) and reconnect the camera:

```bash
sudo cp src/lekiwi_ros2/modules/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

The real camera is started by `gemini2.launch.py`; `sim:=true` and `enable_camera:=false` do not need the driver.

## Camera

Gemini 2 uses `/gemini2/*` in hardware and simulation. Both real and simulated Gemini 2 generate `/gemini2/scan` directly from depth images, including when `pointcloud:=true`; `pointcloud:=true` adds `/gemini2/depth_registered/points`; on real hardware it also publishes `/gemini2/depth_registered/points/compressed` through Cloudini. Compressed RGB is always available in simulation. The real Gemini wrapper requests 640×360 color and 640×400 depth at 15 Hz. See [camera variants](../lekiwi_mujoco/README.md#camera-model) and [driver installation](#gemini-2-driver-real-robot-only). With no payload `pointcloud:=true` publishes the colored cloud without Cloudini compression; Cloudini compression is part of the pan-tilt payload.

To run LeKiwi with the default Gemini 2 geometry but no real camera streaming:

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py enable_camera:=false
```

LeKiwi passes `enable_camera` to the camera launch (`gemini2.launch.py` here, or `pt_bringup`'s with the pan-tilt), which skips the driver and streaming helpers before resolving camera dependencies when false. Camera geometry remains in the URDF. In simulation, the same flag skips the camera plugin and camera processing while keeping the selected URDF and MJCF geometry.

## Checking CPU use

A bare `lekiwi.launch.py` command starts `payload:=""`, `enable_camera:=true`, `laser:=true`, `audio:=true` and `battery_monitor:=true`, plus `lekiwi_navigation` (an empty `mission` and `map_name` start SLAM mapping and Nav2). The standalone `lekiwi_control/control.launch.py` and `lekiwi_bringup/laser.launch.py` also default to `payload:=""`.

On the robot, wait for startup to settle, then list the busiest processes and the running nodes:

```bash
ps -eo pid,pcpu,comm,args --sort=-pcpu | head -30
ros2 node list
```

Look for `orbbec_camera`, `slam_toolbox`, the Nav2 costmap nodes and `laser_scan_filter_chain`. To compare configurations, launch the same payload, navigation stack, camera driver, laser filter, audio and battery components at the same camera rate.

## Using it with another payload

Add the payload's name to `_VALID_PAYLOADS` in `lekiwi.launch.py`. If it blocks part of the LiDAR's view, add `config/<name>_laser_filter.yaml` and `laser.launch.py` applies it automatically. The [repository README](../lekiwi_description/README.md#using-it-on-another-robot) lists the rest of the wiring; a few launch branches still test for `pantilt` explicitly.

## Tests

```bash
pytest test -q
```

The tests check that the launch arguments are declared and validated, and that the LiDAR filter configuration resolves for each payload.
