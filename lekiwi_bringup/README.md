# LeKiwi Bringup

The top-level launch files for LeKiwi. One command starts the whole system (control, navigation, sensors, audio, and the Gemini 2 camera) on the real robot or in the MuJoCo simulation. This package owns `hardware.yaml`, sensor settings, point-cloud compression and the udev rules.

## Contents

| Path | Purpose |
|------|---------|
| `launch/lekiwi.launch.py` | The whole system; includes the other packages' launch files |
| `launch/laser.launch.py` | The LD06 LiDAR driver, with an optional custom scan filter |
| `launch/tof.launch.py` | The VL53L5CX ToF imager (`tof_imager_ros`), configured and activated on start (`lekiwi.launch.py` starts it 20 s after launch) |
| `launch/gemini2.launch.py` | The Orbbec Gemini 2 driver for the base |
| `patches/orbbec-kilted-*.patch` | Two patches for building the Orbbec driver on Kilted: QoS API and `-Werror` ([build steps](#gemini-2-driver-real-robot-only)) |
| `config/hardware.yaml` | Fitted assemblies and sensor switches |
| `config/laser.yaml` | LiDAR serial port and settings |
| `config/gemini2.yaml`, `src/pcl_compressor_node.cpp` | Real camera stream settings (resolution, frame rates, depth registration, filters, laser power, optional IMU) and Cloudini compression for the camera cloud |
| `config/tof.yaml` | ToF imager resolution, ranging rate, frame and publish period |
| `config/battery_overrides.yaml` | LeKiwi's overrides on top of the battery monitor's defaults (state file, 5 Hz rate) |
| `config/diagnostic_aggregator.yaml` | The single top-level health view on `/diagnostics_agg` |
| `udev/99-lekiwi.rules` | Stable device names for the LiDAR and servo bus |

## Requirements

Build the packages and drivers described in the [repository installation instructions](../README.md#installation). The C++ point-cloud compressor requires `cloudini_lib`, `cloudini_ros`, `point_cloud_interfaces`, `rclcpp` and `rclcpp_components`; make these packages available in the workspace or underlay before building bringup. The real Gemini 2 driver is built separately below.

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
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=map                          # SLAM mapping
ros2 launch lekiwi_bringup lekiwi.launch.py mission:=amcl map_name:=livingroom1   # localize on a saved map
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home         # MuJoCo simulation
ros2 launch lekiwi_bringup laser.launch.py                                        # LiDAR alone
ros2 launch lekiwi_bringup tof.launch.py                                          # ToF imager alone
```

### Launch arguments

The main arguments are in the [repository README](../README.md#launch-arguments). `--show-arguments` lists them all. These are the ones it does not cover:

| Argument | Default | Meaning |
|----------|---------|---------|
| `hardware_config` | `config/hardware.yaml` in this package | Hardware composition profile, forwarded to control and model generation |
| `enable_camera` | `true` | Real or simulated camera publishing on/off; camera geometry remains selected |
| `pointcloud` | `false` | One colored cloud in hardware or simulation; the real driver also starts Cloudini compression. Off by default |
| `sts_serial_port`, `use_mock` | `""` | Servo serial port and mock mode; empty uses `lekiwi_control`'s `urdf_config.yaml` |
| `mujoco_model` | `""` | `sim` only: a pre-built MJCF; empty generates one at launch |
| `use_sim_time` | `false` | Use `/clock`; forced on with `sim:=true` |

## Configuration

| File | What it sets |
|------|--------------|
| `config/hardware.yaml` | Independent hardware switches; defaults enable every group |
| `config/gemini2.yaml` | The real camera stream: color (15 FPS) and depth (5 FPS) profiles, depth registration, filters, laser power and the optional IMU, plus the Cloudini topics, XYZ resolution (1 cm) and point filtering |
| `config/laser.yaml` | LD06 serial port (`/dev/ttyLIDAR`), baud rate, range and the angle crop. `product_name` is `LDLiDAR_LD19` (the LD06 and LD19 are the same device) |
| `config/tof.yaml` | `tof_imager_ros` parameters: 8x8 or 4x4 resolution, continuous or autonomous mode, ranging rate (10 Hz; max 15 Hz at 8x8), publish period and `frame_id: tof_frame` |
| `config/battery_overrides.yaml` | Only what differs from the battery monitor's own defaults: the state file location, a 5 Hz `battery_state` rate with the charger-detection windows and ETA smoothing scaled to it |
| `config/diagnostic_aggregator.yaml` | How motor, IMU, patrol and battery diagnostics roll up (with `diagnostics:=true`) |

The rest of the system is configured in the packages that own it; see their READMEs.

## How it works

`lekiwi.launch.py` includes the other packages' launch files and passes the shared arguments on, so each keeps its own defaults:

| Started | By | Condition |
|---------|----|-----------|
| Control and teleop | `lekiwi_control` | always (MuJoCo model with `sim:=true`) |
| EKF | `lekiwi_navigation` | always; wheel odometry + IMU by default (`fusion_mode:=odom` with `imu: false`) |
| SLAM or AMCL, Nav2, waypoint patrol and target tracker | `lekiwi_navigation` | `lidar: true` |
| Gemini 2 driver and streaming helpers | `gemini2.launch.py` | real hardware, `gemini2: true`; streaming follows `enable_camera` |
| Diagnostic aggregator | this package | `diagnostics:=true` |
| LiDAR | `laser.launch.py` | real hardware and `lidar: true` in `hardware.yaml` |
| ToF imager (`tof/pointcloud` in `tof_frame`) | `tof.launch.py`, 20 s after launch | real hardware and `sparkfun_sensor: true` in `hardware.yaml` |
| Audio | `lekiwi_audio` | real hardware and `respeaker: true` in `hardware.yaml` |
| Battery monitor and events (one process, `battery_nodes`) | `ina260_battery_monitor` | real hardware and `battery_monitor: true` in `hardware.yaml` |

In simulation, control, the selected navigation path and optional diagnostics run. Enabled simulated sensors come from MuJoCo plugins; real sensor drivers, audio and the battery monitor are skipped. `sim:=true` also forces simulation time and mock hardware. The launch file turns off the controller's own `odom -> base_footprint`; the EKF publishes it.

## Gemini 2 driver (real robot only)

Build [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) from source (Kilted has no apt package). It is the submodule `modules/OrbbecSDK_ROS2`, pinned to a tested commit. Two patches from this repository are applied to it: `orbbec-kilted-qos.patch` moves an example node to the current `rclcpp::QoS` API, and `orbbec-kilted-werror.patch` stops Kilted's `message_filters` deprecation warnings from failing the build (the package compiles with `-Werror`). Once both are applied, a plain `colcon build` of the workspace builds it. The submodule is configured with `ignore = dirty`, so the applied patches do not show up as changes.

```bash
sudo apt-get update
sudo apt-get install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev libssl-dev \
  ros-kilted-backward-ros ros-kilted-image-transport ros-kilted-image-transport-plugins \
  ros-kilted-image-publisher ros-kilted-camera-info-manager ros-kilted-diagnostic-updater \
  ros-kilted-statistics-msgs ros-kilted-xacro

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

## ToF imager

The SparkFun VL53L5CX (8×8 zones, 45° field of view) faces up from the rear plate. `tof.launch.py` runs [`tof_imager_ros`](https://github.com/adityakamath/tof_imager_ros) as a lifecycle node with `config/tof.yaml`, configures it and activates it once configured. The 8×8 point cloud is `tof/pointcloud` (`sensor_msgs/PointCloud2`, ranging at the 10 Hz set in `config/tof.yaml`) in `tof_frame`: +z along the sensor's boresight (up), +x to the robot's right and +y forward. The frame's yaw is set in the URDF (see the [description README](../lekiwi_description/README.md#tof-imager)). In simulation the same topic comes from the MuJoCo model (see [`lekiwi_mujoco`](../lekiwi_mujoco/README.md#tof-imager)); the same `sparkfun_sensor` hardware setting selects it there.

Real robot only, once per robot: enable I2C on the Raspberry Pi and install the sensor library into the system Python that ROS uses (`pip3 install --break-system-packages ./modules/tof_imager_ros/vl53l5cx_python`; the [`tof_imager_ros` README](https://github.com/adityakamath/tof_imager_ros#how-to-use) has the I2C steps). If the node exits at start, `ros2 launch` shows why: `No module named 'vl53l5cx'` means the library is missing from that Python, and `Sensor not alive` means wiring or the I2C address. Check it with:

```bash
ros2 lifecycle get /tof_imager         # active
ros2 topic hz /tof/pointcloud          # published only while something subscribes
```

Set `sparkfun_sensor: false` in `hardware.yaml` if no sensor is fitted.

The ToF imager starts 20 s after launch, once the controllers are active. It reads the sensor only while `tof/pointcloud` has a subscriber.

## Raspberry Pi 5 system settings

Per-robot operating-system settings (not part of the repository) used on the reference robot, a Raspberry Pi 5 with Ubuntu 24.04 and Kilted:

| Setting | Where | Purpose |
|---------|-------|---------|
| `dtparam=i2c_arm_baudrate=400000` | `/boot/firmware/config.txt` | 400 kHz I2C bus shared by the BNO055, the battery monitor and the ToF imager |
| `usb_max_current_enable=1` | `/boot/firmware/config.txt` | Full USB current for the Gemini 2; use a 5 V / 5 A supply |
| `dtparam=audio=off`, `camera_auto_detect=0` | `/boot/firmware/config.txt` | Disable unused interfaces |
| `rtprio 99` and `memlock unlimited` for the `realtime` group | `/etc/security/limits.d/` | Real-time scheduling for the control loop; for sessions started through Tailscale SSH also set `LimitRTPRIO` and `LimitMEMLOCK` in a systemd drop-in for `tailscaled` |

`config.txt` changes take effect after a reboot. Optionally `export ROS_LOG_DIR=/dev/shm/ros_log` keeps ROS logs in RAM (they are lost on reboot).

## Camera

`gemini2: false` in `hardware.yaml` removes the camera, mount and processing.
`enable_camera:=false` keeps the fitted camera geometry but skips streaming.
The real wrapper requests 640×360 color and 640×400 depth, with color at 15 FPS and depth at 5 FPS, all from `config/gemini2.yaml`;
simulation uses the plugin YAML rate (5 Hz).
`config/gemini2.yaml` also sets `depth_registration` (on: depth is aligned to color; off: depth keeps its own size), `enable_noise_removal_filter`, `enable_frame_sync`, `enable_ldp`, `enable_imu` and `laser_energy_level` (IR projector power, -1 = device default); all are required. `pointcloud:=true` needs `depth_registration: true` and the launch stops with an error otherwise.

The Gemini 2 also has an accelerometer and gyroscope, off by default. Set `enable_imu: true` in `config/gemini2.yaml` and relaunch to get raw samples (`sensor_msgs/Imu`, no orientation) on `/gemini2/gyro_accel/sample`, in `gemini2_accel_gyro_optical_frame`.

`pointcloud:=true` enables `/gemini2/depth_registered/points`. On real hardware
it also starts `PCLCompressorNode`, which drops invalid/duplicate voxel points,
compresses XYZ at 1 cm resolution while preserving RGB, and publishes Cloudini
messages on `/gemini2/depth_registered/points/compressed`. Its settings are in
`config/gemini2.yaml`. Simulation publishes the raw colored cloud without
this compressor. Direct `gemini2.launch.py` calls additionally accept
`pointcloud_decimation` (default 2), `serial_number` and `usb_port`.

If the depth stream never appears, check the Orbbec SDK log in `~/.ros/Log/gemini2/`: a healthy start logs `Stream state changed from STARTING to STREAMING@Depth`, and a stalled one stays at `STARTING` while color slows to about half rate. `dmesg` then usually shows the camera resetting or disconnecting. Stop the launch with a single Ctrl-C and let it exit; a hard kill can leave the camera streaming, so reconnect it before the next start. A 320×200 depth request stalled or reset the camera in several runs on our unit; the driver lists 1280×800, 640×400, 424×266 and 320×200 as the supported depth sizes, and the published depth is the color size either way.

To run LeKiwi with the default Gemini 2 geometry but no real camera streaming:

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py enable_camera:=false
```

## Checking CPU use

A bare `lekiwi.launch.py` command reads `hardware.yaml` and starts the configured hardware plus navigation. The default file enables every group. `fusion_mode` defaults to `base` (wheel odometry and IMU); set `fusion_mode:=odom` when `imu: false`.

On the robot, wait for startup to settle, then list the busiest processes and the running nodes:

```bash
ps -eo pid,pcpu,comm,args --sort=-pcpu | head -30
ros2 node list
```

Look for `orbbec_camera`, `slam_toolbox`, `nav2_container` (all the Nav2 nodes with the default `use_composition:=true`, or the separate Nav2 processes with `false`), `battery_nodes` and `laser_scan_filter_chain`. To compare configurations, launch the same navigation stack, camera driver, laser filter, audio and battery components at the same camera rate.

## Tests

```bash
pytest test -q
```

The tests check launch arguments, hardware-profile forwarding, the LiDAR, IMU and battery switches, navigation and real battery startup, camera processing and explicit LiDAR filter paths without starting hardware drivers.

Configure robot composition only in
[`config/hardware.yaml`](config/hardware.yaml).
It selects the fitted assemblies. Launch normally after editing it; no hardware flags are needed. See
[Configure the robot](../lekiwi_description/README.md#configure-the-robot).

With the default profile, the LiDAR, BNO055 IMU and (on the real robot) battery monitoring run, and the velocity
path passes through the collision monitor. `lidar: false` skips the LiDAR driver, SLAM, Nav2 and the collision monitor, leaving only the EKF; commands then go straight to the base. `imu: false` needs `fusion_mode:=odom`. `battery_monitor: false` skips the battery nodes.

The SparkFun sensor and Gemini 2 currently have one supported model each. To add
another sensor on the permanent plate, replace its sensor mesh/frame and driver
integration together. A replacement camera needs its own mesh, mount, frames and
driver configuration; disabling `gemini2` removes both camera and mount. No
replacement model choices are exposed until their assets and drivers are added.
