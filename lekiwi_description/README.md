# LeKiwi Description

Base URDF and split component meshes, including the tray and optional sensors.

## Contents

| Path | Purpose |
|------|---------|
| `urdf/base.urdf.xacro` | The standalone base: a `base_footprint` root, the base module and the static Gemini 2 |
| `urdf/base.module.xacro` | The base's links and joints as a `lekiwi_base_module` macro, including the Pi case, tray and shared sensor plate |
| `urdf/ld06.module.xacro`, `bno055.module.xacro`, `vl53l5cx.module.xacro`, `respeaker.module.xacro` | One macro per sensor assembly, called from the base module when its `hardware.yaml` switch is on |
| `urdf/base.common.xacro` | Geometry constants: colours, wheel geometry and the mounting offsets of the LiDAR, IMU, microphones, ToF imager and Gemini 2 |
| `urdf/gemini2.module.xacro` | Fixed Gemini 2 mesh and camera frames |
| `urdf/base.control.xacro` | The `<ros2_control>` block for the wheels and IMU, and the xacro arguments below |
| `urdf/base.urdf` | Pre-generated base (tray and Gemini 2) URDF |

## Requirements

ROS 2 with `xacro`.

## Running

There is no launch file here. Expand the xacro yourself, for example to check the model in RViz, or let `lekiwi_control` do it at launch. The wheel controller configuration must be passed in:

```bash
cd urdf
xacro base.urdf.xacro hardware_config:=$(ros2 pkg prefix lekiwi_bringup)/share/lekiwi_bringup/config/hardware.yaml base_controller_config:=$(ros2 pkg prefix lekiwi_control)/share/lekiwi_control/config/control.yaml
```

## Configuration

Pass these to `xacro` as `name:=value`. `lekiwi_control` fills them in from its `urdf_config.yaml`.

| Argument | Default | Meaning |
|----------|---------|---------|
| `hardware_config` | required | Path to the bringup hardware profile |
| `base_controller_config` | required | Path to `lekiwi_control/config/control.yaml`; the wheel geometry comes from it |
| `serial_port` | `/dev/ttySERVO` | Servo bus serial port |
| `baud_rate` | `1000000` | Servo bus baud rate |
| `use_mock` | `false` | Simulated motor responses instead of hardware |
| `use_sync_write` | `true` | Send all servo commands in one bus write |
| `left_motor_id`, `back_motor_id`, `right_motor_id` | `7`, `8`, `9` | Wheel servo IDs |
| `sts3215_max_vel_steps` | `3400` | Servo maximum speed in steps/s |
| `internal_max_vel`, `internal_max_acc`, `internal_acc_coeff` | `254`, `254`, `100` | Wheel servo speed profile, written to each servo at start-up |
| `internal_control_period` | `20` (launch uses `10`) | The servos' internal control-loop period register (1–254, written to EEPROM), set for the wheel servos |
| `proportional_acc_max` | `25` | Scale wheel acceleration together; the largest move gets this acceleration (0–254). `0` disables scaling |
| `imu_i2c_bus`, `imu_i2c_addr`, `imu_axis_remap`, `imu_sensor_mode` | `1`, `28`, `P1`, `NDOF` | IMU I2C bus, I2C address (`0x28`), mounting orientation and fusion mode |
| `ros2_control_hardware_type` | `real` | Hardware plugin: `real`, `gazebo` or `mujoco` |
| `mujoco_model`, `mujoco_headless` | `""`, `false` | `mujoco` only: the generated MJCF and whether to skip the viewer |

## Frames

```text
base_footprint
└── base_link
    ├── rpi5_case_link                  ← Raspberry Pi 5 case assembly
    ├── lidar_plate_link           ← shared LiDAR/ToF plate assembly
    ├── laser_link ─ laser_frame        ← LiDAR
    ├── imu_link ─ imu_frame            ← IMU
    ├── tray_link                       ← tray and spacers assembly
    ├── gemini2_mount_link              ← camera mount
    ├── gemini2_link ─ gemini2_imu_frame, gemini2_link_model_origin   ← static Gemini 2 (base only)
    ├── respeaker_board_link            ← board and mounting assemblies
    ├── speaker_link                    ← speaker and mount assembly
    ├── mic_link                        ← microphone array
    ├── tof_link ─ tof_frame            ← VL53L5CX ToF imager, facing up; tof_frame has +x to the robot's right, +y forward, +z up
    ├── left_wheel_link, back_wheel_link, right_wheel_link
```

The split base components have fixed joints to `base_link`, with their CAD coordinates and the same mesh rotation as `lekiwi_base.stl`. The Gemini 2 mount is lowered by 7 mm to seat its underside on the base upper plate (`gemini2_mount_base_offset_z`). Their mass remains part of the existing lumped base inertia.

The merged `tray_assembly.stl` includes the tray and spacers in one fixed link. `speaker_assembly.stl` combines the speaker and mount, and `respeaker_board_assembly.stl` combines the control board and its mounting parts. These meshes retain their CAD positions and base rotation. The board assembly is reduced to 150,000 faces and shared with MuJoCo.

## ToF imager

The SparkFun VL53L5CX sits on the holder pad on the rear plate (base mesh coordinates: centre x = −40 mm, y = +70 mm, pad top z = 95.5 mm), facing up. `sparkfun_vl53l5cx.stl` has its origin at a board corner, so `tof_base_offset_x/y/z` are the pad centre shifted by 12.7 mm in x and y, and `tof_offset_rz = π` turns the board so its 7-pad header faces +y and the Qwiic connectors lie on ±x, as in the CAD.

`tof_frame` is the chip's top-face centre (`tof_frame_offset_x/y/z`) and the frame `tof_imager_ros` publishes its point cloud in. Its +z is the boresight, so ranges point up. `tof_frame_offset_rz = π/2` turns the cloud onto the robot: with it, +x is the robot's right and +y forward, so a sheet moved from back to front moves the points the same way. The driver's pixel order is fixed by the sensor, so this yaw is the one value to re-measure if the sensor is remounted rotated. A frame cannot mirror, and the driver already produces a right-handed cloud. After changing an offset, regenerate `base.urdf` and the MuJoCo models (see below).

## Regenerating the pre-built URDF

Run from this package directory. The generated `urdf/base.urdf` uses `../meshes/` paths; runtime Xacro retains `package://` mesh URIs.

```bash
python3 test/test_urdf_xacro.py --write
```

## Tests

```bash
pytest test -q
```

## Configure the robot

Edit [`lekiwi_bringup/config/hardware.yaml`](../lekiwi_bringup/config/hardware.yaml), then launch normally:

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py
```

That file is the single source for robot composition in bringup, control, Xacro
and generated MuJoCo models.

```yaml
lidar: true
imu: true
battery_monitor: true
rpi5: true
respeaker: true
sparkfun_sensor: true
gemini2: true
shelf: true
```

| Setting | Effect |
| --- | --- |
| `lidar` | LD19/LD06 LiDAR and its frames; the shared mounting plate stays while the ToF sensor is on. `false` also stops SLAM, Nav2 and the collision monitor |
| `imu` | BNO055 IMU, its frames and controller; `false` needs `fusion_mode:=odom` |
| `battery_monitor` | INA260 battery monitor and events (real robot only) |
| `shelf` | Tray and shelf spacers |
| `rpi5` | Raspberry Pi 5 case; false removes the case for a replacement computer |
| `respeaker` | Microphone, control board, speaker, and all their mounts |
| `sparkfun_sensor` | VL53L5CX only; the shared mounting plate stays while the LiDAR is on |
| `gemini2` | Base Gemini 2 and mount, plus camera streaming |

The switches are independent. Each assembly's links, mounts and joints live in
one Xacro conditional. Missing settings keep their defaults, while unknown keys
and non-boolean switches are rejected at launch. Defaults preserve the full base
robot. The `rpi5` switch controls the Pi 5 case; meshes for other computers can be added later.

For a separate profile, pass only `hardware_config:=/path/to/hardware.yaml` to
bringup or control. Direct Xacro expansion requires `hardware_config:=/path/to/hardware.yaml`;
Hardware selection has no per-device launch arguments. Runtime options
such as `sim`, `mission`, `pointcloud` and camera stream settings remain launch
arguments.

Setting `gemini2: false` disables streaming in either mode and removes the base camera
and mount. An explicit pre-built `mujoco_model` must match the selected hardware.
Base inertia remains a lumped estimate when fixed mounts are removed.

`lidar_plate_assembly.stl` combines the LiDAR and SparkFun mounting plates in
one fixed link that stays while either sensor is enabled and is removed when both are disabled.

The SparkFun sensor and Gemini 2 currently have one supported model each. To add
another sensor on the shared plate, replace its sensor mesh/frame and driver
integration together. A replacement camera needs its own mesh, mount, frames and
driver configuration; disabling `gemini2` removes both camera and mount. No
replacement model choices are exposed until their assets and drivers are added.
