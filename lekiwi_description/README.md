# LeKiwi Description

URDF/xacro models of LeKiwi: the standalone base, and the base with the pan-tilt payload mounted. The MuJoCo model, the controllers and the launch files are all generated from these files.

## Contents

| Path | Purpose |
|------|---------|
| `urdf/base/base.urdf.xacro` | The standalone base: a `base_footprint` root and the base module |
| `urdf/base/base.module.xacro` | The base's links and joints as a `lekiwi_base_module` macro |
| `urdf/base/base.common.xacro` | Geometry constants: colours, wheel geometry and component offsets |
| `urdf/base/base.control.xacro` | The `<ros2_control>` block for the wheels and IMU, and the xacro arguments below |
| `urdf/base_pantilt/` | The base with the pan-tilt: the same files, plus the pan-tilt module from `pt_description` and a combined `<ros2_control>` block for the shared servo bus |
| `urdf/*/*.urdf` | Pre-generated URDFs of the two variants |
| `meshes/` | STL files for the base, wheels, LiDAR, IMU and microphone array (`*_mujoco.stl` are simplified for the simulation) |

## Requirements

ROS 2 Kilted with `xacro`, and `pt_description` from [pantilt_ros2](../payloads/pantilt_ros2/README.md) for the pan-tilt variant.

## Running

There is no launch file here. Expand the xacro yourself, for example to check the model in RViz, or let `lekiwi_control` do it at launch. The wheel controller configuration must be passed in:

```bash
cd urdf
xacro base/base.urdf.xacro base_controller_config:=$(ros2 pkg prefix lekiwi_control)/share/lekiwi_control/config/control.yaml
xacro base_pantilt/base_pantilt.urdf.xacro base_controller_config:=<same path> pantilt_config:=pt101
```

## Configuration

Pass these to `xacro` as `name:=value`. `lekiwi_control` fills them in from its `urdf_config.yaml`.

| Argument | Default | Meaning |
|----------|---------|---------|
| `base_controller_config` | required | Path to `lekiwi_control/config/control.yaml`; the wheel geometry comes from it |
| `serial_port` | `/dev/ttySERVO` | Servo bus serial port |
| `baud_rate` | `1000000` | Servo bus baud rate |
| `use_mock` | `false` | Simulated motor responses instead of hardware |
| `use_sync_write` | `true` | Send all servo commands in one bus write |
| `left_motor_id`, `back_motor_id`, `right_motor_id` | `7`, `8`, `9` | Wheel servo IDs |
| `sts3215_max_vel_steps` | `3400` | Servo maximum speed in steps/s |
| `internal_max_vel`, `internal_max_acc`, `internal_acc_coeff` | `254`, `254`, `100` | Wheel servo speed profile, written to each servo at start-up |
| `imu` | `true` | A physical BNO055 is present; `false` leaves out the IMU sensor |
| `imu_i2c_bus`, `imu_i2c_addr`, `imu_axis_remap`, `imu_sensor_mode` | `1`, `28`, `P1`, `NDOF` | IMU I2C bus, I2C address (`0x28`), mounting orientation and fusion mode |
| `ros2_control_hardware_type` | `real` | Hardware plugin: `real`, `gazebo` or `mujoco` |
| `mujoco_model`, `mujoco_headless` | `""`, `false` | `mujoco` only: the generated MJCF and whether to skip the viewer |
| `pantilt_config` | `pt101` | `base_pantilt` only: pan-tilt mesh variant, `pt100` or `pt101` |
| `pantilt_internal_max_vel`, `pantilt_internal_max_acc`, `pantilt_internal_acc_coeff` | `65`, `50`, `0` | `base_pantilt` only: pan-tilt servo speed profile |

## Frames

```text
base_footprint
└── base_link
    ├── laser_link ─ laser_frame        ← LiDAR
    ├── imu_link ─ imu_frame            ← IMU
    ├── mic_link                        ← microphone array
    ├── left_wheel_link, back_wheel_link, right_wheel_link
    └── pantilt_base_link               ← base_pantilt only; the pan-tilt hangs off this mount
```

## Regenerating the pre-built URDFs

The checked-in `.urdf` files are for tools that want plain URDF. The local meshes use relative paths, and the pan-tilt meshes use raw GitHub URLs from `pantilt_ros2`. Regenerate them after any xacro change:

```bash
python3 test/test_urdf_xacro.py --write
```

## Using it on another robot

A payload adds a `urdf/base_<name>/` folder that includes the payload's own module and mounts it on `base_link` at a fixed joint (see `base_pantilt.urdf.xacro`). The [repository README](../README.md#payloads) lists the other pieces of wiring a payload needs.

## Tests

```bash
pytest test -q
```

The tests expand every xacro file with mock hardware arguments, check that the output is valid XML with the expected links, joints and interfaces, and check that the pre-built URDFs are not stale.
