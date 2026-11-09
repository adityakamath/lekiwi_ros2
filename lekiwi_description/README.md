# LeKiwi Description

URDF/xacro models of LeKiwi: the standalone base with its tray and static Gemini 2 camera (the recommended configuration), and the base with the legacy pan-tilt payload mounted. The MuJoCo model, the controllers and the launch files are all generated from these files.

## Contents

| Path | Purpose |
|------|---------|
| `urdf/base/base.urdf.xacro` | The standalone base: a `base_footprint` root, the base module and the static Gemini 2 |
| `urdf/base/base.module.xacro` | The base's links and joints as a `lekiwi_base_module` macro; `payload:=true` swaps the tray for the payload plate |
| `urdf/base/camera.module.xacro` | The static Gemini 2 (`gemini2_camera` macro): `gemini2_link`, `gemini2_imu_frame` and the mesh, with the same frames as the pan-tilt's camera |
| `urdf/base/base.common.xacro` | Geometry constants: colours, wheel geometry and component offsets |
| `urdf/base/base.control.xacro` | The `<ros2_control>` block for the wheels and IMU, and the xacro arguments below |
| `urdf/base_pantilt/` | The base with the pan-tilt: the same files, plus the pan-tilt module from `pt_description` and a combined `<ros2_control>` block for the shared servo bus |
| `urdf/*/*.urdf` | Pre-generated base (tray and Gemini 2) and PT101/Gemini 2 pan-tilt URDFs |
| `meshes/` | STL files for the base, the two top plates (`container_plate`, `payload_plate`), Gemini 2, wheels, LiDAR, IMU and microphone array (`*_mujoco.stl` are simplified for the simulation). The files hold geometry only; colours are set in the URDF |

## Requirements

ROS 2 Kilted with `xacro`. The base needs nothing else. `pt_description` from [pantilt_ros2](../payloads/pantilt_ros2/README.md) is needed only for the legacy pan-tilt variant.

## Running

There is no launch file here. Expand the xacro yourself, for example to check the model in RViz, or let `lekiwi_control` do it at launch. The wheel controller configuration must be passed in:

```bash
cd urdf
xacro base/base.urdf.xacro base_controller_config:=$(ros2 pkg prefix lekiwi_control)/share/lekiwi_control/config/control.yaml
xacro base_pantilt/base_pantilt.urdf.xacro base_controller_config:=<same path>
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
| `internal_control_period` | `20` | The servos' internal control-loop period register (1–254, written to EEPROM), set for the wheel servos |
| `proportional_acc_max`, `proportional_vel_max` | `25`, `0` | Make the wheels finish ramping together: the wheel with the largest move gets this acceleration (0–254) or velocity (steps/s), and the others are scaled to match. `0` disables each; with `proportional_vel_max` at 0, each joint uses its commanded velocity |
| `imu` | `true` | A physical BNO055 is present; `false` leaves out the IMU sensor |
| `imu_i2c_bus`, `imu_i2c_addr`, `imu_axis_remap`, `imu_sensor_mode` | `1`, `28`, `P1`, `NDOF` | IMU I2C bus, I2C address (`0x28`), mounting orientation and fusion mode |
| `ros2_control_hardware_type` | `real` | Hardware plugin: `real`, `gazebo` or `mujoco` |
| `mujoco_model`, `mujoco_headless` | `""`, `false` | `mujoco` only: the generated MJCF and whether to skip the viewer |
| `enable_camera` | `true` | `mujoco` only: expose the Gemini 2 camera IMU sensor interface when streaming is enabled (base and `base_pantilt`) |
| `pantilt_internal_max_vel`, `pantilt_internal_max_acc`, `pantilt_internal_acc_coeff` | `65`, `50`, `0` | `base_pantilt` only: pan-tilt servo speed profile |

## Frames

```text
base_footprint
└── base_link
    ├── laser_link ─ laser_frame        ← LiDAR
    ├── imu_link ─ imu_frame            ← IMU
    ├── plate_link                      ← top plate: container_plate, or payload_plate with a payload
    ├── gemini2_link ─ gemini2_imu_frame, gemini2_link_model_origin   ← static Gemini 2 (base only)
    ├── mic_link                        ← microphone array
    ├── left_wheel_link, back_wheel_link, right_wheel_link
    └── pantilt_base_link               ← base_pantilt only; the pan-tilt, with its own gemini2_link, hangs off this mount
```

## Regenerating the pre-built URDFs

The checked-in files are `urdf/base/base.urdf` and `urdf/base_pantilt/base_pantilt_gemini2.urdf`, which uses PT101. These files are for tools that want plain URDF. LeKiwi meshes, including the Gemini 2, use `../../meshes/` paths, while pan-tilt meshes use `https://raw.githubusercontent.com/adityakamath/pantilt_ros2/main/pt_description/meshes/`. Runtime Xacro retains `package://` mesh URIs. Regenerate them after any xacro change:

```bash
python3 test/test_urdf_xacro.py --write
```

## Using it on another robot

A payload adds a `urdf/base_<name>/` folder that includes the payload's own module and mounts it on `base_link` at a fixed joint (see `base_pantilt.urdf.xacro`). The base's top plate is `container_plate.stl` by default; a payload swaps it for `payload_plate.stl` by passing `payload="true"` to `lekiwi_base_module`. A payload's URDF does not include the base's static camera; a payload with a camera adds its own.

The rest of the wiring, where `<name>` is the `payload` value:

1. **Registered** in `_VALID_PAYLOADS` in `lekiwi_bringup/launch/lekiwi.launch.py`, which forwards `payload` to the control launch (which also starts teleop) and the laser launch.
2. **Control and teleop:** the payload's own control package supplies its controller (`lekiwi_control` passes it to the spawner with `--param-file`) and its servo profile, and one `controller_manager` drives the base and the payload. Add the joystick layout as `lekiwi_control/config/<name>_teleop.yaml`, loaded on top of `base_teleop.yaml`.
3. **Sensors:** a payload that blocks part of the LiDAR's view adds `lekiwi_bringup/config/<name>_laser_filter.yaml`, which `laser.launch.py` applies automatically.
4. **Simulation:** the payload's MuJoCo package builds its model from the robot's URDF, and `lekiwi_mujoco` attaches it at the URDF's mount joint.

A few launch-file branches still test for `pantilt` explicitly (in `lekiwi.launch.py` and `control.launch.py`), so extend them for a new payload.

## Tests

```bash
pytest test -q
```

The tests expand every xacro file with mock hardware arguments, check that the output is valid XML with the expected links, joints and interfaces, and check that the pre-built URDFs are not stale.
