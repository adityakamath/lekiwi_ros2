# LeKiwi MuJoCo

Simulation of the mobile base, tray and configured sensors, generated from the base URDF.

- **Standalone:** a native viewer, a model builder and a benchmark, with no ROS needed.
- **ROS simulation:** `sim:=true` runs the same launch files as the real robot against the model in `mujoco_ros2_control`.

## Contents

| Path | Purpose |
|------|---------|
| `lekiwi_mujoco/build_mujoco_models.py` | Builds the model from the URDF and controller config |
| `lekiwi_mujoco/mujoco_parameters.py` | Applies the physics and servo parameters from `config/mujoco.yaml` to the model |
| `lekiwi_mujoco/simulation.py` | The simulation without ROS: reset, command, step |
| `lekiwi_mujoco/mujoco_preview.py`, `benchmark_mujoco.py` | Native viewer and motion benchmark |
| `lekiwi_mujoco/gemini2_depth.py`, `gemini2_cloud.py` | ROS helper nodes for the simulated Gemini 2: registered depth, and the optional colored cloud |
| `lekiwi_mujoco/tof_pointcloud.py` | ROS helper node for the simulated ToF imager: rotates the plugin's cloud into `tof_frame` as `/tof/pointcloud` |
| `lekiwi_mujoco/paths.py` | Locates the sibling packages' files, with `*_SHARE` environment overrides |
| `config/` | Physics and servo parameters, ROS plugin configuration, camera rate, laser filters |
| `mjcf/` | MJCF sources, the `scenes/` and the pre-built `lekiwi_base.xml` model |

## Simplified meshes

MuJoCo's STL loader accepts at most 200,000 faces per mesh. The chassis now uses
`lekiwi_base.stl` directly (75,602 faces), with the Pi case, sensor mounts,
speaker and top-plate spacers drawn as separate fixed components from the URDF.
Superseded combined base and tray STLs have been removed.

Only the wheel needs a separate simplified visual copy:

| Original | Faces | MuJoCo copy | Faces |
| --- | ---: | --- | ---: |
| `omni_wheel.stl` | 543,010 | `omni_wheel_mujoco.stl` | 196,710 |

`respeaker_board_assembly.stl` combines the control board and its mounts and is
reduced to 150,000 faces. Both URDF and MuJoCo use that same file.
`speaker_assembly.stl` and `tray_assembly.stl` combine the speaker/mount and
tray/spacers respectively; both fit MuJoCo's limit without simplification.

The split CAD meshes remain visual only. Contacts still use the approximate
chassis cylinder and passive rollers; removing assemblies does not change that
coarse chassis collision envelope. Masses and inertia come from the selected
URDF, including its lumped base estimate.

## Optional hardware

ROS launch, direct Xacro expansion and the standalone builder load the same
[`hardware.yaml`](../lekiwi_bringup/config/hardware.yaml). All hardware switches
default to true. Disabled assemblies lose their geometry and corresponding ROS
nodes and sensor interfaces. `shelf` controls the tray and its spacers together.

```bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true hardware_config:=/path/to/hardware.yaml
python3 -m lekiwi_mujoco.build_mujoco_models --output /tmp/robot.xml --hardware-config /path/to/hardware.yaml
```

An explicit `mujoco_model` must already match the hardware configuration.

## Camera model

```bash
python3 -m lekiwi_mujoco.build_mujoco_models # rebuild the robot selected in hardware.yaml
python3 -m lekiwi_mujoco.mujoco_preview
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true                        # base with Gemini 2
```

The camera assembly is built from the base URDF. Generated MJCF assets keep the existing relative filesystem paths. Gemini simulation uses `/gemini2/*`. The Gemini profile follows [Orbbec's specifications](https://www.orbbec.com/products/stereo-vision-camera/gemini-2/), [datasheet](https://www.orbbec.com/wp-content/uploads/2023/04/ORBBEC_Datasheet_Gemini-2.pdf), and the [ROS SDK launch configuration](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/launch/gemini2.launch.py):

- RGB: 640×360, nominal 55° vertical / approximately 86° horizontal field of view, upright at the home pose. Rendering runs at 5 Hz, configured in the camera plugin YAML.
- Registered depth: ideal depth rendered into the RGB view, with the same intrinsics and `gemini2_color_optical_frame`. `/gemini2/depth/image_raw` is `16UC1` in millimeters; invalid values and depths outside 0.15–10 m are zero. Both color and depth camera-info topics are published.
- Optional point cloud: `pointcloud:=true` starts one colored XYZRGB cloud at `/gemini2/depth_registered/points`. The helper removes invalid XYZ points while preserving valid RGB data. No cloud converters run by default.

An explicit `--model` or `mujoco_model` selects an existing model and must match the chosen camera.

## ToF imager

The VL53L5CX is simulated in the ROS simulation only, with the same plugin as the LiDAR: `build_mujoco_models --lidar plugin` (what the launch uses) adds a second `mujoco.plugin.lidar` instance, `tof`, on a site in `tof_frame`. It casts 8×8 rays, one per zone centre, over the sensor's 45° square field of view, to 4 m at 15 Hz (`tof:` in `config/mujoco.yaml`). The standalone tools (`--lidar rangefinder`) have no ToF, because 64 more rangefinders would be cast on every physics step.

- **Axes.** The plugin publishes `/tof/pointcloud_raw` in the site's axes (x along the rays' centre, no transform), so `tof_pointcloud` maps them onto `tof_frame` (+z along the boresight) and republishes `/tof/pointcloud`, as the real driver does. The frame's yaw comes from the URDF (`tof_frame_offset_rz`), so the model and the real sensor agree.
- **Ray origin.** The plugin excludes no body, and the sensor sits inside the chassis collision cylinder's volume, which every ray would hit from inside. The ray origin is `site_height` (11 mm) above `tof_frame`, over the cylinder's top, and the node adds that offset back to the points.
- **No-hits** are NaN. The real driver writes zeros for them.
- **Not simulated:** target status, multiple targets per zone, crosstalk, reflectivity and the 4×4 mode.

## Requirements

- Python packages installed into your active Python environment (for ROS simulation, use the interpreter that ROS and colcon use): `pip install -r requirements.txt` (`mujoco`, `numpy`, `xacro`, `PyYAML`). Add `pytest<8` to run the tests.
- For the ROS simulation only, on Kilted:
  - `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-mujoco-3d-lidar ros-kilted-laser-filters` (0.1.2 or newer; older releases have no camera or native lidar plugin)
  - [mujoco_ros2_plugins](../modules/mujoco_ros2_plugins/README.md), from the `modules/` submodule, built in the same workspace. It provides the simulated emergency-stop service (default `/emergency_stop`; configurable with `mujoco_plugins.emergency_stop_plugin.service_name`).

## Running

### Standalone

Run these from this directory. `mujoco_preview` opens a GUI window, so on macOS run it with
`mjpython`; everything else (including `mujoco_preview` on Linux) runs with plain `python3`.

Activate the environment containing MuJoCo first; `mjpython` is installed with
MuJoCo. If your shell reports `command not found: mjpython`, check that this
environment is active. In a Pixi workspace, run `pixi shell` from the directory
containing `pixi.toml`, then return to `lekiwi_mujoco` before running these commands.
Standalone use does not require a ROS launch or sourcing a ROS workspace.

```sh
# macOS
mjpython -m lekiwi_mujoco.mujoco_preview      # native viewer

# Linux
python3 -m lekiwi_mujoco.mujoco_preview       # native viewer

# Any platform
python3 -m lekiwi_mujoco.build_mujoco_models --output /tmp/robot.xml --absolute
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

In the viewer, use arrow keys to translate, Shift+Left/Right to rotate,
Space to toggle emergency stop, X to reset and P to pause. The on-screen panel
lists the controls. The same tools are also installed as commands (`ros2 run lekiwi_mujoco <tool>` or
`pip install -e .`) - `build_mujoco_models` and `benchmark_mujoco` work fine that way on any
platform. The installed `mujoco_preview` command only works on Linux; on macOS, always launch it
as `mjpython -m lekiwi_mujoco.mujoco_preview` instead.

The viewer draws native LiDAR rays only when they hit geometry. In open space
(such as outside the maze entrance), missing yellow rays represent no-hit readings,
not disabled sensors.

From Python:

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("/tmp/lekiwi.xml", absolute=True))))
sim.reset()
sim.step(10, action=[1.0, 0.0, 0.0])  # normalized forward, lateral and yaw commands
```

### Scenes

`--scene <name>` (standalone) or `mujoco_scene:=<name>` (ROS) selects the environment. Layouts, objects and fixed cameras are described in the [scenes README](mjcf/scenes/README.md).

| Scene | What it is | Preview |
|-------|------------|---------|
| `flat` | Standard floor (default) | `mjpython -m lekiwi_mujoco.mujoco_preview` |
| `arena` | 4 × 4 m test course with gates, a slalom, a docking pocket and pushable props | `... --scene arena` |
| `home` | 8 × 8 m furnished apartment with pushable props | `... --scene home` |
| `maze` | 7 × 7-cell maze with dead ends and clearings | `... --scene maze` |
| `none` | No scene, or pass a scene MJCF path | |

### ROS simulation

```sh
colcon build --packages-up-to lekiwi_bringup && . install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home   # full stack, headless
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco \
    mujoco_headless:=true use_sim_time:=true mujoco_scene:=home            # control only
```

For the lightest camera setup, keep the default `pointcloud:=false`, lower `camera_publish_rate` in the camera plugin YAML if needed, or use `enable_camera:=false` to remove all simulated camera processing while retaining camera geometry.

`sim:=true` needs no display; `mujoco_gui:=true` opens the MuJoCo viewer. To watch a headless run from another machine, start `foxglove_bridge` and connect Foxglove to `ws://<host>:8765`. The battery monitor and audio are not simulated. The sensors and services come from plugins configured in `config/`:

| Topic or service | What it is |
|------------------|------------|
| `/scan` | Enabled LD06: native LiDAR publishes `/scan_raw`; the scan filter converts no-hit values to infinity |
| `/tof/pointcloud` | The VL53L5CX: a second native lidar instance on `tof_frame` (8×8 rays over 45°, 4 m, 15 Hz) publishes `/tof/pointcloud_raw` in the plugin's axes; `tof_pointcloud` rotates it into `tof_frame` (+z along the sensor's boresight, up, as the real driver; its x and y axes follow the robot, see `tof_frame_offset_rz` in `base.common.xacro`). No-hits are NaN, where the real driver writes zeros |
| `/gemini2/color/image_raw`, `/gemini2/depth/image_raw`, `/gemini2/color/camera_info` | Gemini camera images and shared intrinsics at 5 Hz |
| `/gemini2/depth_registered/points` | Optional colored cloud (`pointcloud:=true`) |
| `/gemini2/gyro_accel/sample` | Camera-mounted six-axis IMU |
| `/free_joint_state_publisher/free_joint_states` | Ground-truth base pose and velocity |
| `/emergency_stop` (`std_srvs/SetBool`, configurable service name) | Disables torque on every motor while enabled, matching the real robot's `sts_hardware_interface`; joints coast/drift freely, not held or braked; releasing hands control back |
| `/external_wrench_plugin/apply_wrench` | Push a body for a test, for example to trigger Nav2 recoveries |
| `/mujoco_ros2_control_node/{reset_world,set_free_joint_state,set_pause,step_simulation}` | Reset, teleport the base, pause and single-step the simulation |
| `/joint_states`, `/imu_sensor_broadcaster/imu`, `/base_controller/odom` | From ros2_control on the simulated hardware |

Notes for control-only runs: `control.launch.py` publishes `odom -> base_footprint` from wheel odometry (`enable_odom_tf:=true`), while `lekiwi.launch.py` turns it off (the EKF publishes it). Without Nav2 nothing consumes `/cmd_vel_presafety`, so publish to `/base_controller/cmd_vel`. The controller enforces no speed limits; teleop scales and Nav2 do. The native LiDAR plugin only loads inside ROS's MuJoCo, so the standalone tools use per-ray rangefinders and have no simulated ToF.

## Configuration

Geometry is expanded from `lekiwi_description/urdf/base.urdf.xacro`; the base and each sensor assembly come from their own `*.module.xacro` files (`ld06`, `bno055`, `vl53l5cx`, `respeaker`, `gemini2`), so the model follows the `hardware.yaml` switches. Rebuild `mjcf/lekiwi_base.xml` after changing URDF, physics or robot MJCF sources. Scene-only changes are loaded when a viewer or launch rebuilds its selected scene.

| File | What it sets |
|------|--------------|
| `config/mujoco.yaml` | Timestep and solver, contact and roller parameters, wheel actuator gains, wheel servo armature and friction (BAM-identified STS3215), the LiDAR, the ToF imager, and the standalone base speed |
| `config/mujoco_ros2_control_plugins.yaml` | ROS plugins: LiDAR and ToF topics, ground-truth pose, emergency stop and external wrench |
| `config/mujoco_camera_plugin.yaml`, `mujoco_camera_gemini2.yaml` | The camera plugin and the Gemini 2 render camera's topics, for the base |
| `mjcf/` | Robot MJCF sources and generated models. Robot geometry, masses and limits are derived from the URDF at build time |
| `mjcf/scenes/` | `flat.xml` (standard floor), `arena.xml` (4 × 4 m test course), `home.xml` (8 × 8 m apartment), and `maze.xml` (7 × 7 cells). Scene geometry is authored in these files |

## Limitations

Contact, friction, servo dynamics and inertias are uncalibrated approximations, with no torque-speed curves, backlash or sensor noise. The model supports one unprefixed robot only.

## Tests

```sh
# From this package directory
python3 -m pytest test -q

# The simulated ToF imager: plugin instance, plugin parameters and the cloud rotation
python3 -m pytest test/test_tof.py -q

# Scene checks: pushing/reset behavior and maze connectivity/clearance
python3 -m pytest test/test_arena.py test/test_home.py test/test_maze.py -q
```

If unrelated ROS pytest plugins fail during collection, disable automatic plugin
loading for these standalone tests:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```
