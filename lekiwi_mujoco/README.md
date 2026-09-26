# LeKiwi MuJoCo

MuJoCo models of LeKiwi (`base`, `pt100`, `pt101`), generated from the URDF and the controller configuration. The package gives you three ways to use them:

- **Standalone:** a native viewer, a model builder and a benchmark, with no ROS needed.
- **ROS simulation:** `sim:=true` runs the same launch files as the real robot against the model in `mujoco_ros2_control`.
- **Payloads:** a payload's own MuJoCo package builds its model, and this package mounts it on the base.

## Contents

| Path | Purpose |
|------|---------|
| `lekiwi_mujoco/build_mujoco_models.py` | Builds the model from the URDF and controller config, and attaches the payload |
| `lekiwi_mujoco/mujoco_parameters.py` | Applies the physics and servo parameters from `config/mujoco.yaml` to the model |
| `lekiwi_mujoco/simulation.py` | The simulation without ROS: reset, command, step |
| `lekiwi_mujoco/mujoco_preview.py`, `benchmark_mujoco.py` | Native viewer and motion benchmark |
| `config/` | Physics and servo parameters, ROS plugin configuration, camera rate, laser filters |
| `mjcf/` | MJCF sources, the `scenes/` and the pre-built `lekiwi_*.xml` models |

## Requirements

- Python packages installed into your active Python environment (for ROS simulation, use the interpreter that ROS and colcon use): `pip install -r requirements.txt` (`mujoco`, `numpy`, `xacro`, `PyYAML`). Add `pytest<8` to run the tests.
- The payload submodule for the pan-tilt models: run `git submodule update --init payloads/pantilt_ros2` from the `lekiwi_ros2` repository root. It provides `pt_description` and `pt_mujoco`.
- For the ROS simulation only, on Kilted:
  - `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-mujoco-3d-lidar ros-kilted-laser-filters` (0.1.2 or newer; older releases have no camera or native lidar plugin)
  - [mujoco_ros2_plugins](../modules/mujoco_ros2_plugins/README.md), from the `modules/` submodule, built in the same workspace. It provides the simulated `/emergency_stop`.

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
mjpython -m lekiwi_mujoco.mujoco_preview --variant pt101      # native viewer

# Linux
python3 -m lekiwi_mujoco.mujoco_preview --variant pt101       # native viewer

# Any platform
python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101 --output /tmp/robot.xml --absolute
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

In the viewer, click the window first, then use the arrow keys to translate, Shift+Left/Right to
rotate, Alt/Option+arrows to pan and tilt (pt100/pt101 only), Space to toggle the emergency stop
(disables torque on every motor; wheels and payload drift/coast freely rather than locking or
holding), X to reset and P to pause. A cyan trail shows the path
travelled; X clears it. The on-screen panel lists the exact controls available for the loaded
variant. The same tools are also installed as commands (`ros2 run lekiwi_mujoco <tool>` or
`pip install -e .`) - `build_mujoco_models` and `benchmark_mujoco` work fine that way on any
platform. The installed `mujoco_preview` command only works on Linux; on macOS, always launch it
as `mjpython -m lekiwi_mujoco.mujoco_preview` instead.

Always build models with `build_mujoco_models` rather than plain xacro: it takes the payload frames, inertias and limits from the URDF. `--scene` selects the environment (`flat`, `arena`, `home`, `maze`, `none` or a scene file) and `--lidar` the LiDAR model (`rangefinder` or `plugin`). With `pt100`/`pt101`, the rangefinders in the payload's blind arc (`config/mujoco_laser_filter_pantilt.yaml`'s mask) are removed from the model entirely, matching what the real robot's `laser_filters` chain discards - so `mujoco_preview`'s rangefinder rays only ever show the unblocked ~295°, with no separate runtime filtering needed. With no arguments it regenerates the committed `mjcf/lekiwi_*.xml` files, which you should do after changes to robot URDF, physics config or robot MJCF sources. These committed models use `flat`; scene-only edits are loaded when the viewer rebuilds the selected scene at launch. If using `--model`, rebuild that model explicitly.

The viewer draws native LiDAR rays only when they hit geometry. In open space
(such as outside the maze entrance), missing yellow rays represent no-hit readings,
not disabled sensors. The same payload blind-sector mask applies in every scene.

From Python:

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/lekiwi.xml", absolute=True))))
sim.reset()
sim.step(10, action=[1., 0., 0., 0., 0.])   # normalized [vx, vy, yaw, pan, tilt] rates
```

### Test arena

`arena` is a 4 × 4 m robot test course with a clear 1.3 × 1.3 m central maneuvering
area. Along the perimeter are three north gates (0.6, 0.8 and 1.0 m clear width,
marked with one, two and three blue floor ticks), an east slalom, a southwest
docking pocket with a flared entrance and a 1 m inner width, and a southern pushing area with a 180 g box
and 60 g ball. The docking pocket opens east. All other obstacles are fixed.
The dock has a centerline, a robot-center stop line, and a contrasting back-wall
target. Push the box east into the orange floor target, then reposition south of
it and push north into the blue target. The lane behind the gates is 0.70 m wide;
the outer slalom lane has at least 0.60 m clearance.
Reset restores both movable objects. Prop masses and friction are illustrative.

Warm walls, colored landmarks, and a bounded sand-colored floor distinguish the
course; the standard blue checkerboard continues outside. Floor markings are
visual only. Select `arena_top` or `arena_overview` for a view of the entire course.

```sh
mjpython -m lekiwi_mujoco.mujoco_preview --variant pt101 --scene arena
```

### Home scene

Use `--scene home` with the standalone viewer or `mujoco_scene:=home` with ROS.
The 8 × 8 m apartment has a living room, kitchen and dining area, an enlarged bedroom, and
a 2.8 × 2.8 m room in the northwest corner with glass partitions and a 1 m doorway.
The glass partitions are translucent, solid collision geometry. Low cutaway walls expose the layout from above; doorways, table legs,
and furniture provide navigation obstacles. The robot starts in the clear central
hall. `home_overview` and `home_top` are fixed cameras available in the viewer.

Three objects have free joints and can be pushed: a 180 g cardboard box east of
spawn, an 80 g foam block northwest of spawn, and a 60 g toy ball near the dining
area. They use low friction; furniture stays fixed. Reset restores the objects as
well as the robot. The apartment flooring stops at the outer walls, with the standard blue MuJoCo
checkerboard outside. Rugs and floor finishes are visual only, so they introduce no
steps or changes in wheel traction. Prop masses and friction are illustrative,
not calibrated to physical household objects.

```sh
# macOS standalone apartment
mjpython -m lekiwi_mujoco.mujoco_preview --variant pt101 --scene home
```

### Maze scene

`--scene maze` opens a 7 × 7-cell maze with 1.12 m clear corridors, multiple
dead ends, and two 2 × 2-cell clearings (2.32 m clear width). The robot starts
outside the blue west entrance at `(0, 0)`. Follow the east-pointing entry
chevrons; the orange exit is on the east boundary at `(9.2, 7.2)`.
The maze uses warm off-white walls and the home scene’s wood-colored floor finish.
This finish extends 0.5 m beyond the outer walls; the standard blue
checkerboard floor continues elsewhere. Clearings use the same floor as corridors.
All cells are reachable, with a continuous route through the maze. No route is
painted on the floor. `maze_top` and `maze_overview` provide fixed camera views.

```sh
mjpython -m lekiwi_mujoco.mujoco_preview --variant pt101 --scene maze
```

### ROS simulation

```sh
colcon build --packages-up-to lekiwi_bringup && . install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=home   # full stack, headless
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco payload:=pantilt \
    mujoco_headless:=true use_sim_time:=true mujoco_scene:=home            # control only
```

`sim:=true` needs no display; `mujoco_gui:=true` opens the MuJoCo viewer. To watch a headless run from another machine, start `foxglove_bridge` and connect Foxglove to `ws://<host>:8765`. The battery monitor and audio are not simulated. The sensors and services come from plugins configured in `config/`:

| Topic or service | What it is |
|------------------|------------|
| `/scan` | Native LiDAR plugin (360 rays, 5 Hz) publishes `/scan_raw`; `laser_filters` turns no-hit into `inf` and masks the pan-tilt |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` | Simulated camera from `pt_mujoco` (rate lowered to 5 Hz here), in frame `oak_rgb_camera_optical_frame`; the image is upside down like the real, inverted mount |
| `/oak/scan` | With the pan-tilt: a laser scan sliced from the depth image, as on the real robot |
| `/free_joint_state_publisher/free_joint_states` | Ground-truth base pose and velocity |
| `/emergency_stop` (`std_srvs/SetBool`) | Disables torque on every motor while enabled, matching the real robot's `sts_hardware_interface`; joints coast/drift freely, not held or braked; releasing hands control back |
| `/external_wrench_plugin/apply_wrench` | Push a body for a test, for example to trigger Nav2 recoveries |
| `/mujoco_ros2_control_node/{reset_world,set_free_joint_state,set_pause,step_simulation}` | Reset, teleport the base, pause and single-step the simulation |
| `/joint_states`, `/imu_sensor_broadcaster/imu`, `/base_controller/odom` | From ros2_control on the simulated hardware |

Notes for control-only runs: `control.launch.py` publishes `odom -> base_footprint` from wheel odometry (`enable_odom_tf:=true`), while `lekiwi.launch.py` turns that off because the EKF publishes it. Without Nav2 nothing consumes `/cmd_vel_presafety`, so publish to `/base_controller/cmd_vel`. The controller enforces no speed limits; teleop scales and Nav2 do. The native LiDAR plugin only loads inside ROS's MuJoCo, so the standalone tools use per-ray rangefinders.

## Configuration

| File | What it sets |
|------|--------------|
| `config/mujoco.yaml` | Timestep and solver, contact and roller parameters, wheel actuator gains, wheel servo armature and friction (BAM-identified STS3215), the LiDAR, and the standalone base speed |
| `config/mujoco_ros2_control_plugins.yaml` | ROS plugins: LiDAR, ground-truth pose, emergency stop and external wrench |
| `config/mujoco_camera_pantilt.yaml` | Lowers the pan-tilt camera rate for the Raspberry Pi |
| `config/mujoco_laser_filter*.yaml` | Laser filter chain, with and without the pan-tilt mask |
| `mjcf/` | Robot MJCF sources and generated models. Robot geometry, masses and limits are derived from the URDF at build time |
| `mjcf/scenes/` | `flat.xml` (standard floor), `arena.xml` (4 × 4 m test course), `home.xml` (8 × 8 m apartment), and `maze.xml` (7 × 7 cells). Scene geometry is authored in these files |

Changes to `config/mujoco.yaml` are applied when the model is built, so rebuild or relaunch to see them. If the package cannot find its files, point it at them with `LEKIWI_MUJOCO_SHARE`, `LEKIWI_DESCRIPTION_SHARE`, `PT_DESCRIPTION_SHARE`, `PT_MUJOCO_SHARE` or `LEKIWI_CONTROL_SHARE`, or with the builder's `--control-package`, `--description-package` and `--pt-package`.

## Limitations

Contact, friction, servo dynamics and inertias are uncalibrated approximations, with no torque-speed curves, backlash or sensor noise. The model supports one unprefixed robot only.

## Using a payload

The payload's own package builds its model from the robot's URDF (for the pan-tilt, `pt_mujoco`'s `build_payload_spec`), and this package attaches it at the URDF's mount joint (`pantilt_mount_joint`). The payload's frames, inertias and servo parameters stay in its package; only the mount comes from here. See the [`pt_mujoco` README](../payloads/pantilt_ros2/pt_mujoco/README.md) for the payload side.

## Tests

```sh
# From this package directory
python3 -m pytest test -q

# Scene checks: pushing/reset behavior and maze connectivity/clearance
python3 -m pytest test/test_arena.py test/test_home.py test/test_maze.py -q
```

If unrelated ROS pytest plugins fail during collection, disable automatic plugin
loading for these standalone tests:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```
