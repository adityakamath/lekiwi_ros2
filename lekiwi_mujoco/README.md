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

- Python packages, installed into the interpreter that ROS and colcon use: `pip install -r requirements.txt` (`mujoco`, `numpy`, `xacro`, `PyYAML`). Add `pytest<8` to run the tests.
- The payload submodule for the pan-tilt models: `git submodule update --init payloads/pantilt_ros2`. It provides `pt_description` and `pt_mujoco`.
- For the ROS simulation only, on Kilted:
  - `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-mujoco-3d-lidar ros-kilted-laser-filters` (0.1.2 or newer; older releases have no camera or native lidar plugin)
  - [mujoco_ros2_plugins](../modules/mujoco_ros2_plugins/README.md), from the `modules/` submodule, built in the same workspace. It provides the simulated `/emergency_stop`.

## Running

### Standalone

Run these from this directory (`mjpython` instead of `python3` on macOS for the viewer):

```sh
python3 -m lekiwi_mujoco.mujoco_preview --variant pt101      # native viewer
python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101 --output /tmp/robot.xml --absolute
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

In the viewer, click the window first, then use the arrow keys to translate, Shift+Left/Right to rotate, Alt/Option+arrows to pan and tilt, X to reset and P to pause. A cyan trail shows the path travelled; X clears it. The same tools are installed as commands (`ros2 run lekiwi_mujoco <tool>` or `pip install -e .`).

Always build models with `build_mujoco_models` rather than plain xacro: it takes the payload frames, inertias and limits from the URDF. `--scene` selects the environment (`flat`, `arena`, `none` or a scene file) and `--lidar` the LiDAR model (`rangefinder` or `plugin`). With no arguments it regenerates the committed `mjcf/lekiwi_*.xml` files, which you should do after any change to the URDF, the config or the MJCF.

From Python:

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/lekiwi.xml", absolute=True))))
sim.reset()
sim.step(10, action=[1., 0., 0., 0., 0.])   # normalized [vx, vy, yaw, pan, tilt] rates
```

### ROS simulation

```sh
colcon build --packages-up-to lekiwi_bringup && . install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=arena   # full stack, headless
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco payload:=pantilt \
    mujoco_headless:=true use_sim_time:=true mujoco_scene:=arena            # control only
```

`sim:=true` needs no display; `mujoco_gui:=true` opens the MuJoCo viewer. To watch a headless run from another machine, start `foxglove_bridge` and connect Foxglove to `ws://<host>:8765`. The battery monitor and audio are not simulated. The sensors and services come from plugins configured in `config/`:

| Topic or service | What it is |
|------------------|------------|
| `/scan` | Native LiDAR plugin (360 rays, 5 Hz) publishes `/scan_raw`; `laser_filters` turns no-hit into `inf` and masks the pan-tilt |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` | Simulated camera from `pt_mujoco` (rate lowered to 5 Hz here), in frame `oak_rgb_camera_optical_frame`; the image is upside down like the real, inverted mount |
| `/oak/scan` | With the pan-tilt: a laser scan sliced from the depth image, as on the real robot |
| `/free_joint_state_publisher/free_joint_states` | Ground-truth base pose and velocity |
| `/emergency_stop` (`std_srvs/SetBool`) | Wheels commanded to zero and the pan-tilt held while enabled; releasing hands control back |
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
| `mjcf/` | The MJCF sources (base, 16 contact rollers per wheel, servo defaults) and the scenes: `flat.xml` (floor only) and `arena.xml` (6 x 6 m walled room with obstacles). Geometry, masses and limits in them are overwritten from the URDF at build time |

Changes to `config/mujoco.yaml` are applied when the model is built, so rebuild or relaunch to see them. If the package cannot find its files, point it at them with `LEKIWI_MUJOCO_SHARE`, `LEKIWI_DESCRIPTION_SHARE`, `PT_DESCRIPTION_SHARE`, `PT_MUJOCO_SHARE` or `LEKIWI_CONTROL_SHARE`, or with the builder's `--control-package`, `--description-package` and `--pt-package`.

## Limitations

Contact, friction, servo dynamics and inertias are uncalibrated approximations, with no torque-speed curves, backlash or sensor noise. The model supports one unprefixed robot only.

## Using a payload

The payload's own package builds its model from the robot's URDF (for the pan-tilt, `pt_mujoco`'s `build_payload_spec`), and this package attaches it at the URDF's mount joint (`pantilt_mount_joint`). The payload's frames, inertias and servo parameters stay in its package; only the mount comes from here. See the [`pt_mujoco` README](../payloads/pantilt_ros2/pt_mujoco/README.md) for the payload side.

## Tests

```sh
pytest lekiwi_mujoco/test -q       # about 3 minutes on a Raspberry Pi
```
