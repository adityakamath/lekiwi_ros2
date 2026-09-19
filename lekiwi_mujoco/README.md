# LeKiwi MuJoCo

MuJoCo models of LeKiwi (`base`, `pt100`, `pt101`) generated from the URDF, plus a standalone
(no ROS) viewer and benchmark. The ROS simulation (`sim:=true`) runs these models in
`mujoco_ros2_control`.

## Requirements

- Python: `pip install -r requirements.txt` (`mujoco==3.13.0`, `numpy`, `xacro`, `PyYAML`) into the
  interpreter used by colcon/ROS launch. Add `pytest<8` for the tests.
- Payload submodule: `git submodule update --init payloads/pantilt_ros2` (provides `pt_description` and `pt_mujoco`, which builds the pan-tilt model from the robot's URDF; it is found in the checkout when not installed).
- ROS sim only (Kilted): `sudo apt install ros-kilted-mujoco-ros2-control
  ros-kilted-mujoco-ros2-control-plugins ros-kilted-mujoco-3d-lidar ros-kilted-laser-filters`
  (0.1.2 or newer; older releases have no camera or native lidar plugin). The simulation's
  `/emergency_stop` comes from `modules/mujoco_ros2_plugins`, built in the same workspace.

## Standalone use

From this directory (no install, no ROS; `mjpython` on macOS for the viewer):

```sh
python3 -m lekiwi_mujoco.mujoco_preview --variant pt101      # native viewer
python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101 --output /tmp/robot.xml --absolute
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

The same tools are installed as `mujoco_preview`, `build_mujoco_models` and `benchmark_mujoco`
(`pip install -e .` or `ros2 run lekiwi_mujoco <tool>`). `build_mujoco_models` takes
`--scene flat|arena|none|<path>` and `--lidar rangefinder|plugin`; with no arguments it
regenerates the committed `mjcf/lekiwi_*.xml` snapshots (do this after any URDF, config or MJCF
edit). Use it instead of plain xacro: it syncs payload frames, inertias and limits from the URDF.

Viewer keys: arrows translate, Shift+Left/Right rotates, Alt/Option+arrows pan/tilt, X reset,
P pause; click the viewport first. A cyan trail shows the path travelled (cleared by X).

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/lekiwi.xml", absolute=True))))
sim.reset()
sim.step(10, action=[1., 0., 0., 0., 0.])   # normalized [vx, vy, yaw, pan, tilt] rates
```

## ROS simulation

```sh
colcon build --packages-up-to lekiwi_bringup && . install/setup.bash
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true mujoco_scene:=arena   # full stack, headless
ros2 launch lekiwi_control control.launch.py ros2_control_hardware_type:=mujoco payload:=pantilt \
    mujoco_headless:=true use_sim_time:=true mujoco_scene:=arena  # control only, in the arena scene
```

`sim:=true` needs no display (`mujoco_gui:=true` opens the MuJoCo viewer). The battery monitor and
audio are not simulated. Sensors come from plugins configured in `config/`:

| Topic | Source |
|---|---|
| `/scan` | Native `mujoco.plugin.lidar` (360 rays, 5 Hz) publishes `/scan_raw`; `laser_filters` (`config/mujoco_laser_filter*.yaml`) turns no-hit into `inf` and masks the pan-tilt into `/scan` |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info` | `CameraPlugin` (configured by `pt_mujoco`, rate lowered to 5 Hz here), headless EGL; frame `oak_rgb_camera_optical_frame`; image is upside down like the real, inverted OAK-D mount |
| `/oak/scan` | With the pan-tilt: `depthimage_to_laserscan` slices the simulated depth image, as the real bringup does (`pt_mujoco/config/mujoco_depth_to_scan.yaml`) |
| `/free_joint_state_publisher/free_joint_states` | Ground-truth base pose and velocity |
| `/emergency_stop` (`std_srvs/SetBool`) | [`mujoco_ros2_plugins`](../modules/mujoco_ros2_plugins/README.md): the service `sts_hardware_interface` provides on the real robot. While enabled it commands the wheels to zero and holds the pan-tilt where it was; releasing it hands the commands back |
| `/external_wrench_plugin/apply_wrench` | `ExternalWrenchPlugin`: pushes a body for a test, e.g. to trigger Nav2 recoveries |
| `/mujoco_ros2_control_node/{reset_world,set_free_joint_state,set_pause,step_simulation}` | Core services: reset, teleport the base, pause and single-step the simulation |
| `/joint_states`, `/imu_sensor_broadcaster/imu`, `/base_controller/odom` | ros2_control on the simulated hardware |

Notes for control-only runs: `control.launch.py` publishes `odom -> base_footprint` from wheel
odometry (`enable_odom_tf:=true`); `lekiwi.launch.py` disables it because the EKF publishes it.
Without Nav2 nothing consumes `/cmd_vel_presafety`, so publish to `/base_controller/cmd_vel`.
The controller enforces no speed limits (teleop scales and Nav2 do). The native lidar plugin only
loads in ROS's MuJoCo, so standalone tools use per-ray rangefinders.

## Configuration

| File | Controls |
|---|---|
| `config/mujoco.yaml` | Timestep/solver, contact and roller parameters, wheel actuator gains, wheel servo armature/friction (BAM-identified STS3215), native lidar, standalone base speed |
| `config/mujoco_ros2_control_plugins.yaml`, `mujoco_camera_pantilt.yaml` | ROS plugins (lidar, ground-truth pose, emergency stop); the camera plugin config comes from `pt_mujoco` and the second file only lowers its rate for the Pi |
| `mjcf/base_shared.xml`, `base_subtree.xml`, `sts3215.mjcf.xacro` | Shared MJCF (16 contact rollers per wheel; wheel joint damping forced to 0); geometry, masses and limits are overwritten from the URDF at build time; the pan-tilt payload is not here, `pt_mujoco` builds it and it is attached at the URDF's `pantilt_mount_joint` |
| `mjcf/scenes/flat.xml`, `arena.xml` | Floor only; 6 x 6 m walled room with obstacles |

Asset lookup uses the source checkout, ROS prefixes or the Python `share` directory; override with
`LEKIWI_MUJOCO_SHARE`, `LEKIWI_DESCRIPTION_SHARE`, `PT_DESCRIPTION_SHARE`, `PT_MUJOCO_SHARE`, `LEKIWI_CONTROL_SHARE`
or the builder's `--control-package`, `--description-package`, `--pt-package`.

## Limitations

Contact, friction, servo dynamics and inertias are uncalibrated approximations (no torque-speed
curves, backlash, or sensor noise). Only one unprefixed robot per model. To watch a Pi-hosted
sim from a laptop, run `foxglove_bridge` on the Pi and connect Foxglove to `ws://<pi>:8765`.

## Tests

`pytest lekiwi_mujoco/test -q` (about 3 minutes on a Raspberry Pi). `test_without_ros.py` runs the
tools with ROS imports blocked and needs `mujoco` and `xacro` pip-installed. CI runs the suite in
the ROS container and on Linux, macOS and Windows.
