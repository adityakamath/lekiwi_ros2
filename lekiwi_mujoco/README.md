# LeKiwi MuJoCo

One robot, three variants (`base`, `pt100`, `pt101`). The URDF owns geometry and
inertias; controller configuration owns command limits; MuJoCo adds contact and
actuator approximations. No ROS environment is needed for standalone use.

## Quick start

From the repository root, with the payload submodule initialized (`git submodule update --init payloads/pantilt_ros2`):

```sh
python -m pip install -e ./lekiwi_mujoco            # add '[gym]' for Gymnasium
cd lekiwi_mujoco && python3 -m lekiwi_mujoco.mujoco_preview --variant pt101   # mjpython on macOS
```

Every tool runs three ways: `python3 -m lekiwi_mujoco.<module>` from this directory (no
install, no ROS), the console script after `pip install -e .`, or
`ros2 run lekiwi_mujoco <tool>` after a colcon build.

| Tool | Purpose |
|---|---|
| `build_mujoco_models` | Generate runnable MJCFs (`--variant`, `--output`, `--absolute`, `--scene flat/none/<path>`). With no arguments it regenerates the three committed snapshots. |
| `mujoco_preview` | Native passive viewer with keyboard control (below). |
| `benchmark_mujoco` | Open-loop motion benchmark over all variants (`--output motion.json`). |

Headless work uses `Simulation`; Gymnasium uses `LeKiwiEnv` (`pip install -e './lekiwi_mujoco[gym]'`,
no reward or task defined). Install `pytest<8` to run the tests: `pytest lekiwi_mujoco/test -q`.

## Modules

| Module | Role |
|---|---|
| `build_mujoco_models.py`, `mujoco_parameters.py` | MjSpec composition; URDF-derived parameter sync (masses, inertias, limits, roller contacts, payload frames). |
| `simulation.py` | `Simulation`/`RobotControl`/`RobotBindings`: commands, saturation, pan/tilt slew, stepping, reset, state. |
| `mujoco_preview.py` | Viewer, keyboard input, travel trail. |
| `mujoco_env.py`, `observations.py` | Optional Gymnasium adapter and its [observation contract](docs/observations.md). |
| `benchmark_mujoco.py`, `paths.py` | Benchmark; asset discovery. |

Only a single unprefixed robot per model is supported (multi-instance/prefix support was
removed and can return if needed). No fleet framework, terrain generator, recorder or
training task is included.

## Viewer

Arrow keys translate, Shift+Left/Right rotates, Alt/Option+Left/Right pans and
Alt/Option+Up/Down tilts (Alt takes priority over Shift and suppresses base motion).
Hold to move, release to stop; pan/tilt hold their last angle. X resets, P pauses (both
case-insensitive, both clear held keys). Opposing keys cancel; losing focus stops motion.
Click the viewport first: the first key press attaches GLFW press/release callbacks on
the viewer thread. A thin cyan travel trail (1 cm samples, latest 1,000 points, visual
only, 3 mm above the flat floor) is cleared by X.

Options: `--variant`, `--scene`, `--model <xml>` (load a prebuilt XML with its embedded
limits instead of regenerating), `--island-colors` (debug), `--telemetry state.json`.
The viewer regenerates its model on every launch; restart after configuration edits.
On Windows use `py -m venv .venv` then `.venv\Scripts\python -m pip install -r requirements.txt`.

## With ROS 2

Install `requirements.txt` into the interpreter used by colcon and ROS launch (MuJoCo
and xacro are Python dependencies, not ROS runtime imports), then:

```sh
colcon build --packages-up-to lekiwi_bringup
. install/setup.sh
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true payload:=pantilt pantilt_config:=pt101
```

`control.launch.py` generates the model with `build_mujoco_models` (`mujoco_scene` argument:
`flat`, `none` or a scene path) and hands it to `mujoco_ros2_control`, which hosts the
physics, controllers and IMU/lidar. `sim:=true` is headless by default; `mujoco_gui:=true`
attaches the MuJoCo Simulate viewer and needs a display. Headless `sim:=true` was verified
on the Raspberry Pi (Kilted, 2026-09-18): base, joint-state, IMU and pan-tilt controllers
all activate. The simulated OAK-D camera does not publish (the `CameraPlugin` is not
shipped in `mujoco_ros2_control_plugins` 0.0.3); its GLFW init failure only logs a warning.
Use the existing waypoint services and Nav2 to test navigation; no alternative follower or
ROS viewer is provided. Plan for viewing a Pi-hosted simulation on a laptop:
[remote-viewer plan](docs/remote-viewer-plan.md).

## Data discovery and explicit paths

Assets are found in the source checkout, ROS prefixes, or the Python `share` directory.
Override with `LEKIWI_MUJOCO_SHARE`, `LEKIWI_DESCRIPTION_SHARE`, `PT_DESCRIPTION_SHARE`,
`LEKIWI_CONTROL_SHARE`, or pass `--control-package`, `--description-package`,
`--pt-package` (source/share directories, not modules); the build functions take
`control_dir`, `description_dir`, `pt_package`. Standalone URDF expansion requires
`base_controller_config:=/path/to/lekiwi_control/config/base/control.yaml` (supplied by
the launch layer and the builder). Committed XML files are source-checkout snapshots
with relative mesh paths; regenerate them after URDF/config/MJCF edits (previewing and
Gymnasium regenerate temporary models automatically). Use the generator rather than
plain xacro: it applies payload and camera-frame corrections that xacro alone skips.

## Scene and parameter ownership

Do not edit masses, inertias or limits in generated XML. `build_robot_spec(variant)` builds
the robot only, `compose_scene(robot, scene)` attaches it to `mjcf/scenes/flat.xml` (floor,
lights, materials; the only scene shipped), `build_spec` does both. A robot-only model has
no ground. `config/mujoco.yaml` owns timestep, integrator and solver iterations, overriding
any external scene.

| Quantity | Source |
|---|---|
| Wheel radius, mount radius, angular offset | `lekiwi_control/config/base/control.yaml`, read by URDF (`base_controller_config`); scales wheel visuals and roller contacts. Mismatched mount positions/axes fail generation. |
| Other frames, mesh transforms and scales | Expanded LeKiwi/payload URDF, copied into MjSpec bodies/geoms. |
| Mass, COM, inertia tensor | URDF `<inertial>` on every represented link; no CAD-density fallback; nonphysical tensors fail. Fixed frames without inertia stay massless. Totals: 2.56 kg base, 3.779226 kg with either payload. |
| Wheel hub and 16 virtual rollers | URDF wheel inertia partitioned via `roller_mass_fraction` (parallel-axis subtraction); aggregate mass/COM/tensor match the URDF. |
| Effort ceiling | URDF joint `limit effort` sets actuator force range (not a torque-speed curve). |
| Velocity/position limits | Controller config and URDF hardware/joint limits (see below). |
| Contacts, friction, bearings, servo armature/friction/gains | `config/mujoco.yaml`: explicit uncalibrated assumptions; chassis collision is a cylinder. |
| Floor friction, appearance | Selected scene XML. |

**STS3215 servo defaults** (`mjcf/sts3215.mjcf.xacro`, copied from `so_arm_mujoco`) hold
parameters identified for a real 12 V STS3215 via Rhoban's BAM project. Wheel joints use
`class="sts3215"`; `sync_robot_parameters()` then overwrites armature/frictionloss for every
actuated joint from `config/mujoco.yaml`'s `actuators` block, which is where the BAM values
take effect. Wheel damping is overridden to 0: BAM identified it for a small-range
position-controlled joint, and on a continuously spinning velocity-controlled wheel it
measurably fights the actuator. Pan/tilt still use the uncalibrated values in `config/mujoco.yaml`
(pantilt_ros2 keeps its own copy).

**Command limits.** Base x/y/yaw limits and wheel kinematics come from `control.yaml`;
wheel-speed ceilings from the expanded URDF and `urdf_config.yaml`; pan/tilt uses the smaller
of the URDF joint velocity and hardware `max_velocity` (ignoring the real-mode 1e6 sentinel;
default 4.433 rad/s, ±1.5708 rad). All are generated into the XML metadata and wheel
`ctrlrange`, not hard-coded in the keyboard handler. Held base keys request axis maxima,
combined motion is scaled uniformly to satisfy every wheel limit, and pan/tilt targets are
slew-limited. These constrain commands, not physical state; ROS keeps its own controller
enforcement.

## Model notes and limitations

Each wheel uses a ring of 16 passive axisymmetric contact rollers (tangential axes, real
contact bodies) so lateral slip works; wheel radius 0.051 m, wheelbase radius 0.132239 m.
The roller count, friction, bearing loss, servo gains, chassis inertia and some payload
inertias are uncalibrated approximations; there are no measured torque-speed curves,
backlash, tyre deformation, battery sag or sensor noise/latency. A 2 ms step with
implicit-fast integration is used. Do not retune coefficients solely to force perfect
command tracking. The OAK camera uses MuJoCo's -Z view / +Y up convention mapped to the
mechanical +X forward / +Z up frame; payload transforms come from the URDF (this corrects
the pinned MJCF tilt offset from 0.1025 m to 0.0541441 m). Lens center, intrinsics and
exact extrinsics still need calibration.

## Simulation API

`Simulation` owns physics, command saturation, pan/tilt slew, reset and state; rendering and
pacing stay outside it.

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

path = build("pt101", "/tmp/lekiwi.xml", absolute=True)
sim = Simulation(mujoco.MjModel.from_xml_path(str(path)))
sim.reset()  # 0.5 s settling, then the clock restarts at zero
sim.step(10, action=[1., 0., 0., 0., 0.])
sim.stop()
```

Actions are normalized rates: forward, leftward, counterclockwise, then pan and tilt for
payload variants. `command([vx, vy, yaw_rate])` takes m/s, m/s, rad/s. `step(count)` advances
whole physics ticks; `steps_for(seconds)` needs an exact multiple of the timestep
(Gymnasium: 0.02 s per action). `reset(settle_seconds=0)` restores the compiled initial
state and clears command history and slider goals; `stop()` zeros wheels and holds pan/tilt
targets without teleporting anything. Gymnasium defaults to ideal named sensors
([contract v1](docs/observations.md)); `observation_mode="privileged"` returns raw
`qpos`/`qvel`/`sensors`. `render()` is an overview, not a camera observation.

## Testing

The suite covers loading/settling all variants, actuator/sensor interfaces and wheel mass,
motion in several directions, pan/tilt setpoints, camera direction, URDF body-frame and
mesh comparisons at three poses per payload, speed saturation, parameter propagation and
rejection, generated-file freshness, the trail, and a subprocess check that the tools run
with ROS imports blocked (`test_without_ros.py`; needs `mujoco` and `xacro` pip-installed
in the interpreter, since it runs in isolated mode). CI runs it in the ROS container and
in a separate Linux/macOS/Windows job. The native viewer on Windows is not interactively
verified.

## Open items

- **Remote viewing:** Pi-hosted physics with a laptop viewer over ROS 2 Zenoh
  ([plan](docs/remote-viewer-plan.md)); then waypoint markers and the trail in that viewer,
  consuming existing Nav2 state.
- **ROS/Python parity:** the Python core and the ROS path differ and the ROS side is
  unverified against the plugin: base acceleration (no ramp here; YAML declares limits),
  pan/tilt (rate-integrated targets here; absolute forward position commands in ROS; do not
  reinterpret one as the other), command timeout (none here; 0.5 s in ROS YAML) and
  reset/clock behavior. Run identical timestamped command sequences through both and compare.
- **Motor calibration:** measure wheel and pan/tilt response, braking and load dependence;
  relate firmware settings (e.g. `pantilt_internal_max_vel: 65`) to MuJoCo parameters.
- **Contact calibration:** measure straight, strafe and turn trajectories, slip and stopping
  distance on a known surface; separate contact from motor error before retuning.
- **Sensors and camera:** verify IMU frames/heading, lidar ray conventions and ROS message
  correspondence; ROS camera simulation is disabled. Ideal signals are not calibrated
  measurements.
- **Inertia audit:** check camera mass is not double-counted and link masses/COMs against
  physical data, correcting in the URDF only.
- **Windows:** native viewer and no-ROS installation.
