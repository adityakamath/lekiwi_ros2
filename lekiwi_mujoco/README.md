# LeKiwi MuJoCo

One robot, three variants (`base`, `pt100`, `pt101`). URDF owns geometry and
inertias; controller configuration owns command limits. MuJoCo adds contact and
actuator approximations. No ROS environment is needed for standalone use.

## Start here

From the repository root, with the payload submodule initialized:

```sh
python -m pip install -e ./lekiwi_mujoco
# macOS; use python instead of mjpython on Linux/Windows:
cd lekiwi_mujoco && mjpython -m lekiwi_mujoco.mujoco_preview --variant pt101
```

Arrow keys translate, Shift+Left/Right rotates, Alt/Option+arrows moves pan/tilt.
Hold to move, release to stop. X resets; P pauses. Models regenerate automatically.

For headless experiments, use `Simulation`. For Gymnasium, install the optional
adapter with `python -m pip install -e './lekiwi_mujoco[gym]'` and use `LeKiwiEnv`.
The adapter defines no reward or training task. Install pytest to run the tests.

## Package responsibilities

- `build_mujoco_models.py` and `mujoco_parameters.py`: MjSpec composition and URDF conversion.
- `simulation.py`: commands, physics stepping and reset.
- `mujoco_preview.py`: native viewer and keyboard input.
- `mujoco_env.py`: optional Gymnasium adapter; `observations.py` defines its readings.
- `paths.py` locates assets.

Scenes remain separate XML assets. Only a single unprefixed robot is supported; no
fleet or multi-instance framework, terrain generator, recorder framework or training
tasks is included. The benchmark uses the same core, compiling each variant once and
resetting between motions. Historical model compatibility is no longer maintained.

## With ROS 2

Install the Python dependencies from `requirements.txt` into the interpreter used
by colcon and ROS launch; MuJoCo and Gymnasium are Python dependencies, not ROS
runtime imports. Build and source the ROS workspace normally:

```sh
colcon build --packages-up-to lekiwi_mujoco
. install/setup.sh
ros2 run lekiwi_mujoco build_mujoco_models --variant pt101 --output /tmp/lekiwi.xml --absolute --control-package /path/to/lekiwi_control
ros2 run lekiwi_mujoco mujoco_preview --variant pt101 --control-package /path/to/lekiwi_control
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true payload:=pantilt
```

For the native viewer on macOS, use `mjpython` with the source wrapper; MuJoCo's
macOS viewer requires that launcher. Normal Python suffices for headless work.
The existing ROS physics bridge remains `mujoco_ros2_control`.

## Data discovery and explicit paths

The package discovers assets in the source checkout, ROS prefixes, or the Python
installation's `share` directory. Outside a checkout, set `LEKIWI_DESCRIPTION_SHARE`
and `PT_DESCRIPTION_SHARE` if those asset packages are not installed in a discoverable
prefix. `LEKIWI_MUJOCO_SHARE` overrides this package's asset directory.

The builder and preview accept `--control-package`, `--description-package` and
`--pt-package`. These are package source/share directories, not Python modules.
Gymnasium/build functions expose equivalent `control_dir`, `description_dir` and
`pt_package` arguments. Source checkouts auto-discover sibling control data;
`LEKIWI_CONTROL_SHARE` can also identify it. ROS control launch supplies the control
and description paths explicitly. No ROS middleware is started by the Python tools.

Standalone URDF expansion now requires
`base_controller_config:=/path/to/lekiwi_control/config/base/control.yaml`.
The integration layer supplies this input instead of making `lekiwi_description`
depend on `lekiwi_control`.

Committed XML models are source-checkout snapshots. For an installed layout or after
URDF/configuration edits, regenerate with an explicit output path. The viewer and
Gymnasium regenerate temporary models automatically. Masses/inertias remain URDF-owned.

See [model usage](mjcf/README.md) and [parameter ownership](mjcf/PARAMETERS.md).

## Shared headless execution

The viewer, Gymnasium and benchmark/test runtime use `Simulation` for physics,
command saturation, pan/tilt target slew, reset and state reporting. Rendering and
wall-clock pacing stay outside the core. This aligns the Python paths; equivalence
with ROS controllers still requires integration testing.

```python
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation

path = build("pt101", "/tmp/lekiwi.xml", absolute=True)
sim = Simulation(mujoco.MjModel.from_xml_path(str(path)))
sim.reset()  # default 0.5 s settling, then simulation clock starts at zero
sim.step(10, action=[1., 0., 0., 0., 0.])
sim.stop()
```

Actions are normalized rates: forward, leftward, counterclockwise rotation,
then pan and tilt for payload variants. Configured velocity limits scale them;
wheel and body constraints use uniform saturation. Pan/tilt targets are bounded
and rate-limited each physics tick. These constrain commands, not measured motor
motion. `command([vx, vy, yaw_rate])` accepts m/s, m/s and rad/s. `step(count)`
advances an integer number of physics ticks; `steps_for(seconds)` requires an
exact multiple of the physics timestep. Gymnasium defaults to 0.02 s per action.

`reset(settle_seconds=0)` restores the exact compiled initial state without settling.
Reset clears command history and pending slider goals. Viewer position-slider
requests persist until reached or replaced, subject to the same slew limits.
`stop()` zeros wheel commands and holds the current pan/tilt command targets;
it does not teleport joints or guarantee instantaneous physical stopping.

`Simulation` resolves body, joint, actuator, sensor and camera IDs once against the
compiled model (unprefixed names, one robot per model). Multi-instance/prefix support
was removed; it can be reintroduced if a concrete need appears.

## Observations

Gymnasium now defaults to named ideal sensors and actuated-joint readings. See
[observation contract v1](docs/observations.md) for units, frames, validity masks
and reset timing. Use `observation_mode="privileged"` to retain the earlier raw
`qpos`, `qvel`, `sensors` interface. Camera overview rendering is not a sensor.
See [ROS/Python parity status](docs/ros-parity.md) for Pixi verification limits.

## Standalone verification on this MacBook

The native PT101 viewer was launched with a clean environment (no ROS/Pixi
activation). All three variants also generated, stepped, stopped and reset in an
isolated Python subprocess with ROS imports blocked and no Gymnasium import.
Run that regression check with:

```sh
python -m pytest lekiwi_mujoco/test/test_without_ros.py -q
```

No ROS installation is required. The source checkout and initialized payload
submodule supply URDF/mesh/configuration files; those data files are still required.
A standalone Python virtual environment plus the package dependencies suffices.
This verifies the local simulation foundation, not future algorithm correctness
or equivalence with ROS controllers. Native Windows validation remains open.

The native viewer shows a thin cyan XY travel trail, sampled every centimetre
and limited to the latest 1,000 points. X clears it. It is visual-only, projected
3 mm above the default flat floor; terrain-following trails are not implemented.

For planned Pi-hosted ROS simulation with a local laptop viewer, see the
[remote-viewer plan](docs/remote-viewer-plan.md). Pi prerequisites were inspected;
the remote simulation/viewer connection is not implemented or validated yet.
