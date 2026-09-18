# MuJoCo models and validation

The portable XML files are `lekiwi_base.xml`, `lekiwi_pt100_oakd_s2.xml` and
`lekiwi_pt101_oakd_s2.xml`. Their relative mesh paths require the repository mesh assets
and, for payload models, the pinned `payloads/pantilt_ros2` submodule.

See [scene and parameter ownership](PARAMETERS.md) for the authoritative inputs and scene selection.

## Test without ROS

From the repository root:

```sh
git submodule update --init payloads/pantilt_ros2
python -m venv .venv
. .venv/bin/activate
pip install -r lekiwi_mujoco/requirements.txt 'pytest<8'
python3 -m lekiwi_mujoco.build_mujoco_models
pytest lekiwi_mujoco/test/test_mujoco_models.py -q
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

These are headless model tests, with no window or waypoint follower. The benchmark applies
fixed body-velocity commands through wheel inverse kinematics and measures MuJoCo motion.
`simulation.py` supplies shared wheel commands and headless execution; tests and benchmarks use it directly.

## Native model inspection

```sh
# macOS (use python on Linux):
mjpython3 -m lekiwi_mujoco.mujoco_preview --variant pt101
```

The launcher regenerates the selected model from the current xacro/URDF and robot
configuration on each launch, then opens it through MuJoCo's passive viewer and steps
physics in real time. Restart after configuration edits; it does not hot-reload during motion.
It does not implement navigation. Use the Control
panel for wheel/pan/tilt actuator sliders, P to pause/resume and X to reset.
Keyboard controls: Up/Down forward/backward, Left/Right strafe left/right,
Shift+Left/Right rotate left/right. Alt (Windows/Linux) or Option (macOS) plus
Left/Right pans left/right; Alt/Option plus Up/Down tilts up/down. Alt/Option takes
priority over Shift and suppresses base motion while held. Both left/right modifiers
work; R/F/T/G are left to the native viewer.
X reset and P pause remain case insensitive. Hold motion keys to keep moving; release
arrows to stop the wheel command. Pan/tilt targets move at each joint's configured velocity limit while held and hold
their last angle on release, clamped to joint limits. Shift can change while an arrow
stays held. Opposing keys cancel; losing window focus stops keyboard motion.
The first key press attaches GLFW press/release callbacks on the viewer UI thread.
Subsequent robot-key events are consumed and other keys are forwarded to MuJoCo.
Reset and pause clear held inputs; release/repress motion keys to resume. Click the
viewport before using keys. `--variant base`
and `--variant pt100` select other models; `--model` explicitly loads a prebuilt XML with its embedded limits; regenerate that XML
after configuration edits. Legacy XML without limit metadata must be regenerated.
Optional `--telemetry state.json` writes simulation time, commands and warning counts.

## Parametric command limits and platform support

`config/base/control.yaml` in `lekiwi_control` supplies the base x/y/yaw velocity
limits and wheel kinematics. Expanded URDF wheel joint limits supply wheel-speed ceilings,
using the motor configuration from `lekiwi_control/config/base/urdf_config.yaml`.
Pan/tilt uses the smaller of its URDF joint velocity and hardware `max_velocity` parameter;
the real-mode URDF's 1e6 sentinel is therefore not mistaken for a physical limit.
The generator embeds this metadata and wheel actuator `ctrlrange` in all three XML files.
The preview and ROS launch regenerate on startup. Committed XML snapshots require running
the generator after edits. No manually synchronized velocity constants are required.

Held base keys request the configured axis maxima; combined translation/rotation is
scaled uniformly to satisfy every wheel limit. Wheel sliders are also constrained by base
and wheel command limits. Pan/tilt keyboard targets use each joint's full configured velocity limit; slider position
changes are rate-limited too. Enabled controller velocity overrides further reduce this
ceiling. Position bounds are the intersection of URDF, hardware and enabled controller
limits. The default pan/tilt limits are 4.433204477 rad/s and [-1.5708, 1.5708] rad;
these numbers are generated from configuration, not constants in the keyboard handler.
These are actuator command limits, not artificial clamps on physics state: actual motion
can overshoot or be driven by external forces, and motor dynamics remain uncalibrated.
ROS retains its own controller limit enforcement; the viewer's position-target rate limiter
is not injected into the ROS control plugin.

Keep Shift for rotation. Either Shift key works with the same GLFW key codes on macOS,
Windows and Linux. Option is Alt, and Command is Super/Windows, so neither is a better
portable default. On Windows use:

```powershell
py -m venv .venv
.venv\Scripts\python -m pip install -r lekiwi_mujoco/requirements.txt
python3 -m lekiwi_mujoco.mujoco_preview --variant pt101
```

Use `mjpython` on macOS and `python` on Linux. Keyboard input uses GLFW callbacks, not
macOS-specific key APIs. CI is configured to run model/key tests on Windows, macOS and
Linux; the native Windows viewer still needs an interactive verification on Windows.

## Test with ROS

Build and source the repository using its normal ROS setup, then run:

```sh
colcon test --packages-select lekiwi_mujoco lekiwi_description
colcon test-result --verbose
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true payload:=""
# Or, instead of the base-only launch:
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true payload:=pantilt pantilt_config:=pt101
```

Install `lekiwi_mujoco/requirements.txt` in the Python environment used by ROS launch and colcon.
Model generation now requires the MuJoCo Python package as well as xacro and PyYAML;
Gymnasium is used only by the optional learning interface. The ament model tests skip
when these Python dependencies are unavailable.
The dedicated non-ROS CI job installs those dependencies and executes the model tests.
ROS uses the existing `mujoco_ros2_control` physics engine, GUI and controllers. Use the
existing waypoint recording/patrol services and Nav2 `/follow_waypoints` action to test
navigation. No alternative follower, demo waypoint YAML, ROS observer window or ROS viewer launch
option is provided. Dynamic trails/waypoint overlays were removed with the custom viewers;
the XML models do not implement those overlays themselves.

## Model changes and physical limitations

The former contact cylinders had ordinary isotropic friction and resisted an omni wheel's required lateral slip. Each wheel now uses an effective ring of **16 passive axisymmetric rollers**. The roller axes are tangential; lateral wheel motion is permitted by their own joint rotation. These are actual contact bodies, not forces applied to the chassis or a kinematic position override. The wheel visual meshes, mounts, radius (0.051 m), wheelbase radius (0.132239 m), actuator names and wheel-axis signs remain unchanged.

Roller mass/inertia is subtracted from each URDF wheel inertia, preserving its aggregate mass, center of mass and tensor. Every represented link now uses its URDF inertial data; sensor mass is no longer inferred from mesh volume. Current totals are 2.56 kg for the base and 3.779226 kg for either payload. These are URDF assumptions, not measurements or generator constants. See [parameter ownership](PARAMETERS.md).

The roller count/profile, surface friction, bearing loss, servo armature/friction/gains, chassis inertia and several payload inertias remain uncalibrated approximations. Sixteen effective rollers are not asserted to reproduce the CAD roller count or double-row construction. The contact profile has slight radius ripple. The model does not include measured motor torque-speed curves, gear backlash, tyre deformation, battery sag, or real sensor noise/latency. A 2 ms step and implicit-fast integration improve numerical consistency; do not retune physical coefficients solely to force perfect command tracking.

The imported OAK camera now uses MuJoCo's `-Z` view / `+Y` up convention mapped to the mechanical frame's `+X` forward / `+Z` up. Tests verify its neutral world direction and that it follows both joints. Lens-center translation, intrinsics/distortion, and exact physical optical extrinsics still need calibration. The generator takes payload body, joint and mesh transforms directly from the expanded URDF. This corrects the pinned MJCF tilt offset from 0.1025 m to the URDF value of 0.0541441 m, lowering the tilt/camera assembly by 48.3559 mm. Joint limits and effort ceilings follow the URDF; simulation actuator gains are explicit in `config/mujoco.yaml`.

## Regeneration and validation

Edit `base_shared.xml` / `base_subtree.xml` and the `.mjcf.xacro` entry points, then regenerate all committed snapshots:

```sh
python3 -m lekiwi_mujoco.build_mujoco_models
pip install 'pytest<8'
pytest lekiwi_mujoco/test/test_mujoco_models.py -q
python3 -m lekiwi_mujoco.benchmark_mujoco --output motion.json
```

The same generator is used by ROS launch, with absolute mesh paths for its temporary model. It expands the pinned payload and synchronizes payload geometry with the URDF and applies the camera-frame correction in the enclosing LeKiwi assembly. Directly running xacro bypasses that integration correction; use this generator for runnable models. `--pt-package` supports an explicit installed payload share directory.

The tests cover loading/settling all three models, preserved actuator/sensor interfaces and wheel mass, motion in multiple directions, pan/tilt setpoints, camera direction, independent URDF body-frame and mesh-bound comparisons at three pan/tilt poses per payload, speed saturation and generated-file freshness. A dedicated non-ROS CI job runs them. The updated ROS launch path still needs an end-to-end run on a sourced Linux ROS workspace; the development host is macOS with a Pixi ROS Kilted environment, but its MuJoCo ROS control plugin is unavailable.
