# Scene and parameter ownership

Runnable XML files are generated snapshots. Edit the authoritative inputs below;
do not edit masses, inertias or limits in the generated XML. Preview, Gymnasium and
ROS regenerate from current sources at startup. Explicit `--model`/`model_path`/
`mujoco_model` paths load a snapshot as supplied; regenerate it after source edits.

## Robot and scene

`build_robot_spec(variant)` builds only the robot. `compose_scene(robot, scene)`
attaches it using native MjSpec. `build_spec(variant, scene='flat')` combines both.
The default world's floor, materials, textures, lights and viewer settings live in
`scenes/flat.xml`; robot assets contain none of those world elements. Only this
one scene ships. A caller may supply another scene MJCF path later. Relative scene
mesh/texture assets are resolved against that scene before composition.

The simulation profile `config/mujoco.yaml` owns timestep, integrator and solver
iterations for both robot-only and composed models; it overrides these settings
in an external scene. Existing single-robot names remain unchanged. No multi-instance
support, terrain generator, obstacle library or multi-robot controller is implemented.

```sh
python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101 --output /tmp/robot.xml --absolute --scene none
python3 -m lekiwi_mujoco.build_mujoco_models --variant pt101 --output /tmp/world.xml --absolute --scene flat
# Use mjpython on macOS, python on Linux/Windows:
mjpython3 -m lekiwi_mujoco.mujoco_preview --variant pt101 --scene flat
ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true payload:=pantilt mujoco_scene:=flat
```

Gymnasium accepts `scene='flat'`, `scene=None` or a scene XML path. A robot-only
model has no ground; free dynamics will fall under gravity if stepped.

## Authoritative inputs

| Quantity | Source | Conversion/validation |
|---|---|---|
| Wheel radius, mount radius, angular offset | `lekiwi_control/config/base/control.yaml` | URDF reads these values; MuJoCo reads the expanded URDF for mounts and scales wheel visuals/roller contacts. Kinematics use the same values. Mismatched URDF mount positions/axes fail generation. |
| Other base and payload frames, mesh transforms/scales | Expanded LeKiwi/payload URDF | Copied into represented MjSpec bodies/geoms. The lidar visual remains on its measurement frame to avoid self-hits, with its relative transform adjusted. |
| Link mass, center of mass, full inertia tensor and inertial orientation | Expanded URDF `<inertial>` | Used on every represented link with inertial data. No mass fallback from imported MJCF or CAD density. Nonphysical tensors fail generation. |
| Wheel hub and virtual roller mass/inertia | URDF wheel inertia plus profile `roller_mass_fraction` | The fixed 16-roller construction partitions the wheel mass and subtracts roller inertia using the parallel-axis theorem. Aggregate mass, COM and tensor match URDF at the reference pose. Invalid residual hub inertia fails. |
| Missing inertial data | URDF omission | Fixed frames remain massless. Moving links must have URDF inertia. An inertial URDF link with no MuJoCo body fails rather than being silently discarded. Adding inertia to an existing fixed camera frame is picked up automatically. |
| Motor effort ceiling | URDF joint `limit effort` | Sets MuJoCo actuator force range. This is not a measured torque-speed curve. |
| Velocity/position command limits | Controller configuration and expanded URDF hardware/joint limits | Existing intersection/saturation rules remain. |
| Simulation contact shapes, friction, passive bearings and servo gains/armature | `lekiwi_mujoco/config/mujoco.yaml` | Explicit uncalibrated assumptions. Roller lengths scale with wheel radius; chassis collision remains an explicit cylinder approximation. |
| CAD geometry | Mesh files and URDF mesh scale | CAD files are not deformed automatically except the wheel's uniform size scaling. The nominal wheel CAD radius is declared once in URDF; it is a mesh reference measurement. |
| Floor friction and world appearance | Selected scene XML | Independent of robot contact/actuation configuration. |
| Motor EEPROM settings, acceleration firmware behavior | Existing hardware YAML/URDF | Still hardware settings; no invented mapping to MuJoCo dynamics. Calibration remains separate work. |

The shared wheel geometry is read by URDF through the `base_controller_config` xacro
argument. Pass `lekiwi_control/config/base/control.yaml` explicitly when expanding
URDF; ROS launch and the model builder supply this input. The generator
resolves the checkout or installed control package. Controller kinematic dimensions
must match the intended physical robot; changing them resizes/repositions the model,
but does not invent new masses or inertias. Update URDF inertias if the hardware changed.

Generated XML necessarily contains numerical masses and inertia values. They are
outputs, not an additional source to maintain. The pinned payload MJCF has its own
original inertials; the adapter replaces those with current expanded URDF values
without modifying that dependency.

Current URDF totals are 2.56 kg for the base and 3.779226 kg for PT100/PT101.
These are descriptive results, not generator constants or calibrated measurements.
Sensor masses formerly inferred from mesh volume now follow URDF. Camera frames with
no URDF inertia currently contribute no separate mass; add correct inertial data to
URDF if the camera is not already included in another link's mass budget.

## Verification

Tests exercise separate robot/scene compilation and XML round trips, external-scene
asset paths, changed wheel dimensions for all three variants, URDF mass edits,
full rotated inertia tensors, wheel aggregate inertia, added camera inertia, motor
force/profile updates and rejection of inconsistent parameters. Existing dynamics,
keyboard and Gymnasium checks continue to run. Physical calibration, ROS/native
control parity and interactive Windows validation remain separate open gaps.
