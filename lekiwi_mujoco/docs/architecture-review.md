# LeKiwi simulation: open items

## Execution modes and algorithm portability

| Open item | Gap and required verification |
|---|---|
| **1. Full ROS on Raspberry Pi; viewer on MacBook or Windows laptop** | Implement and verify authoritative headless MuJoCo physics and ROS controllers on the Pi, with local native MuJoCo rendering on the laptop over ROS 2 Zenoh. Follow [the remote-viewer plan](remote-viewer-plan.md). The laptop must not advance a second independent simulation. Verify ground-truth pose/joint-state synchronization and passive roller coverage before deciding whether a small additional state publisher is needed. Verify model/assets, state and clock synchronization, pause/reset, reconnects and command routing through ROS. Measure Pi simulation rate and viewing latency. |
| **2. Full ROS on MacBook or Windows through Pixi; viewer on the same laptop** | Establish and verify a Pixi-managed ROS 2 environment containing MuJoCo, `mujoco_ros2_control` and the LeKiwi controllers on each platform. Investigate the current macOS MuJoCo vendor restriction, missing dependencies and Windows build/runtime support. Verify native-library/Python compatibility, ROS launch, local viewing and keyboard input. If native execution is blocked, document the blocker and evaluate a Linux VM or WSL fallback explicitly; do not count a fallback as verified native support. |
| **3. Standalone portability and algorithm validation** | Verify no-ROS installation and native viewer interaction on Windows. Validate each concrete algorithm/feature workflow, including matching headless and viewed behavior. Future algorithm correctness and transfer to ROS remain separate from basic local execution. |
| **Transfer algorithms and features from standalone simulation to ROS** | Define the minimal inputs/outputs needed by a concrete algorithm: observations, commands, units, frames, timestamps and lifecycle/reset behavior. Keep algorithm logic independent of ROS messages and transport where practical; connect it to ROS through a small adapter or verify an equivalent ROS implementation. Exercise matching input scenarios and compare outputs and robot behavior before treating the port as equivalent. Reuse existing ROS controllers/navigation instead of building replacements solely for simulation. Do not introduce a general plugin framework before a concrete feature requires it. |
| **End-to-end ROS behavior and Python parity** | Validate ROS control/navigation against the updated models in modes 1 and 2, and compare command semantics with mode 3. Resolve or explicitly document position-versus-rate pan/tilt commands, acceleration constraints, saturation, command timeout, stopping, reset and clock behavior. Compare commanded and measured trajectories; record the ROS path (recording is not part of lekiwi yet). Node/configuration tests alone do not establish controller, physics-bridge or algorithm-port equivalence. |

## Simulation fidelity

| Open item | Gap and required verification |
|---|---|
| **Sensor correspondence and camera integration** | Verify IMU frames, heading reference, lidar ray conventions and ROS message correspondence. Establish device sampling rates, noise and latency only as required by experiments. Investigate disabled ROS camera simulation and verify optical pose/intrinsics before using images as robot observations. Ideal simulator signals are not calibrated hardware measurements. |
| **Motor calibration** | Command limits do not establish firmware-equivalent dynamics. Measure wheel and pan/tilt response, acceleration, braking and load dependence; determine how firmware settings such as `pantilt_internal_max_vel: 65` relate to physical motion and MuJoCo actuator parameters. |
| **Contact calibration** | Effective roller geometry, friction and bearing losses remain approximations. Measure straight, strafe and turn trajectories, slip and stopping distance on a known flat surface. Separate contact errors from motor-model errors before adjusting parameters. |
| **Physical inertia audit** | Verify whether the camera mass is included in another link before adding separate inertia. Check other link masses, centers of mass and inertia tensors against physical data. Make corrections in URDF only. |

## Deferred requirements

| Open item | Gap and scope |
|---|---|
| **Waypoint and remote trail visualization** | Add available waypoint markers and carry the travel trail into the chosen remote viewer. Prefer 2D overlays or thin ground-plane marks. Consume existing ROS navigation state; do not implement another waypoint follower. Include this in remote-viewing evaluation. |
| **Experiment provenance** | When a concrete experiment needs reproducibility, associate model/configuration versions, dependency versions, seeds and reset/time epochs with its recordings. Choose the smallest sufficient metadata mechanism then; a general manifest framework and dataset conversion are not current prerequisites. |
