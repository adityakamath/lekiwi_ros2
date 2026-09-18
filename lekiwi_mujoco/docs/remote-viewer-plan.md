# Pi simulation with a local native MuJoCo viewer

Proposed first implementation; the remote connection and viewer are not implemented or verified.

## Status — 2026-09-18

- Headless `sim:=true` runs on the Pi with all controllers active. `mujoco_gui` defaults to
  `false`: the earlier GLFW crash came from requesting the interactive viewer on a
  display-less host, so no X server, GPU or OpenGL is needed. The camera-init warning is harmless.
- The laptop side may be ROS 2 based (rclpy, `rmw_zenoh_cpp` and MuJoCo in a Pixi
  environment); it never steps physics.
- Still open: check whether `/joint_states`, TF and odometry already carry enough state
  for the viewer (roller coverage), then Zenoh router endpoints between Pi and laptop
  (deployment configuration, not repository code).

## Verified Pi prerequisites — 2026-09-17

Read-only SSH inspection of `ubuntu@lekiwi` confirmed:

- ARM64 host with ROS 2 Kilted under `/opt/ros/kilted`.
- `mujoco_ros2_control` 0.0.3-1noble.20260611.084259.
- `omni_wheel_drive_controller` 5.13.0-1noble.20260305.030959.
- `rmw_zenoh_cpp` 0.6.6-1noble.20260225.112004.
- Python MuJoCo installed in the user's Python 3.12 environment.
- `zenohd` 1.9.0 running as a systemd service and listening on TCP port 7447.
- Installed library contains `odom_free_joint_name` and `odom_topic` configuration keys, with `/simulator/floating_base_state` as the default topic string. Live publication remains unverified.
- Installed pause, reset and step service definitions. Pause takes `bool paused`; reset takes `string keyframe`. Availability and behavior of running services remain unverified.

The existing checkout is `~/ros2_ws/src/lekiwi_ros2`, on `main`, with local
untracked content in the lidar submodule. Preserve it and use an isolated
simulation checkout. No simulation/controller process was observed during this
inspection. No Pi files or services were changed. Router compatibility and
cross-machine communication have not yet been tested.

## Architecture

- Pi: headless MuJoCo, mujoco_ros2_control, existing ROS controllers and a Zenoh router.
- Laptop: native MuJoCo viewer plus a small ROS subscriber/command adapter in Pixi.
- Transport: ROS 2 rmw_zenoh_cpp, with explicitly configured router endpoints over LAN/VPN. Match ROS distribution, message definitions, domain ID and compatible middleware versions. Do not depend on multicast discovery across the VPN.
- The Pi alone advances physics. The laptop loads matching MJCF/assets and updates render state from received snapshots. It never calls mj_step or runs a second controller loop. Camera orbit/zoom stays local.

The laptop needs ROS client libraries and rmw_zenoh, but not mujoco_ros2_control or the robot controller binaries. This avoids the currently blocked native macOS simulation plugin build. First verify that MuJoCo and the ROS Python client can coexist in the laptop's Pixi interpreter.

## State and controls

Inspect the Pi's installed package revision and actual topics/services before selecting a state interface. Upstream documents free-joint odometry plus pause/reset/step interfaces, but installed-version support must be checked. JointState alone cannot reproduce the floating base, and controller odometry can drift relative to simulation ground truth.

Reuse existing joint state, simulator ground-truth base pose and clock topics if they provide sufficient synchronized state for viewing. Check passive roller coverage. If exact visual state cannot be obtained, add only a small simulation-state publisher at the existing plugin boundary, with simulation timestamp and a complete, consistently sampled pose state. Avoid rebuilding the bridge or inventing a general transport framework.

Start around 30 state updates/s; render locally independently. Keep only the newest sample, show stale/disconnected status, and handle backward clock jumps/reset epochs. Check matching model identity and state layout before applying updates. Copy matching assets once rather than sending meshes every frame.

Keyboard commands go through existing ROS controller topics. Preserve base velocity and pan/tilt position semantics; do not write remote MuJoCo actuator arrays directly. Verify stop-on-release and a Pi-side stale-command timeout. Reuse supported pause/reset services. Disable unsupported viewer actions rather than making local-only state changes. Trails follow received ground-truth base positions and clear on reset.

## Alternatives

Foxglove Bridge is useful for optional plots, TF, sensors and navigation diagnostics. It supplies a WebSocket connection for Foxglove, not automatic state synchronization for the native MuJoCo viewer. Rosbridge is a fallback if a ROS-free laptop client becomes necessary; it still requires a native-viewer state adapter. Neither is required in the initial Zenoh design. Do not implement multiple transports initially. A raw Zenoh Python subscriber is not automatically a ROS rmw_zenoh subscriber; use the ROS client rather than reimplementing its wire protocol.

## Verification sequence

1. Prepare an isolated simulation checkout on the Pi and confirm its launch command uses simulation-only hardware interfaces. Preserve the existing workspace and its local changes.
2. Run headless on the Pi and confirm active controllers, ground-truth state, simulation clock and supported services.
3. Establish Zenoh communication with explicit LAN/VPN endpoints. Verify messages in both directions.
4. Show received robot motion in the laptop viewer, first read-only; verify base orientation, wheel/pan/tilt motion and model agreement.
5. Enable keyboard commands, pause/reset where supported, and the trail. Test disconnect/reconnect and stale-command stopping.
6. Measure real-time factor on Pi and state age/render responsiveness on laptop. Test Windows separately; Mac success does not establish Windows support.

## References

- [ROS 2 rmw_zenoh setup and configuration](https://github.com/ros2/rmw_zenoh)
- [MuJoCo ROS control hardware and simulation interfaces](https://github.com/ros-controls/mujoco_ros2_control/blob/main/mujoco_ros2_control/docs/hardware_interface.rst)
- [Foxglove Bridge](https://docs.foxglove.dev/docs/fleet/bridge)
