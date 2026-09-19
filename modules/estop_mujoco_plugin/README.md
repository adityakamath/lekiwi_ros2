# estop_mujoco_plugin

An emergency-stop plugin for [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control).
It serves `/emergency_stop` (`std_srvs/SetBool`) the way a real hardware interface does
(e.g. [sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)), so the same
joystick buttons, toggles and scripts that stop a real robot stop the simulated one. It knows nothing
about any particular robot: it works from the actuators in the MuJoCo model.

## Behaviour

While the stop is enabled, the plugin overrides every actuator command immediately before each physics
step, so it wins over whatever the controllers write. Releasing it hands the commands back.

| Actuator | While stopped |
|---|---|
| Velocity servo (e.g. wheels) | commanded to zero |
| Joint position servo (e.g. pan-tilt, arm joints) | holds the angle it had when the stop was enabled, limited to its control range |
| Anything else (torque motors, tendons) | commanded to zero |

Enabling twice keeps the first held position; a world reset relatches from the new state. Controllers
stay active throughout.

## Use

Build it in the workspace with `mujoco_ros2_control` 0.1.2 or newer (`sudo apt install
ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins`), then add it to the robot's
`mujoco_plugins` parameters:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      emergency_stop_plugin:
        type: "estop_mujoco_plugin/EmergencyStopPlugin"
```

```sh
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"    # stop
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"   # release
```

A robot package that loads the plugin should list `estop_mujoco_plugin` as an `exec_depend`: a plugin
class that cannot be found is a fatal error in `mujoco_ros2_control`, not a skip.

## Limitations

The service name is fixed to `/emergency_stop`, so a model with several independent robots shares one
stop. The stop is a command override, not a torque cut, so a robot on position servos stays powered and
holds. Only simulation is covered: on real hardware the stop comes from the hardware interface.

## Tests

`colcon test --packages-select estop_mujoco_plugin`. The C++ tests run the stop against real MuJoCo
models (velocity and position servos, range limits, latching, release, reset); the Python test checks
the plugin registration.
