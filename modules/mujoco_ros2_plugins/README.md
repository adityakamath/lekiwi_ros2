# mujoco_ros2_plugins

Plugins for [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) that give a
simulation what a real robot's hardware interfaces provide. They know nothing about any particular
robot: they work from the actuators and joints in the MuJoCo model. All plugins live in one library and
share one header, `mujoco_ros2_plugins.hpp`.

| Plugin class | What it does |
|---|---|
| `mujoco_ros2_plugins/EmergencyStopPlugin` | Serves `/emergency_stop` (`std_srvs/SetBool`) like [sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface), so the joystick buttons, toggles and scripts that stop a real robot stop the simulated one |

## Adding a plugin

Add `src/<name>_plugin.cpp` that includes `mujoco_ros2_plugins.hpp`, derives from `PluginBase` and ends
with `MUJOCO_ROS2_PLUGINS_EXPORT(mujoco_ros2_plugins::<Name>Plugin)`, then list the file in `PLUGIN_SOURCES`
in `CMakeLists.txt` and the class in `plugins.xml`. Put logic that needs no ROS in that header (as
`EmergencyStop`) so it can be tested with plain MuJoCo.

## EmergencyStopPlugin

### Behaviour

While the stop is enabled, the plugin overrides every actuator command immediately before each physics
step, so it wins over whatever the controllers write. Releasing it hands the commands back.

| Actuator | While stopped |
|---|---|
| Velocity servo (e.g. wheels) | commanded to zero |
| Joint position servo (e.g. pan-tilt, arm joints) | holds the angle it had when the stop was enabled, limited to its control range |
| Anything else (torque motors, tendons) | commanded to zero |

Enabling twice keeps the first held position; a world reset relatches from the new state. Controllers
stay active throughout.

### Use

Build it in the workspace with `mujoco_ros2_control` 0.1.2 or newer (`sudo apt install
ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins`), then add it to the robot's
`mujoco_plugins` parameters:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      emergency_stop_plugin:
        type: "mujoco_ros2_plugins/EmergencyStopPlugin"
```

```sh
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"    # stop
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"   # release
```

A robot package that loads the plugin should list `mujoco_ros2_plugins` as an `exec_depend`: a plugin
class that cannot be found is a fatal error in `mujoco_ros2_control`, not a skip.

### Limitations

The service name is fixed to `/emergency_stop`, so a model with several independent robots shares one
stop. The stop is a command override, not a torque cut, so a robot on position servos stays powered and
holds. Only simulation is covered: on real hardware the stop comes from the hardware interface.

## Tests

`colcon test --packages-select mujoco_ros2_plugins`. The C++ tests run the stop against real MuJoCo
models (velocity and position servos, range limits, latching, release, reset); the Python test checks
the plugin registration.
