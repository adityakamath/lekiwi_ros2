# LeKiwi Base Control

ROS 2 control package for the LeKiwi 3-wheel omnidirectional mobile robot.

## Overview

This package provides complete robot control using standard ROS 2 components:

- **Omnidirectional Drive Controller**: Converts cmd_vel to wheel velocities and computes odometry
- **Joint State Broadcaster**: Publishes real-time motor states
- **Hardware Interface**: Communicates with Feetech STS3215 servo motors

## Quick Start

See [docs/getting_started.md](docs/getting_started.md) for detailed setup and usage instructions.

## Hardware

- **Platform**: 3-wheel omnidirectional robot
- **Motors**: Feetech STS3215 servos (IDs: 7, 8, 9)
- **Communication**: Serial (1000000 baud) via `/dev/ttySERVO`
- **Wheel Layout**: 60°, 180°, 300° from +X axis

## Features

- Standard ROS 2 controllers (no custom code)
- Position-based odometry using encoder feedback
- Multi-turn position tracking for continuous operation
- Velocity and acceleration limits
- Emergency stop functionality
- Real-time control at 50 Hz

## Package Structure

```text
lekiwi_base_control/
├── config/
│   └── base_controllers.yaml     # Controller parameters
├── launch/
│   └── base_control.launch.py    # Launch file
├── urdf/
│   ├── base.urdf.xacro            # Robot description
│   └── base_properties.xacro     # Shared properties
└── docs/
    ├── getting_started.md         # Usage guide
    └── architecture.md            # System design
```

## Basic Usage

```bash
# Launch the robot control system
ros2 launch lekiwi_base_control base_control.launch.py

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}"

# Monitor odometry
ros2 topic echo /odom
```

## Documentation

- [Getting Started Guide](docs/getting_started.md) - Setup and basic usage
- [Architecture Documentation](docs/architecture.md) - System design and components

## Dependencies

- ROS 2 (Humble, Jazzy, or Kilted)
- ros2_control
- ros2_controllers (omni_wheel_drive_controller)
- sts_hardware_interface

## License

Apache-2.0
