# Getting Started with LeKiwi Base Control

This guide covers building, launching, and operating the LeKiwi omnidirectional robot.

## Prerequisites

- ROS 2 (Humble, Jazzy, or Kilted) installed
- Workspace set up with required packages:
  - `sts_hardware_interface`
  - `lekiwi_base_control`

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface lekiwi_base_control
source install/setup.bash
```

## Hardware Setup

1. **Connect the Robot**:
   - Ensure the USB-to-serial adapter is connected
   - Verify the device is available at `/dev/ttySERVO`
   - If using a different port, update `base_properties.xacro` accordingly

2. **Power On**:
   - Connect battery/power supply to the motors
   - Motors should be in a safe position (off the ground or with wheels free)

3. **Verify Communication** (optional):

   ```bash
   # Check that the serial port exists
   ls -l /dev/ttySERVO
   ```

## Launching the Robot

```bash
ros2 launch lekiwi_base_control base_control.launch.py
```

You should see output indicating:

- Robot description loaded from `base.urdf.xacro`
- Controller manager started
- Hardware interface connected to motors (IDs: 7, 8, 9)
- Controllers spawned successfully:
  - `joint_state_broadcaster`
  - `omni_wheel_controller`

## Basic Operations

### 1. Send Velocity Commands

**Move Forward (X-axis)**:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

**Strafe Sideways (Y-axis)**:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {z: 0.0}}"
```

**Rotate in Place**:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

**Stop the Robot**:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

### 2. Monitor Robot State

**View Joint States**:

```bash
ros2 topic echo /joint_states
```

Shows position and velocity for all three wheels (left, back, right).

**View Odometry**:

```bash
ros2 topic echo /odom
```

Shows robot pose and velocity estimates based on position feedback.

**View TF Transforms**:

```bash
# Install tf2_tools if needed
ros2 run tf2_ros tf2_echo odom base_link
```

### 3. Controller Status

**List Active Controllers**:

```bash
ros2 control list_controllers
```

**View Hardware Interface Status**:

```bash
ros2 control list_hardware_interfaces
```

Should show command and state interfaces for all three wheel joints.

## Velocity and Acceleration Limits

The robot has the following maximum limits (automatically calculated from hardware constraints):

### Velocity Limits (80% of hardware maximum)

Motor-level: 2720 steps/s = 4.17 rad/s

Robot-level (based on omnidirectional kinematics):
- **Linear X**: 0.246 m/s
- **Linear Y**: 0.213 m/s (limiting direction)
- **Angular Z**: 1.61 rad/s (92.23 deg/s)

### Acceleration Limits (100% of hardware maximum)

Motor-level: 25,400 steps/s² = 38.97 rad/s²

Robot-level (based on omnidirectional kinematics):
- **Linear X**: 2.29 m/s²
- **Linear Y**: 1.99 m/s² (limiting direction)
- **Angular Z**: 15.03 rad/s² (861 deg/s²)

Commands exceeding these limits will be automatically clamped by the controller to prevent motor damage.

## Configuration

All robot parameters are centralized in `urdf/base_properties.xacro`:

- **Wheel geometry**: radius, thickness, base offset
- **Motor configuration**: IDs, operating mode, serial port
- **Hardware limits**: max velocity/acceleration in steps
- **Derived limits**: automatically calculated for each axis

To change motor limits:

1. Edit `base_properties.xacro`:
   - Adjust `max_velocity_steps` (default: 2720 = 80% of 3400 hardware max)
   - Adjust `max_acceleration_steps` (default: 25400 = 100% hardware max)
2. Derived values in URDF (rad/s) automatically recalculate
3. Manually update corresponding values in `base_controllers.yaml` (m/s, rad/s)
4. Rebuild: `colcon build --packages-select lekiwi_base_control`

## Troubleshooting

### Motors Not Responding

1. Check serial connection:

   ```bash
   ls -l /dev/ttySERVO
   ```

2. Verify motor power is connected
3. Check motor IDs are correct (7, 8, 9) in `base_properties.xacro`
4. Review controller manager logs for errors:

   ```bash
   ros2 topic echo /rosout
   ```

### Incorrect Motor Behavior

1. Verify operating mode is set to velocity mode (mode=1)
2. Check that motors are initialized properly
3. Ensure baud rate matches (1000000)
4. Verify sync write is enabled for coordinated motion

### Odometry Drifting

- Ensure `position_feedback: true` in `base_controllers.yaml`
- Verify wheels are not slipping on the surface
- Check that `enable_multi_turn: true` in `base.urdf.xacro`
- Confirm wheel radius and base offset are accurate

### Controllers Not Loading

1. Check controller configuration exists: `config/base_controllers.yaml`
2. Verify URDF loads correctly:

   ```bash
   ros2 run robot_state_publisher robot_state_publisher \
     --ros-args -p robot_description:="$(xacro /path/to/base.urdf.xacro)"
   ```

3. List available controller types:

   ```bash
   ros2 control list_controller_types
   ```

### Emergency Stop

If the robot needs to be stopped immediately:

```bash
# Method 1: Send zero velocity (immediate)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" --once

# Method 2: Stop publishing (triggers timeout after 0.5s)
# Press Ctrl+C on cmd_vel publisher
```

## Performance Tips

1. **Smooth Motion**: Use acceleration limits to prevent jerky movements
2. **Accurate Odometry**: Keep wheels clean and ensure good surface contact
3. **Optimal Performance**: Run at 50 Hz control frequency (default)
4. **Battery Life**: Lower `max_velocity_steps` to reduce power consumption

## Testing

### Unit Testing

```bash
# Test hardware interface (if tests exist)
colcon test --packages-select sts_hardware_interface
colcon test-result --verbose
```

### Manual Testing Checklist

- [ ] Motors respond to velocity commands
- [ ] All three wheels move independently
- [ ] Odometry updates at 50 Hz
- [ ] Joint states published correctly
- [ ] Emergency stop works (cmd_vel timeout)
- [ ] Velocity limits enforced
- [ ] Robot moves in expected directions

## Next Steps

- See [architecture.md](architecture.md) for system design details
- Integrate with navigation stack (nav2)
- Add sensors (lidar, camera, IMU) to URDF
- Tune controller parameters for your specific hardware
- Create custom launch files for different operating modes

## Configuration Reference

### Key Files

| File | Purpose | Location |
|------|---------|----------|
| `base_properties.xacro` | Single source of truth for all properties | `urdf/` |
| `base.urdf.xacro` | Robot structure definition | `urdf/` |
| `base_controllers.yaml` | Controller configuration | `config/` |
| `base_control.launch.py` | Main launch file | `launch/` |

### Important Parameters

| Parameter | Location | Default | Description |
|-----------|----------|---------|-------------|
| `max_velocity_steps` | `base_properties.xacro` | 2720 | Motor velocity limit (80% of 3400 max) |
| `max_acceleration_steps` | `base_properties.xacro` | 25400 | Motor acceleration limit (100% max) |
| `serial_port` | `base_properties.xacro` | /dev/ttySERVO | Serial device path |
| `baud_rate` | `base_properties.xacro` | 1000000 | Serial communication speed |
| `left_motor_id` | `base_properties.xacro` | 7 | Left wheel motor ID |
| `back_motor_id` | `base_properties.xacro` | 8 | Back wheel motor ID |
| `right_motor_id` | `base_properties.xacro` | 9 | Right wheel motor ID |
| `update_rate` | `base_controllers.yaml` | 50 | Control loop frequency (Hz) |
| `cmd_vel_timeout` | `base_controllers.yaml` | 0.5 | Command timeout (seconds) |

## Additional Resources

- [ROS 2 Control Documentation](https://control.ros.org/)
- [Omni Wheel Drive Controller](https://control.ros.org/master/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)
- [STS Hardware Interface README](../sts_hardware_interface/README.md)
