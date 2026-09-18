# Observation contract v1

Gymnasium defaults to `observation_mode="sensors"`. Raw simulator arrays remain
available only through `observation_mode="privileged"` or direct core access.
This is a deliberate change from the earlier default. No training task is added.

| Field | Units and interpretation |
|---|---|
| time | Seconds in the current simulation epoch, shape (1,). Reset returns zero after settling. |
| joint_position | Radians, unwrapped for continuous wheels; wheel joints then optional pan/tilt, ordered by `env.contract.metadata()['joint_names']`. |
| joint_velocity | rad/s in the same order. Passive rollers and free-base state excluded. |
| imu_orientation_wxyz | Ideal IMU-site orientation relative to simulation world, quaternion wxyz. Convert to xyzw for ROS messages. Not a calibrated BNO055 heading estimate. |
| imu_angular_velocity | rad/s in the IMU site's local axes. |
| imu_specific_force | m/s² in IMU-site axes, including support against gravity; not gravity-subtracted world acceleration. |
| lidar_range | Metres, numeric ray-index order. Invalid/no-hit entries are zero and must be masked. |
| lidar_valid | 0/1 validity mask. Zero range alone does not indicate invalidity. |

All values are copies sampled synchronously at the end of a simulation step.
There is no sensor noise, latency, rolling scan or device-rate emulation. Seed
handling does not imply randomized sensors. Controller step duration is distinct
from physical device sampling rate. Reset begins a new time epoch; external
recorders must distinguish epochs rather than assuming time remains monotonic.

`env.contract.metadata()` returns compiled ray-site positions, quaternions,
parent body IDs and cutoffs. Rays follow local site +Z. These are model-derived
geometry, not measured LD06 calibration. Do not assume that array index alone is
a ROS LaserScan angle or that all rays were captured sequentially.

IMU values use the compiled `imu_site` frame, attached to the URDF-derived IMU
body. Hardware axis-remap, orientation reference, covariance, noise, device
latency and ROS frame correspondence still require measurement/integration tests.
The default interface exposes ideal counterparts of observable signals, not a
claim of hardware-equivalent measurements.

Camera images are not in this observation contract. `render()` remains an
external overview for visualization, not a robot-camera observation. ROS camera
integration remains disabled. Odometry, base world pose, contacts and full joint
state are privileged/debug information rather than sensor observations.
