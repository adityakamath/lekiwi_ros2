"""Versioned ideal sensor interface, independent of ROS and Gymnasium."""
import numpy as np


class ObservationContract:
    version = 1

    def __init__(self, model, robot):
        self.model, self.robot = model, robot
        self.joints = tuple(robot.joint_ids)
        self.qpos = [int(model.jnt_qposadr[robot.joint_ids[n]]) for n in self.joints]
        self.qvel = [int(model.jnt_dofadr[robot.joint_ids[n]]) for n in self.joints]
        self.lidar = sorted((n for n in robot.sensor_ids if n.startswith('lidar-')),
                            key=lambda n: int(n.split('-')[-1]))
        self.slices = {name: slice(int(model.sensor_adr[sid]), int(model.sensor_adr[sid] + model.sensor_dim[sid]))
                       for name, sid in robot.sensor_ids.items()}
        for name in ('bno055_quat', 'bno055_gyro', 'bno055_accel'):
            if name not in self.slices:
                raise ValueError(f'Missing observation sensor: {name}')

    def read(self, data):
        ranges = np.array([data.sensordata[self.slices[n]][0] for n in self.lidar])
        valid = np.isfinite(ranges) & (ranges >= 0)
        return {
            'time': np.array([data.time]),
            'joint_position': data.qpos[self.qpos].copy(),
            'joint_velocity': data.qvel[self.qvel].copy(),
            'imu_orientation_wxyz': data.sensordata[self.slices['bno055_quat']].copy(),
            'imu_angular_velocity': data.sensordata[self.slices['bno055_gyro']].copy(),
            'imu_specific_force': data.sensordata[self.slices['bno055_accel']].copy(),
            'lidar_range': np.where(valid, ranges, 0.),
            'lidar_valid': valid.astype(np.int8),
        }

    def metadata(self):
        # Each ray follows its compiled site's local +Z axis; do not infer scan angles.
        rays = []
        for name in self.lidar:
            sid = self.robot.sensor_ids[name]
            site = int(self.model.sensor_objid[sid])
            rays.append({'name': name, 'site_id': site,
                         'position_in_parent': self.model.site_pos[site].tolist(),
                         'quaternion_wxyz_in_parent': self.model.site_quat[site].tolist(),
                         'parent_body_id': int(self.model.site_bodyid[site]),
                         'cutoff_m': float(self.model.sensor_cutoff[sid])})
        return {'version': self.version, 'joint_names': list(self.joints),
                'ideal_sensors': True, 'latency_seconds': 0., 'noise_model': None,
                'sampling': 'synchronous at end of step; reset starts a new time epoch',
                'lidar_rays': rays}
