#!/usr/bin/env python3
"""
Tests for lekiwi_navigation config files and launch argument surface.

Config tests: validate YAML structure and required keys without starting any nodes.
Launch tests: validate argument declarations using `ros2 launch --show-arguments`.
"""

import os
import subprocess

import pytest
import yaml

_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_CFG_NAV2 = os.path.join(_PKG_SRC, 'config', 'nav2')
_CFG_RL = os.path.join(_PKG_SRC, 'config', 'robot_localization')


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _show_arguments(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', '--show-arguments', package, launch_file],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout + result.stderr


# ── nav2.yaml ────────────────────────────────────────────────────────────────

class TestNav2Yaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'nav2.yaml'))

    def test_required_top_level_keys(self):
        for key in ('bt_navigator', 'controller_server', 'planner_server',
                    'behavior_server', 'velocity_smoother', 'collision_monitor',
                    'local_costmap', 'global_costmap', 'waypoint_follower'):
            assert key in self.cfg, f"Missing top-level key '{key}' in nav2.yaml"

    def test_controller_frequency_positive(self):
        freq = self.cfg['controller_server']['ros__parameters']['controller_frequency']
        assert isinstance(freq, (int, float)) and freq > 0

    def test_mppi_motion_model_is_omni(self):
        model = self.cfg['controller_server']['ros__parameters']['FollowPath']['motion_model']
        assert model == 'Omni', "MPPI motion_model must be 'Omni' for holonomic drive"

    def test_velocity_limits_consistent_across_sections(self):
        """vx_max in MPPI must match max_velocity in velocity_smoother."""
        mppi = self.cfg['controller_server']['ros__parameters']['FollowPath']
        smoother = self.cfg['velocity_smoother']['ros__parameters']
        assert mppi['vx_max'] == smoother['max_velocity'][0], \
            "MPPI vx_max and velocity_smoother max_velocity[0] must match"
        assert mppi['wz_max'] == smoother['max_velocity'][2], \
            "MPPI wz_max and velocity_smoother max_velocity[2] must match"

    def test_robot_radius_set(self):
        local = self.cfg['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
        global_ = self.cfg['global_costmap']['global_costmap']['ros__parameters']['robot_radius']
        assert local > 0 and global_ > 0

    def test_costmap_filters_default_disabled(self):
        """Filters must default to disabled; laser.launch.py enables them at runtime."""
        local_params = self.cfg['local_costmap']['local_costmap']['ros__parameters']
        assert local_params['keepout_filter']['enabled'] is False
        assert local_params['speed_filter']['enabled'] is False


# ── slam_toolbox.yaml ─────────────────────────────────────────────────────────

class TestSlamToolboxYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'slam_toolbox.yaml'))

    def test_slam_toolbox_section_present(self):
        assert 'slam_toolbox' in self.cfg

    def test_required_frame_params(self):
        params = self.cfg['slam_toolbox']['ros__parameters']
        for key in ('odom_frame', 'map_frame', 'base_frame', 'scan_topic'):
            assert key in params, f"Missing key '{key}' in slam_toolbox.yaml"

    def test_frames_match_nav2_convention(self):
        params = self.cfg['slam_toolbox']['ros__parameters']
        assert params['odom_frame'] == 'odom'
        assert params['map_frame'] == 'map'
        assert params['base_frame'] == 'base_footprint'


# ── amcl.yaml ────────────────────────────────────────────────────────────────

class TestAmclYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'amcl.yaml'))

    def test_amcl_section_present(self):
        assert 'amcl' in self.cfg

    def test_omni_motion_model(self):
        model = self.cfg['amcl']['ros__parameters']['robot_model_type']
        assert 'Omni' in model, "AMCL must use OmniMotionModel for holonomic robot"

    def test_max_particles_positive(self):
        assert self.cfg['amcl']['ros__parameters']['max_particles'] > 0

    def test_frames_match_nav2_convention(self):
        params = self.cfg['amcl']['ros__parameters']
        assert params['odom_frame_id'] == 'odom'
        assert params['base_frame_id'] == 'base_footprint'
        assert params['global_frame_id'] == 'map'


# ── EKF configs ──────────────────────────────────────────────────────────────

class TestEkfYaml:
    """Tests run against all three EKF config variants."""

    @pytest.fixture(params=['ekf.yaml', 'ekf_imu.yaml', 'ekf_odom.yaml'])
    def cfg(self, request):
        return _load(os.path.join(_CFG_RL, request.param))

    def test_ekf_node_section_present(self, cfg):
        assert 'ekf_node' in cfg

    def test_frequency_positive(self, cfg):
        freq = cfg['ekf_node']['ros__parameters']['frequency']
        assert isinstance(freq, (int, float)) and freq > 0

    def test_two_d_mode_is_bool(self, cfg):
        assert isinstance(cfg['ekf_node']['ros__parameters']['two_d_mode'], bool)

    def test_frames_match_convention(self, cfg):
        params = cfg['ekf_node']['ros__parameters']
        assert params['odom_frame'] == 'odom'
        assert params['base_link_frame'] == 'base_footprint'
        assert params['map_frame'] == 'map'

    def test_publish_tf_enabled(self, cfg):
        assert cfg['ekf_node']['ros__parameters']['publish_tf'] is True


class TestEkfBaseYaml:
    """Checks specific to the base fusion config (wheel odom + IMU)."""

    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_RL, 'ekf.yaml'))

    def test_odom0_and_imu0_both_present(self):
        params = self.cfg['ekf_node']['ros__parameters']
        assert 'odom0' in params, "ekf.yaml must fuse wheel odom (odom0)"
        assert 'imu0' in params, "ekf.yaml must fuse IMU (imu0)"

    def test_odom0_fuses_velocities_not_position(self):
        """Wheel odom should contribute vx/vy but not absolute position."""
        config = self.cfg['ekf_node']['ros__parameters']['odom0_config']
        # Indices 0-2 are x/y/z position - must be false for odometry-only fusion.
        assert config[0] is False and config[1] is False, \
            "ekf.yaml odom0 should not fuse absolute position"
        # Indices 6-7 are vx/vy - must be true.
        assert config[6] is True and config[7] is True, \
            "ekf.yaml odom0 must fuse vx and vy"


# ── EKF isolation ─────────────────────────────────────────────────────────────

class TestEkfImuYaml:
    """ekf_imu.yaml (IMU only) must NOT fuse wheel odometry."""

    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_RL, 'ekf_imu.yaml'))

    def test_no_odom0_source(self):
        assert 'odom0' not in self.cfg['ekf_node']['ros__parameters'], \
            "ekf_imu.yaml must not fuse wheel odometry (odom0)"

    def test_imu0_present(self):
        assert 'imu0' in self.cfg['ekf_node']['ros__parameters'], \
            "ekf_imu.yaml must fuse IMU (imu0)"


class TestEkfOdomYaml:
    """ekf_odom.yaml (odometry only) must NOT fuse IMU."""

    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_RL, 'ekf_odom.yaml'))

    def test_no_imu0_source(self):
        assert 'imu0' not in self.cfg['ekf_node']['ros__parameters'], \
            "ekf_odom.yaml must not fuse IMU (imu0)"

    def test_odom0_present(self):
        assert 'odom0' in self.cfg['ekf_node']['ros__parameters'], \
            "ekf_odom.yaml must fuse wheel odometry (odom0)"


# ── Cross-file velocity limit consistency ─────────────────────────────────────

class TestCrossFileVelocityConsistency:
    """
    nav2.yaml comments state several limits must match base_controller values in
    lekiwi_control/config/base/control.yaml. This test enforces that contract.
    """

    def setup_method(self):
        self.nav2 = _load(os.path.join(_CFG_NAV2, 'nav2.yaml'))
        # lekiwi_navigation lives at src/lekiwi_ros2/lekiwi_navigation;
        # lekiwi_control lives at src/lekiwi_ros2/lekiwi_control — same monorepo.
        ctrl_path = os.path.normpath(os.path.join(
            _PKG_SRC, '..', 'lekiwi_control', 'config', 'base', 'control.yaml'
        ))
        self.ctrl = _load(ctrl_path)

    def _ctrl_linear(self, axis):
        return self.ctrl['base_controller']['ros__parameters']['linear'][axis]

    def _ctrl_angular(self):
        return self.ctrl['base_controller']['ros__parameters']['angular']['z']

    def test_behavior_server_backup_limits_match_controller(self):
        bs = self.nav2['behavior_server']['ros__parameters']
        assert bs['backup']['acceleration_limit'] == self._ctrl_linear('x')['max_acceleration']
        assert bs['backup']['deceleration_limit'] == -self._ctrl_linear('x')['max_acceleration']

    def test_behavior_server_rotation_limits_match_controller(self):
        bs = self.nav2['behavior_server']['ros__parameters']
        assert bs['max_rotational_vel'] == self._ctrl_angular()['max_velocity']
        assert bs['rotational_acc_lim'] == self._ctrl_angular()['max_acceleration']

    def test_velocity_smoother_limits_match_controller(self):
        vs = self.nav2['velocity_smoother']['ros__parameters']
        assert vs['max_velocity'][0] == self._ctrl_linear('x')['max_velocity']
        assert vs['max_velocity'][1] == self._ctrl_linear('y')['max_velocity']
        assert vs['max_velocity'][2] == self._ctrl_angular()['max_velocity']
        assert vs['max_accel'][0] == self._ctrl_linear('x')['max_acceleration']
        assert vs['max_accel'][1] == self._ctrl_linear('y')['max_acceleration']


# ── collision_toggle.yaml, map_saver.yaml, waypoint_recorder.yaml ─────────────

class TestCollisionToggleYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'collision_toggle.yaml'))

    def test_section_present(self):
        assert 'collision_toggle_node' in self.cfg

    def test_required_keys_present(self):
        params = self.cfg['collision_toggle_node']['ros__parameters']
        for key in ('button', 'target_node', 'parameter_name'):
            assert key in params, f"Missing key '{key}' in collision_toggle.yaml"

    def test_targets_collision_monitor(self):
        node = self.cfg['collision_toggle_node']['ros__parameters']['target_node']
        assert node == '/collision_monitor'


class TestMapSaverYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'map_saver.yaml'))

    def test_section_present(self):
        assert 'map_saver_node' in self.cfg

    def test_save_timeout_positive(self):
        t = self.cfg['map_saver_node']['ros__parameters']['save_timeout']
        assert isinstance(t, (int, float)) and t > 0


class TestWaypointRecorderYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_NAV2, 'waypoint_recorder.yaml'))

    def test_section_present(self):
        assert 'waypoint_recorder_node' in self.cfg

    def test_required_keys_present(self):
        params = self.cfg['waypoint_recorder_node']['ros__parameters']
        for key in ('number_of_loops', 'frame_id', 'marker_topic', 'max_retries'):
            assert key in params, f"Missing key '{key}' in waypoint_recorder.yaml"

    def test_frame_id_is_map(self):
        assert self.cfg['waypoint_recorder_node']['ros__parameters']['frame_id'] == 'map'

    def test_max_retries_positive(self):
        retries = self.cfg['waypoint_recorder_node']['ros__parameters']['max_retries']
        assert isinstance(retries, int) and retries > 0


# ── nav2.launch.py argument surface ──────────────────────────────────────────

class TestNav2LaunchArgs:
    EXPECTED_ARGS = ['params_file', 'map_name', 'use_sim_time', 'autostart', 'log_level']

    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_navigation', 'nav2.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in nav2.launch.py"


# ── navigation.launch.py and slam.launch.py argument surface ─────────────────

class TestNavigationLaunchArgs:
    EXPECTED_ARGS = ['fusion_mode', 'mission', 'map_name', 'use_sim_time']

    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_navigation', 'navigation.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in navigation.launch.py"


class TestSlamLaunchArgs:
    EXPECTED_ARGS = ['mission', 'map_name', 'use_sim_time']

    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_navigation', 'slam.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in slam.launch.py"


class TestEkfLaunchArgs:
    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_navigation', 'ekf.launch.py')
        for arg in ('fusion_mode', 'use_sim_time'):
            assert arg in output, f"Expected argument '{arg}' not in ekf.launch.py"
