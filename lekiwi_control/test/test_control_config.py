#!/usr/bin/env python3
"""
Tests for lekiwi_control config files and launch argument surface.

Config tests: validate YAML structure and required keys without starting any nodes.
Launch tests: validate argument declarations using `ros2 launch --show-arguments`.
"""

import os
import subprocess

import yaml

# Resolve source package root (works with both symlink-install and regular install).
_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_CFG_BASE = os.path.join(_PKG_SRC, 'config', 'base')
_CFG_PANTILT = os.path.join(_PKG_SRC, 'config', 'payloads', 'pantilt')


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _show_arguments(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', '--show-arguments', package, launch_file],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout + result.stderr


# ── urdf_config.yaml ─────────────────────────────────────────────────────────

class TestUrdfConfig:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'urdf_config.yaml'))

    def test_required_keys_present(self):
        for key in ('serial_port', 'baud_rate', 'use_mock', 'use_sync_write',
                    'left_motor_id', 'back_motor_id', 'right_motor_id',
                    'sts3215_max_vel_steps', 'proportional_acc_max'):
            assert key in self.cfg, f"Missing key '{key}' in urdf_config.yaml"

    def test_baud_rate_is_positive_int(self):
        assert isinstance(self.cfg['baud_rate'], int) and self.cfg['baud_rate'] > 0

    def test_motor_ids_are_distinct(self):
        ids = [self.cfg['left_motor_id'], self.cfg['back_motor_id'], self.cfg['right_motor_id']]
        assert len(ids) == len(set(ids)), "Motor IDs must be distinct"


class TestUrdfConfigPantilt:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_PANTILT, 'urdf_config.yaml'))

    def test_required_keys_present(self):
        for key in ('pan_motor_id', 'tilt_motor_id',
                    'pan_center_steps', 'tilt_center_steps',
                    'pan_joint_lower', 'pan_joint_upper',
                    'tilt_joint_lower', 'tilt_joint_upper'):
            assert key in self.cfg, f"Missing key '{key}' in pantilt/urdf_config.yaml"

    def test_joint_limits_valid(self):
        assert self.cfg['pan_joint_lower'] < self.cfg['pan_joint_upper']
        assert self.cfg['tilt_joint_lower'] < self.cfg['tilt_joint_upper']

    def test_pan_tilt_motor_ids_distinct(self):
        assert self.cfg['pan_motor_id'] != self.cfg['tilt_motor_id']


# ── control.yaml ─────────────────────────────────────────────────────────────

class TestControlYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'control.yaml'))

    def test_controller_manager_present(self):
        assert 'controller_manager' in self.cfg

    def test_update_rate_positive(self):
        rate = self.cfg['controller_manager']['ros__parameters']['update_rate']
        assert isinstance(rate, (int, float)) and rate > 0

    def test_base_controller_present(self):
        assert 'base_controller' in self.cfg

    def test_velocity_limits_consistent(self):
        params = self.cfg['base_controller']['ros__parameters']
        assert params['linear']['x']['max_velocity'] > 0
        assert params['linear']['y']['max_velocity'] > 0
        assert params['angular']['z']['max_velocity'] > 0


# ── teleop.yaml ──────────────────────────────────────────────────────────────

class TestTeleopYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'teleop.yaml'))

    def test_joy_teleop_present(self):
        assert 'joy_teleop' in self.cfg

    def test_drive_action_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        assert 'teleop' in actions, "Missing 'teleop' drive action"

    def test_deadman_button_defined(self):
        teleop = self.cfg['joy_teleop']['ros__parameters']['teleop']
        assert 'deadman_buttons' in teleop and len(teleop['deadman_buttons']) > 0

    def test_toggle_services_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        for svc in ('estop_toggle', 'twist_switch_toggle'):
            assert svc in actions, f"Missing toggle service '{svc}' in teleop.yaml"


# ── toggles.yaml ─────────────────────────────────────────────────────────────

class TestTogglesYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'toggles.yaml'))

    def test_bool_toggle_node_present(self):
        assert 'bool_toggle_node' in self.cfg

    def test_toggles_list_non_empty(self):
        toggles = self.cfg['bool_toggle_node']['ros__parameters']['toggles']
        assert isinstance(toggles, list) and len(toggles) > 0

    def test_each_toggle_has_required_fields(self):
        params = self.cfg['bool_toggle_node']['ros__parameters']
        for name in params['toggles']:
            assert name in params, f"Toggle '{name}' missing from params"
            assert 'trigger_service' in params[name]
            assert 'target_service' in params[name]
            assert 'initial_state' in params[name]

    def test_teleop_yaml_does_not_contain_bool_toggle_node(self):
        """Confirms the decoupling: bool_toggle_node config lives only in toggles.yaml."""
        teleop_cfg = _load(os.path.join(_CFG_BASE, 'teleop.yaml'))
        assert 'bool_toggle_node' not in teleop_cfg


# ── twist_switch.yaml ─────────────────────────────────────────────────────────

class TestTwistSwitchYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'twist_switch.yaml'))

    def test_twist_switch_node_present(self):
        """Key must match TeleopSwitchNode's own hardcoded node name ('twist_switch_node') -
        see control.launch.py's control_support_node, which no longer remaps it."""
        assert 'twist_switch_node' in self.cfg

    def test_required_topic_params(self):
        params = self.cfg['twist_switch_node']['ros__parameters']
        for key in ('input_topic_default', 'input_topic_switched', 'output_topic'):
            assert key in params, f"Missing key '{key}' in twist_switch.yaml"


# ── pantilt/control.yaml ─────────────────────────────────────────────────────

class TestControlYamlPantilt:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_PANTILT, 'control.yaml'))

    def test_pantilt_controller_registered(self):
        types = self.cfg['controller_manager']['ros__parameters']
        assert 'pantilt_controller' in types, \
            "pantilt/control.yaml must register pantilt_controller in controller_manager"

    def test_pantilt_controller_type(self):
        ct = self.cfg['controller_manager']['ros__parameters']['pantilt_controller']['type']
        assert 'ForwardCommandController' in ct

    def test_pan_tilt_joint_limits_valid(self):
        limits = self.cfg['controller_manager']['ros__parameters']['joint_limits']
        for joint in ('pan_joint', 'tilt_joint'):
            assert joint in limits, f"Missing joint_limits entry for '{joint}'"
            assert limits[joint]['min_position'] < limits[joint]['max_position']

    def test_pantilt_controller_uses_position_interface(self):
        iface = self.cfg['pantilt_controller']['ros__parameters']['interface_name']
        assert iface == 'position'


# ── pantilt/teleop.yaml ──────────────────────────────────────────────────────

class TestTeleopYamlPantilt:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_PANTILT, 'teleop.yaml'))

    def test_pantilt_control_action_present(self):
        assert 'pantilt_control' in self.cfg['joy_teleop']['ros__parameters']

    def test_pantilt_control_publishes_to_correct_topic(self):
        action = self.cfg['joy_teleop']['ros__parameters']['pantilt_control']
        assert action['topic_name'] == '/pantilt_controller/commands'

    def test_pantilt_uses_float64_multiarray(self):
        action = self.cfg['joy_teleop']['ros__parameters']['pantilt_control']
        assert action['interface_type'] == 'std_msgs/msg/Float64MultiArray'

    def test_pantilt_shares_l1_deadman(self):
        """Pan-tilt deadman must match the base drive deadman (L1 = button 9)."""
        base_teleop = _load(os.path.join(_CFG_BASE, 'teleop.yaml'))
        base_deadman = base_teleop['joy_teleop']['ros__parameters']['teleop']['deadman_buttons']
        pantilt_deadman = self.cfg['joy_teleop']['ros__parameters']['pantilt_control']['deadman_buttons']
        assert pantilt_deadman == base_deadman, \
            "Pan-tilt deadman must match base drive deadman button(s)"


# ── bno055_diagnostics.yaml ──────────────────────────────────────────────────

class TestBno055DiagnosticsYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'bno055_diagnostics.yaml'))

    def test_section_present(self):
        assert 'bno055_diagnostics' in self.cfg

    def test_required_keys_present(self):
        params = self.cfg['bno055_diagnostics']['ros__parameters']
        for key in ('i2c_bus', 'i2c_addr', 'sensor_mode'):
            assert key in params, f"Missing key '{key}' in bno055_diagnostics.yaml"

    def test_sensor_mode_is_ndof(self):
        """NDOF is the only mode that provides fully fused orientation output."""
        mode = self.cfg['bno055_diagnostics']['ros__parameters']['sensor_mode']
        assert mode == 'NDOF', f"Expected sensor_mode=NDOF, got {mode!r}"


# ── control.launch.py argument surface ───────────────────────────────────────

class TestControlLaunchArgs:
    EXPECTED_ARGS = [
        'payload', 'pantilt_config', 'sts_serial_port', 'use_mock',
        'diagnostics', 'use_sim_time', 'joy',
    ]

    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_control', 'control.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in control.launch.py"


# ── teleop.launch.py argument surface ────────────────────────────────────────

class TestTeleopLaunchArgs:
    EXPECTED_ARGS = ['payload', 'use_sim_time', 'joy']

    def test_expected_args_declared(self):
        output = _show_arguments('lekiwi_control', 'teleop.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in teleop.launch.py"
