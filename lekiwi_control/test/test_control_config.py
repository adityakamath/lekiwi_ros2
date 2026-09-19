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
_CFG_BASE = os.path.join(_PKG_SRC, 'config')
_CFG_PANTILT = _CFG_BASE


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
                    'sts3215_max_vel_steps', 'proportional_acc_max', 'proportional_vel_max',
                    'internal_max_vel', 'internal_max_acc', 'internal_acc_coeff',
                    'internal_control_period'):
            assert key in self.cfg, f"Missing key '{key}' in urdf_config.yaml"

    def test_baud_rate_is_positive_int(self):
        assert isinstance(self.cfg['baud_rate'], int) and self.cfg['baud_rate'] > 0

    def test_motor_ids_are_distinct(self):
        ids = [self.cfg['left_motor_id'], self.cfg['back_motor_id'], self.cfg['right_motor_id']]
        assert len(ids) == len(set(ids)), "Motor IDs must be distinct"


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

    def test_no_dead_limit_blocks(self):
        # omni_wheel_drive_controller declares no velocity/acceleration parameters; speed limits
        # are the teleop axis scales (base_teleop.yaml) and Nav2's.
        params = self.cfg['base_controller']['ros__parameters']
        assert 'linear' not in params and 'angular' not in params


# ── base_teleop.yaml ──────────────────────────────────────────────────────────────

class TestTeleopYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'base_teleop.yaml'))

    def test_joy_teleop_present(self):
        assert 'joy_teleop' in self.cfg

    def test_drive_action_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        assert 'teleop' in actions, "Missing 'teleop' drive action"

    def test_axis_scales_are_positive_speed_limits(self):
        axes = self.cfg['joy_teleop']['ros__parameters']['teleop']['axis_mappings']
        for name in ('twist-linear-x', 'twist-linear-y', 'twist-angular-z'):
            assert axes[name]['scale'] > 0

    def test_deadman_button_defined(self):
        teleop = self.cfg['joy_teleop']['ros__parameters']['teleop']
        assert 'deadman_buttons' in teleop and len(teleop['deadman_buttons']) > 0

    def test_toggle_services_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        for svc in ('estop_toggle', 'twist_switch_toggle'):
            assert svc in actions, f"Missing toggle service '{svc}' in base_teleop.yaml"


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
        teleop_cfg = _load(os.path.join(_CFG_BASE, 'base_teleop.yaml'))
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


# ── collision_toggle.yaml ────────────────────────────────────────────────────
# Moved from lekiwi_navigation - the node now runs inside lekiwi_control's
# control_support_node (its target, collision_monitor, stays in lekiwi_navigation).

class TestCollisionToggleYaml:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_BASE, 'collision_toggle.yaml'))

    def test_section_present(self):
        assert 'collision_toggle_node' in self.cfg

    def test_required_keys_present(self):
        params = self.cfg['collision_toggle_node']['ros__parameters']
        for key in ('button', 'target_node', 'parameter_name'):
            assert key in params, f"Missing key '{key}' in collision_toggle.yaml"

    def test_targets_collision_monitor(self):
        node = self.cfg['collision_toggle_node']['ros__parameters']['target_node']
        assert node == '/collision_monitor'


# ── pantilt/pantilt_teleop.yaml ──────────────────────────────────────────────────────

class TestTeleopYamlPantilt:
    def setup_method(self):
        self.cfg = _load(os.path.join(_CFG_PANTILT, 'pantilt_teleop.yaml'))

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
        base_teleop = _load(os.path.join(_CFG_BASE, 'base_teleop.yaml'))
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
