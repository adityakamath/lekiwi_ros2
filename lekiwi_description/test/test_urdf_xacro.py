#!/usr/bin/env python3
"""
Tests for lekiwi_description URDF/Xacro files.

Tests that all xacro files parse to valid XML without errors, using mock
hardware arguments so no physical device is required.
"""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_URDF_BASE = os.path.join(_PKG_SRC, 'urdf', 'base', 'base.urdf.xacro')
_URDF_PANTILT = os.path.join(_PKG_SRC, 'urdf', 'base_pantilt', 'base_pantilt.urdf.xacro')

# Mock hardware args matching urdf_config.yaml defaults so the xacro parses cleanly.
_BASE_ARGS = [
    'serial_port:=/dev/ttySERVO',
    'use_mock:=true',
    'baud_rate:=1000000',
    'use_sync_write:=true',
    'left_motor_id:=7',
    'back_motor_id:=8',
    'right_motor_id:=9',
    'sts3215_max_vel_steps:=3400',
    'proportional_acc_max:=25',
]

_PANTILT_ARGS = _BASE_ARGS + [
    'pantilt_config:=pt101',
    'pan_motor_id:=1',
    'tilt_motor_id:=2',
    'pan_center_steps:=2048',
    'tilt_center_steps:=2646',
    'proportional_vel_max:=0',
    'pan_joint_lower:=-1.5708',
    'pan_joint_upper:=1.5708',
    'tilt_joint_lower:=-1.5708',
    'tilt_joint_upper:=1.5708',
]


def _xacro(xacro_file, args):
    """Run xacro and return (stdout, stderr, returncode)."""
    result = subprocess.run(
        ['xacro', xacro_file] + args,
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout, result.stderr, result.returncode


# ── base URDF ────────────────────────────────────────────────────────────────

class TestBaseUrdf:
    def setup_method(self):
        self.stdout, self.stderr, self.rc = _xacro(_URDF_BASE, _BASE_ARGS)

    def test_xacro_exits_cleanly(self):
        assert self.rc == 0, f"xacro failed:\n{self.stderr}"

    def test_output_is_valid_xml(self):
        assert self.stdout.strip(), "xacro produced empty output"
        try:
            ET.fromstring(self.stdout)
        except ET.ParseError as e:
            pytest.fail(f"base.urdf.xacro output is not valid XML: {e}")

    def test_robot_element_present(self):
        root = ET.fromstring(self.stdout)
        assert root.tag == 'robot', f"Root element should be 'robot', got '{root.tag}'"

    def test_base_link_present(self):
        root = ET.fromstring(self.stdout)
        link_names = [link.get('name') for link in root.findall('link')]
        assert 'base_link' in link_names or 'base_footprint' in link_names, \
            "Expected base_link or base_footprint in URDF"

    def test_wheel_joints_present(self):
        root = ET.fromstring(self.stdout)
        joint_names = [j.get('name') for j in root.findall('joint')]
        for joint in ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'):
            assert joint in joint_names, f"Missing wheel joint '{joint}' in base URDF"

    def test_ros2_control_block_present(self):
        root = ET.fromstring(self.stdout)
        assert root.find('ros2_control') is not None, \
            "base URDF must contain a ros2_control block"

    def test_no_xacro_warnings(self):
        assert 'ERROR' not in self.stderr.upper(), \
            f"xacro reported errors:\n{self.stderr}"


# ── base_pantilt URDF ─────────────────────────────────────────────────────────

class TestBasePantiltUrdf:
    def setup_method(self):
        self.stdout, self.stderr, self.rc = _xacro(_URDF_PANTILT, _PANTILT_ARGS)

    def test_xacro_exits_cleanly(self):
        assert self.rc == 0, f"xacro failed:\n{self.stderr}"

    def test_output_is_valid_xml(self):
        assert self.stdout.strip(), "xacro produced empty output"
        try:
            ET.fromstring(self.stdout)
        except ET.ParseError as e:
            pytest.fail(f"base_pantilt.urdf.xacro output is not valid XML: {e}")

    def test_robot_element_present(self):
        root = ET.fromstring(self.stdout)
        assert root.tag == 'robot'

    def test_pantilt_joints_present(self):
        root = ET.fromstring(self.stdout)
        joint_names = [j.get('name') for j in root.findall('joint')]
        for joint in ('pan_joint', 'tilt_joint'):
            assert joint in joint_names, f"Missing pantilt joint '{joint}' in base_pantilt URDF"

    def test_wheel_joints_still_present(self):
        root = ET.fromstring(self.stdout)
        joint_names = [j.get('name') for j in root.findall('joint')]
        for joint in ('left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'):
            assert joint in joint_names, f"Missing wheel joint '{joint}' in base_pantilt URDF"

    def test_ros2_control_block_present(self):
        root = ET.fromstring(self.stdout)
        assert root.find('ros2_control') is not None

    def test_no_xacro_errors(self):
        assert 'ERROR' not in self.stderr.upper(), \
            f"xacro reported errors:\n{self.stderr}"


# ── ros2_control interface structure ─────────────────────────────────────────

class TestBaseUrdfInterfaces:
    """Wheel joints must expose velocity command + position/velocity state interfaces."""

    def setup_method(self):
        stdout, _, _ = _xacro(_URDF_BASE, _BASE_ARGS)
        self.root = ET.fromstring(stdout)

    def test_wheel_command_interfaces_are_velocity(self):
        rc = self.root.find('ros2_control[@name="lekiwi_base"]')
        assert rc is not None, "ros2_control block 'lekiwi_base' not found"
        joints = rc.findall('.//joint')
        assert len(joints) == 3, f"Expected 3 wheel joints in ros2_control, got {len(joints)}"
        for j in joints:
            cmd_ifaces = [i.get('name') for i in j.findall('command_interface')]
            assert 'velocity' in cmd_ifaces, \
                f"Wheel joint '{j.get('name')}' must have velocity command interface"

    def test_wheel_state_interfaces_include_position_and_velocity(self):
        rc = self.root.find('ros2_control[@name="lekiwi_base"]')
        for j in rc.findall('.//joint'):
            state_ifaces = [i.get('name') for i in j.findall('state_interface')]
            assert 'position' in state_ifaces, \
                f"Wheel joint '{j.get('name')}' must have position state interface"
            assert 'velocity' in state_ifaces, \
                f"Wheel joint '{j.get('name')}' must have velocity state interface"

    def test_imu_sensor_block_present(self):
        rc = self.root.find('ros2_control[@name="lekiwi_imu"]')
        assert rc is not None, "ros2_control block 'lekiwi_imu' not found"
        sensor = rc.find('.//sensor[@name="bno055"]')
        assert sensor is not None, "IMU sensor block 'bno055' not found in lekiwi_imu"

    def test_imu_orientation_state_interfaces_present(self):
        rc = self.root.find('ros2_control[@name="lekiwi_imu"]')
        sensor = rc.find('.//sensor[@name="bno055"]')
        ifaces = [i.get('name') for i in sensor.findall('state_interface')]
        for expected in ('orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'):
            assert expected in ifaces, f"IMU sensor missing state interface '{expected}'"

    def test_gazebo_imu_sensor_present(self):
        assert self.root.find('.//gazebo[@reference="imu_frame"]/sensor[@name="imu"]') is not None, \
            "Gazebo imu_frame sensor should be present when imu:=true"


class TestBaseUrdfNoImu:
    """imu:=false must omit all IMU sensor blocks but keep lekiwi_base intact."""

    def setup_method(self):
        stdout, _, _ = _xacro(_URDF_BASE, _BASE_ARGS + ['imu:=false'])
        self.root = ET.fromstring(stdout)

    def test_imu_sensor_block_absent(self):
        assert self.root.find('ros2_control[@name="lekiwi_imu"]') is None, \
            "ros2_control block 'lekiwi_imu' should be absent when imu:=false"

    def test_base_sensor_block_still_present(self):
        assert self.root.find('ros2_control[@name="lekiwi_base"]') is not None, \
            "ros2_control block 'lekiwi_base' should still be present when imu:=false"

    def test_gazebo_imu_sensor_absent(self):
        assert self.root.find('.//gazebo[@reference="imu_frame"]/sensor[@name="imu"]') is None, \
            "Gazebo imu_frame sensor should be absent when imu:=false"


class TestBaseUrdfMujocoImu:
    """mujoco hardware type must gate its own bno055 sensor block on imu, like real hardware."""

    def _render(self, imu):
        stdout, stderr, rc = _xacro(
            _URDF_BASE, _BASE_ARGS + ['ros2_control_hardware_type:=mujoco', f'imu:={imu}'])
        assert rc == 0, f"xacro failed:\n{stderr}"
        return ET.fromstring(stdout)

    def test_mujoco_bno055_sensor_present_when_imu_true(self):
        root = self._render('true')
        rc = root.find('ros2_control[@name="lekiwi_base"]')
        assert rc.find('.//sensor[@name="bno055"]') is not None, \
            "mujoco lekiwi_base block should contain a bno055 sensor when imu:=true"
        assert rc.find('.//sensor[@name="lidar"]') is not None, "lidar sensor should still be present"

    def test_mujoco_bno055_sensor_absent_when_imu_false(self):
        root = self._render('false')
        rc = root.find('ros2_control[@name="lekiwi_base"]')
        assert rc.find('.//sensor[@name="bno055"]') is None, \
            "mujoco lekiwi_base block should omit the bno055 sensor when imu:=false"
        assert rc.find('.//sensor[@name="lidar"]') is not None, \
            "lidar sensor should still be present when imu:=false"


class TestBasePantiltUrdfInterfaces:
    """Pan/tilt joints must expose position command interface."""

    def setup_method(self):
        stdout, _, _ = _xacro(_URDF_PANTILT, _PANTILT_ARGS)
        self.root = ET.fromstring(stdout)

    def test_pantilt_command_interfaces_are_position(self):
        # pan_joint and tilt_joint live in a separate ros2_control block from wheels.
        all_rc = self.root.findall('ros2_control')
        pantilt_joints = {}
        for rc in all_rc:
            for j in rc.findall('.//joint'):
                name = j.get('name')
                if name in ('pan_joint', 'tilt_joint'):
                    pantilt_joints[name] = j
        assert len(pantilt_joints) == 2, \
            f"Expected pan_joint and tilt_joint in ros2_control, found: {list(pantilt_joints)}"
        for name, j in pantilt_joints.items():
            cmd_ifaces = [i.get('name') for i in j.findall('command_interface')]
            assert 'position' in cmd_ifaces, \
                f"Joint '{name}' must have position command interface"


# ── Stale pre-built .urdf consistency check ───────────────────────────────────

class TestPrebuiltUrdfConsistency:
    """
    The committed .urdf files should be regenerated when xacro sources change.
    This test regenerates them with mock args and checks for meaningful differences
    (ignoring the path comment line that always differs).
    """

    def _strip_comments(self, xml_str: str) -> str:
        """Remove XML comment lines (which carry only the source path)."""
        import re
        return re.sub(r'<!--.*?-->', '', xml_str, flags=re.DOTALL).strip()

    def test_base_urdf_not_stale(self):
        prebuilt = os.path.join(_PKG_SRC, 'urdf', 'base', 'base.urdf')
        if not os.path.exists(prebuilt):
            pytest.skip("base.urdf not present; nothing to compare against")
        generated, _, rc = _xacro(_URDF_BASE, _BASE_ARGS)
        assert rc == 0
        with open(prebuilt) as f:
            existing = f.read()
        # Normalize both to ignore path-comment differences.
        assert self._strip_comments(generated) == self._strip_comments(existing), \
            "base.urdf is stale — regenerate with: xacro base.urdf.xacro ... > base.urdf"

    def test_base_pantilt_urdf_not_stale(self):
        prebuilt = os.path.join(_PKG_SRC, 'urdf', 'base_pantilt', 'base_pantilt.urdf')
        if not os.path.exists(prebuilt):
            pytest.skip("base_pantilt.urdf not present; nothing to compare against")
        generated, _, rc = _xacro(_URDF_PANTILT, _PANTILT_ARGS)
        assert rc == 0
        with open(prebuilt) as f:
            existing = f.read()
        assert self._strip_comments(generated) == self._strip_comments(existing), \
            "base_pantilt.urdf is stale — regenerate with: xacro base_pantilt.urdf.xacro ... > base_pantilt.urdf"


# ── pantilt_config variant ────────────────────────────────────────────────────

class TestBasePantiltUrdfVariants:
    """Both pt100 and pt101 mesh variants must parse successfully."""

    @pytest.mark.parametrize('variant', ['pt100', 'pt101'])
    def test_pantilt_variant_parses(self, variant):
        args = [a for a in _PANTILT_ARGS if not a.startswith('pantilt_config')]
        args.append(f'pantilt_config:={variant}')
        _, stderr, rc = _xacro(_URDF_PANTILT, args)
        assert rc == 0, f"xacro failed for pantilt_config:={variant}:\n{stderr}"
