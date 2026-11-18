#!/usr/bin/env python3
"""
Tests for lekiwi_description URDF/Xacro files.

Tests that all xacro files parse to valid XML without errors, using mock
hardware arguments so no physical device is required.
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_URDF_BASE = os.path.join(_PKG_SRC, 'urdf', 'base.urdf.xacro')
_URDF_DOCK = os.path.join(_PKG_SRC, 'urdf', 'dock.urdf.xacro')

# Mock hardware args matching urdf_config.yaml defaults so the xacro parses cleanly.
_BASE_ARGS = [
    'hardware_config:=' + os.path.join(_PKG_SRC, '..', 'lekiwi_bringup', 'config', 'hardware.yaml'),
    'base_controller_config:=' + os.path.join(_PKG_SRC, '..', 'lekiwi_control', 'config', 'control.yaml'),
    'serial_port:=/dev/ttySERVO',
    'use_mock:=true',
    'baud_rate:=1000000',
    'use_sync_write:=true',
    'left_motor_id:=7',
    'back_motor_id:=8',
    'right_motor_id:=9',
    'sts3215_max_vel_steps:=3400',
    'proportional_acc_max:=25',
    'internal_max_vel:=254',
    'internal_max_acc:=254',
    'internal_acc_coeff:=100',
]

def _portable(xml):
    """The pre-built base.urdf points at the local meshes by relative path."""
    return xml.replace('package://lekiwi_description/meshes/', '../meshes/')


def _xacro(xacro_file, args):
    """Run xacro and return (stdout, stderr, returncode)."""
    result = subprocess.run(
        ['xacro', xacro_file] + args,
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout, result.stderr, result.returncode


# ── base URDF ────────────────────────────────────────────────────────────────

def test_robot_structure():
    stdout, stderr, rc = _xacro(_URDF_BASE, _BASE_ARGS)
    assert rc == 0, stderr
    root = ET.fromstring(stdout)
    assert root.tag == 'robot'
    links = {link.get('name') for link in root.findall('link')}
    assert {'base_link', 'base_footprint'} <= links
    joints = {joint.get('name') for joint in root.findall('joint')}
    assert {'left_wheel_joint', 'back_wheel_joint', 'right_wheel_joint'} <= joints
    assert root.find("ros2_control[@name='lekiwi_base']") is not None


class TestBaseStaticGemini2:
    """The base carries a static Gemini 2 on the front camera mount."""

    def setup_method(self):
        self.stdout, _, self.rc = _xacro(_URDF_BASE, _BASE_ARGS)
        self.root = ET.fromstring(self.stdout)

    def test_camera_frames_and_mount(self):
        links = {link.get('name') for link in self.root.findall('link')}
        assert {'gemini2_link', 'gemini2_imu_frame', 'gemini2_link_model_origin'} <= links
        joint = self.root.find("joint[@name='gemini2_link_center_joint']")
        assert joint.find('parent').get('link') == 'base_link'
        assert [float(v) for v in joint.find('origin').get('xyz').split()] == pytest.approx([0.09365, 0, 0.0723])

    def test_camera_comes_from_lekiwi_description_only(self):
        mesh = self.root.find("link[@name='gemini2_link_model_origin']/visual/geometry/mesh").get('filename')
        assert mesh == 'package://lekiwi_description/meshes/gemini2.stl'
        assert os.path.isfile(os.path.join(_PKG_SRC, 'meshes', 'gemini2.stl'))


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

    def test_internal_register_params_are_exposed(self):
        rc = self.root.find('ros2_control[@name="lekiwi_base"]')
        for j in rc.findall('.//joint'):
            params = {p.get('name'): p.text for p in j.findall('param')}
            for key in ('internal_max_vel', 'internal_max_acc', 'internal_acc_coeff'):
                assert key in params, f"Wheel joint '{j.get('name')}' missing '{key}' param"

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
            "Gazebo imu_frame sensor should be present with the default hardware profile"


# ── Stale pre-built .urdf consistency check ───────────────────────────────────

class TestPrebuiltUrdfConsistency:
    """
    The committed base.urdf should be regenerated when xacro sources change.
    This test regenerates them with mock args and checks for meaningful differences
    (ignoring the path comment line that always differs).
    """

    def _strip_comments(self, xml_str: str) -> str:
        """Remove XML comment lines (which carry only the source path)."""
        import re
        return re.sub(r'<!--.*?-->', '', xml_str, flags=re.DOTALL).strip()

    def test_base_urdf_not_stale(self):
        prebuilt = os.path.join(_PKG_SRC, 'urdf', 'base.urdf')
        assert os.path.isfile(prebuilt), 'The checked-in base.urdf is missing'
        generated, _, rc = _xacro(_URDF_BASE, _BASE_ARGS)
        assert rc == 0
        with open(prebuilt) as f:
            existing = f.read()
        # Normalize both to ignore path-comment differences.
        assert self._strip_comments(_portable(generated)) == self._strip_comments(existing), \
            'base.urdf is stale: regenerate with python3 test/test_urdf_xacro.py --write'

    def test_dock_urdf_not_stale(self):
        prebuilt = os.path.join(_PKG_SRC, 'urdf', 'dock.urdf')
        assert os.path.isfile(prebuilt), 'The checked-in dock.urdf is missing'
        generated, _, rc = _xacro(_URDF_DOCK, _BASE_ARGS)
        assert rc == 0
        with open(prebuilt) as f:
            existing = f.read()
        assert self._strip_comments(_portable(generated)) == self._strip_comments(existing), \
            'dock.urdf is stale: regenerate with python3 test/test_urdf_xacro.py --write'


if __name__ == '__main__' and '--write' in sys.argv:
    for name in ('base', 'dock'):
        result = subprocess.run(['xacro', f'{name}.urdf.xacro'] + _BASE_ARGS,
                                cwd=os.path.join(_PKG_SRC, 'urdf'),
                                capture_output=True, text=True, check=True)
        with open(os.path.join(_PKG_SRC, 'urdf', f'{name}.urdf'), 'w') as out:
            out.write(_portable(result.stdout))


# Hardware presence must remove whole assemblies, without dangling TF references.
_AUDIO_LINKS = {'mic_link', 'respeaker_board_link', 'speaker_link'}
_TOF_LINKS = {'tof_link', 'tof_frame'}
_LASER_LINKS = {'laser_link', 'laser_frame'}
_IMU_LINKS = {'imu_link', 'imu_frame'}
_CAMERA_LINKS = {'gemini2_link', 'gemini2_imu_frame', 'gemini2_link_model_origin', 'gemini2_mount_link'}


@pytest.mark.parametrize('settings,removed,retained', [
    ({'rpi5': False}, {'rpi5_case_link'}, _LASER_LINKS | _AUDIO_LINKS),
    ({'imu': False}, _IMU_LINKS, _LASER_LINKS | _AUDIO_LINKS | _TOF_LINKS | _CAMERA_LINKS),
    ({'lidar': False}, _LASER_LINKS, _IMU_LINKS | _AUDIO_LINKS | _TOF_LINKS | _CAMERA_LINKS | {'lidar_plate_link'}),
    ({'respeaker': False}, _AUDIO_LINKS, _LASER_LINKS | _TOF_LINKS | _CAMERA_LINKS),
    ({'gemini2': False}, _CAMERA_LINKS, _AUDIO_LINKS | _TOF_LINKS),
    ({'shelf': False}, {'tray_link'}, _AUDIO_LINKS | _LASER_LINKS),
    ({'sparkfun_sensor': False}, _TOF_LINKS, _AUDIO_LINKS | _LASER_LINKS),
    ({'respeaker': False, 'gemini2': False, 'shelf': False, 'sparkfun_sensor': False},
     _AUDIO_LINKS | _TOF_LINKS | _CAMERA_LINKS |
     {'tray_link'},
     _LASER_LINKS | {'imu_link', 'imu_frame', 'base_link', 'base_footprint', 'rpi5_case_link', 'left_wheel_link', 'back_wheel_link', 'right_wheel_link'}),
])
def test_hardware_presence_removes_complete_assemblies(settings, removed, retained, tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump(settings))
    stdout, stderr, rc = _xacro(_URDF_BASE, _BASE_ARGS + [f'hardware_config:={config}'])
    assert rc == 0, stderr
    root = ET.fromstring(stdout)
    links = {link.get('name') for link in root.findall('link')}
    assert not removed & links
    assert retained <= links
    for joint in root.findall('joint'):
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links
    for gazebo in root.findall('gazebo[@reference]'):
        assert gazebo.get('reference') in links


def test_disabled_gemini2_has_no_simulated_control_sensor(tmp_path):
    config = tmp_path / 'hardware.yaml'
    config.write_text('gemini2: false\n')
    stdout, stderr, rc = _xacro(_URDF_BASE, _BASE_ARGS + [
        f'hardware_config:={config}', 'ros2_control_hardware_type:=mujoco'])
    assert rc == 0, stderr
    assert ET.fromstring(stdout).find("ros2_control/sensor[@name='gemini2']") is None


@pytest.mark.parametrize('shelf', ['true', 'false'])
def test_tray_assembly_follows_shelf_setting(tmp_path, shelf):
    wrapper = tmp_path / 'plate.xacro'
    wrapper.write_text(f'''<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test">
      <xacro:include filename="{_PKG_SRC}/urdf/base.module.xacro"/>
      <xacro:property name="theta" value="${{2*pi/3}}"/>
      <xacro:property name="max_torque_nm" value="3"/>
      <xacro:property name="max_velocity_rads" value="10"/>
      <link name="base_footprint"/>
      <xacro:lekiwi_base_module parent="base_footprint">
        <origin xyz="0 0 0"/>
      </xacro:lekiwi_base_module>
    </robot>''')
    config = tmp_path / 'hardware.yaml'
    config.write_text('shelf: ' + shelf + '\n')
    stdout, stderr, rc = _xacro(str(wrapper), _BASE_ARGS + [f'hardware_config:={config}'])
    assert rc == 0, stderr
    root = ET.fromstring(stdout)
    has_tray = shelf == 'true'
    meshes = {m.get('filename').split('/')[-1] for m in root.findall('.//visual/geometry/mesh')}
    assert ('tray_assembly.stl' in meshes) == has_tray
    plate = root.find("link[@name='tray_link']")
    assert (plate is not None) == has_tray
    if plate is not None:
        offset = root.find("joint[@name='tray_joint']/origin").get('xyz')
        assert list(map(float, offset.split())) == pytest.approx([0, 0, 0])
    links = {link.get('name') for link in root.findall('link')}
    for joint in root.findall('joint'):
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links


@pytest.mark.parametrize('backend', ['real', 'mujoco'])
def test_disabled_imu_has_no_links_or_control_interfaces(tmp_path, backend):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'imu': False}))
    stdout, stderr, rc = _xacro(_URDF_BASE, _BASE_ARGS + [
        f'hardware_config:={config}', f'ros2_control_hardware_type:={backend}'])
    assert rc == 0, stderr
    root = ET.fromstring(stdout)
    assert root.find("ros2_control/sensor[@name='bno055']") is None
    assert root.find("ros2_control[@name='lekiwi_imu']") is None
    assert root.find("gazebo[@reference='imu_frame']") is None
    assert not {'imu_link', 'imu_frame'} & {link.get('name') for link in root.findall('link')}
    assert not {'imu_joint', 'imu_frame_joint'} & {joint.get('name') for joint in root.findall('joint')}


@pytest.mark.parametrize('backend', ['real', 'mujoco'])
def test_imu_interfaces_are_present_by_default(tmp_path, backend):
    config = tmp_path / 'hardware.yaml'
    config.write_text(yaml.safe_dump({'respeaker': False}))
    stdout, stderr, rc = _xacro(_URDF_BASE, _BASE_ARGS + [
        f'hardware_config:={config}', f'ros2_control_hardware_type:={backend}'])
    assert rc == 0, stderr
    root = ET.fromstring(stdout)
    sensor = root.find("ros2_control/sensor[@name='bno055']")
    assert (sensor is not None) is True
    assert (root.find("ros2_control[@name='lekiwi_imu']") is not None) == (backend == 'real')
    assert (root.find("gazebo[@reference='imu_frame']") is not None) is True
    if sensor is not None:
        names = {entry.get('name') for entry in sensor.findall('state_interface')}
        assert {'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'} <= names
