#!/usr/bin/env python3
"""
Smoke tests for lekiwi_bringup launch argument surfaces.

Each test calls `ros2 launch --show-arguments` on a launch file and checks that
the expected arguments are declared and that invalid argument combinations are
rejected early by the launch setup function.

These tests do NOT start any nodes - they only validate the launch surface.
"""

import subprocess


def _show_arguments(package, launch_file, extra_args=None):
    """Run `ros2 launch --show-arguments` and return stdout + returncode."""
    cmd = ['ros2', 'launch', '--show-arguments', package, launch_file]
    if extra_args:
        cmd += extra_args
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    return result.stdout + result.stderr, result.returncode


def _dry_run(package, launch_file, extra_args=None):
    """Run the launch in dry-run mode (no nodes started) to exercise OpaqueFunction logic."""
    cmd = ['ros2', 'launch', '--noninteractive', package, launch_file, 'use_sim_time:=true']
    if extra_args:
        cmd += extra_args
    # We give it 5 s max - enough to hit OpaqueFunction validation and exit.
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    return result.stdout + result.stderr, result.returncode


# ── lekiwi.launch.py argument surface ────────────────────────────────────────

class TestLekiwiBringupArgs:
    """Checks that the top-level launch declares all expected arguments."""

    EXPECTED_ARGS = [
        'payload', 'pantilt_config', 'diagnostics', 'control_mock',
        'fusion_mode', 'pointcloud', 'octomap', 'mission', 'map_name',
        'use_sim_time', 'joy',
    ]

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_bringup', 'lekiwi.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not found in lekiwi.launch.py"


# ── lekiwi.launch.py validation rules ────────────────────────────────────────

class TestLekiwiBringupValidation:
    """Checks that invalid argument combinations fail fast with RuntimeError."""

    def test_invalid_payload_rejected(self):
        output, _ = _dry_run('lekiwi_bringup', 'lekiwi.launch.py',
                              ['payload:=invalid_payload'])
        assert 'Unknown payload' in output or 'RuntimeError' in output or 'invalid_payload' in output

    def test_pointcloud_without_pantilt_rejected(self):
        output, _ = _dry_run('lekiwi_bringup', 'lekiwi.launch.py',
                              ['payload:=', 'pointcloud:=true'])
        # Empty payload is also rejected as malformed - either way launch fails.
        assert any(kw in output.lower() for kw in ('pointcloud', 'pantilt', 'malformed', 'runtimeerror'))

    def test_octomap_without_pointcloud_rejected(self):
        output, _ = _dry_run('lekiwi_bringup', 'lekiwi.launch.py',
                              ['payload:=pantilt', 'octomap:=true', 'pointcloud:=false'])
        assert 'octomap' in output.lower() or 'pointcloud' in output.lower()


# ── slam.launch.py argument surface ──────────────────────────────────────────

class TestSlamLaunchArgs:
    """Checks that slam.launch.py declares all expected arguments."""

    EXPECTED_ARGS = ['mission', 'map_name', 'use_sim_time']

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_navigation', 'slam.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not found in slam.launch.py"


# ── slam.launch.py validation rules ──────────────────────────────────────────

class TestSlamLaunchValidation:
    """Checks that slam.launch.py rejects invalid mission/map_name combinations."""

    def test_unknown_mission_rejected(self):
        output, _ = _dry_run('lekiwi_navigation', 'slam.launch.py',
                              ['mission:=invalid'])
        assert 'Unknown mission' in output or 'RuntimeError' in output

    def test_slam_without_map_name_rejected(self):
        output, _ = _dry_run('lekiwi_navigation', 'slam.launch.py',
                              ['mission:=slam'])
        assert 'map_name' in output.lower() or 'requires' in output.lower()

    def test_amcl_without_map_name_rejected(self):
        output, _ = _dry_run('lekiwi_navigation', 'slam.launch.py',
                              ['mission:=amcl'])
        assert 'map_name' in output.lower() or 'requires' in output.lower()


# ── laser.launch.py argument surface ─────────────────────────────────────────

class TestLaserLaunchArgs:
    """Checks that laser.launch.py declares all expected arguments."""

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_bringup', 'laser.launch.py')
        for arg in ('payload', 'custom_filter'):
            assert arg in output, f"Expected argument '{arg}' not found in laser.launch.py"


# ── control.launch.py argument surface ───────────────────────────────────────

class TestControlLaunchArgs:
    """Checks that control.launch.py declares all expected arguments."""

    EXPECTED_ARGS = [
        'payload', 'pantilt_config', 'sts_serial_port', 'use_mock',
        'diagnostics', 'use_sim_time', 'joy',
    ]

    def test_expected_args_declared(self):
        output, _ = _show_arguments('lekiwi_control', 'control.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not found in control.launch.py"
