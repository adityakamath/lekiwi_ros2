"""Pure-Python derivation tests; run directly without ROS with PyYAML installed."""
import ast
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_navigation.velocity_limits import velocity_overrides


class TestVelocityLimits(unittest.TestCase):
    def derive(self, scales, offset=0):
        axes = {name: {'scale': scale, 'offset': offset} for name, scale in zip(
            ('twist-linear-x', 'twist-linear-y', 'twist-angular-z'), scales)}
        config = {'joy_teleop': {'ros__parameters': {'teleop': {'axis_mappings': axes}}}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.yaml'
            path.write_text(yaml.safe_dump(config))
            return velocity_overrides(path)

    def test_changed_and_inverted_scales_propagate(self):
        for scales in ([.3, .4, 1.2], [-.12, .25, -.9]):
            with self.subTest(scales=scales):
                result = self.derive(scales)
                limits = list(map(abs, scales))
                self.assertEqual(result['velocity_smoother']['max_velocity'], limits)
                self.assertEqual(result['velocity_smoother']['min_velocity'], [-v for v in limits])
                self.assertEqual(result['behavior_server']['max_rotational_vel'], limits[2])
                for axis, value in zip(('vx', 'vy', 'wz'), limits):
                    self.assertEqual(result['controller_server'][f'FollowPath.{axis}_max'], value)
                    self.assertEqual(result['controller_server'][f'FollowPath.{axis}_min'], -value)

    def test_custom_mppi_plugin_id_receives_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            teleop = Path(directory) / 'teleop.yaml'
            params = Path(directory) / 'nav2.yaml'
            axes = {name: {'scale': scale, 'offset': 0} for name, scale in zip(
                ('twist-linear-x', 'twist-linear-y', 'twist-angular-z'), (.3, .4, .8))}
            teleop.write_text(yaml.safe_dump({
                'joy_teleop': {'ros__parameters': {'teleop': {'axis_mappings': axes}}}}))
            params.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': {
                'controller_plugins': ['MyController'],
                'MyController': {'plugin': 'nav2_mppi_controller::MPPIController'},
            }}}))
            limits = velocity_overrides(teleop, params)['controller_server']
        self.assertEqual(limits['MyController.vx_max'], .3)
        self.assertEqual(limits['MyController.vy_min'], -.4)
        self.assertEqual(limits['MyController.wz_max'], .8)
        self.assertNotIn('FollowPath.vx_max', limits)

    def test_invalid_scales_and_offset_fail(self):
        for value in (0, float('nan'), float('inf'), True, 'fast'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.derive([.2, value, .8])
        with self.assertRaises(ValueError):
            self.derive([.2, .2, .8], offset=.1)

    def test_launch_applies_overrides_after_yaml(self):
        path = Path(__file__).resolve().parents[1] / 'launch/nav2.launch.py'
        tree = ast.parse(path.read_text())
        found = set()
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
                continue
            if call.func.id != 'Node':
                continue
            kwargs = {k.arg: k.value for k in call.keywords}
            executable = kwargs.get('executable')
            if not isinstance(executable, ast.Constant):
                continue
            name = executable.value
            if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
                text = ast.unparse(kwargs['parameters'])
                self.assertIn(f"[configured_params, speed_limits['{name}']]", text)
                found.add(name)
        self.assertEqual(found, {'controller_server', 'behavior_server', 'velocity_smoother'})


if __name__ == '__main__':
    unittest.main()
