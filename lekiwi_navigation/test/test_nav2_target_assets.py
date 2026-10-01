"""Source asset contracts. Can also run directly without ROS: python this_file.py."""
import ast
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]


class TestTargetAssets(unittest.TestCase):
    def test_button_is_one_shot_setbool(self):
        config = yaml.safe_load((ROOT / 'lekiwi_control/config/base_teleop.yaml').read_text())
        commands = config['joy_teleop']['ros__parameters']
        button = commands['nav2_send_goal']
        self.assertEqual(button['buttons'], [8])
        self.assertEqual(button['service_name'], '/nav2_send_goal')
        for field in ('type', 'interface_type', 'service_request'):
            self.assertEqual(button[field], commands['reset_waypoints'][field])
        self.assertEqual(button['service_request'], {'data': True})
        self.assertNotIn('deadman_buttons', button)
        self.assertFalse(any(8 in command.get('buttons', [])
                             for name, command in commands.items() if name != 'nav2_send_goal'))

    def test_launch_contains_tracker_with_config_and_sim_time(self):
        tree = ast.parse((ROOT / 'lekiwi_navigation/launch/nav2.launch.py').read_text())
        nodes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'Node']
        tracker = [n for n in nodes if any(k.arg == 'executable'
                   and isinstance(k.value, ast.Constant) and k.value.value == 'nav2_target_node'
                   for k in n.keywords)]
        self.assertEqual(len(tracker), 1)
        source = ast.unparse(tracker[0])
        self.assertIn('nav2_target.yaml', source)
        self.assertIn('use_sim_time', source)
        self.assertFalse((ROOT / 'lekiwi_navigation/launch/nav2_target.launch.py').exists())

        launch_source = (ROOT / 'lekiwi_navigation/launch/nav2.launch.py').read_text()
        self.assertNotIn('navigate_to_pose_backend', launch_source)

    def test_config_defaults_match(self):
        config = yaml.safe_load((ROOT / 'lekiwi_navigation/config/nav2/nav2_target.yaml').read_text())
        params = config['nav2_target_node']['ros__parameters']
        tree = ast.parse((ROOT / 'lekiwi_navigation/lekiwi_navigation/nav2_target_node.py').read_text())
        defaults = next(ast.literal_eval(n.value) for n in ast.walk(tree)
                        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
                        and t.id == 'defaults' for t in n.targets))
        self.assertEqual(set(params), set(defaults))
        self.assertGreater(params['marker_scale'], 0)
        self.assertEqual(params['marker_color'], [0.0, 1.0, 0.4, 0.9])
        teleop = yaml.safe_load((ROOT / 'lekiwi_control/config/base_teleop.yaml').read_text())
        axes = teleop['joy_teleop']['ros__parameters']['teleop']['axis_mappings']
        self.assertEqual(params['translation_scale'], 2.0)
        self.assertEqual(params['rotation_scale'], 2.0)
        self.assertEqual(params['translation_scale'] * axes['twist-linear-x']['scale'], 0.4)
        self.assertEqual(params['translation_scale'] * axes['twist-linear-y']['scale'], 0.4)
        self.assertEqual(params['rotation_scale'] * axes['twist-angular-z']['scale'], 1.6)

    def test_audio_phrases(self):
        config = yaml.safe_load((ROOT / 'lekiwi_audio/config/phrases.yaml').read_text())
        phrases = config['services']['/nav2_send_goal']
        self.assertEqual(phrases['true']['success'], 'Sending navigation goal')
        self.assertEqual(phrases['true']['failure'], 'Navigation goal rejected')
        self.assertNotIn('false', phrases)


if __name__ == '__main__':
    unittest.main()
