"""teleop.launch.py loads config/base/base_teleop.yaml plus config/payloads/<name>/<name>_teleop.yaml.

Calls the launch setup function directly with a fake context: no nodes are started.
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node

SOURCE = Path(__file__).resolve().parents[1]


def _parameter_files(**arguments):
    spec = importlib.util.spec_from_file_location('teleop_launch', SOURCE / 'launch/teleop.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({'payload': '', 'use_sim_time': 'false', 'joy': 'false', **arguments})
    node = next(a for a in module.launch_setup(context) if isinstance(a, Node) and a.node_executable == 'joy_teleop')
    return [Path(str(entry._ParameterFile__param_file[0].text)) for entry in node._Node__parameters
            if hasattr(entry, '_ParameterFile__param_file')]


def test_base_only_loads_just_the_base_teleop_file():
    files = _parameter_files()
    assert [f.name for f in files] == ['base_teleop.yaml']
    assert files[0].is_file()


def test_pantilt_payload_adds_its_own_teleop_file_on_top():
    files = _parameter_files(payload='pantilt')
    assert [f.name for f in files] == ['base_teleop.yaml', 'pantilt_teleop.yaml']
    assert all(f.is_file() for f in files)


def test_teleop_files_follow_the_naming_pattern_in_source():
    assert (SOURCE / 'config/base/base_teleop.yaml').is_file()
    assert (SOURCE / 'config/payloads/pantilt/pantilt_teleop.yaml').is_file()
    assert not (SOURCE / 'config/base/teleop.yaml').exists()
    assert not (SOURCE / 'config/payloads/pantilt/teleop.yaml').exists()
