"""laser.launch.py picks the payload's scan filter from config/<payload>_laser_filter.yaml.

Calls the launch setup function directly with a fake context: no nodes are started.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node

SOURCE = Path(__file__).resolve().parents[1]


def _nodes(**arguments):
    spec = importlib.util.spec_from_file_location('laser_launch', SOURCE / 'launch/laser.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({'payload': '', 'custom_filter': '', **arguments})
    return {node.node_executable: node for node in module.launch_setup(context) if isinstance(node, Node)}


def _filter_file(node):
    return str(node._Node__parameters[0]._ParameterFile__param_file[0].text)


def test_pantilt_payload_uses_its_filter_and_remaps_the_raw_scan():
    nodes = _nodes(payload='pantilt')
    assert set(nodes) == {'ldlidar_ros2_node', 'scan_to_scan_filter_chain'}
    assert Path(_filter_file(nodes['scan_to_scan_filter_chain'])).name == 'pantilt_laser_filter.yaml'


def test_base_only_has_no_filter():
    assert set(_nodes()) == {'ldlidar_ros2_node'}


def test_filter_file_ships_in_config_root_not_a_payload_folder():
    assert (SOURCE / 'config/pantilt_laser_filter.yaml').is_file()
    assert not (SOURCE / 'config/payloads').exists()
    share = Path(get_package_share_directory('lekiwi_bringup'))
    assert (share / 'config/pantilt_laser_filter.yaml').is_file()
    assert not (share / 'config/payloads').exists()
