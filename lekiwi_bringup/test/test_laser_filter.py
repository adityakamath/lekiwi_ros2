"""laser.launch.py starts the base LiDAR.

Calls the launch setup function directly with a fake context: no nodes are started.
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node

SOURCE = Path(__file__).resolve().parents[1]


def _nodes(**arguments):
    spec = importlib.util.spec_from_file_location('laser_launch', SOURCE / 'launch/laser.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({'custom_filter': '', **arguments})
    return {node.node_executable: node for node in module.launch_setup(context) if isinstance(node, Node)}


def _filter_file(node):
    return str(node._Node__parameters[0]._ParameterFile__param_file[0].text)




def test_base_only_has_no_filter():
    assert set(_nodes()) == {'ldlidar_ros2_node'}




def test_explicit_custom_filter(tmp_path):
    config = tmp_path / 'filter.yaml'
    config.write_text('laser_scan_filter_chain: {ros__parameters: {}}')
    nodes = _nodes(custom_filter=str(config))
    assert set(nodes) == {'ldlidar_ros2_node', 'scan_to_scan_filter_chain'}
    assert Path(_filter_file(nodes['scan_to_scan_filter_chain'])) == config


def test_missing_custom_filter_is_rejected():
    import pytest
    with pytest.raises(RuntimeError, match='not found'):
        _nodes(custom_filter='/nonexistent/filter.yaml')
