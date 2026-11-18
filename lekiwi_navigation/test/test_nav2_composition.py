"""use_composition must change how the Nav2 nodes run, never which nodes, parameters or remaps."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.substitutions import FindPackageShare

PACKAGES = Path(__file__).resolve().parents[2]
PLAIN_NODES = {'waypoint_recorder_node', 'nav2_target_node'}


def _text(value):
    if hasattr(value, 'text'):
        return value.text
    if isinstance(value, (list, tuple)):
        return '(' + ','.join(_text(item) for item in value) + ')'
    return str(value)


def _entities(monkeypatch, composition):
    monkeypatch.setattr(FindPackageShare, 'find', lambda self, package: str(PACKAGES / package))
    path = PACKAGES / 'lekiwi_navigation/launch/nav2.launch.py'
    spec = importlib.util.spec_from_file_location('nav2_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations['use_composition'] = 'true' if composition else 'false'
    opaque = next(e for e in entities if isinstance(e, OpaqueFunction))
    group = next(a for a in opaque.execute(context) if isinstance(a, GroupAction))
    return group.get_sub_entities()


def _name(value):
    return _text(value).strip('()') if value is not None else 'controller_server'  # nav2 default


def test_composed_components_match_the_process_nodes(monkeypatch):
    processes = [e for e in _entities(monkeypatch, False) if isinstance(e, Node)]
    composed = _entities(monkeypatch, True)
    components = next(e for e in composed if isinstance(e, LoadComposableNodes)
                      )._LoadComposableNodes__composable_node_descriptions
    by_name = {_name(d._ComposableNode__node_name): d for d in components}
    nav2 = {_name(n._Node__node_name): n for n in processes
            if _text(n._Node__package).startswith('nav2_')}
    assert set(by_name) == set(nav2)
    for name, node in nav2.items():
        component = by_name[name]
        assert _text(component._ComposableNode__package).strip('()') == _text(node._Node__package).strip('()')
        assert _text(list(component._ComposableNode__remappings or [])) == \
            _text(list(node._Node__remappings or []))
        assert len(component._ComposableNode__parameters) == len(node._Node__parameters)


def test_plain_nodes_are_never_composed(monkeypatch):
    for composition in (False, True):
        names = {_name(e._Node__node_name) for e in _entities(monkeypatch, composition)
                 if isinstance(e, Node)}
        assert PLAIN_NODES <= names


def test_composed_mode_has_one_container_and_no_nav2_processes(monkeypatch):
    nodes = [e for e in _entities(monkeypatch, True) if isinstance(e, Node)]
    packages = [_text(n._Node__package) for n in nodes]
    assert packages.count('rclcpp_components') == 1
    assert not [p for p in packages if p.startswith('nav2_')]


def test_container_gets_the_params_file_for_the_nested_costmaps(monkeypatch):
    container = next(e for e in _entities(monkeypatch, True) if isinstance(e, Node)
                     and _text(e._Node__package) == 'rclcpp_components')
    assert container._Node__parameters, 'costmaps inside the container would run on defaults'
