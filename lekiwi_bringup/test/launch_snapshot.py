"""Expand launch files without starting anything and describe the nodes they would create."""
import importlib.util
import json
import re
import tempfile
from pathlib import Path

import yaml

import launch.actions
import launch_ros.actions
import launch_ros.descriptions
from launch_ros.descriptions import ParameterFile
from launch import LaunchContext, LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction

TMP = tempfile.gettempdir()
RECORDED = []
SHARE = r'/[^\s\'"]*?/share/(\w+)'


def _describe(value):
    """Reduce launch objects to plain, stable JSON data."""
    if isinstance(value, (str, int, float, bool)) or value is None:
        return _scrub(value) if isinstance(value, str) else value
    if isinstance(value, ParameterFile):
        source = value._ParameterFile__param_file
        if all(hasattr(part, 'text') for part in source):
            return _describe(''.join(part.text for part in source))
        return _describe(source)
    if isinstance(value, Substitution):
        return f'<{type(value).__name__}: {value.describe()}>'
    if isinstance(value, dict):
        return {str(_describe(k)): _describe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, type({}.items()))):
        return [_describe(v) for v in value]
    if isinstance(value, Path):
        return _scrub(str(value))
    if hasattr(value, '_LaunchDescriptionSource__location'):
        return _describe(value._LaunchDescriptionSource__location)
    return f'<{type(value).__name__}>'


def _scrub(text):
    """Replace temporary file paths with their parsed YAML content."""
    if text.startswith(TMP) and text.endswith('.yaml') and Path(text).is_file():
        return {'<temp yaml>': yaml.safe_load(Path(text).read_text())}
    text = re.sub(re.escape(TMP) + r'/[\w.\-]+', '<TMP>', text)
    return re.sub(SHARE, r'<share:\1>', text)


def _recorder(base, kind):
    class Recording(base):
        def __init__(self, *args, **kwargs):
            RECORDED.append({'kind': kind, 'args': _describe(args), 'kwargs': _describe(kwargs)})
            super().__init__(*args, **kwargs)
    Recording.__name__ = base.__name__
    return Recording


def _patch():
    saved = {}
    for owner, name in [
        (launch_ros.actions, 'Node'), (launch_ros.actions, 'LifecycleNode'),
        (launch_ros.actions, 'ComposableNodeContainer'), (launch_ros.actions, 'LoadComposableNodes'), (launch_ros.actions, 'SetParameter'),
        (launch_ros.descriptions, 'ComposableNode'), (launch.actions, 'IncludeLaunchDescription'),
    ]:
        saved[(owner, name)] = getattr(owner, name)
        setattr(owner, name, _recorder(saved[(owner, name)], name))
    return saved


def _restore(saved):
    for (owner, name), original in saved.items():
        setattr(owner, name, original)


def _walk(entities, context):
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            result = entity.execute(context)
            _walk(result or [], context)
        elif hasattr(entity, 'get_sub_entities'):
            _walk(entity.get_sub_entities(), context)


def snapshot(launch_file, overrides=None):
    """Return a JSON-ready description of every node-like action a launch file builds."""
    saved = _patch()
    RECORDED.clear()
    try:
        spec = importlib.util.spec_from_file_location('snapshot_' + launch_file.stem.replace('.', '_'), launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        description = module.generate_launch_description()
        assert isinstance(description, LaunchDescription)
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        context.launch_configurations.update(overrides or {})
        _walk(description.entities, context)
        text = json.dumps(list(RECORDED), default=str)
        return json.loads(re.sub(SHARE, r'<share:\1>', text))
    finally:
        _restore(saved)
