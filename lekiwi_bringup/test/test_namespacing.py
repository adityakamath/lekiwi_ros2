"""One namespacing code path: every test runs for the empty namespace and for a named one.

Expected names and frames are computed from the namespace, so an empty namespace must give
exactly the unprefixed names the robot always had and a named one must prefix every one of them.
"""
import importlib.util
import json
import os
import re
import tempfile
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError
import yaml

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.utilities import evaluate_parameters

from launch_snapshot import snapshot

ROOT = Path(__file__).resolve().parents[2]
UNUSED_MODEL = {'mujoco_model': '/unused/model.xml'}
CASES = {
    'lekiwi': ('lekiwi_bringup/launch/lekiwi.launch.py', {'use_mock': 'true'}),
    'lekiwi_sim': ('lekiwi_bringup/launch/lekiwi.launch.py', {'sim': 'true', **UNUSED_MODEL}),
    'laser': ('lekiwi_bringup/launch/laser.launch.py', {}),
    'tof': ('lekiwi_bringup/launch/tof.launch.py', {}),
    'gemini2': ('lekiwi_bringup/launch/gemini2.launch.py', {'enable_camera': 'true'}),
    'control': ('lekiwi_control/launch/control.launch.py', {'use_mock': 'true', 'diagnostics': 'true'}),
    'control_sim': ('lekiwi_control/launch/control.launch.py',
                    {'ros2_control_hardware_type': 'mujoco', 'pointcloud': 'true', **UNUSED_MODEL}),
    'teleop': ('lekiwi_control/launch/teleop.launch.py', {'joy': 'true'}),
    'ekf': ('lekiwi_navigation/launch/ekf.launch.py', {}),
    'slam': ('lekiwi_navigation/launch/slam.launch.py', {}),
    'nav2': ('lekiwi_navigation/launch/nav2.launch.py', {}),
    'nav2_processes': ('lekiwi_navigation/launch/nav2.launch.py', {'use_composition': 'false'}),
}
CONFIGS = [
    *sorted(str(p.relative_to(ROOT)) for pattern in (
        'lekiwi_bringup/config/*.yaml', 'lekiwi_control/config/*.yaml',
        'lekiwi_navigation/config/*/*.yaml', 'lekiwi_mujoco/config/mujoco_laser_filter.yaml')
        for p in ROOT.glob(pattern)),
]


@pytest.fixture(params=['', 'robot1'], ids=['no_namespace', 'robot1'])
def ns(request):
    return request.param


def prefix(ns):
    return f'{ns}/' if ns else ''


def root(ns):
    return f'/{ns}' if ns else ''


def records(case, ns, **extra):
    path, overrides = CASES[case]
    # What lekiwi.launch.py and PushRosNamespace set for the files it includes.
    context = {'namespace': ns, 'frame_prefix': prefix(ns), 'ros_namespace': os.path.join('/', ns)}
    return snapshot(ROOT / path, {**overrides, **context, **extra})


def nodes(case, ns, **extra):
    return [r['kwargs'] for r in records(case, ns, **extra) if r['kind'] in ('Node', 'LifecycleNode')]


def node(case, ns, name, package=None, **extra):
    (found,) = [n for n in nodes(case, ns, **extra) if (n.get('name') == name or n.get('executable') == name)
                and package in (None, n.get('package'))]
    return found


def dicts(parameters):
    """The plain dictionary parameters of a node, merged."""
    merged = {}
    for item in parameters:
        if isinstance(item, dict) and '<temp yaml>' not in item:
            merged.update(item)
    return merged


def yamls(parameters):
    """The contents of the generated parameter files of a node, merged by node key."""
    merged = {}
    for item in parameters:
        if isinstance(item, dict) and '<temp yaml>' in item:
            merged.update(item['<temp yaml>'])
    return merged


def remaps(kwargs):
    return [tuple(r) for r in kwargs.get('remappings') or []]


def walk(value):
    if isinstance(value, dict):
        for key, item in value.items():
            yield key, item
            yield from walk(item)
    elif isinstance(value, list):
        for item in value:
            yield from walk(item)


# ── configuration guards ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize('config', CONFIGS)
def test_params_files_match_a_node_in_any_namespace(config):
    """A bare top-level key matches only a node in the root namespace."""
    content = yaml.safe_load((ROOT / config).read_text())
    if 'ros__parameters' in json.dumps(content):
        assert all(key.startswith('/') for key in content), f'{config}: use /**/<node> keys'


@pytest.mark.parametrize('config', CONFIGS)
def test_configured_names_are_relative(config):
    """An absolute topic or service name ignores the robot namespace."""
    content = yaml.safe_load((ROOT / config).read_text())
    absolute = [(key, value) for key, value in walk(content)
                if isinstance(value, str) and value.startswith('/')
                and re.search(r'(topic|service|target_node)$', str(key))]
    assert not absolute, f'{config}: {absolute}'


def test_generated_parameter_files_have_no_yaml_aliases(ns):
    """rcl cannot parse '&id001' anchors, which yaml.dump writes when one dict is reused."""
    before = set(Path(tempfile.gettempdir()).glob('*.yaml'))
    for case in ('nav2', 'nav2_processes', 'control_sim', 'lekiwi_sim'):
        records(case, ns)
    created = set(Path(tempfile.gettempdir()).glob('*.yaml')) - before
    assert created
    assert [p for p in created if re.search(r'[&*]id\d+', p.read_text())] == []


# ── lekiwi.launch.py ─────────────────────────────────────────────────────────────────

def _lekiwi_module():
    spec = importlib.util.spec_from_file_location('lekiwi_launch_ns', ROOT / 'lekiwi_bringup/launch/lekiwi.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _lekiwi_context(module, **configurations):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update({'sim': 'true', **UNUSED_MODEL, **configurations})
    return context


def test_robot_is_always_launched_inside_a_namespace_group(ns):
    module = _lekiwi_module()
    (group,) = module.launch_setup(_lekiwi_context(module, namespace=ns))
    assert isinstance(group, GroupAction)
    pushes = [e for e in group.get_sub_entities() if isinstance(e, PushRosNamespace)]
    assert len(pushes) == 1


@pytest.mark.parametrize('namespace', ['/robot1/', 'robot_1', 'a/b'])
def test_slashes_around_the_namespace_are_ignored(namespace):
    module = _lekiwi_module()
    assert module.launch_setup(_lekiwi_context(module, namespace=namespace))


@pytest.mark.parametrize('namespace', ['not a name', '1robot', 'robot-1'])
def test_invalid_namespace_is_rejected(namespace):
    module = _lekiwi_module()
    with pytest.raises(RuntimeError, match='valid ROS name'):
        module.launch_setup(_lekiwi_context(module, namespace=namespace))


@pytest.mark.parametrize('path', [
    'modules/ina260_battery_monitor/launch/battery_monitor.launch.py',
    'modules/sts_hardware_interface/launch/motor_diagnostics.launch.py'])
def test_submodule_headers_follow_the_frame_prefix(path, ns):
    """The battery and motor diagnostics launches name their header frame from the shared frame_prefix."""
    spec = importlib.util.spec_from_file_location('submodule_launch', ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        description = module.generate_launch_description()
    except PackageNotFoundError as error:
        pytest.skip(f'{error} (submodule package is not built in this environment)')
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations['frame_prefix'] = prefix(ns)
    node = next(e for e in description.entities if isinstance(e, Node))
    parameters = {}
    for item in evaluate_parameters(context, node._Node__parameters):
        if isinstance(item, dict):
            parameters.update(item)
    assert parameters['frame_id'] == f'{prefix(ns)}base_link'


# ── control.launch.py ────────────────────────────────────────────────────────────────

def test_control_frames_and_controller_manager(ns):
    p = prefix(ns)
    spawners = [n for n in nodes('control_sim', ns) if n.get('executable') == 'spawner']
    assert spawners
    # They start outside the namespace scope, so each carries the namespace and names the manager relatively.
    assert all(n['arguments'][n['arguments'].index('-c') + 1] == 'controller_manager' for n in spawners)
    assert all(n['namespace'] == os.path.join('/', ns) for n in spawners)
    imu = next(n['arguments'] for n in spawners if n['arguments'][0] == 'gemini2_imu_broadcaster')
    assert imu[imu.index('--controller-ros-args') + 1] == '--ros-args -r ~/imu:=gemini2/gyro_accel/sample'
    assert dicts(node('control_sim', ns, 'robot_state_publisher')['parameters'])['frame_prefix'] == p
    assert dicts(node('control_sim', ns, 'tof_pointcloud')['parameters'])['frame_id'] == f'{p}tof_frame'
    assert dicts(node('control_sim', ns, 'control_support_node')['parameters'])['frame_id'] == f'{p}base_link'


def test_controller_sensor_frames(ns):
    p = prefix(ns)
    controllers = yamls(node('control_sim', ns, 'ros2_control_node', 'mujoco_ros2_control')['parameters'])
    assert controllers['/**/imu_sensor_broadcaster']['ros__parameters']['frame_id'] == f'{p}imu_frame'
    assert controllers['/**/gemini2_imu_broadcaster']['ros__parameters']['frame_id'] == (
        f'{p}gemini2_accel_gyro_optical_frame')
    assert 'enable_odom_tf' in controllers['/**/base_controller']['ros__parameters']


def test_simulated_sensors_use_this_robots_topics_and_frames(ns):
    p, r = prefix(ns), root(ns)
    parameters = node('control_sim', ns, 'ros2_control_node', 'mujoco_ros2_control')['parameters']
    plugins = next(y['/**']['ros__parameters']['mujoco_plugins'] for y in
                   (item['<temp yaml>'] for item in parameters if isinstance(item, dict) and '<temp yaml>' in item)
                   if '/**' in y and 'mujoco_plugins' in json.dumps(y) and 'emergency_stop_plugin' in json.dumps(y))
    assert plugins == {
        'emergency_stop_plugin': {'service_name': f'{r}/emergency_stop'},
        'mujoco_3d_lidar_plugin': {
            'lidar': {'frame_name': f'{p}laser_frame', 'topic': f'{r}/scan_raw'},
            'tof': {'frame_name': f'{p}tof_frame', 'topic': f'{r}/tof/pointcloud_raw'}},
        'mujoco_camera_plugin': {'gemini2_rgb': {
            'frame_name': f'{p}gemini2_color_optical_frame',
            'info_topic': f'{r}/gemini2/color/camera_info', 'image_topic': f'{r}/gemini2/color/image_raw',
            'depth_topic': f'{r}/_gemini2/depth_raw'}},
    }


def test_simulated_camera_nodes(ns):
    p = prefix(ns)
    assert 'republish' not in [n.get('executable') for n in nodes('control_sim', ns)]  # It aborts in a namespace.
    cloud = node('control_sim', ns, 'gemini2_colored_pointcloud')
    assert ('points', '_gemini2/registered_points') in remaps(cloud)
    optical = node('control_sim', ns, 'gemini2_optical_frame_publisher')['arguments']
    assert optical[:4] == ['--frame-id', f'{p}gemini2_link', '--child-frame-id', f'{p}gemini2_color_optical_frame']
    imu = node('control_sim', ns, 'gemini2_imu_tf')['arguments']
    assert imu == ['--frame-id', f'{p}gemini2_color_optical_frame', '--child-frame-id', f'{p}gemini2_accel_gyro_optical_frame']


def test_controller_manager_is_not_remapped(ns):
    """ros2_control deprecates remapping arguments on the controller_manager node and warns about them."""
    for package, case in (('controller_manager', 'control'), ('mujoco_ros2_control', 'control_sim')):
        assert remaps(node(case, ns, 'ros2_control_node', package)) == []


def test_teleop_header_frame(ns):
    parameters = dicts(node('teleop', ns, 'joy_teleop')['parameters'])
    assert parameters['teleop.axis_mappings.header-frame_id.value'] == f'{prefix(ns)}base_link'


# ── sensors ──────────────────────────────────────────────────────────────────────────

def test_sensor_driver_frames(ns):
    assert dicts(node('laser', ns, 'laser_node')['parameters'])['frame_id'] == f'{prefix(ns)}laser_frame'
    assert dicts(node('tof', ns, 'tof_imager')['parameters'])['frame_id'] == f'{prefix(ns)}tof_frame'


def test_gemini2_driver_publishes_prefixed_tf_for_every_stream(ns):
    """The driver names RGB, depth, cloud and IMU frames itself; one prefix puts them all under the robot."""
    found = records('gemini2', ns)
    driver = next(r for r in found if r['kind'] == 'IncludeLaunchDescription')
    arguments = dict(tuple(a) for a in driver['kwargs']['launch_arguments'])
    assert (arguments['publish_tf'], arguments['tf_prefix']) == ('true', prefix(ns))
    assert not [r for r in found if r['kwargs'].get('name') == 'gemini2_optical_frame_publisher']


# ── navigation ───────────────────────────────────────────────────────────────────────

def test_ekf_frames_and_diagnostics(ns):
    p = prefix(ns)
    ekf = node('ekf', ns, 'ekf_node')
    assert dicts(ekf['parameters']) == {
        'use_sim_time': False, 'map_frame': f'{p}map', 'odom_frame': f'{p}odom',
        'base_link_frame': f'{p}base_footprint', 'world_frame': f'{p}odom'}
    assert ('/diagnostics', 'diagnostics') in remaps(ekf)


def test_slam_frames_and_map_topics(ns):
    p = prefix(ns)
    slam = node('slam', ns, 'slam_toolbox')
    assert {'odom_frame': f'{p}odom', 'map_frame': f'{p}map', 'base_frame': f'{p}base_footprint'}.items() \
        <= dicts(slam['parameters']).items()
    assert {('/map', 'map'), ('/map_metadata', 'map_metadata'), ('/diagnostics', 'diagnostics')} <= set(remaps(slam))
    assert ('/diagnostics', 'diagnostics') in remaps(node('slam', ns, 'lifecycle_manager_slam'))
    saver = dicts(node('slam', ns, 'map_saver_node')['parameters'])
    assert (saver['map_frame'], saver['base_frame']) == (f'{p}map', f'{p}base_footprint')


@pytest.mark.parametrize('case', ['nav2', 'nav2_processes'])
def test_nav2_frames_topics_and_tf(case, ns):
    p, r = prefix(ns), root(ns)
    found = records(case, ns)
    assert '/tf' not in json.dumps(found), 'all robots share the global /tf'
    managers = [x['kwargs'] for x in found if str(x['kwargs'].get('name', '')).startswith('lifecycle_manager')]
    assert len(managers) == 3
    assert all(('/diagnostics', 'diagnostics') in remaps(m) for m in managers)
    plain = {n['name']: dicts(n['parameters']) for n in nodes(case, ns) if n.get('package') == 'lekiwi_navigation'}
    assert plain['waypoint_recorder_node'] == {'frame_id': f'{p}map', 'base_frame': f'{p}base_footprint'}
    assert plain['nav2_target_node'] == {
        'use_sim_time': False, 'map_frame': f'{p}map', 'base_frame': f'{p}base_footprint',
        'target_frame': f'{p}nav2_target'}
    if case == 'nav2':
        target = next(x['kwargs']['target_container'] for x in found if x['kind'] == 'LoadComposableNodes')
        assert target == f'{r}/nav2_container'


def test_nav2_override_file_matches_this_robot(ns):
    p = prefix(ns)
    container = next(n for n in nodes('nav2', ns) if n.get('name') == 'nav2_container')
    overrides = next(item for item in container['parameters'] if isinstance(item, dict) and '<temp yaml>' in item
                     and '/**/bt_navigator' in item['<temp yaml>'])['<temp yaml>']
    assert overrides == {
        '/**/bt_navigator': {'ros__parameters': {'robot_base_frame': f'{p}base_footprint', 'global_frame': f'{p}map'}},
        '/**/behavior_server': {'ros__parameters': {
            'robot_base_frame': f'{p}base_footprint', 'global_frame': f'{p}map', 'local_frame': f'{p}odom'}},
        '/**/collision_monitor': {'ros__parameters': {
            'base_frame_id': f'{p}base_footprint', 'odom_frame_id': f'{p}odom'}},
        '/**/local_costmap': {'local_costmap': {'ros__parameters': {
            'robot_base_frame': f'{p}base_footprint', 'global_frame': f'{p}odom'}}},
        '/**/global_costmap': {'global_costmap': {'ros__parameters': {
            'robot_base_frame': f'{p}base_footprint', 'global_frame': f'{p}map'}}},
    }
