"""The plugin is registered under the name robots load it by."""
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
BASE_CLASS = 'mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase'


def test_plugin_description_names_the_class_and_the_upstream_base():
    classes = {c.get('name'): c for c in ET.parse(PACKAGE / 'plugins.xml').getroot().iter('class')}
    assert set(classes) == {'mujoco_ros2_plugins/EmergencyStopPlugin'}
    cls = classes['mujoco_ros2_plugins/EmergencyStopPlugin']
    assert cls.get('type') == 'mujoco_ros2_plugins::EmergencyStopPlugin'
    assert cls.get('base_class_type') == BASE_CLASS
    assert ET.parse(PACKAGE / 'plugins.xml').getroot().get('path') == 'mujoco_ros2_plugins'


def test_the_class_is_exported_and_the_description_is_registered_with_the_loader():
    assert 'MUJOCO_ROS2_PLUGINS_EXPORT(mujoco_ros2_plugins::EmergencyStopPlugin)' in (PACKAGE / 'src/emergency_stop_plugin.cpp').read_text()
    assert BASE_CLASS in (PACKAGE / 'include/mujoco_ros2_plugins/mujoco_ros2_plugins.hpp').read_text()
    assert 'pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)' in \
        (PACKAGE / 'CMakeLists.txt').read_text()


def test_the_package_is_robot_agnostic():
    for path in (*PACKAGE.rglob('*.cpp'), *PACKAGE.rglob('*.hpp'), PACKAGE / 'plugins.xml', PACKAGE / 'CMakeLists.txt'):
        text = path.read_text().lower()
        assert 'lekiwi' not in text and 'pantilt' not in text, path


def test_every_plugin_source_is_built_and_described():
    sources = {f'src/{p.name}' for p in (PACKAGE / 'src').glob('*.cpp')}
    cmake = (PACKAGE / 'CMakeLists.txt').read_text()
    assert all(source in cmake for source in sources)
    assert len(list(ET.parse(PACKAGE / 'plugins.xml').getroot().iter('class'))) == len(sources)
