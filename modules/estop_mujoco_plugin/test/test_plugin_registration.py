"""The plugin is registered under the name robots load it by."""
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
BASE_CLASS = 'mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase'


def test_plugin_description_names_the_class_and_the_upstream_base():
    classes = {c.get('name'): c for c in ET.parse(PACKAGE / 'plugins.xml').getroot().iter('class')}
    assert set(classes) == {'estop_mujoco_plugin/EmergencyStopPlugin'}
    cls = classes['estop_mujoco_plugin/EmergencyStopPlugin']
    assert cls.get('type') == 'estop_mujoco_plugin::EmergencyStopPlugin'
    assert cls.get('base_class_type') == BASE_CLASS
    assert ET.parse(PACKAGE / 'plugins.xml').getroot().get('path') == 'estop_mujoco_plugin'


def test_the_class_is_exported_and_the_description_is_registered_with_the_loader():
    assert 'PLUGINLIB_EXPORT_CLASS' in (PACKAGE / 'src/emergency_stop_plugin.cpp').read_text()
    assert 'pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)' in \
        (PACKAGE / 'CMakeLists.txt').read_text()


def test_the_package_is_robot_agnostic():
    for path in (*PACKAGE.rglob('*.cpp'), *PACKAGE.rglob('*.hpp'), PACKAGE / 'plugins.xml', PACKAGE / 'CMakeLists.txt'):
        text = path.read_text().lower()
        assert 'lekiwi' not in text and 'pantilt' not in text, path
