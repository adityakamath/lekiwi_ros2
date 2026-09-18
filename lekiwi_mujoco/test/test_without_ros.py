"""Exercise standalone entry points with ROS environment and imports unavailable."""
import os
from pathlib import Path
import subprocess
import sys


def test_without_ros(tmp_path):
    package = Path(__file__).resolve().parents[1]
    program = '''
import importlib.abc
import sys
from pathlib import Path

class NoROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy', 'rclcpp', 'ament_index_python',
                                      'launch', 'launch_ros', 'rosbag2_py'}:
            raise AssertionError('Unexpected ROS import: ' + fullname)
sys.meta_path.insert(0, NoROS())
sys.path.insert(0, sys.argv[1])
import mujoco
from lekiwi_mujoco.build_mujoco_models import build
from lekiwi_mujoco.simulation import Simulation
for variant in ('base', 'pt100', 'pt101'):
    path = build(variant, Path(sys.argv[2]) / (variant + '.xml'), absolute=True)
    sim = Simulation(mujoco.MjModel.from_xml_path(str(path)))
    sim.reset()
    sim.command([.1, 0, 0])
    sim.step(20)
    assert sim.data.time > 0
    sim.stop()
    sim.reset()
    assert sim.data.time == 0
assert 'gymnasium' not in sys.modules
print('All three variants generated, stepped, stopped and reset without ROS or Gymnasium')
'''
    environment = {key: os.environ[key] for key in ('HOME', 'PATH', 'TMPDIR', 'SYSTEMROOT') if key in os.environ}
    result = subprocess.run([sys.executable, '-I', '-c', program, str(package), str(tmp_path)],
                            cwd=tmp_path, env=environment, capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
