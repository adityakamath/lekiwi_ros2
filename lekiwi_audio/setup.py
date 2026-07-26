import contextlib
import sys
from glob import glob

from lekiwi_audio.render_phrases import render_missing
from setuptools import find_packages, setup

package_name = 'lekiwi_audio'

# Must run before setup() (its kwargs, incl. data_files' glob below, are evaluated at import
# time) and must not print to stdout (colcon parses setup.py --dry-run's stdout as a dict
# literal).
with contextlib.redirect_stdout(sys.stderr):
    render_missing()

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/sounds', glob('sounds/*.wav')),
    ],
    # Runtime only needs pyyaml - see requirements-build.txt for build-time-only deps.
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Aditya Kamath (Kamath Robotics)',
    maintainer_email='adityakamath@live.com',
    description='Spoken status announcements (e-stop, mode switching, waypoints) for LeKiwi robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'indicator_node = lekiwi_audio.indicator_node:main',
            'render_phrases = lekiwi_audio.render_phrases:main',
        ],
    },
)
