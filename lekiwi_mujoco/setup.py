from pathlib import Path
from setuptools import find_packages, setup

package_name = 'lekiwi_mujoco'
assets = []
for folder in ('mjcf', 'config', 'docs'):
    for directory in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        assets.append(('share/' + package_name + '/' + str(directory),
                       [str(p) for p in sorted(directory.iterdir()) if p.is_file()]))

setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml', 'README.md', 'requirements.txt']), *assets],
    install_requires=['setuptools', 'mujoco==3.13.0', 'numpy>=1.26,<3', 'PyYAML>=6,<7',
                      'xacro>=2.0,<3'],
    extras_require={'gym': ['gymnasium>=1.2,<2']},
    zip_safe=False, license='Apache-2.0',
    maintainer='Aditya Kamath (Kamath Robotics)', maintainer_email='adityakamath@live.com',
    description='LeKiwi MuJoCo simulation infrastructure without a ROS runtime requirement',
    entry_points={'console_scripts': [
        'build_mujoco_models = lekiwi_mujoco.build_mujoco_models:main',
        'mujoco_preview = lekiwi_mujoco.mujoco_preview:main',
        'benchmark_mujoco = lekiwi_mujoco.benchmark_mujoco:main',
    ]},
)
