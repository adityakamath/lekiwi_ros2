from pathlib import Path
from setuptools import find_packages, setup

package_name = 'lekiwi_mujoco'
assets = []
for folder in ('mjcf',):
    for directory in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        assets.append(('share/' + package_name + '/' + str(directory),
                       [str(p) for p in sorted(directory.iterdir()) if p.is_file()]))

setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']), *assets],
    install_requires=['setuptools'],
    zip_safe=False, license='Apache-2.0',
    maintainer='Aditya Kamath (Kamath Robotics)', maintainer_email='adityakamath@live.com',
    description='MuJoCo models for LeKiwi',
)
