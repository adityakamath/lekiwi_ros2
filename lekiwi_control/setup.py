from glob import glob

from setuptools import find_packages, setup

package_name = 'lekiwi_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config/base', glob('config/base/*.yaml')),
        ('share/' + package_name + '/config/payloads/pantilt', glob('config/payloads/pantilt/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Kamath (Kamath Robotics)',
    maintainer_email='adityakamath@live.com',
    description='Control stack for LeKiwi robot: launch, teleop config, and velocity switching',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'twist_switch_node = lekiwi_control.twist_switch_node:main',
        ],
    },
)
