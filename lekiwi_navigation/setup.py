from glob import glob

from setuptools import find_packages, setup

package_name = 'lekiwi_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config/nav2', glob('config/nav2/*.yaml')),
        ('share/' + package_name + '/config/robot_localization', glob('config/robot_localization/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Kamath (Kamath Robotics)',
    maintainer_email='adityakamath@live.com',
    description='Navigation stack for LeKiwi robot: sensor fusion and SLAM',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'map_saver_node = lekiwi_navigation.map_saver_node:main',
            'collision_toggle_node = lekiwi_navigation.collision_toggle_node:main',
        ],
    },
)
