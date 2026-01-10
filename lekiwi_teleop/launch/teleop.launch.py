from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('lekiwi_teleop')

    # Configuration files
    joy_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'joy_config.yaml'
    ])

    teleop_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'teleop_config.yaml'
    ])

    # Joy node - publishes sensor_msgs/Joy messages
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_config]
    )

    # Teleop twist joy node - converts joy messages to cmd_vel
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_config]
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node,
    ])
