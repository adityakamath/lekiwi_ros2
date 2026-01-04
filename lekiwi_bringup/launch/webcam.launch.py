from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'webcam_config.yaml'
    ])
    
    camera_info_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_bringup'),
        'config',
        'webcam_calibration.yaml'
    ])
    
    # Create the file:// URL using PythonExpression
    camera_info_url = ['file://', camera_info_file]
    
    webcam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='webcam_node',
        output='log',
        respawn=True,
        parameters=[
            config_file,
            {'camera_info_url': camera_info_url}
        ]
    )

    return LaunchDescription([
        webcam_node
    ])