from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directories
    lekiwi_bringup_share = FindPackageShare('lekiwi_bringup')
    lekiwi_base_control_share = FindPackageShare('lekiwi_base_control')

    # Include base control launch file
    base_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_base_control_share, '/launch/base_control.launch.py'
        ]),
        launch_arguments={'log_level': 'warn'}.items()
    )
    
    # Include webcam launch file
    webcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_bringup_share, '/launch/webcam.launch.py'
        ]),
        launch_arguments={'log_level': 'warn'}.items()
    )
    
    # Include laser scanner launch file
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            lekiwi_bringup_share, '/launch/laser.launch.py'
        ]),
        launch_arguments={'log_level': 'warn'}.items()
    )

    return LaunchDescription([
        base_control_launch,
        webcam_launch,
        laser_launch,
    ])