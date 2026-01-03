from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'MJPG',
                'camera_frame_id': 'camera_link',
                'camera_info_url': '',
                'framerate': 15.0,
            }],
        ),
    ])
