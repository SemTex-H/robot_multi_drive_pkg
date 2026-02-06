from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick Driver Node
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     parameters=[{'deadzone': 0.1}]
        # ),
        # Custom Controller Node
        Node(
            package='robot_multi_drive',
            executable='robot_control',
            name='robot_control_node'
        ),
        # camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='rear_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'io_method': 'mmap',
                'pixel_format': 'mjpeg',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0
            }]
        ),
        # Recorder node
        Node(
            package='robot_multi_drive',
            executable='cam_record',
            name='cam_record_node'
        )
    ])
