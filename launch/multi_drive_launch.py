from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick Driver Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.1}]
        ),
        # Your Custom Controller Node
        Node(
            package='robot_multi_drive',
            executable='robot_control',
            name='robot_control_node'
        ),
        # Recorder node
        Node(
            package='robot_multi_drive',
            executable='cam_record',
            name='cam_record_node'
        )
    ])
