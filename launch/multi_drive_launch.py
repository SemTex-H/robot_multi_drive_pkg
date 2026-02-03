from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 3. Joystick Driver Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.1}]
        ),
        # 4. Your Custom Controller Node
        Node(
            package='robot_multi_drive',
            executable='motor_control',
            name='motor_control'
        )
    ])
