from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbo_interface',
            executable='smach_qbo',
            name='smach_qbo',
            output='screen'
        )
    ])