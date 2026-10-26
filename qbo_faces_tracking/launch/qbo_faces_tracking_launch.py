from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbo_faces_tracking',
            executable='qbo_faces_tracking_node',
            name='qbo_face_tracker',
            output='screen',
            remappings=[
                ('/face_roi', '/target_roi')  # <-- redirection du ROI
            ],
            parameters=[]  # tu peux mettre ici des params YAML si besoin
        )
    ])