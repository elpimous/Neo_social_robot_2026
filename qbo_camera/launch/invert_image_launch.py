# launch/invert_image_launch.py
# Caméra œil de Néo : usb_cam + rotation 180°
# Auteur : Vincent FOUCAULT

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Driver USB-Cam
        # (si /dev/qbo_eye_camera existe, le préférer à /dev/video0 : la D435 crée aussi des /dev/video*)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device':    '/dev/video0',
                'image_width':     640,
                'image_height':    480,
                'framerate':       30.0,
                'pixel_format':    'yuyv'
            }]
        ),

        # Rotation 180° de l'image
        Node(
            package='qbo_camera',
            executable='qbo_invert_image',
            name='invert_image_node',
            output='screen',
        ),
    ])
