from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    qbo_arduqbo_dir = get_package_share_directory('qbo_arduqbo')
    qbo_camera_dir = get_package_share_directory('qbo_camera')

    return LaunchDescription([
        # Launch principal du robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_arduqbo_dir, 'launch', 'qbo_full.launch.py')
            )
        ),

        # Ajout du launch pour inversion d'image
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_camera_dir, 'launch', 'invert_image_launch.py')
            )
        ),
    ])