from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

# Lancer manuellement le service =>

def generate_launch_description():
    qbo_arduqbo_dir = get_package_share_directory('qbo_arduqbo')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_arduqbo_dir, 'launch', 'qbo_full.launch.py')
            )
        )
    ])
