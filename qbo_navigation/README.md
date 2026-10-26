
# Qbo Navigation (ROS2 Humble)

Package complet pour :
- Création de carte (SLAM)
- Localisation EKF (IMU + Odom)
- Navigation autonome (Nav2)

Compatible avec les topics du robot Q.bo.

Topics utilisés :

/scan  
/qbo_arduqbo/base_ctrl/cmd_vel  
/qbo_arduqbo/base_ctrl/odom  
/cmd_joints  
/joint_states  
/qbo_arduqbo/imu_ctrl/imu_state/data  
/qbo_arduqbo/sens_ctrl/distance_sensors_state/floor_sensor  

---

# 1 Installation des dépendances

ROS2 Humble doit être installé.

Installer les packages nécessaires :

sudo apt update

sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization ros-humble-tf-transformations ros-humble-tf2-ros ros-humble-rviz2 ros-humble-teleop-twist-keyboard

---

# 2 Installation du package

Créer un workspace :

mkdir -p ~/qbo_ws/src
cd ~/qbo_ws/src

Copier le dossier qbo_navigation ici.

Compiler :

cd ~/qbo_ws
colcon build

Source :

source install/setup.bash

---

# 3 Vérifier les topics du robot

ros2 topic list

Les topics suivants doivent exister :

/scan  
/qbo_arduqbo/base_ctrl/odom  
/qbo_arduqbo/base_ctrl/cmd_vel  

---

# 4 Création de carte (SLAM)

Lancer :
ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
ros2 launch qbo_navigation localization.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0.20 0 0 0 base_pan_link laser
ros2 launch qbo_navigation map_creation.launch.py

Dans un autre terminal :

ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r cmd_vel:=/qbo_arduqbo/base_ctrl/cmd_vel


Piloter le robot pour construire la carte.

Sauvegarder la carte :

ros2 run nav2_map_server map_saver_cli -f ~/map

---

# 5 Localisation EKF

Fusion IMU + Odom :

ros2 launch qbo_navigation localization.launch.py

Le topic publié sera :

/odometry/filtered

---

# 6 Navigation autonome

ros2 launch qbo_navigation navigation.launch.py

Puis ouvrir RViz :

rviz2

Ajouter :
- Map
- TF
- LaserScan
- RobotModel

Utiliser "2D Goal Pose" pour envoyer un objectif.

---

# 7 TF nécessaires

Le robot doit publier :

map → odom  
odom → base_link  
base_link → laser  

Vérifier avec :

ros2 run tf2_tools view_frames

---

# 8 Problèmes fréquents

slam_toolbox not found

Installer :

sudo apt install ros-humble-slam-toolbox

config/slam.yaml not found

Le launch utilise maintenant un chemin absolu via :
get_package_share_directory()

---

# 9 Conseils pour Q.bo

Le laser peut être monté sur la tête.

Pour de meilleures cartes :
- garder la tête fixe
- vitesse lente
- éviter les rotations rapides

