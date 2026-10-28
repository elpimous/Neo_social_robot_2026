#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_moving_laser_on_waypoints.launch.py
========================================
Auteur : Vincent FOUCAULT

Néo (QBo) — test d'endurance : navigation autonome au laser sur waypoints.

Lance :
  * (option) ton bringup robot (base arduqbo, TF, tête Dynamixel)  -> arg robot_bringup:=
  * (option) le driver RPLidar A2                                  -> arg start_lidar_driver:=
  * map_server + AMCL
  * Nav2 : planner Smac2D (centrage), contrôleur RPP, behaviors, BT navigator,
           velocity_smoother, collision_monitor -> cmd_vel de la base
  * waypoint_mission : tête verrouillée, lidar marche/arrêt, pauses, 2 h
  * (option) RViz

Usage :
  ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py
  ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py duration_min:=120 use_rviz:=true

Prérequis : qbo_startup.launch.py actif (base arduqbo + tête qbo_dynamixel),
et qbo_navigation/navigation.launch.py ARRÊTÉ (ce launch le remplace).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('qbo_nav_waypoints')
    # Carte de la maison : celle de qbo_navigation (map.yaml), sinon maps/ de ce package
    try:
        default_map = os.path.join(get_package_share_directory('qbo_navigation'), 'map', 'map.yaml')
    except Exception:
        default_map = os.path.join(pkg, 'maps', 'map.yaml')

    # ------------------------------------------------------------------ args
    map_yaml = LaunchConfiguration('map')
    waypoints = LaunchConfiguration('waypoints')
    nav2_params = LaunchConfiguration('nav2_params')
    mission_params = LaunchConfiguration('mission_params')
    duration_min = LaunchConfiguration('duration_min')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    robot_bringup = LaunchConfiguration('robot_bringup')
    start_lidar_driver = LaunchConfiguration('start_lidar_driver')
    lidar_executable = LaunchConfiguration('lidar_executable')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    tilt_level = LaunchConfiguration('tilt_level_rad')
    head_cx = LaunchConfiguration('head_center_x')
    head_cz = LaunchConfiguration('head_center_z')
    laser_up = LaunchConfiguration('laser_above_center')
    tilt_sign = LaunchConfiguration('tilt_sign')
    laser_yaw = LaunchConfiguration('laser_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Carte (yaml map_server) de la maison'),
        DeclareLaunchArgument('waypoints', default_value=os.path.join(pkg, 'config', 'waypoints.yaml'),
                              description='Fichier des waypoints'),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('mission_params', default_value=os.path.join(pkg, 'config', 'mission_params.yaml')),
        DeclareLaunchArgument('duration_min', default_value='120.0', description='Durée du test (min)'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint', description='Frame de la base'),
        DeclareLaunchArgument('laser_frame', default_value='laser', description='frame_id du scan'),
        DeclareLaunchArgument('odom_topic', default_value='/qbo_arduqbo/base_ctrl/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/qbo_arduqbo/base_ctrl/cmd_vel',
                              description='Topic vitesse de la base (sortie du collision_monitor)'),
        DeclareLaunchArgument('robot_bringup', default_value='',
                              description='Chemin d\'un launch bringup à inclure (vide = qbo_startup déjà lancé par systemd)'),
        DeclareLaunchArgument('start_lidar_driver', default_value='true',
                              description='Lancer le driver rplidar_node (false s\'il tourne déjà)'),
        DeclareLaunchArgument('lidar_executable', default_value='rplidar_node',
                              description='rplidar_node (Slamtec, dans qbo_ws/src/rplidar_ros)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyRpLidar'),
        DeclareLaunchArgument('lidar_scan_mode', default_value='Sensitivity'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true',
                              description='TF dynamique tête -> laser (head_laser_tf, depuis /joint_states)'),
        # --- Lidar sur la tête : géométrie + calibration horizontale ---
        DeclareLaunchArgument('tilt_level_rad', default_value='0.0',
                              description='Tilt qui met le laser HORIZONTAL (calibrate_head_level). '
                                          'Sert de consigne de verrouillage ET de zéro de la TF'),
        DeclareLaunchArgument('head_center_x', default_value='0.10',
                              description='Centre tête (axes pan/tilt) en avant de base_link (m)'),
        DeclareLaunchArgument('head_center_z', default_value='0.34',
                              description='Centre tête au-dessus de base_link (m) — base_link = sol + 0.02'),
        DeclareLaunchArgument('laser_above_center', default_value='0.13',
                              description='Centre tête -> plan de mesure du lidar (m)'),
        DeclareLaunchArgument('laser_yaw', default_value='3.14159',
                              description='Angle de montage du lidar sur la tête (rad). '
                                          'diagnose_base_lidar le 28/10/2026 : +169° puis +184° '
                                          '(précision ±8°) => 180°'),
        DeclareLaunchArgument('tilt_sign', default_value='1.0',
                              description='-1.0 si un tilt positif fait LEVER la tête'),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200',
                              description='A2M8 : 115200 — A2M12 : 256000'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    # ------------------------------------------------- param rewriting (Nav2)
    nav2_configured = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={
            'robot_base_frame': base_frame,
            'base_frame_id': base_frame,
            'odom_topic': odom_topic,
            'yaml_filename': map_yaml,
            'cmd_vel_out_topic': cmd_vel_topic,
        },
        convert_types=True)

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = dict(output='screen', arguments=['--ros-args', '--log-level', log_level])

    # ------------------------------------------------------- robot bringup
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup),
        condition=IfCondition(PythonExpression(["'", robot_bringup, "' != ''"])))

    # ------------------------------------------------------- Verrou tête (permanent)
    # Indépendant de la mission : la tête est tenue dès le lancement, même pendant
    # l'enregistrement des waypoints ou si la mission s'arrête.
    head_lock = Node(
        package='qbo_nav_waypoints', executable='head_lock', name='head_lock', output='screen',
        parameters=[{'tilt_level_rad': ParameterValue(tilt_level, value_type=float)}])

    # ------------------------------------------------------- RPLidar A2
    lidar = Node(
        package='rplidar_ros',
        executable=lidar_executable,
        name='rplidar_node',
        output='screen',
        condition=IfCondition(start_lidar_driver),
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': ParameterValue(lidar_baudrate, value_type=int),
            'frame_id': laser_frame,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': lidar_scan_mode,
            'scan_frequency': 10.0,
        }])

    # ------------------------------------------------------- TF laser DYNAMIQUE
    # Lidar au sommet de la tête : base_link -> head_pan_link -> head_tilt_link -> laser,
    # calculée depuis les angles MESURÉS des servos (yaw de montage 180° repris de qbo_navigation)
    laser_tf = Node(
        package='qbo_nav_waypoints', executable='head_laser_tf', name='head_laser_tf',
        output='screen', condition=IfCondition(publish_laser_tf),
        parameters=[{
            'base_frame': 'base_link',
            'laser_frame': laser_frame,
            'head_center_x': ParameterValue(head_cx, value_type=float),
            'head_center_z': ParameterValue(head_cz, value_type=float),
            'laser_above_center': ParameterValue(laser_up, value_type=float),
            'laser_yaw': ParameterValue(laser_yaw, value_type=float),
            'tilt_level_rad': ParameterValue(tilt_level, value_type=float),
            'tilt_sign': ParameterValue(tilt_sign, value_type=float),
        }])

    # ------------------------------------------------------- localisation
    localization = GroupAction([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[nav2_configured], remappings=tf_remaps, **common),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[nav2_configured], remappings=tf_remaps, **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'bond_timeout': 4.0,
                          'node_names': ['map_server', 'amcl']}]),
    ])

    # ------------------------------------------------------- navigation
    navigation = GroupAction([
        Node(package='nav2_controller', executable='controller_server',
             parameters=[nav2_configured],
             remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[nav2_configured], remappings=tf_remaps, **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[nav2_configured],
             remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[nav2_configured], remappings=tf_remaps, **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             parameters=[nav2_configured],
             remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', 'cmd_vel_smoothed')], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[nav2_configured], remappings=tf_remaps, **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'bond_timeout': 4.0,
                          'node_names': ['controller_server', 'planner_server', 'behavior_server',
                                         'bt_navigator', 'velocity_smoother', 'collision_monitor']}]),
    ])

    # ------------------------------------------------------- mission
    mission = Node(
        package='qbo_nav_waypoints',
        executable='waypoint_mission',
        name='waypoint_mission',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='15',
        parameters=[mission_params, {
            'waypoints_file': waypoints,
            'duration_min': ParameterValue(duration_min, value_type=float),
            'cmd_vel_topic': cmd_vel_topic,
            'odom_topic': odom_topic,
            'level.base_frame': base_frame,
            'level.laser_frame': laser_frame,
            'head.tilt_level_rad': ParameterValue(tilt_level, value_type=float),
        }])

    # ------------------------------------------------------- RViz
    rviz_cfg = os.path.join(pkg, 'rviz', 'neo_nav.rviz')   # scan rouge, zones collision monitor
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_cfg], condition=IfCondition(use_rviz))

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        *args,
        LogInfo(msg=['[Néo] Test navigation laser — carte : ', map_yaml, ' — waypoints : ', waypoints]),
        bringup,
        head_lock,
        laser_tf,
        lidar,
        localization,
        navigation,
        mission,
        rviz,
    ])
