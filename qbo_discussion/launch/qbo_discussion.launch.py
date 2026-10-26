#!/usr/bin/env python3
"""
Launch file pour le système de discussion complet de Néo
Inclut STT, TTS et le node de discussion RAG
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Déclaration des arguments de lancement
    declare_json_path = DeclareLaunchArgument(
        'json_path',
        default_value='/home/nvidia/qbo_ws/src/qbo_discussion/qbo_discussion/rag/neo_json_random.json',
        description='Chemin vers le fichier JSON contenant les données Q&A'
    )
    
    declare_confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.80',
        description='Seuil de confiance minimal pour accepter une réponse'
    )
    
    declare_model_name = DeclareLaunchArgument(
        'model_name',
        default_value='intfloat/multilingual-e5-large',
        description='Nom du modèle SentenceTransformer à utiliser'
    )
    
    # CORRECTION : Ajout de l'argument user_name manquant
    declare_user_name = DeclareLaunchArgument(
        'user_name',
        default_value='Vincent',
        description='Nom de l\'utilisateur pour personnaliser les réponses'
    )
    
    # Lancement du node STT (Speech-to-Text)
    stt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qbo_stt'),
                'launch',
                'qbo_stt_launch.py'
            ])
        ]),
        launch_arguments={
            # Ajoutez ici les arguments spécifiques au STT si nécessaire
        }.items()
    )
    
    # Lancement du node TTS (Text-to-Speech)
    tts_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qbo_tts'),
                'launch',
                'qbo_tts_launch.py'
            ])
        ]),
        launch_arguments={
            # Ajoutez ici les arguments spécifiques au TTS si nécessaire
        }.items()
    )
    
    # Node de discussion RAG
    discussion_node = Node(
        package='qbo_discussion',
        executable='qbo_discussion_node',
        name='qbo_discussion_node',
        output='screen',
        parameters=[{
            'json_path': LaunchConfiguration('json_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_name': LaunchConfiguration('model_name'),
            'user_name': LaunchConfiguration('user_name')
        }],
        remappings=[
            # Ajoutez des remappings si nécessaire
            # ('input_topic', 'remapped_input'),
            # ('output_topic', 'remapped_output')
        ]
    )
    
    return LaunchDescription([
        # Arguments de lancement
        declare_json_path,
        declare_confidence_threshold,
        declare_model_name,
        declare_user_name,
        
        # Nodes et launch files
        stt_launch,
        tts_launch,
        discussion_node
    ])