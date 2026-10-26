# QBO Discussion Package

Package ROS2 pour la discussion avec le robot Neo via RAG.

## Installation

```bash
# Installer import nécessaire
pip3 install sentence-transformers tf-keras faiss-cpu

```
```bash
# Compiler le package
colcon build --packages-select qbo_discussion

# Sourcer l'environnement
source install/setup.bash
```

## Utilisation

```bash
# Lancer le node
ros2 run qbo_discussion qbo_discussion_node

# Ou avec le fichier de lancement
ros2 launch qbo_discussion qbo_discussion.launch.py
```

## Topics

- **Input**: `/listened` (std_msgs/String) - Messages d'entrée à traiter
- **Output**: `/tts_input` (std_msgs/String) - Réponses générées pour le TTS

## Configuration

Éditez le fichier `config/qbo_discussion_params.yaml` pour modifier les paramètres.
