# qbo_dynamixel

**Version :** 0.1.3

## 📦 Description du package

Ce package ROS 2 (`Humble`) est conçu pour contrôler des servos Dynamixel AX12A / AX18A sur la plateforme Qbo équipée d'une Jetson Orin NX (JetPack 6.1, Ubuntu 22.04) et de la carte A608 de chez Seeed.

Il permet :

- Le contrôle Pan & Tilt via le topic `/cmd_joints`
- La publication d'états moteurs (`/joint_states`, `/dynamixel_state`)
- La supervision via `/diagnostics`
- Le contrôle du couple via le service `/torque_enable`
- Une configuration dynamique des servos via un fichier YAML

Étapes rapides (résumé) :

1. Installer les dépendances ROS 2 nécessaires.
2. Identifier le port série et scanner les servos pour valider les IDs.
3. Renseigner le fichier YAML (ports, IDs, limites, vitesse).
4. Compiler le package et sourcer l'environnement.
5. Lancer le nœud `qbo_dynamixel` avec le fichier de paramètres.
6. Tester les commandes `/cmd_joints` et vérifier `/diagnostics`.

---

## ✅ Installation des dépendances

```bash
sudo apt install ros-humble-dynamixel-workbench
sudo apt install ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-diagnostic-updater
```

---

## 🧪 Utilitaire Python pour scanner le bus Dynamixel

Permet de détecter les servos connectés (adapter le port et la vitesse à 57600 / 1000000 / 115200 selon les cas).

```bash
# Vérifier les ports disponibles :
  ls /dev/ttyUSB*
  ls /dev/ttyAMA*

# Installer la dépendance :
sudo apt install python3-serial

# Ajouter l'utilisateur au groupe dialout, puis redémarrer la session :
sudo usermod -a -G dialout $USER
groups

# Lancer le scan :
python3 src/qbo_arduqbo/scan_dxl.py
```

---

## ⚙️ Compilation du package

```bash
colcon build --packages-select qbo_arduqbo --cmake-clean-cache --allow-overriding qbo_arduqbo
source install/setup.bash
```

---

## ▶️ Commandes de test

### 🚀 Lancement du contrôleur Dynamixel

```bash
ros2 run qbo_arduqbo qbo_dynamixel   --ros-args --params-file src/qbo_arduqbo/config/dynamixel_config.yaml
```

---

### 🎮 Commande de mouvement (publisher)

```bash
ros2 topic pub -1 /cmd_joints sensor_msgs/JointState   "{name: ['head_pan_joint'], position: [0.0], velocity: [2.0]}"
ros2 topic pub -1 /cmd_joints sensor_msgs/JointState   "{name: ['head_tilt_joint'], position: [0.2], velocity: [2.0]}"
```

---

### 🧩 Contrôle du couple (services)

```bash
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable   "{torque_enable: false}"

ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable   "{torque_enable: false}"
```

---

### 📡 Surveillance des topics

```bash
ros2 topic echo /dynamixel_state
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

# Activer le torque automatique off
```bash
ros2 param set /qbo_dynamixel auto_torque_off true
```

# Désactiver le torque automatique off
```bash
ros2 param set /qbo_dynamixel auto_torque_off false
```

# Modifier le timeout (en secondes)
```bash
ros2 param set /qbo_dynamixel auto_torque_off_timeout 5.0
```

---

### 🧩 Chagement des paramétres à la volé (rosparam)

```bash
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.max_angle_degrees 60.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.min_angle_degrees -60.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.neutral 512

ros2 param set /qbo_dynamixel dynamixel.motors.head_tilt_joint.max_angle_degrees 15.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_tilt_joint.min_angle_degrees -25.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_tilt_joint.neutral 520
```

---

## 📁 Structure YAML attendue (extrait)

```yaml
qbo_dynamixel:
  ros__parameters:
    dynamixel.usb_port: "/dev/ttyDmx"
    dynamixel.baud_rate: 1000000
    dynamixel.protocol_version: 1.0

    dynamixel_state_rate_hz: 1
    dynamixel_joint_rate_hz: 15

    # DIAGNOSTICS
    diagnostic_temp_warn: 65.0
    diagnostic_voltage_min: 8.0
    diagnostic_voltage_max: 12.5
    diagnostic_position_error: 20

    auto_torque_off: true
    auto_torque_off_timeout: 20.0

    dynamixel.motor_keys: ["motor_1", "motor_2"]
    # Servo 1 : head_pan_joint (<--->)
    dynamixel.motors.motor_1.name: head_pan_joint
    dynamixel.motors.motor_1.id: 1
    dynamixel.motors.motor_1.invert: false
    dynamixel.motors.motor_1.max_angle_degrees: 70.0  # 1.22 Rad
    dynamixel.motors.motor_1.min_angle_degrees: -70.0 # -1.22 Rad
    dynamixel.motors.motor_1.range: 300.0
    dynamixel.motors.motor_1.ticks: 1024
    dynamixel.motors.motor_1.neutral: 530
    dynamixel.motors.motor_1.torque_limit: 800
    dynamixel.motors.motor_1.max_speed: 2.0

    # Servo 2 : head_tilt_joint (￪↓)
    dynamixel.motors.motor_2.name: head_tilt_joint
    dynamixel.motors.motor_2.id: 8
    dynamixel.motors.motor_2.invert: false
    dynamixel.motors.motor_2.max_angle_degrees: 20.0  # 0.35 Rad
    dynamixel.motors.motor_2.min_angle_degrees: -30.0 # -0.52 Rad
    dynamixel.motors.motor_2.range: 300.0
    dynamixel.motors.motor_2.ticks: 1024
    dynamixel.motors.motor_2.neutral: 520
    dynamixel.motors.motor_2.torque_limit: 800
    dynamixel.motors.motor_2.max_speed: 2.0
```

```bash
**Auteur : Sylvain Zwolinski
**Licence : BSD-3-Clause
