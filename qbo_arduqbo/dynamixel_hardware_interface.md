# Néo Robot — Documentation Technique Complète

> ROS2 Humble — Jetson Orin NX 16Go — JetPack 6.1 — Ubuntu 22.04
> **Auteurs :** Sylvain Zwolinski, Vincent — **Licence :** BSD-3-Clause

---

## Table des matières

1. [Vue d'ensemble du système](#1-vue-densemble-du-système)
2. [Configuration UDEV](#2-configuration-udev)
3. [Dépendances](#3-dépendances)
4. [Installation et compilation](#4-installation-et-compilation)
5. [qbo_dynamixel — Tête Pan/Tilt](#5-qbo_dynamixel--tête-pantilt)
6. [qbo_arduqbo — Cartes QBoard](#6-qbo_arduqbo--cartes-qboard)
7. [Commandes de test](#7-commandes-de-test)
8. [Dépannage](#8-dépannage)
9. [Améliorations à venir](#9-améliorations-à-venir)
10. [Historique de version](#10-historique-de-version)

---

## 1. Vue d'ensemble du système

Néo est un robot mobile équipé d'une tête Pan/Tilt, d'une base mobile, de capteurs, d'un LCD et d'une gestion batterie. Deux packages ROS2 gèrent le matériel :

| Package | Rôle | Port |
|---|---|---|
| `qbo_dynamixel` | Servos AX-12A tête (Pan + Tilt) | `/dev/ttyDmx` |
| `qbo_arduqbo` | QBoard1 (base/IMU), QBoard2 (nez/bouche/audio) | `/dev/ttyQboard1`, `/dev/ttyQboard2` |

**Architecture générale :**

```
dynamixel_config.yaml          qboards_config.yaml
        │                               │
        ▼                               ▼
qbo_dynamixel (node)          qbo_arduqbo (node)
        │                               │
        ├── DynamixelHardware           ├── BaseController    → /odom, /cmd_vel
        │   ├── ping retry (×5)         ├── ImuController     → /imu_state
        │   ├── centrage neutre         ├── BatteryController → /diagnostics
        │   ├── clamp pos/vitesse       ├── LcdController     → /cmd_lcd
        │   └── /diagnostics            ├── NoseController    → /cmd_nose
        │                               ├── MouthController   → /cmd_mouth
        ├── /cmd_joints sub             └── SensorController  → /distance_sensors
        └── /torque_enable srv
```

---

## 2. Configuration UDEV

Les règles UDEV créent des liens symboliques stables dans `/dev/` indépendamment de l'ordre de branchement USB.

### Identifier les périphériques

```bash
lsusb
udevadm info -a -n /dev/ttyUSB0 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}|KERNELS'
ls -l /dev/serial/by-path/
```

### Créer le fichier de règles

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

```udev
# ttyQboard1 — FTDI FT232 (VID 0403, PID 6001)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyQboard1", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyQboard2 — interface 00 du FT4232H (VID 0403, PID 6010)
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", \
  ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="ttyQboard2", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyDmx — interface 01 du même FT4232H (Dynamixel RS485)
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", \
  ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="ttyDmx", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyRpLidar — Silicon Labs CP210x
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyRpLidar"
```

### Appliquer et vérifier

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/ttyDmx /dev/ttyQboard1 /dev/ttyQboard2
```

### Si ttyDmx pose problème — service systemd alternatif

```bash
sudo nano /etc/systemd/system/ttyDmx.service
```

```ini
[Unit]
Description=Créer le lien /dev/ttyDmx pour le port FTDI interface 01
After=dev-ttyUSB2.device
Wants=dev-ttyUSB2.device

[Service]
Type=oneshot
ExecStart=/bin/ln -sf /dev/ttyUSB2 /dev/ttyDmx
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable ttyDmx.service
sudo systemctl start ttyDmx.service
```

### Permissions série

```bash
sudo usermod -aG dialout $USER
newgrp dialout   # sans reloguer
```

---

## 3. Dépendances

### Packages ROS2

```bash
sudo apt install \
  ros-humble-dynamixel-workbench-toolbox \
  ros-humble-dynamixel-workbench \
  ros-humble-diagnostic-updater \
  ros-humble-hardware-interface \
  ros-humble-sensor-msgs \
  ros-humble-tf2-geometry-msgs
```

### Python

```bash
pip3 install dynamixel-sdk --break-system-packages
sudo apt install python3-serial
```

---

## 4. Installation et compilation

```bash
cd ~/qbo_ws

# Build complet
colcon build --packages-select qbo_msgs qbo_arduqbo
source install/setup.bash

# Build propre si erreurs de cache
rm -rf build/qbo_arduqbo install/qbo_arduqbo
colcon build --packages-select qbo_arduqbo --cmake-clean-cache --allow-overriding qbo_arduqbo
source install/setup.bash
```

### Vérifier les servos avant de lancer

```bash
python3 - << 'EOF'
from dynamixel_sdk import *
PORT = "/dev/ttyDmx"
for baud in [57600, 1000000, 115200]:
    ph = PortHandler(PORT)
    pm = PacketHandler(1.0)
    ph.setBaudRate(baud)
    ph.openPort()
    found = []
    for id in range(1, 20):
        model, result, _ = pm.ping(ph, id)
        if result == 0:
            found.append((id, model))
    ph.closePort()
    if found:
        print(f"✅ Baud {baud}: {found}")
    else:
        print(f"❌ Baud {baud}: rien")
EOF
```

Résultat attendu : `✅ Baud 1000000: [(1, 12), (2, 12)]`

---

## 5. qbo_dynamixel — Tête Pan/Tilt

### Servos configurés

| Joint | ID | Neutre | Limites | Max vitesse |
|---|---|---|---|---|
| `head_pan_joint` | 1 | tick 530 | ±70° (±1.22 rad) | 5.0 rad/s |
| `head_tilt_joint` | 2 | tick 465 | -30°/+20° (-0.52/+0.35 rad) | 3.0 rad/s |

**Protocole :** Dynamixel 1.0 — RS485 @ 1 Mbps — `/dev/ttyDmx`

### Fonctionnement

**Initialisation (`on_init`) :**
1. Lecture de tous les paramètres depuis le yaml
2. Init `DynamixelWorkbench` sur le port RS485
3. Ping avec retry (5 tentatives × 100ms) — nécessaire pour charger la table de registres du modèle
4. Configuration : `Torque_Limit`, `torqueOn`, `Goal_Position = neutral` (centrage auto)
5. Init `diagnostic_updater` → `/diagnostics`

**Lecture (`read`) :**

Fréquence configurable via `dynamixel_state_rate_hz` dans le yaml. Pour chaque servo :
```
itemRead Present_Position    → ticks → rad
itemRead Present_Speed       → bit10=direction, bits0-9=magnitude → rad/s
itemRead Present_Load        → bit10=direction, bits0-9=magnitude
itemRead Present_Temperature → °C
itemRead Torque_Enable       → bool
```
Si lecture échoue : dernière valeur conservée, `WARN_THROTTLE` 2s, pas de crash.

**Écriture de position (`/cmd_joints`) :**
```
position (rad) → clamp([min_angle, max_angle]) → ticks → Goal_Position
velocity (rad/s) → min(vel, max_speed) → × 85.9 → clamp([1,1023]) → Moving_Speed
```

**Conversions AX-12A :**

| Grandeur | Formule |
|---|---|
| rad → ticks | `round(rad / (π/512)) + neutral` |
| ticks → rad | `(ticks - neutral) × (π/512)` |
| rad/s → ticks vitesse | `rad/s × 85.9` (0.111 RPM/tick) |
| Vitesse max @ 11.1V | ≈ 5.7 rad/s (54.6 RPM) |

**Diagnostics sur `/diagnostics` :**

| Champ | Seuil |
|---|---|
| Position (rad), Velocity, Torque load | — |
| Temperature (C) | WARN ≥ 60°C / ERROR ≥ 70°C |
| Torque enabled | true/false |

### Configuration YAML

```yaml
dynamixel_hardware:
  ros__parameters:
    dynamixel.usb_port: "/dev/ttyDmx"
    dynamixel.baud_rate: 1000000
    dynamixel.protocol_version: 1.0
    dynamixel_state_rate_hz: 1.0        # ⚠️ float obligatoire (1.0 pas 1)
    dynamixel_joint_rate_hz: 15.0

    diagnostic_temp_warn: 65.0
    diagnostic_voltage_min: 8.0
    diagnostic_voltage_max: 12.5
    auto_torque_off: true
    auto_torque_off_timeout: 20.0

    dynamixel.motor_keys: ["motor_1", "motor_2"]

    dynamixel.motors.motor_1.name: head_pan_joint
    dynamixel.motors.motor_1.id: 1
    dynamixel.motors.motor_1.neutral: 530
    dynamixel.motors.motor_1.ticks: 1024
    dynamixel.motors.motor_1.torque_limit: 800
    dynamixel.motors.motor_1.max_speed: 5.0
    dynamixel.motors.motor_1.min_angle_degrees: -70.0
    dynamixel.motors.motor_1.max_angle_degrees: 70.0
    dynamixel.motors.motor_1.invert: false

    dynamixel.motors.motor_2.name: head_tilt_joint
    dynamixel.motors.motor_2.id: 2
    dynamixel.motors.motor_2.neutral: 465
    dynamixel.motors.motor_2.ticks: 1024
    dynamixel.motors.motor_2.torque_limit: 800
    dynamixel.motors.motor_2.max_speed: 3.0
    dynamixel.motors.motor_2.min_angle_degrees: -30.0
    dynamixel.motors.motor_2.max_angle_degrees: 20.0
    dynamixel.motors.motor_2.invert: false
```

### Topics & Services

| Topic/Service | Type | Description |
|---|---|---|
| `/cmd_joints` sub | `sensor_msgs/JointState` | Commandes position + vitesse |
| `/diagnostics` pub | `diagnostic_msgs/DiagnosticArray` | Température, charge, position, torque |
| `/head_pan_joint/torque_enable` | `qbo_msgs/srv/TorqueEnable` | Torque ON/OFF servo pan |
| `/head_tilt_joint/torque_enable` | `qbo_msgs/srv/TorqueEnable` | Torque ON/OFF servo tilt |

### Lancement

```bash
# Debug direct (recommandé)
~/qbo_ws/install/qbo_arduqbo/lib/qbo_arduqbo/qbo_dynamixel \
  --ros-args --params-file ~/qbo_ws/src/qbo_arduqbo/config/dynamixel_config.yaml

# Via ros2 run
ros2 run qbo_arduqbo qbo_dynamixel \
  --ros-args --params-file ~/qbo_ws/src/qbo_arduqbo/config/dynamixel_config.yaml

# Via launch complet Néo
ros2 launch qbo_arduqbo qbo_full.launch.py
```

---

## 6. qbo_arduqbo — Cartes QBoard

### Lancement

```bash
ros2 run qbo_arduqbo qbo_arduqbo \
  --ros-args --params-file ~/qbo_ws/src/qbo_arduqbo/config/qboards_config.yaml
```

### Configuration YAML (extrait)

```yaml
qbo_arduqbo:
  ros__parameters:
    enable_qboard1: true
    port1: "/dev/ttyQboard1"
    baud1: 500000              # ×4.3 vs 115200 — réduit latence transmission
    timeout1: 10.0             # nécessaire pour calibration IMU

    enable_qboard2: true
    port2: "/dev/ttyQboard2"
    baud2: 115200
    timeout2: 3.0

    enable_base: true
    enable_battery: true
    enable_imu_base: true
    enable_lcd: true
    enable_nose: true
    enable_mouth: true
    enable_audio: false
    enable_sensors: true
```

### Contrôleurs

**BaseController** — base mobile, odométrie, TF

| Topic/Service | Description |
|---|---|
| `/qbo_arduqbo/base_ctrl/cmd_vel` sub | Commande vitesse Twist |
| `/qbo_arduqbo/base_ctrl/odom` pub | Odométrie |
| `stop_base` srv | Arrêt d'urgence |
| `set_odometry` srv | Reset position |
| `unlock_motors_stall` srv | Déblocage moteurs |

**ImuController** — IMU, calibration

| Topic/Service | Description |
|---|---|
| `/qbo_arduqbo/imu_ctrl/imu_state/data` pub | Données IMU |
| `/qbo_arduqbo/imu_ctrl/imu_state/calibrate` srv | Calibration |

**BatteryController** — tension, autonomie (→ `/diagnostics` uniquement)

**LcdController** — `/qbo_arduqbo/lcd_ctrl/cmd_lcd` (`qbo_msgs/msg/LCD`)

**NoseController** — `/qbo_arduqbo/nose_ctrl/cmd_nose` (`qbo_msgs/msg/Nose`) + service `test_leds`

**MouthController** — `/qbo_arduqbo/mouth_ctrl/cmd_mouth` (`qbo_msgs/msg/Mouth`) + service `test_leds`

**SensorController** — capteurs distance SRF10/Sharp IR → `/qbo_arduqbo/sens_ctrl/distance_sensors_state/<name>`

---

## 7. Commandes de test

### Tête Dynamixel

```bash
# Centrer
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint', 'head_tilt_joint'], position: [0.0, 0.0], velocity: [1.0, 1.0]}"

# Pan droite lent
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [0.5], velocity: [0.5]}"

# Pan gauche max (-70°) rapide
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [-1.22], velocity: [5.0]}"

# Tilt haut (+20° max)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_tilt_joint'], position: [0.35], velocity: [1.0]}"

# Tilt bas (-30° max)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_tilt_joint'], position: [-0.52], velocity: [1.0]}"

# Mouvement combiné
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint', 'head_tilt_joint'], position: [0.8, -0.3], velocity: [2.0, 1.0]}"

# Torque OFF / ON
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"
ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
```

### Base mobile

```bash
ros2 topic pub /qbo_arduqbo/base_ctrl/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
ros2 service call /qbo_arduqbo/base_ctrl/stop_base std_srvs/srv/Empty "{}"
ros2 service call /qbo_arduqbo/base_ctrl/set_odometry qbo_msgs/srv/SetOdometry \
  "{x: 0.0, y: 0.0, theta: 0.0}"
```

### Expressivité

```bash
# Nez rouge
ros2 topic pub -1 /qbo_arduqbo/nose_ctrl/cmd_nose qbo_msgs/msg/Nose "{color: 4}"

# LCD
ros2 topic pub -1 /qbo_arduqbo/lcd_ctrl/cmd_lcd qbo_msgs/msg/LCD "{msg: 'Hello Neo'}"

# Bouche
ros2 topic pub -1 /qbo_arduqbo/mouth_ctrl/cmd_mouth qbo_msgs/msg/Mouth \
  "{mouth_image: [true,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true]}"
```

### Surveillance

```bash
ros2 topic echo /diagnostics
ros2 topic echo /joint_states
ros2 topic echo /qbo_arduqbo/base_ctrl/odom
ros2 topic echo /qbo_arduqbo/imu_ctrl/imu_state/data
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

### Système

```bash
# Ports disponibles
ls -la /dev/ttyDmx /dev/ttyQboard1 /dev/ttyQboard2

# Processus occupant les ports
fuser /dev/ttyDmx /dev/ttyQboard1 /dev/ttyQboard2

# Tuer un processus bloquant
kill $(pgrep -f qbo_dynamixel)
pkill -f qbo_arduqbo

# Baud rate effectif
stty -F /dev/ttyQboard1
stty -F /dev/ttyDmx
```

---

## 8. Dépannage

### `Cannot ping servo ID X après 5 tentatives`

```bash
# Vérifier le port
ls -la /dev/ttyDmx

# Scanner tous les IDs et baud rates
python3 - << 'EOF'
from dynamixel_sdk import *
for baud in [57600, 1000000, 115200]:
    ph = PortHandler("/dev/ttyDmx")
    pm = PacketHandler(1.0)
    ph.setBaudRate(baud)
    ph.openPort()
    found = [(id, m) for id in range(1,20) for m, r, _ in [pm.ping(ph, id)] if r == 0]
    ph.closePort()
    print(f"{'✅' if found else '❌'} Baud {baud}: {found if found else 'rien'}")
EOF

# Permissions
groups && sudo usermod -aG dialout nvidia
```

### `Overload error` — servo en protection

```bash
# Couper l'alimentation 5 secondes puis rebrancher
# Vérifier les limites yaml (min/max_angle_degrees)
# Réduire torque_limit (ex: 400 au lieu de 800)
```

### `No Q.bo board detected on any port`

```bash
# Port occupé ?
fuser /dev/ttyQboard1
sudo kill -9 $(fuser /dev/ttyQboard1 2>/dev/null)

# La carte envoie-t-elle des données ?
timeout 3 cat /dev/ttyQboard1 | xxd | head -5

# Symlinks absents ?
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/ttyQboard*
```

### Segfault au redémarrage rapide

Bus RS485 pas vidé. Attendre 2-3 secondes.

```bash
fuser /dev/ttyDmx   # vérifier qu'aucun processus ne tient le port
```

### Température servo élevée (>50°C)

Normal en continu. AX-12A supporte jusqu'à 70°C.
- Réduire `torque_limit: 400`
- Réduire `max_speed: 2.0`
- Activer `auto_torque_off: true`

---

## 9. Améliorations à venir

**Court terme**
- `auto_torque_off` — éteindre le torque après timeout d'inactivité (paramètre yaml existant, à implémenter)
- Publisher `/joint_states` standard pour RViz/URDF

**Moyen terme**
- Action server `/move_head` — feedback progression, utilisable depuis les behaviours de Néo
- Profil de mouvement trapézoïdal — accélération/décélération progressive
- Intégration `ros2_control` complète via `controller_manager`

**Long terme**
- Comportements de tête : suivi de visage, regard aléatoire, nod/shake expressif
- Détection de blocage mécanique via `Present_Load`
- Calibration automatique des neutres au démarrage
---

## 10. Historique de version

| Version | Date | Changements |
|---|---|---|
| 1.0 | Mars 2026 | Init — lecture position, écriture Goal_Position |
| 1.1 | Mars 2026 | Ping avec retry, séparation ping/config |
| 1.2 | Mars 2026 | Lecture température/vitesse/torque, diagnostics |
| 1.3 | Mars 2026 | Services torque_enable, lecture tolérante |
| 1.4 | Mars 2026 | Centrage au démarrage, clamp min/max_angle |
| 1.5 | Mars 2026 | Contrôle vitesse Moving_Speed, clamp max_speed |
| 1.6 | Mars 2026 | Config 100% yaml, suppression valeurs hardcodées |
| 1.7 | Mars 2026 | Fusion doc : dynamixel + arduqbo + udev en 1 fichier |
