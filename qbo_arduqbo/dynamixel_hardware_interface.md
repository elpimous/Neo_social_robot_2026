# Dynamixel Hardware Interface — Néo Robot

> Interface ROS2 Humble pour la commande de servomoteurs AX-12A via `DynamixelWorkbench`, intégrée dans le robot **Néo** (Jetson Orin NX 16Go).

---

## Table des matières

1. [Vue d'ensemble](#vue-densemble)
2. [Architecture](#architecture)
3. [Fonctionnement détaillé](#fonctionnement-détaillé)
4. [Dépendances](#dépendances)
5. [Installation](#installation)
6. [Configuration YAML](#configuration-yaml)
7. [Commandes de test](#commandes-de-test)
8. [Topics & Services](#topics--services)
9. [Avantages de cette implémentation](#avantages)
10. [Améliorations à venir](#améliorations-à-venir)
11. [Dépannage](#dépannage)

---

## Vue d'ensemble

Ce package fournit un nœud ROS2 (`qbo_dynamixel`) qui pilote les servomoteurs Dynamixel AX-12A de la tête de Néo :

| Joint | ID | Direction | Neutre | Limites |
|---|---|---|---|---|
| `head_pan_joint` | 1 | Pan (gauche/droite) | tick 530 | ±70° (±1.22 rad) |
| `head_tilt_joint` | 2 | Tilt (haut/bas) | tick 465 | -30° / +20° (-0.52 / +0.35 rad) |

**Protocole** : Dynamixel Protocol 1.0 — RS485 @ 1 Mbps via `/dev/ttyDmx` (FT4232H, interface 01 Qboard2)

---

## Architecture

```
dynamixel_config.yaml
        │
        ▼
qbo_dynamixel (main)
        │
        ├── DynamixelHardware (on_init)
        │       ├── ping ID1 → AX-12A modèle 12
        │       ├── ping ID2 → AX-12A modèle 12
        │       ├── Torque_Limit + torqueOn
        │       ├── Goal_Position → neutral (centrage)
        │       └── diagnostic_updater → /diagnostics
        │
        ├── Subscriber /cmd_joints (JointState)
        │       ├── Moving_Speed (clampé à max_speed yaml)
        │       └── Goal_Position (clampé à min/max_angle yaml)
        │
        ├── Service /head_pan_joint/torque_enable
        ├── Service /head_tilt_joint/torque_enable
        │
        └── Read loop @ 1 Hz (via config.yaml)
                ├── Present_Position → position (rad)
                ├── Present_Speed    → velocity (rad/s)
                ├── Present_Load     → torque_load
                ├── Present_Temperature → temperature (°C)
                └── Torque_Enable    → torque_enabled (bool)
```

---

## Fonctionnement détaillé

ImuController (imu_ctrl)

Rôle : lecture IMU, calibration IMU.

Paramètres

    rate (double) : fréquence de publication.
    topic (string) : base du topic IMU.

Topics

    Publie : <topic>/data (sensor_msgs/msg/Imu).
    Publie : <topic>/is_calibrated (std_msgs/msg/Bool).

Services

    <topic>/calibrate (qbo_msgs/srv/CalibrateIMU).

Diagnostics (IMU Status)

    Criticité : ERROR si IMU incomplète, WARN si non calibrée, OK sinon.
    Champs : présence gyro/accel, modèles et adresses I2C, IMU calibrated, Last calibration.

Exemples

ros2 topic echo /qbo_arduqbo/imu_ctrl/imu_state/data
ros2 topic echo /qbo_arduqbo/imu_ctrl/imu_state/is_calibrated
ros2 service call /qbo_arduqbo/imu_ctrl/imu_state/calibrate qbo_msgs/srv/CalibrateIMU "{}"

BatteryController (battery_ctrl)

Rôle : mesure tension et état batterie, estimation autonomie/puissance.

Paramètres

    error_battery_level (double) : seuil batterie vide.
    warn_battery_level (double) : seuil batterie faible.
    capacity_ah (double) : capacité batterie.
    nominal_voltage (double) : tension nominale.
    battery_type (string) : type de batterie.

Topics

    Aucun topic applicatif dédié (diagnostics uniquement).
    Abonné à /diagnostics pour récupérer la puissance A608.

Services

    Aucun.

Diagnostics (Battery Status)

    Criticité : STALE si pas de communication QBoard3, WARN/ERROR selon seuils, OK sinon.
    Champs : Voltage, Type, Nominal Voltage, Capacity, Status, Charge Mode, External Power, PC On, Boards On, Charge Mode Description, Estimated Runtime, Estimated Power, Estimated Extras.

Exemples

ros2 topic echo /diagnostics

LcdController (lcd_ctrl)

Rôle : affichage LCD (ligne commandée + infos diagnostics).

Paramètres

    rate (double) : fréquence d'update.
    topic (string) : topic d'affichage LCD.

Topics

    Abonne : <topic> (qbo_msgs/msg/LCD).
    Abonne : /diagnostics (pour afficher réseau, batterie, temperature CPU).

Services

    Aucun.

Diagnostics (LCD Status)

    Criticité : OK si LCD présent, ERROR sinon.
    Champs : LCD Present, LCD Model, I2C Address.

Exemples

ros2 topic pub -1 /qbo_arduqbo/lcd_ctrl/cmd_lcd qbo_msgs/msg/LCD "{msg: 'Hello Q.bo'}"

NoseController (nose_ctrl)

Rôle : LED RGB du nez, test.

Paramètres

    rate (double) : fréquence de boucle.
    topic (string) : topic couleur nez.

Topics

    Abonne : <topic> (qbo_msgs/msg/Nose).

Services

    <node>/test_leds (qbo_msgs/srv/TestLeds).

Diagnostics (Nose Status)

    Criticité : WARN si la derniere commande/test a echoue, OK sinon.
    Champs : color_code, color_name.

Exemples

ros2 topic pub -1 /qbo_arduqbo/nose_ctrl/cmd_nose qbo_msgs/msg/Nose "{color: 4}"
ros2 service call /qbo_arduqbo/nose_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"

MouthController (mouth_ctrl)

Rôle : matrice LED 4x5, test.

Paramètres

    rate (double) : fréquence de boucle.
    topic (string) : topic motif bouche.

Topics

    Abonne : <topic> (qbo_msgs/msg/Mouth).

Services

    <node>/test_leds (qbo_msgs/srv/TestLeds).

Diagnostics (Mouth Status)

    Criticité : WARN si le test LED a echoue, OK sinon.

Exemples

ros2 topic pub -1 /qbo_arduqbo/mouth_ctrl/cmd_mouth qbo_msgs/msg/Mouth "{mouth_image: [true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true]}"

ros2 service call /qbo_arduqbo/mouth_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"

SensorController (sens_ctrl)

Rôle : capteurs distance SRF10/VL53L1X (I2C) et Sharp IR (ADC).

Paramètres

    rate (double) : fréquence de boucle (default 15.0).
    topic (string) : base des topics publies (default srf10_state).
    sensors.<group>.<name>.type : srf10, VL53L1X, gp2d12, gp2d120, GP2Y0A21YK.
    sensors.<group>.<name>.address : adresse I2C/ADC.
    sensors.<group>.<name>.frame_id : frame du capteur.
    sensors.<group>.<name>.topic : topic de publication (optionnel).
    sensors.<group>.<name>.publish_if_obstacle : publier uniquement si obstacle.

Topics

    Publie : <base_topic>/<name> (sensor_msgs/msg/PointCloud) pour chaque capteur.

Services

    Aucun.

Diagnostics (Sensors Status)

    Criticité : WARN si aucun capteur configure ou si un capteur ne repond plus, OK sinon.
    Champs : Missing SRF10, Missing ADC, SRF10 count, ADC count, Board configured.

Exemples

ros2 topic echo /qbo_arduqbo/sens_ctrl/distance_sensors_state/front_right_srf10

### Initialisation (`on_init`)

1. Lecture des paramètres depuis le yaml (port, baud, protocol, joints...)
2. Initialisation de `DynamixelWorkbench` sur `/dev/ttyDmx` @ 1 Mbps
3. **Ping avec retry** (5 tentatives × 100ms) pour chaque servo — nécessaire car `DynamixelWorkbench` doit charger la table de registres du modèle avant tout `itemRead/itemWrite`
4. Configuration : `Torque_Limit`, `torqueOn`, `Goal_Position = neutral`
5. Enregistrement des callbacks dans `diagnostic_updater`

### Lecture (`read`)

Appelée à 1 Hz dans la boucle principale. Lit 5 registres par servo :

```
Present_Position  → ticks → rad (via ticksToAngle)
Present_Speed     → ticks → rad/s (× 0.01194)
Present_Load      → torque_load brut
Present_Temperature → °C
Torque_Enable     → bool
```

Si la lecture échoue (torque OFF, overload), la dernière valeur connue est conservée sans planter le nœud (`WARN_THROTTLE` + `continue`).

### Écriture de position

Le topic `/cmd_joints` accepte un `JointState` avec :
- `name[]` : noms des joints à commander
- `position[]` : consigne en **radians**
- `velocity[]` : vitesse souhaitée en **rad/s** (optionnel)

Pipeline de traitement :
```
position (rad)
    └── angleToTicks()
            ├── clamp([min_angle, max_angle])  ← sécurité mécanique
            ├── invert si nécessaire
            └── tick = round(rad / rad_per_tick) + neutral

velocity (rad/s)
    └── min(velocity, max_speed)               ← clamp yaml
            └── speed_tick = vel × 85.9        ← AX-12: 0.111 RPM/tick
                    └── clamp([1, 1023])
                            └── Moving_Speed
```

### Conversions AX-12A

| Grandeur | Formule |
|---|---|
| rad → ticks | `tick = round(rad / (π / 512)) + neutral` |
| ticks → rad | `rad = (tick - neutral) × (π / 512)` |
| rad/s → ticks vitesse | `tick = rad/s × 85.9` (car 0.111 RPM/tick) |
| Vitesse max (11.1V) | ≈ 5.7 rad/s (54.6 RPM) |

### Diagnostics

Publié sur `/diagnostics` via `diagnostic_updater` à chaque `read()` :

| Champ | Valeur |
|---|---|
| Position (rad) | position courante |
| Velocity (rad/s) | vitesse courante |
| Torque load | charge moteur brute |
| Temperature (C) | température interne |
| Torque enabled | true / false |
| Status | OK / WARN (≥60°C) / ERROR (≥70°C) |

---

## Dépendances

### Packages ROS2 à installer

```bash
sudo apt install \
  ros-humble-dynamixel-workbench-toolbox \
  ros-humble-diagnostic-updater \
  ros-humble-hardware-interface \
  ros-humble-sensor-msgs
```

### Package Python (pour le scan de diagnostic)

```bash
pip3 install dynamixel-sdk --break-system-packages
```

### Règle udev requise

Le port `/dev/ttyDmx` est créé par une règle udev pointant vers l'interface 01 du FT4232H (VID `0403`, PID `6010`) :

```
# /etc/udev/rules.d/99-usb-serial.rules
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", \
  ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="ttyDmx"
```

Recharger après modification :
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Installation

### 1. Cloner et builder

```bash
cd ~/qbo_ws
colcon build --packages-select qbo_msgs qbo_arduqbo
source install/setup.bash
```

### 2. Vérifier que les servos sont détectables

```bash
python3 - << 'EOF'
from dynamixel_sdk import *
ph = PortHandler("/dev/ttyDmx")
pm = PacketHandler(1.0)
ph.setBaudRate(1000000)
ph.openPort()
for id in range(1, 10):
    model, result, _ = pm.ping(ph, id)
    if result == 0:
        print(f"✅ ID {id} trouvé, modèle {model}")
ph.closePort()
EOF
```

Résultat attendu :
```
✅ ID 1 trouvé, modèle 12
✅ ID 2 trouvé, modèle 12
```

### 3. Lancer le nœud

```bash
# Via le binaire direct (recommandé pour debug)
~/qbo_ws/install/qbo_arduqbo/lib/qbo_arduqbo/qbo_dynamixel \
  --ros-args --params-file /home/nvidia/qbo_ws/src/qbo_arduqbo/config/dynamixel_config.yaml

# Via ros2 run
ros2 run qbo_arduqbo qbo_dynamixel \
  --ros-args --params-file /home/nvidia/qbo_ws/src/qbo_arduqbo/config/dynamixel_config.yaml

# Via launch (avec les autres nœuds Néo)
ros2 launch qbo_arduqbo qbo_arduqbo.launch.py
```

---

## Configuration YAML

Fichier : `config/dynamixel_config.yaml`

```yaml
dynamixel_hardware:
  ros__parameters:
    dynamixel.usb_port: "/dev/ttyDmx"
    dynamixel.baud_rate: 1000000
    dynamixel.protocol_version: 1.0
    dynamixel.motor_keys: ["motor_1", "motor_2"]

    # Servo 1 : Pan (gauche/droite)
    dynamixel.motors.motor_1.name: head_pan_joint
    dynamixel.motors.motor_1.id: 1
    dynamixel.motors.motor_1.neutral: 530          # tick position centrale
    dynamixel.motors.motor_1.ticks: 1024
    dynamixel.motors.motor_1.torque_limit: 800     # /1023
    dynamixel.motors.motor_1.max_speed: 3.0        # rad/s max
    dynamixel.motors.motor_1.min_angle_degrees: -70.0
    dynamixel.motors.motor_1.max_angle_degrees: 70.0
    dynamixel.motors.motor_1.invert: false

    # Servo 2 : Tilt (haut/bas)
    dynamixel.motors.motor_2.name: head_tilt_joint
    dynamixel.motors.motor_2.id: 2
    dynamixel.motors.motor_2.neutral: 465
    dynamixel.motors.motor_2.ticks: 1024
    dynamixel.motors.motor_2.torque_limit: 800
    dynamixel.motors.motor_2.max_speed: 2.0        # rad/s max (tilt plus délicat)
    dynamixel.motors.motor_2.min_angle_degrees: -30.0
    dynamixel.motors.motor_2.max_angle_degrees: 20.0
    dynamixel.motors.motor_2.invert: false
```

> **Paramètres clés :**
> - `neutral` : position tick au centre mécanique — à calibrer physiquement
> - `torque_limit` : limite le couple (protection mécanique) — 800/1023 ≈ 78%
> - `max_speed` : vitesse max acceptée par le software — indépendante du yaml `max_speed` AX-12
> - `min/max_angle_degrees` : limites de sécurité — toute commande hors plage est clampée

---

## Commandes de test

### Centrer la tête

```bash
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint', 'head_tilt_joint'], position: [0.0, 0.0], velocity: [1.0, 1.0]}"
```

### Pan gauche/droite

```bash
# Pan à droite (+0.5 rad ≈ 28°)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [0.5], velocity: [1.0]}"

# Pan à gauche (-0.5 rad)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [-0.5], velocity: [1.0]}"

# Pan max droite (70° = 1.22 rad)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [1.22], velocity: [2.0]}"
```

### Tilt haut/bas

```bash
# Tilt haut (+0.3 rad ≈ 17°)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_tilt_joint'], position: [0.3], velocity: [0.5]}"

# Tilt bas (-0.4 rad ≈ -23°)
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_tilt_joint'], position: [-0.4], velocity: [0.5]}"
```

### Mouvement combiné

```bash
ros2 topic pub -1 /cmd_joints sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint', 'head_tilt_joint'], position: [0.5, -0.3], velocity: [2.0, 1.0]}"
```

### Contrôle du torque

```bash
# Désactiver le torque (mode passif — tête libre à la main)
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"
ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"

# Réactiver le torque
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
```

### Lire l'état des joints

```bash
ros2 topic echo /cmd_joints
```

### Lire les diagnostics

```bash
ros2 topic echo /diagnostics
# ou avec rqt
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

### Vérifier les ports série

```bash
ls -la /dev/ttyDmx /dev/ttyQboard*
fuser /dev/ttyDmx   # vérifier qu'aucun autre processus n'occupe le port
```

---

## Topics & Services

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_joints` | `sensor_msgs/JointState` | Entrée | Commandes position + vitesse |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Sortie | Température, charge, position, torque |

### Services

| Service | Type | Description |
|---|---|---|
| `/head_pan_joint/torque_enable` | `qbo_msgs/srv/TorqueEnable` | Active/désactive le torque du servo pan |
| `/head_tilt_joint/torque_enable` | `qbo_msgs/srv/TorqueEnable` | Active/désactive le torque du servo tilt |

### Format du message `/cmd_joints`

```
name:     ['head_pan_joint', 'head_tilt_joint']   # noms des joints à commander
position: [0.0, 0.0]                               # rad — clampé aux limites yaml
velocity: [1.0, 0.5]                               # rad/s — clampé à max_speed yaml
                                                   # velocity: [] pour conserver la vitesse actuelle
```

---

## Avantages

### Robustesse
- **Ping avec retry** au démarrage — résout les collisions de bus RS485 entre servos en daisy-chain
- **Lecture tolérante** — un servo qui ne répond pas (torque OFF, overload) ne plante plus le nœud
- **Clamp de position** — toute commande hors `[min_angle, max_angle]` est automatiquement limitée, empêchant les overload errors et les dommages mécaniques
- **Protection overload** — `Torque_Limit` configurable par servo dans le yaml

### Flexibilité
- **100% piloté par le yaml** — port, IDs, neutres, limites, vitesses max : aucune valeur hardcodée dans le code
- **Servos dynamiques** — ajouter un 3ème servo ne nécessite qu'une entrée yaml, pas de modification C++
- **Services torque** par servo — permet le mode passif (tête déplaçable à la main) servo par servo

### Observabilité
- **Diagnostics ROS2** complets sur `/diagnostics` : position, vitesse, température, charge, état torque
- **Alertes température** : WARN à 60°C, ERROR à 70°C
- **Logs détaillés** : chaque commande loggue la vitesse demandée, clampée, et le tick résultant

### Performance (Jetson Orin NX)
- Latency timer à 1ms sur le port USB-série (règle udev `latency_timer`)
- `WARN_THROTTLE` à 2s pour éviter le flood de logs en cas d'erreur répétée
- Lecture de 5 registres par servo par cycle (position, vitesse, charge, température, torque)

---

## Améliorations à venir

### Court terme

- **Sync Read** — lire les 5 registres des 2 servos en un seul paquet RS485 au lieu de 10 lectures individuelles, divisant par ~5 le temps de lecture
- **Auto torque-off** — éteindre le torque après N secondes d'inactivité (`auto_torque_off_timeout` déjà dans le yaml)

### Moyen terme

- **Action server `/move_head`** — interface de haut niveau avec feedback de progression, utilisable depuis le comportement de Néo
- **Profil de mouvement trapézoïdal** — accélération/décélération progressive pour des mouvements plus naturels
- **Intégration ros2_control** — exposer le hardware interface via le vrai `controller_manager` pour utiliser les controllers standard (joint_trajectory_controller, etc.)
- **Publisher `/joint_states`** — publier l'état des joints sur le topic standard pour la visualisation URDF dans RViz

### Long terme

- **Comportements de tête** — nœud de haut niveau : suivi de visage, regard aléatoire, nod/shake expressif
- **Détection d'obstacle mécanique** — utiliser `Present_Load` pour détecter un blocage et stopper le mouvement
- **Calibration automatique** — procédure de recherche automatique des positions neutres au démarrage

---

## Dépannage

### `Cannot ping servo ID X après 5 tentatives`

```bash
# Vérifier que le port existe
ls -la /dev/ttyDmx

# Scanner les IDs et baud rates disponibles
python3 -c "
from dynamixel_sdk import *
ph = PortHandler('/dev/ttyDmx')
pm = PacketHandler(1.0)
ph.setBaudRate(1000000)
ph.openPort()
for id in range(1, 10):
    model, result, _ = pm.ping(ph, id)
    if result == 0:
        print(f'ID {id} OK modele {model}')
ph.closePort()
"

# Vérifier les permissions
groups  # doit contenir dialout
sudo usermod -aG dialout nvidia  # si absent, puis reloguer
```

### `Overload error` sur un servo

Le servo a été forcé hors de ses limites mécaniques. Il passe en protection et ne répond plus.

```bash
# Couper l'alimentation 5 secondes, rebrancher
# Puis vérifier que les limites yaml sont correctes :
# min_angle_degrees / max_angle_degrees correspondent à la plage mécanique réelle
```

### Port occupé par un ancien processus

```bash
fuser /dev/ttyDmx
sudo kill -9 $(fuser /dev/ttyDmx 2>/dev/null)
# ou
pkill -f qbo_dynamixel
```

### Température servo 2 élevée (>50°C)

Normal en fonctionnement continu. L'AX-12A supporte jusqu'à 70°C.

- Réduire `torque_limit` dans le yaml (ex: 400 au lieu de 800)
- Réduire `max_speed`
- Activer `auto_torque_off` quand le robot est inactif

### Segmentation fault au redémarrage rapide

Le bus RS485 n'a pas eu le temps de se vider. Attendre 2-3 secondes entre deux lancements.

---------------------------------------------------------------------

## Notes de version

| Version | Date | Changements |
|---|---|---|
| 1.0 | Mars 2026 | Init — lecture position, écriture Goal_Position |
| 1.1 | Mars 2026 | Ping avec retry, séparation ping/config         |
| 1.2 | Mars 2026 | Lecture température/vitesse/torque, diagnostics |
| 1.3 | Mars 2026 | Services torque_enable, lecture tolérante       |
| 1.4 | Mars 2026 | Centrage au démarrage, clamp min/max_angle      |
| 1.5 | Mars 2026 | Contrôle vitesse Moving_Speed, clamp max_speed  |
| 1.6 | Mars 2026 | Config 100% yaml, suppression valeurs hardcodées|
| 1.7 | Mars 2026 | Relax moteurs, selon valeur yaml                |
