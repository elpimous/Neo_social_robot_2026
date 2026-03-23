# 🤖 Dynamixel Controller — QBo / AX-12A (ROS2)

Contrôleur ROS2 optimisé pour servos **AX-12A (Protocol 1.0)** utilisant `DynamixelWorkbench`.

---

# 📦 Fonctionnalités

* ✅ Contrôle multi-servos
* ✅ **Sync Write (écriture groupée)** → performance maximale
* ✅ Lecture position en temps réel (`/joint_states`)
* ✅ Publication TF (quaternions)
* ✅ Paramètres dynamiques (ROS2)
* ✅ Auto torque OFF (sécurité)
* ✅ Diagnostics complets (température, tension, charge…)

---

# 🏗️ Architecture

## 🔁 Boucle principale (50 Hz)

```
/cmd_joints
   ↓
jointCmdCallback()
   ↓
setAngle() (pas de write direct)
   ↓
flushSyncWrite() (1 seule trame série)
   ↓
itemRead() (position)
   ↓
/joint_states + TF
```

---

# ⚡ Sync Write (optimisation clé)

Sans Sync Write :

```
2 servos → 4 writes → 4 trames
```

Avec Sync Write :

```
2 servos → 1 seule trame
```

➡️ **4× moins de trafic série**

---

# 🔧 Installation

```bash
cd ~/qbo_ws
colcon build --packages-select qbo_arduqbo
source install/setup.bash
```

---

# 🚀 Lancement

```bash
ros2 launch qbo_arduqbo dynamixel.launch.py
```

---

# 🎮 Commandes de base

## 📡 Publier une commande (position seule)

```bash
ros2 topic pub /cmd_joints sensor_msgs/msg/JointState "{
  name: ['head_pan_joint', 'head_tilt_joint'],
  position: [0.5, -0.3]
}"
```

---

## ⚡ Position + vitesse

```bash
ros2 topic pub /cmd_joints sensor_msgs/msg/JointState "{
  name: ['head_pan_joint', 'head_tilt_joint'],
  position: [0.5, -0.3],
  velocity: [1.0, 0.5]
}"
```

---

## 🎯 Commande d’un seul moteur

```bash
ros2 topic pub /cmd_joints sensor_msgs/msg/JointState "{
  name: ['head_pan_joint'],
  position: [0.2],
  velocity: [0.8]
}"
```

---

# 📊 Lecture des données

## 🔍 Position des joints

```bash
ros2 topic echo /joint_states
```

---

## 🌡️ Température

```bash
ros2 topic echo /diagnostics | grep Temperature
```

---

## 🔋 Tension

```bash
ros2 topic echo /diagnostics | grep Voltage
```

---

## ⚙️ Charge (load)

```bash
ros2 topic echo /diagnostics | grep Load
```

---

## 🧠 Tout voir (debug complet)

```bash
ros2 topic echo /diagnostics
```

---

# 🔌 Services (Torque)

## Activer torque

```bash
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
```

---

## Désactiver torque

```bash
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"
```

---

# ⚙️ Paramètres dynamiques

## Modifier torque_limit

```bash
ros2 param set /dynamixel_controller dynamixel.motors.head_pan_joint.torque_limit 500
```

---

## Modifier angle max

```bash
ros2 param set /dynamixel_controller dynamixel.motors.head_pan_joint.max_angle_degrees 45
```

---

# 🧪 Tests rapides

## Centrer les moteurs

```bash
ros2 topic pub /cmd_joints sensor_msgs/msg/JointState "{
  name: ['head_pan_joint', 'head_tilt_joint'],
  position: [0.0, 0.0]
}"
```

---

## Mouvement sinus simple

```bash
ros2 topic pub /cmd_joints sensor_msgs/msg/JointState -r 10 "{
  name: ['head_pan_joint'],
  position: [0.5]
}"
```

---

# 🧠 TF (Transformations)

* Utilise des **quaternions**
* Générés via `setRPY()`
* Exemple :

```
pan  → rotation Z
tilt → rotation Y
```

---

# 📡 Topics ROS2

| Topic           | Type       | Description      |
| --------------- | ---------- | ---------------- |
| `/cmd_joints`   | JointState | Commande moteurs |
| `/joint_states` | JointState | Retour position  |
| `/tf`           | TF         | Transformations  |

---

# ⚠️ Limitations

* ❌ Protocol 1.0 → pas de Sync Read
* ❌ Pas de retour vitesse réelle
* ❌ Pas de contrôle couple direct
* ✔️ Lecture position uniquement

---

# 🔥 Conseils performance

* ✔️ Utiliser Sync Write (déjà fait)
* ✔️ Éviter logs à 50Hz
* ✔️ Garder baudrate élevé (≥ 500000)

---

# 🐛 Debug

## Vérifier connexion

```bash
ls /dev/ttyUSB*
```

---

## Tester ping

```bash
ros2 run dynamixel_workbench_controllers find_dynamixel
```

---

## Vérifier paramètres

```bash
ros2 param list
```

---

# 🚀 Conclusion

Ce contrôleur est :

* ⚡ **Optimisé bus série (Sync Write)**
* 🧠 **Propre (zero alloc hot path)**
* 🤖 **Adapté robot temps réel**

---

# Vincent FOUCAULT 23 Mars 2026