# Néo — Aide-mémoire des commandes (navigation laser sur waypoints)

**Auteur : Vincent FOUCAULT** — Jetson Orin NX `nvidia@192.168.1.23`, ROS 2 Humble

> Règle d'or : **toujours commencer par `cd ~`** dans chaque terminal (un terminal resté dans
> un dossier supprimé fait planter Nav2 : `getcwd(): No such file or directory`).

---

## 0. Connexion et tmux

```bash
# Depuis le PC ROG
ssh nvidia@192.168.1.23          # terminal normal
ssh -X nvidia@192.168.1.23       # avec affichage déporté (pour RViz)
```

Tout lancer dans **tmux** (survit à une coupure SSH) :

| Action | Commande |
|---|---|
| Créer la session | `tmux new -s neo` |
| Revenir dans la session | `tmux attach -t neo` |
| Nouvelle fenêtre | `Ctrl+B` puis `C` |
| Fenêtre suivante / précédente | `Ctrl+B` puis `N` / `P` |
| Aller à la fenêtre n° X | `Ctrl+B` puis `0`…`9` |
| Quitter sans rien arrêter | `Ctrl+B` puis `D` |
| Tout fermer | `tmux kill-server` |

---

## TERMINAL 1 — Base + tête (qbo_startup)

```bash
cd ~
source ~/qbo_ws/install/setup.bash
ros2 launch qbo_bringup qbo_startup.launch.py
```

À relancer **après chaque flash du firmware** ou redémarrage.

Vérifications rapides (dans un autre terminal) :

```bash
ros2 topic hz /qbo_arduqbo/base_ctrl/odom      # ~20 Hz attendus (Ctrl+C)
ros2 topic echo /joint_states --once           # head_pan_joint / head_tilt_joint
```

---

## TERMINAL 2 — Navigation sur waypoints (avec boîte noire)

```bash
cd ~
source ~/qbo_ws/install/setup.bash
ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py
```

Options utiles (à ajouter à la fin de la commande) :

| Option | Effet |
|---|---|
| `duration_min:=10` | test court (défaut : 120 min) |
| `waypoints:=$HOME/neo_waypoints.yaml` | autre fichier de waypoints |
| `use_rviz:=true` | RViz lancé avec (déconseillé : charge le Jetson) |
| `monitor:=false` | sans boîte noire |
| `tilt_level_rad:=0.0123` | calibration d'horizontalité du lidar |

Démarrage : la mission attend un **2D Pose Estimate** dans RViz (terminal 3), puis part seule.
Arrêt sécurisé : `Ctrl+C`.

---

## TERMINAL 3 — RViz (2D Pose Estimate, surveillance)

Depuis le PC ROG, connexion avec `ssh -X`, puis :

```bash
cd ~
source ~/qbo_ws/install/setup.bash
rviz2 -d ~/qbo_ws/install/qbo_nav_waypoints/share/qbo_nav_waypoints/rviz/neo_nav.rviz
```

- Si RViz plante au démarrage : `export LIBGL_ALWAYS_SOFTWARE=1` avant `rviz2`.
- **2D Pose Estimate** : cliquer la vraie position de Néo, glisser dans sa direction,
  jusqu'à ce que le scan rouge se pose sur les murs.
- **Fermer RViz une fois la mission partie** (il tourne sur le Jetson et le ralentit).
- `Segmentation fault` à la fermeture de RViz : bug connu de RViz2 Humble, sans conséquence.

---

## TERMINAL 4 — Diagnostics pendant que Néo roule

```bash
cd ~
source ~/qbo_ws/install/setup.bash
```

| But | Commande |
|---|---|
| Fluidité : vitesse à chaque étage (30 s) | `ros2 run qbo_nav_waypoints speed_trace` |
| Idem sur 60 s | `ros2 run qbo_nav_waypoints speed_trace --ros-args -p duration_s:=60` |
| Boîte noire en direct | `tail -f $(ls -t ~/.ros/qbo_nav_test/blackbox_*.log \| head -1)` |
| Logs de la mission uniquement | `tail -f ~/.ros/log/latest/launch.log \| grep --line-buffered -E "waypoint_mission\|patience\|Running spin"` |
| Capteur de sol (doit rester 0.11–0.36 m) | `ros2 topic echo /qbo_arduqbo/sens_ctrl/distance_sensors_state/floor_sensor --field points` |
| Tension batterie | `ros2 topic echo /diagnostics \| grep -A1 "key: Voltage"` |
| Charge CPU / RAM | `htop` |

---

## Résultats d'un test

```bash
cat $(ls -t ~/.ros/qbo_nav_test/*_resume.txt | head -1)     # résumé
ls -t ~/.ros/qbo_nav_test/                                    # CSV par trajet, boîtes noires, traces
```

---

## Après un PLANTAGE (Jetson injoignable / redémarré)

```bash
uptime                                                         # quelques minutes = le Jetson a REDÉMARRÉ
tail -60 $(ls -t ~/.ros/qbo_nav_test/blackbox_*.log | sed -n 2p)   # boîte noire de la session plantée
journalctl -b -1 -p warning --no-pager | tail -50             # messages système avant la panne
```

Si `uptime` est long : seul le Wi-Fi a décroché (le Jetson n'a pas redémarré).

---

## Outils ponctuels

### Enregistrer X waypoints (nombre libre, minimum 2)

**Prérequis :** terminal 1 (qbo_startup) + terminal 2 (navigation) lancés, RViz ouvert
(terminal 3) et **2D Pose Estimate fait** (le scan rouge doit coller aux murs).

**Terminal 4 — méthode 1 : clic dans RViz** (remplacer `X` par le nombre voulu) :

```bash
cd ~
source ~/qbo_ws/install/setup.bash
ros2 run qbo_nav_waypoints record_waypoints --ros-args -p count:=X -p output:=$HOME/neo_waypoints.yaml
```

Dans RViz, outil **Publish Point** : cliquer les X points **dans l'ordre de passage**.
L'outil s'arrête seul après le X-ième point et écrit le fichier. L'orientation de chaque point
est calculée vers le suivant.

**Terminal 4 — méthode 2 : robot placé à la main** (orientation réelle enregistrée) :

```bash
ros2 run qbo_nav_waypoints record_waypoints --ros-args -p mode:=robot -p count:=X -p output:=$HOME/neo_waypoints.yaml
```

Déplacer Néo (téléop ou à la main) sur chaque point, puis **Entrée** pour l'enregistrer.

Bonnes pratiques :
- au moins 50 cm de dégagement autour de chaque point, jamais dans une porte ;
- points reliés par des passages larges (≥ 70 cm) ;
- la boucle est fermée automatiquement : WP1 → … → WPX → WP1.

**Vérifier puis utiliser :**

```bash
cat ~/neo_waypoints.yaml
ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py waypoints:=$HOME/neo_waypoints.yaml
```

**En faire les waypoints par défaut** (sans `waypoints:=`) :

```bash
cp ~/neo_waypoints.yaml ~/qbo_ws/src/qbo_nav_waypoints/config/waypoints.yaml
```

(⚠️ écrasé à la prochaine mise à jour du package : envoyer le fichier à Claude pour l'intégrer au dépôt GitHub.)

### Diagnostic base / lidar (odométrie, angle du lidar, seuil de démarrage)
Terminal 1 + terminal 2 lancés, **aucun objectif en cours**, 1,2 m dégagés devant Néo :

```bash
ros2 run qbo_nav_waypoints diagnose_base_lidar
```

### Calibration de l'horizontalité du lidar (niveau à bulle sur le RPLidar)

```bash
ros2 run qbo_nav_waypoints calibrate_head_level        # touches + - ++ -- p q
```

### Test manuel de la base (roues en l'air : seul le recul et la rotation passent)

```bash
ros2 topic pub -r 10 /qbo_arduqbo/base_ctrl/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05}}"
ros2 topic pub -r 10 /qbo_arduqbo/base_ctrl/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```

(Roues en l'air, l'avance est bloquée par le capteur anti-chute : c'est normal.)

---

## Mise à jour du package depuis GitHub

```bash
cd ~
cd /tmp && rm -rf Neo_social_robot_2026 && git clone --depth 1 https://github.com/elpimous/Neo_social_robot_2026.git
rm -rf ~/qbo_ws/src/qbo_nav_waypoints && cp -r /tmp/Neo_social_robot_2026/qbo_nav_waypoints ~/qbo_ws/src/
cd ~/qbo_ws && colcon build --symlink-install --packages-select qbo_nav_waypoints && source install/setup.bash
cd ~
```

## Flash du firmware QBoard1 (ATmega**1280**)

```bash
pkill -INT -f "ros2 launch"; sleep 3          # arrête TOUT, y compris qbo_startup
cd /tmp && rm -rf Neo_social_robot_2026 && git clone --depth 1 https://github.com/elpimous/Neo_social_robot_2026.git
cd /tmp/Neo_social_robot_2026/QBoard1Firm
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega1280 . && \
arduino-cli upload  --fqbn arduino:avr:mega:cpu=atmega1280 -p /dev/ttyQboard1 .
cd ~                                          # ne pas rester dans /tmp/...
```

Puis **relancer le terminal 1 (qbo_startup)**.

## Tout arrêter proprement

```bash
pkill -INT -f "ros2 launch"; sleep 3
pkill -f "qbo_nav_waypoints|nav2_|rplidar_node|rviz2"
ros2 daemon stop
```

## Préparation du Jetson (une seule fois, déjà faite le 29/10/2026)

```bash
sudo bash ~/qbo_ws/src/qbo_nav_waypoints/scripts/neo_debug_setup.sh   # journal persistant, Wi-Fi sans veille
sudo nvpmodel -m 0 && sudo jetson_clocks                                # pleine puissance
sudo apt install -y tmux
```
