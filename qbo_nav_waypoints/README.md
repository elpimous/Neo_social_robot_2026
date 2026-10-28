# qbo_nav_waypoints — Néo : test d'endurance navigation laser

**Auteur : Vincent FOUCAULT** — ROS2 Humble, Jetson Orin NX, RPLidar A2

Néo boucle sur 5 waypoints de la carte de la maison pendant **2 h** :
tête verrouillée laser horizontal, lidar en marche pendant les trajets et
**arrêté pendant chaque pause** (5–30 s aléatoire), passage des portes **au
milieu**, chaîne de sécurité complète jusqu'aux moteurs.

```
ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py
```

---

## 1. Installation (sur le Jetson)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
cd ~/qbo_ws/src && git pull            # ou copie du dossier qbo_nav_waypoints
cd ~/qbo_ws && colcon build --symlink-install --packages-select qbo_nav_waypoints
source install/setup.bash
```

## 2. Config Néo prise en compte (relevée dans le dépôt)

| Élément | Valeur |
|---|---|
| Base / odom | `base_footprint`, `/qbo_arduqbo/base_ctrl/odom`, cmd `/qbo_arduqbo/base_ctrl/cmd_vel` |
| Lidar | `rplidar_node`, `/dev/ttyRpLidar`, 115200, mode Sensitivity, 10 Hz, services `/start_motor` `/stop_motor` |
| TF laser | **dynamique** (`head_laser_tf`) : `base_link -> head_pan_link -> head_tilt_link -> laser`, depuis les angles mesurés des servos, yaw de montage 180° |
| Carte | `qbo_navigation/map/map.yaml` |
| Tête | `qbo_dynamixel` : `/cmd_joints` (JointState), `head_pan_joint` / `head_tilt_joint`, 0 rad = neutre |
| Rayon robot | 0.20 m |

**Tête et auto torque OFF** : `qbo_dynamixel` coupe le couple après 20 s sans
commande. La mission renvoie la consigne toutes les 5 s (sans vitesse, pour ne
pas réécrire `Moving_Speed`) : la tête reste verrouillée pendant les 2 h.
Ne lance pas `head_tracker_cpp` / `ball_tracking_cpp` / `qbo_faces_tracking`
pendant le test : ils commanderaient la tête en même temps.

### Lidar au sommet de la tête (Ø ~20 cm, plan laser à ~0.49 m du sol)

Le plan laser suit le tilt de la tête : 3° vers le bas => le faisceau touche le
sol vers 9 m (faux obstacles, AMCL perturbé). D'où :

- **TF dynamique** calculée depuis `/joint_states` : AMCL et les costmaps voient
  la vraie inclinaison. Plus de `/joint_states` => plus de TF => Nav2 s'arrête.
- **Verrou serré** : tête en place à 1.1° près, dérive > 2° en roulant => arrêt
  et reverrouillage ; contrôle d'horizontalité avant chaque trajet (≤ 1.5°).
- **Une seule valeur de calibration**, `tilt_level_rad`, sert à la fois de
  consigne de verrouillage du tilt et de zéro de la TF.

**Calibration (une fois)** — niveau à bulle posé sur le RPLidar :

```bash
ros2 run qbo_nav_waypoints calibrate_head_level     # + - ++ -- p q
# => affiche tilt_level_rad:=<valeur> à reporter au lancement
```

Géométrie (mètre ruban, arguments du launch) : `head_center_x` (0.10, centre de
la tête en avant de l'axe des roues, repris de qbo_navigation), `head_center_z`
(0.34 au-dessus de base_link), `laser_above_center` (0.13). La hauteur influe peu
en 2D ; `head_center_x` compte pour la précision AMCL. Si la tête *monte* quand
le tilt augmente, ajoute `tilt_sign:=-1.0`.

**Angle mort** : à 0.49 m, le lidar ne voit pas ce qui est plus bas (pieds de
chaise, jouets, animaux, marches). Le collision monitor non plus. Pour le test
de 2 h, dégage le sol du parcours et reste dans les parages au premier tour.

**Prérequis au lancement** : `qbo_startup.launch.py` actif (base + tête), et
`qbo_navigation/navigation.launch.py` **arrêté** (ce launch le remplace :
lidar, TF laser, AMCL, Nav2). Si le lidar tourne déjà ailleurs :
`start_lidar_driver:=false publish_laser_tf:=false`.

## 3. Enregistrer les 5 waypoints

Nav2 lancé (ou juste map_server + RViz), puis :

```bash
# Clic sur la carte (RViz > "Publish Point") — orientation auto vers le WP suivant
ros2 run qbo_nav_waypoints record_waypoints --ros-args \
  -p output:=$HOME/qbo_ws/src/qbo_nav_waypoints/config/waypoints.yaml
# (puis recompiler, ou passer waypoints:=<fichier> au launch)

# ou : Néo placé physiquement sur chaque point, Entrée à chaque fois
ros2 run qbo_nav_waypoints record_waypoints --ros-args -p mode:=robot \
  -p output:=$HOME/qbo_ws/src/qbo_nav_waypoints/config/waypoints.yaml
```

Évite de poser un waypoint **dans** une porte ou à moins de 40 cm d'un mur.

## 4. Lancer le test

```bash
ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py tilt_level_rad:=<valeur calibrée>
# options utiles
#   use_rviz:=true                 visualisation
#   duration_min:=120              durée
#   map:=/chemin/autre_carte.yaml
#   robot_bringup:=$(ros2 pkg prefix qbo_bringup)/share/qbo_bringup/launch/qbo_startup.launch.py
#                                  si qbo_startup n'est PAS lancé par systemd
```

Au lancement :

- **Tête** : le nœud `head_lock` la verrouille immédiatement (pan 0, tilt
  `tilt_level_rad`) et la tient en permanence, mission active ou non.
- **RViz** (`rviz/neo_nav.rviz`) : carte, **scan lidar en rouge**, costmaps,
  chemin, particules AMCL, zones STOP (rouge) / RALENTI (orange).
  AMCL démarre sur une pose *provisoire* à l'origine de la carte, pour que le
  scan s'affiche tout de suite (souvent décalé des murs, c'est normal).
- Si `initial_pose.enabled: false` → avec **2D Pose Estimate**, clique la
  vraie position de Néo et glisse dans sa direction, jusqu'à ce que le scan
  rouge se superpose aux murs. La mission ne démarre **que** sur ce clic.

**Ctrl+C** = arrêt sécurisé : but annulé, vitesse nulle, lidar arrêté, résumé écrit.

## 5. Séquence d'un cycle

```
verrou tête ─► laser horizontal (TF mesurée) ─► lidar ON + 5 scans frais ─► AMCL OK
   │
   └─► [boucle] pré-vol (tête, niveau, scans) ─► clear costmaps ─► NavigateToPose
          surveillance : timeout | scan > 0.6 s | tête dérive > 4.5° | AMCL perdu
          ─► arrivée : vitesse 0 ─► LIDAR OFF ─► pause 5–30 s ─► LIDAR ON ─► WP suivant
```

Arrêts sécurisés (robot immobilisé, lidar coupé) : localisation perdue,
tête impossible à verrouiller, laser non horizontal, lidar qui ne redémarre
pas, 3 trajets consécutifs en échec.

## 6. Pourquoi Néo passe les portes au milieu

1. **Costmap global** : inflation 0.75 m, `cost_scaling_factor` 2.5 → chaque
   montant crée une pente de coût qui couvre toute l'ouverture ; le coût est
   minimal pile au centre.
2. **Smac Planner 2D** `cost_travel_multiplier: 3.0` → le chemin suit cette
   vallée de coût minimal (milieu des portes et couloirs).
3. **Regulated Pure Pursuit** à look-ahead court → suit le chemin sans couper,
   ralentit automatiquement près des montants.
4. **Collision monitor** à zones étroites (largeur du robot) → ne bloque pas
   dans une porte, stoppe net devant un vrai obstacle.

Réglages : centrage plus strict → `cost_travel_multiplier` 4.0 ; si Néo refuse
une porte étroite (« no valid path ») → baisser `inflation_radius` global à
0.6 ou vérifier `robot_radius` et la carte (pixels gris dans l'ouverture).

## 7. Résultats

`~/.ros/qbo_nav_test/nav_test_AAAAMMJJ_HHMMSS.csv` — une ligne par trajet :
résultat, tentatives, durée, distance odom, recoveries, erreur d'arrivée
(cm/°), σ AMCL, durée de pause.
`..._resume.txt` — taux de réussite, distance totale, causes d'échec.

## 8. Notes RPLidar

Le driver `rplidar_ros` expose `/start_motor` et `/stop_motor` (std_srvs/Empty).
Pendant la pause, le driver peut journaliser des erreurs « grab scan » : c'est
normal. Aucun `expected_update_rate` n'est configuré dans les costmaps, donc
Nav2 tolère l'absence de scans à l'arrêt ; c'est le nœud mission qui interdit
tout mouvement tant que les scans ne sont pas frais.
