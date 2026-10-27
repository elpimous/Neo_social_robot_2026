#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_mission.py — Test d'endurance de navigation laser de Néo (QBo)
=======================================================================
Auteur : Vincent FOUCAULT

Boucle autonome sur N waypoints (5 par défaut) pendant une durée donnée (2 h) :

  1. Verrouille la tête pan/tilt (consigne republiée en continu) et vérifie
     qu'elle a rejoint sa position, puis que le laser est horizontal (TF).
  2. Démarre le moteur du RPLidar et attend des scans frais et valides.
  3. Initialise / attend la localisation AMCL.
  4. Pour chaque trajet WP(i) -> WP(i+1) :
       - pré-vol : tête en place, laser horizontal, scans frais
       - nettoyage des costmaps, envoi de l'objectif NavigateToPose
       - surveillance continue : timeout, scans périmés (lidar planté),
         dérive de la tête, perte de localisation => annulation + arrêt
       - nouvelles tentatives, puis saut au WP suivant si échec
  5. À l'arrivée : arrêt complet, ARRÊT DU LIDAR, pause aléatoire 5–30 s,
     redémarrage du lidar, scans frais, trajet suivant.
  6. Fin (durée atteinte, Ctrl+C ou défaut grave) : annulation, vitesse nulle,
     lidar arrêté, résumé écrit.

Journal : CSV par trajet + résumé texte dans log_dir.
"""

import csv
import datetime
import math
import os
import random
import signal
import threading
import time
import traceback

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


class MissionAbort(Exception):
    """Défaut grave : la mission s'arrête de façon sécurisée."""


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')
        self.cbg = ReentrantCallbackGroup()
        self._stop_evt = threading.Event()
        self._declare_all()
        self._read_params()

        # ---------- État partagé ----------
        self._head_pos = {}
        self._head_stamp = 0.0
        self._head_lock_active = False
        self._scan_last = 0.0
        self._scan_count = 0
        self._amcl_pose = None
        self._amcl_std_xy = float('inf')
        self._amcl_std_yaw = float('inf')
        self._amcl_count = 0
        self._odom_dist = 0.0
        self._last_odom_xy = None
        self._fb_recoveries = 0
        self._goal_handle = None

        # ---------- Waypoints ----------
        self.frame_id, self.initial_pose, self.waypoints = self._load_waypoints(self.p_waypoints_file)

        # ---------- ROS I/O ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=self.cbg)
        self.cli_clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', callback_group=self.cbg)
        self.cli_clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap', callback_group=self.cbg)
        self.cli_lidar_start = self.create_client(Empty, self.p_lidar_start_srv, callback_group=self.cbg)
        self.cli_lidar_stop = self.create_client(Empty, self.p_lidar_stop_srv, callback_group=self.cbg)

        self.pub_cmd_vel = self.create_publisher(Twist, self.p_cmd_vel_topic, 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        if self.p_head_cmd_type == 'float64_array':
            self.pub_head = self.create_publisher(Float64MultiArray, self.p_head_cmd_topic, 10)
        else:
            self.pub_head = self.create_publisher(JointState, self.p_head_cmd_topic, 10)

        self.create_subscription(LaserScan, self.p_scan_topic, self._scan_cb,
                                 qos_profile_sensor_data, callback_group=self.cbg)
        self.create_subscription(JointState, self.p_head_js_topic, self._joint_cb, 20,
                                 callback_group=self.cbg)
        self.create_subscription(Odometry, self.p_odom_topic, self._odom_cb, 20,
                                 callback_group=self.cbg)
        amcl_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, self.p_amcl_topic, self._amcl_cb,
                                 amcl_qos, callback_group=self.cbg)

        if self.p_head_enabled:
            self.create_timer(1.0 / max(self.p_head_rate, 0.05), self._head_timer_cb,
                              callback_group=self.cbg)

        # ---------- Journal ----------
        self._init_logs()
        seed = self.p_seed if self.p_seed >= 0 else int(time.time())
        self.rng = random.Random(seed)
        self.get_logger().info(
            f'Mission prête : {len(self.waypoints)} waypoints, durée {self.p_duration_min:.0f} min, '
            f'pauses {self.p_pause_min:.0f}–{self.p_pause_max:.0f} s, seed={seed}, log={self.csv_path}')

    # =====================================================================
    # Paramètres
    # =====================================================================
    def _declare_all(self):
        d = self.declare_parameter
        d('waypoints_file', '')
        d('duration_min', 120.0)
        d('max_laps', 0)
        d('pause_min_s', 5.0)
        d('pause_max_s', 30.0)
        d('random_seed', -1)
        d('start_from_nearest', True)
        d('leg_timeout_s', 240.0)
        d('max_attempts_per_leg', 3)
        d('retry_wait_s', 3.0)
        d('max_consecutive_failed_legs', 3)
        d('log_dir', '~/.ros/qbo_nav_test')
        d('cmd_vel_topic', '/qbo_arduqbo/base_ctrl/cmd_vel')
        d('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        d('amcl_pose_topic', '/amcl_pose')
        d('initial_pose_timeout_s', 600.0)
        d('loc.check_enabled', True)
        d('loc.max_xy_std_m', 0.60)
        d('loc.max_yaw_std_rad', 0.80)
        d('head.enabled', True)
        d('head.command_type', 'joint_state')
        d('head.command_topic', '/cmd_joints')
        d('head.joint_states_topic', '/joint_states')
        d('head.joint_names', ['head_pan_joint', 'head_tilt_joint'])
        d('head.lock_positions_rad', [0.0, 0.0])
        d('head.tilt_joint', 'head_tilt_joint')
        d('head.tilt_level_rad', float('nan'))   # si fourni (launch) : remplace la consigne tilt
        d('head.speed_rad_s', 0.6)
        d('head.reach_tolerance_rad', 0.02)
        d('head.drift_tolerance_rad', 0.035)
        d('head.republish_hz', 0.2)
        d('head.lock_timeout_s', 10.0)
        d('head.settle_time_s', 1.0)
        d('head.require_feedback', True)
        d('level.enabled', True)
        d('level.base_frame', 'base_footprint')
        d('level.laser_frame', 'laser')
        d('level.tolerance_deg', 1.5)
        d('lidar.control_enabled', True)
        d('lidar.stop_during_pause', True)
        d('lidar.start_service', '/start_motor')
        d('lidar.stop_service', '/stop_motor')
        d('lidar.scan_topic', '/scan')
        d('lidar.spinup_s', 2.0)
        d('lidar.fresh_scans_required', 5)
        d('lidar.start_timeout_s', 10.0)
        d('lidar.start_attempts', 3)
        d('lidar.max_scan_age_s', 0.6)

    def _read_params(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p_waypoints_file = os.path.expanduser(g('waypoints_file'))
        self.p_duration_min = float(g('duration_min'))
        self.p_max_laps = int(g('max_laps'))
        self.p_pause_min = float(g('pause_min_s'))
        self.p_pause_max = max(float(g('pause_max_s')), self.p_pause_min)
        self.p_seed = int(g('random_seed'))
        self.p_start_nearest = bool(g('start_from_nearest'))
        self.p_leg_timeout = float(g('leg_timeout_s'))
        self.p_max_attempts = max(1, int(g('max_attempts_per_leg')))
        self.p_retry_wait = float(g('retry_wait_s'))
        self.p_max_consec_fail = max(1, int(g('max_consecutive_failed_legs')))
        self.p_log_dir = os.path.expanduser(g('log_dir'))
        self.p_cmd_vel_topic = g('cmd_vel_topic')
        self.p_odom_topic = g('odom_topic')
        self.p_amcl_topic = g('amcl_pose_topic')
        self.p_init_pose_timeout = float(g('initial_pose_timeout_s'))
        self.p_loc_check = bool(g('loc.check_enabled'))
        self.p_loc_xy = float(g('loc.max_xy_std_m'))
        self.p_loc_yaw = float(g('loc.max_yaw_std_rad'))
        self.p_head_enabled = bool(g('head.enabled'))
        self.p_head_cmd_type = g('head.command_type')
        self.p_head_cmd_topic = g('head.command_topic')
        self.p_head_js_topic = g('head.joint_states_topic')
        self.p_head_names = list(g('head.joint_names'))
        self.p_head_lock = [float(v) for v in g('head.lock_positions_rad')]
        self.p_head_speed = float(g('head.speed_rad_s'))
        self.p_head_reach = float(g('head.reach_tolerance_rad'))
        self.p_head_drift = float(g('head.drift_tolerance_rad'))
        self.p_head_rate = float(g('head.republish_hz'))
        self.p_head_timeout = float(g('head.lock_timeout_s'))
        self.p_head_settle = float(g('head.settle_time_s'))
        self.p_head_need_fb = bool(g('head.require_feedback'))
        self.p_level_enabled = bool(g('level.enabled'))
        self.p_base_frame = g('level.base_frame')
        self.p_laser_frame = g('level.laser_frame')
        self.p_level_tol = float(g('level.tolerance_deg'))
        self.p_lidar_ctrl = bool(g('lidar.control_enabled'))
        self.p_lidar_stop_pause = bool(g('lidar.stop_during_pause'))
        self.p_lidar_start_srv = g('lidar.start_service')
        self.p_lidar_stop_srv = g('lidar.stop_service')
        self.p_scan_topic = g('lidar.scan_topic')
        self.p_lidar_spinup = float(g('lidar.spinup_s'))
        self.p_lidar_fresh = max(1, int(g('lidar.fresh_scans_required')))
        self.p_lidar_timeout = float(g('lidar.start_timeout_s'))
        self.p_lidar_attempts = max(1, int(g('lidar.start_attempts')))
        self.p_scan_max_age = float(g('lidar.max_scan_age_s'))

        if len(self.p_head_names) != len(self.p_head_lock):
            raise ValueError('head.joint_names et head.lock_positions_rad doivent avoir la même taille')
        # Le lidar est sur la tête : la consigne tilt = valeur qui met le laser horizontal,
        # la même que celle utilisée par head_laser_tf (une seule source : arg tilt_level_rad)
        tilt_level = float(g('head.tilt_level_rad'))
        tilt_name = g('head.tilt_joint')
        if not math.isnan(tilt_level) and tilt_name in self.p_head_names:
            self.p_head_lock[self.p_head_names.index(tilt_name)] = tilt_level

    def _load_waypoints(self, path):
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(f'Fichier de waypoints introuvable : "{path}"')
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        frame = data.get('frame_id', 'map')
        wps = []
        for i, w in enumerate(data.get('waypoints', []) or []):
            wps.append({
                'name': str(w.get('name', f'WP{i + 1}')),
                'x': float(w['x']), 'y': float(w['y']),
                'yaw': math.radians(float(w.get('yaw_deg', 0.0))),
            })
        if len(wps) < 2:
            raise ValueError('Il faut au moins 2 waypoints dans ' + path)
        # Garde-fou : waypoints d'exemple non remplis
        pts = {(round(w['x'], 2), round(w['y'], 2)) for w in wps}
        if len(pts) < len(wps):
            raise ValueError(
                f'Waypoints confondus dans {path} : remplis les coordonnées réelles '
                '(ros2 run qbo_nav_waypoints record_waypoints)')
        ip = data.get('initial_pose', {}) or {}
        init = None
        if bool(ip.get('enabled', False)):
            init = {'x': float(ip.get('x', 0.0)), 'y': float(ip.get('y', 0.0)),
                    'yaw': math.radians(float(ip.get('yaw_deg', 0.0)))}
        return frame, init, wps

    # =====================================================================
    # Callbacks
    # =====================================================================
    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        valid = 0
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                valid += 1
        if valid / n >= 0.05:          # scan exploitable (moteur lancé, pas un scan vide)
            self._scan_last = time.monotonic()
            self._scan_count += 1

    def _joint_cb(self, msg: JointState):
        hit = False
        for name, pos in zip(msg.name, msg.position):
            if name in self.p_head_names:
                self._head_pos[name] = pos
                hit = True
        if hit:
            self._head_stamp = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        if self._last_odom_xy is not None:
            dist = math.hypot(p.x - self._last_odom_xy[0], p.y - self._last_odom_xy[1])
            if dist < 0.5:             # ignore les sauts (reset odom)
                self._odom_dist += dist
        self._last_odom_xy = (p.x, p.y)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self._amcl_pose = msg.pose.pose
        self._amcl_std_xy = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        self._amcl_std_yaw = math.sqrt(max(c[35], 0.0))
        self._amcl_count += 1

    def _fb_cb(self, fb_msg):
        try:
            self._fb_recoveries = int(fb_msg.feedback.number_of_recoveries)
        except Exception:
            pass

    def _head_timer_cb(self):
        if self._head_lock_active:
            self._publish_head_cmd()

    # =====================================================================
    # Outils
    # =====================================================================
    def _sleep(self, seconds):
        """Attente interruptible. Retourne False si arrêt demandé."""
        return not self._stop_evt.wait(max(0.0, seconds))

    @staticmethod
    def _wait_future(fut, timeout):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.02)
        return True

    def _call_srv(self, client, request, label, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'Service indisponible : {label}')
            return False
        fut = client.call_async(request)
        if not self._wait_future(fut, timeout):
            self.get_logger().warn(f'Service sans réponse : {label}')
            return False
        return True

    def publish_zero(self, duration=0.5):
        t_end = time.monotonic() + duration
        while time.monotonic() < t_end:
            self.pub_cmd_vel.publish(Twist())
            time.sleep(0.05)

    def request_stop(self):
        self._stop_evt.set()

    # =====================================================================
    # Tête
    # =====================================================================
    def _publish_head_cmd(self, with_speed=False):
        """Envoie la consigne de verrouillage. La vitesse n'est transmise qu'au
        verrouillage initial (qbo_dynamixel réécrit Moving_Speed à chaque
        message qui en contient une) ; les renvois périodiques ne servent qu'à
        garder le couple actif (auto torque OFF après 20 s sans commande)."""
        if self.p_head_cmd_type == 'float64_array':
            msg = Float64MultiArray()
            msg.data = list(self.p_head_lock)
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.p_head_names)
            msg.position = list(self.p_head_lock)
            if with_speed:
                msg.velocity = [self.p_head_speed] * len(self.p_head_names)
        self.pub_head.publish(msg)

    def head_error(self):
        """Erreur max (rad) entre consigne et position mesurée, None si inconnue."""
        if time.monotonic() - self._head_stamp > 1.0:
            return None
        errs = []
        for name, target in zip(self.p_head_names, self.p_head_lock):
            if name not in self._head_pos:
                return None
            errs.append(abs(self._head_pos[name] - target))
        return max(errs) if errs else None

    def lock_head(self):
        if not self.p_head_enabled:
            return True
        self._head_lock_active = True
        self._publish_head_cmd(with_speed=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.p_head_timeout:
            err = self.head_error()
            if err is not None and err <= self.p_head_reach:
                if not self._sleep(self.p_head_settle):
                    return False
                err = self.head_error()
                if err is not None and err <= self.p_head_reach:
                    self.get_logger().info(f'Tête verrouillée (erreur {math.degrees(err):.1f}°)')
                    return True
            elif err is None and not self.p_head_need_fb \
                    and time.monotonic() - t0 > self.p_head_settle + 1.0:
                self.get_logger().warn('Tête : pas de retour /joint_states, verrou supposé (require_feedback=false)')
                return True
            if not self._sleep(0.1):
                return False
        err = self.head_error()
        self.get_logger().error(
            'Tête : position de verrouillage NON atteinte '
            + (f'(erreur {math.degrees(err):.1f}°)' if err is not None else '(aucun retour /joint_states)'))
        return False

    def check_level(self):
        """Inclinaison (°) du plan laser par rapport à la base, via TF.
        Le lidar étant sur la tête, cette TF (head_laser_tf) suit les angles
        MESURÉS des servos : c'est une vraie mesure d'horizontalité."""
        if not self.p_level_enabled:
            return True, 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            try:
                if self.tf_buffer.can_transform(self.p_base_frame, self.p_laser_frame, Time()):
                    tf = self.tf_buffer.lookup_transform(self.p_base_frame, self.p_laser_frame, Time())
                    q = tf.transform.rotation
                    zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)   # axe z laser projeté sur z base
                    tilt = math.degrees(math.acos(min(1.0, abs(zz))))  # abs : lidar monté à l'envers OK
                    return tilt <= self.p_level_tol, tilt
            except Exception:
                pass
            time.sleep(0.1)
        self.get_logger().error(f'TF {self.p_base_frame} -> {self.p_laser_frame} indisponible')
        return False, float('nan')

    # =====================================================================
    # Lidar
    # =====================================================================
    def scan_age(self):
        return time.monotonic() - self._scan_last if self._scan_last > 0 else float('inf')

    def start_lidar(self):
        for attempt in range(1, self.p_lidar_attempts + 1):
            if self._stop_evt.is_set():
                return False
            if self.p_lidar_ctrl:
                self._call_srv(self.cli_lidar_start, Empty.Request(), 'start_motor')
                if not self._sleep(self.p_lidar_spinup):
                    return False
            c0 = self._scan_count
            t0 = time.monotonic()
            while time.monotonic() - t0 < self.p_lidar_timeout:
                if self._scan_count - c0 >= self.p_lidar_fresh:
                    self.get_logger().info('Lidar en marche, scans frais OK')
                    return True
                if not self._sleep(0.05):
                    return False
            self.get_logger().warn(f'Lidar : pas de scans frais (tentative {attempt}/{self.p_lidar_attempts})')
            if self.p_lidar_ctrl:
                self._call_srv(self.cli_lidar_stop, Empty.Request(), 'stop_motor')
                self._sleep(1.0)
        return False

    def stop_lidar(self):
        if not (self.p_lidar_ctrl and self.p_lidar_stop_pause):
            return True
        ok = self._call_srv(self.cli_lidar_stop, Empty.Request(), 'stop_motor')
        if ok:
            self.get_logger().info('Lidar arrêté (pause)')
        return ok

    # =====================================================================
    # Localisation
    # =====================================================================
    def loc_ok(self):
        if not self.p_loc_check:
            return True
        return self._amcl_std_xy <= self.p_loc_xy and self._amcl_std_yaw <= self.p_loc_yaw

    def init_localization(self):
        c0 = self._amcl_count
        if self.initial_pose is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.frame_id
            msg.pose.pose.position.x = self.initial_pose['x']
            msg.pose.pose.position.y = self.initial_pose['y']
            qx, qy, qz, qw = yaw_to_quat(self.initial_pose['yaw'])
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance[0] = 0.25 ** 2
            msg.pose.covariance[7] = 0.25 ** 2
            msg.pose.covariance[35] = math.radians(15) ** 2
            for _ in range(3):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_initial_pose.publish(msg)
                if not self._sleep(0.5):
                    return False
            timeout = 20.0
            self.get_logger().info('Pose initiale envoyée à AMCL')
        else:
            timeout = self.p_init_pose_timeout
            if self._amcl_count == 0:
                self.get_logger().warn('En attente d\'un "2D Pose Estimate" dans RViz...')
        t0 = time.monotonic()
        last_log = t0
        while time.monotonic() - t0 < timeout:
            if self._amcl_count > c0 or (self.initial_pose is None and self._amcl_count > 0):
                self.get_logger().info(
                    f'Localisation OK (σxy={self._amcl_std_xy:.2f} m, σyaw={math.degrees(self._amcl_std_yaw):.0f}°)')
                return True
            if time.monotonic() - last_log > 15.0:
                self.get_logger().info('... toujours en attente de la localisation AMCL')
                last_log = time.monotonic()
            if not self._sleep(0.2):
                return False
        return False

    def nearest_index(self):
        if self._amcl_pose is None:
            return 0
        p = self._amcl_pose.position
        return min(range(len(self.waypoints)),
                   key=lambda i: math.hypot(self.waypoints[i]['x'] - p.x, self.waypoints[i]['y'] - p.y))

    # =====================================================================
    # Navigation
    # =====================================================================
    def clear_costmaps(self):
        self._call_srv(self.cli_clear_local, ClearEntireCostmap.Request(), 'clear local costmap', 2.0)
        self._call_srv(self.cli_clear_global, ClearEntireCostmap.Request(), 'clear global costmap', 2.0)

    def _make_pose(self, wp):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wp['x']
        ps.pose.position.y = wp['y']
        _, _, qz, qw = yaw_to_quat(wp['yaw'])
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _cancel_active_goal(self):
        gh = self._goal_handle
        if gh is None:
            return
        try:
            fut = gh.cancel_goal_async()
            self._wait_future(fut, 3.0)
        except Exception:
            pass
        self._goal_handle = None

    def navigate_to(self, wp):
        """Retourne un statut : success / aborted / canceled / timeout / lidar_stale /
        head_drift / loc_lost / rejected / send_timeout / shutdown."""
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(wp)
        self._fb_recoveries = 0
        send_fut = self.nav_client.send_goal_async(goal, feedback_callback=self._fb_cb)
        if not self._wait_future(send_fut, 10.0):
            return 'send_timeout'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return 'rejected'
        self._goal_handle = gh
        res_fut = gh.get_result_async()

        t0 = time.monotonic()
        drift_since = None
        reason = None
        while not res_fut.done():
            if self._stop_evt.is_set():
                reason = 'shutdown'
            elif time.monotonic() - t0 > self.p_leg_timeout:
                reason = 'timeout'
            elif self.scan_age() > self.p_scan_max_age:
                reason = 'lidar_stale'
            elif not self.loc_ok():
                reason = 'loc_lost'
            elif self.p_head_enabled:
                err = self.head_error()
                if err is not None and err > self.p_head_drift:
                    drift_since = drift_since or time.monotonic()
                    if time.monotonic() - drift_since > 0.5:
                        reason = 'head_drift'
                else:
                    drift_since = None
            if reason:
                break
            time.sleep(0.05)

        if reason:
            self.get_logger().warn(f'Trajet interrompu : {reason}')
            self._cancel_active_goal()
            self._wait_future(res_fut, 5.0)
            self.publish_zero(0.5)
            return reason

        self._goal_handle = None
        status = res_fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'success'
        if status == GoalStatus.STATUS_CANCELED:
            return 'canceled'
        return 'aborted'

    def preflight(self):
        """Tête en place + laser horizontal + scans frais. Lève MissionAbort sinon."""
        if self.p_head_enabled:
            err = self.head_error()
            if err is None or err > self.p_head_reach:
                if not self.lock_head():
                    raise MissionAbort('tête impossible à verrouiller')
        ok, tilt = self.check_level()
        if not ok:
            raise MissionAbort(f'laser non horizontal ({tilt:.1f}° > {self.p_level_tol:.1f}°)')
        if self.scan_age() > self.p_scan_max_age:
            if not self.start_lidar():
                raise MissionAbort('lidar ne redémarre pas')

    # =====================================================================
    # Journal
    # =====================================================================
    def _init_logs(self):
        os.makedirs(self.p_log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.p_log_dir, f'nav_test_{stamp}.csv')
        self.summary_path = os.path.join(self.p_log_dir, f'nav_test_{stamp}_resume.txt')
        self._csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(['heure', 'trajet', 'tour', 'depart', 'arrivee', 'resultat', 'tentatives',
                            'duree_s', 'distance_m', 'recoveries', 'erreur_xy_m', 'erreur_yaw_deg',
                            'sigma_xy_m', 'pause_s', 'note'])
        self._csv_file.flush()
        self.stats = {'legs': 0, 'ok': 0, 'fail': 0, 'dist': 0.0, 'reasons': {}, 'recoveries': 0}

    def _log_leg(self, **kw):
        self._csv.writerow([
            datetime.datetime.now().isoformat(timespec='seconds'), kw['leg'], kw['lap'], kw['src'], kw['dst'],
            kw['result'], kw['attempts'], f"{kw['duration']:.1f}", f"{kw['dist']:.2f}", kw['recov'],
            f"{kw['err_xy']:.3f}" if kw['err_xy'] is not None else '',
            f"{kw['err_yaw']:.1f}" if kw['err_yaw'] is not None else '',
            f'{self._amcl_std_xy:.3f}', f"{kw['pause']:.1f}", kw.get('note', '')])
        self._csv_file.flush()

    def _write_summary(self, t_start, reason_end):
        el = time.monotonic() - t_start
        s = self.stats
        rate = 100.0 * s['ok'] / s['legs'] if s['legs'] else 0.0
        lines = [
            'Néo — test d\'endurance navigation laser (qbo_nav_waypoints)',
            f'Fin : {reason_end}',
            f'Durée : {el / 60:.1f} min',
            f'Trajets : {s["legs"]}  réussis : {s["ok"]}  échoués : {s["fail"]}  ({rate:.1f} % de réussite)',
            f'Distance parcourue (odom) : {s["dist"]:.1f} m',
            f'Recoveries Nav2 : {s["recoveries"]}',
            'Causes d\'échec/interruption : ' + (', '.join(f'{k}={v}' for k, v in s['reasons'].items()) or 'aucune'),
            f'Détail : {self.csv_path}',
        ]
        try:
            with open(self.summary_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception:
            pass
        for ln in lines:
            self.get_logger().info(ln)

    # =====================================================================
    # Mission
    # =====================================================================
    def run(self):
        t_start = time.monotonic()
        end_reason = 'inconnue'
        try:
            end_reason = self._mission(t_start)
        except MissionAbort as e:
            end_reason = f'ARRÊT SÉCURISÉ : {e}'
            self.get_logger().error(end_reason)
        except Exception as e:  # noqa: BLE001
            end_reason = f'EXCEPTION : {e}'
            self.get_logger().error(end_reason + '\n' + traceback.format_exc())
        finally:
            self._cancel_active_goal()
            self.publish_zero(0.5)
            if self.p_lidar_ctrl:
                self._call_srv(self.cli_lidar_stop, Empty.Request(), 'stop_motor', 2.0)
            self._write_summary(t_start, end_reason)
            try:
                self._csv_file.close()
            except Exception:
                pass

    def _mission(self, t_start):
        log = self.get_logger()

        log.info('Attente du serveur NavigateToPose (Nav2)...')
        while not self.nav_client.wait_for_server(timeout_sec=5.0):
            if self._stop_evt.is_set():
                return 'arrêt demandé'
            log.info('... Nav2 pas encore actif')

        if not self.lock_head():
            raise MissionAbort('tête impossible à verrouiller')
        ok, tilt = self.check_level()
        if not ok:
            raise MissionAbort(f'laser non horizontal ({tilt:.1f}°) — corrige head.lock_positions_rad')
        log.info(f'Laser horizontal ({tilt:.2f}°)')

        if not self.start_lidar():
            raise MissionAbort('lidar : aucun scan exploitable')
        if not self.init_localization():
            raise MissionAbort('localisation AMCL non initialisée')

        n = len(self.waypoints)
        idx = self.nearest_index() if self.p_start_nearest else 0
        start_idx = idx
        prev_name = 'départ'
        lap = 1
        leg = 0
        consec_fail = 0
        duration_s = self.p_duration_min * 60.0
        log.info(f'=== MISSION DÉMARRÉE : premier objectif {self.waypoints[idx]["name"]} ===')

        while not self._stop_evt.is_set():
            wp = self.waypoints[idx]
            leg += 1
            log.info(f'--- Trajet {leg} (tour {lap}) : {prev_name} -> {wp["name"]} ---')

            attempts = 0
            result = 'none'
            recov_total = 0
            t_leg = time.monotonic()
            d0 = self._odom_dist
            while attempts < self.p_max_attempts and not self._stop_evt.is_set():
                attempts += 1
                self.preflight()
                self.clear_costmaps()
                result = self.navigate_to(wp)
                recov_total += self._fb_recoveries
                if result in ('success', 'shutdown'):
                    break
                self.stats['reasons'][result] = self.stats['reasons'].get(result, 0) + 1
                if result == 'loc_lost':
                    raise MissionAbort(
                        f'localisation perdue (σxy={self._amcl_std_xy:.2f} m) — robot immobilisé')
                if result == 'head_drift':
                    self.lock_head()
                log.warn(f'Échec tentative {attempts}/{self.p_max_attempts} ({result})')
                if not self._sleep(self.p_retry_wait):
                    break

            if result == 'shutdown' or self._stop_evt.is_set():
                return 'arrêt demandé (Ctrl+C)'

            dur = time.monotonic() - t_leg
            dist = self._odom_dist - d0
            self.stats['legs'] += 1
            self.stats['dist'] += dist
            self.stats['recoveries'] += recov_total

            err_xy = err_yaw = None
            if result == 'success' and self._amcl_pose is not None:
                p = self._amcl_pose
                err_xy = math.hypot(p.position.x - wp['x'], p.position.y - wp['y'])
                dyaw = quat_to_yaw(p.orientation) - wp['yaw']
                err_yaw = abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw))))

            pause = 0.0
            if result == 'success':
                self.stats['ok'] += 1
                consec_fail = 0
                log.info(f'Arrivé à {wp["name"]} en {dur:.0f} s, {dist:.1f} m'
                         + (f', erreur {err_xy * 100:.0f} cm / {err_yaw:.0f}°' if err_xy is not None else ''))
            else:
                self.stats['fail'] += 1
                consec_fail += 1
                log.error(f'{wp["name"]} non atteint après {attempts} tentatives ({result}) -> waypoint suivant')
                if consec_fail >= self.p_max_consec_fail:
                    self._log_leg(leg=leg, lap=lap, src=prev_name, dst=wp['name'], result=result,
                                  attempts=attempts, duration=dur, dist=dist, recov=recov_total,
                                  err_xy=None, err_yaw=None, pause=0.0)
                    raise MissionAbort(f'{consec_fail} trajets consécutifs en échec')

            # ---------- Pause au waypoint : robot immobile, lidar coupé ----------
            elapsed = time.monotonic() - t_start
            mission_over = elapsed >= duration_s or (self.p_max_laps > 0 and lap >= self.p_max_laps
                                                     and (idx + 1) % n == start_idx)
            if result == 'success':
                self.publish_zero(0.5)
                self.stop_lidar()
                pause = self.rng.uniform(self.p_pause_min, self.p_pause_max)
                if not mission_over:
                    log.info(f'Pause {pause:.0f} s à {wp["name"]} '
                             f'(écoulé {elapsed / 60:.1f}/{self.p_duration_min:.0f} min)')
                    if not self._sleep(pause):
                        self._log_leg(leg=leg, lap=lap, src=prev_name, dst=wp['name'], result=result,
                                      attempts=attempts, duration=dur, dist=dist, recov=recov_total,
                                      err_xy=err_xy, err_yaw=err_yaw, pause=pause, note='interrompu')
                        return 'arrêt demandé (Ctrl+C)'

            self._log_leg(leg=leg, lap=lap, src=prev_name, dst=wp['name'], result=result,
                          attempts=attempts, duration=dur, dist=dist, recov=recov_total,
                          err_xy=err_xy, err_yaw=err_yaw, pause=pause)

            if mission_over:
                return f'mission terminée ({elapsed / 60:.1f} min, {lap} tour(s))'

            # ---------- Redémarrage pour le trajet suivant ----------
            if result == 'success' or self.scan_age() > self.p_scan_max_age:
                if not self.start_lidar():
                    raise MissionAbort('lidar ne redémarre pas après la pause')

            prev_name = wp['name']
            idx = (idx + 1) % n
            if idx == start_idx:
                lap += 1

        return 'arrêt demandé'


def main(args=None):
    # Gestion propre du Ctrl+C : on veut annuler le but, stopper le robot et le lidar
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)

    def _sigterm(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        node = WaypointMission()
    except Exception as e:  # noqa: BLE001
        rclpy.logging.get_logger('waypoint_mission').fatal(str(e))
        rclpy.try_shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    mission = threading.Thread(target=node.run, name='mission', daemon=True)
    mission.start()

    try:
        while rclpy.ok() and mission.is_alive():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C : arrêt sécurisé en cours...')
    finally:
        node.request_stop()
        t_end = time.monotonic() + 10.0
        while mission.is_alive() and time.monotonic() < t_end:
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception:
                break
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
