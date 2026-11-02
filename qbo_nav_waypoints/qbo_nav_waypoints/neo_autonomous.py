#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
neo_autonomous.py — Néo se promène seul dans la maison
======================================================
Auteur : Vincent FOUCAULT

Comportement :
  1. Choisit au hasard sa prochaine destination parmi les waypoints (jamais celui
     où il est, de préférence ceux qu'il n'a pas visités depuis longtemps).
  2. En roulant, la tête regarde où il va : l'angle de pan suit le point du chemin
     planifié situé ~0.8 m devant lui (proportionnel au virage, anticipé avant que
     les roues tournent). Tilt bloqué à l'horizontale.
  3. Arrivé, il s'arrête 5 à 10 s et regarde autour de lui de façon réaliste :
     coups d'œil (profil « minimum jerk » = départ/arrivée doux) puis fixations,
     petits ajustements, parfois un regard en arrière ; avant de repartir, il
     regarde vers sa prochaine destination.
  4. Le lidar étant sur la tête, sa TF (head_laser_tf) suit le pan MESURÉ ; la
     vitesse de la tête est plafonnée (60°/s) pour ne pas déformer le scan.
  A. Allure variable : vitesse de croisière tirée à chaque trajet, ralentie dans
     les passages étroits (jamais sous 0.20 m/s : zone morte de la base).
  B. Batterie : « fatigué » => plus lent et pauses plus longues ; « faible » =>
     il rejoint son point de repos et s'arrête.

Sécurités : lidar muet ou localisation perdue => but annulé, attente ; trop
d'échecs consécutifs => arrêt. Ctrl+C : but annulé, tête recentrée, lidar arrêté.
"""

import csv
import math
import os
import random
import signal
import statistics
import threading
import time
from collections import deque
from datetime import datetime

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState, LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def min_jerk(s):
    """Profil minimum jerk (0..1 -> 0..1) : accélération et freinage doux, comme un mouvement humain."""
    s = max(0.0, min(1.0, s))
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


class Stop(Exception):
    pass


class NeoAutonomous(Node):

    def __init__(self):
        super().__init__('neo_autonomous')
        d = self.declare_parameter
        # ---- général
        d('waypoints_file', '')
        d('duration_min', 0.0)                 # 0 = illimité
        d('random_seed', -1)
        d('log_dir', '~/.ros/qbo_nav_test')
        d('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        d('base_frame', 'base_footprint')
        d('map_frame', 'map')
        d('leg_timeout_s', 240.0)
        d('max_consecutive_failures', 4)
        d('initial_pose_timeout_s', 900.0)
        # ---- pauses (3)
        d('pause.min_s', 3.0)
        d('pause.max_s', 5.0)
        d('pause.look_back_prob', 0.2)
        d('pause.fix_min_s', 0.6)
        d('pause.fix_max_s', 2.0)
        d('pause.min_saccade_deg', 15.0)
        d('pause.micro_deg', 2.5)
        d('pause.look_next_s', 1.0)            # regard vers la prochaine destination avant de repartir
        # ---- tête (2, 4)
        d('head.command_topic', '/cmd_joints')
        d('head.joint_states_topic', '/joint_states')
        d('head.pan_joint', 'head_pan_joint')
        d('head.tilt_joint', 'head_tilt_joint')
        d('head.tilt_level_rad', 0.0)          # réécrit par le launch
        d('head.pan_limit_deg', 60.0)          # butée absolue (servo ±70°, ménage le câble du cou)
        d('head.travel_limit_deg', 45.0)       # amplitude max EN ROULANT (tête qui suit le virage)
        d('pause.look_around', True)           # tête qui regarde autour pendant les pauses
        d('pause.limit_deg', 45.0)             # amplitude max PENDANT LES PAUSES (si look_around)
        d('head.max_speed_deg_s', 45.0)        # lidar sur la tête : scan redressé (deskew), vitesse modérée
        d('head.rate_hz', 20.0)
        d('head.lookahead_m', 0.8)
        d('head.gain', 1.0)                    # pan = gain x angle du virage
        d('head.smoothing_s', 0.35)            # constante de temps du filtre (déplacement)
        d('head.near_goal_m', 0.35)            # près du but : retour progressif face
        d('head.max_error_deg', 12.0)          # écart consigne/mesure toléré
        # ---- allure (A)
        d('speed.controller_node', '/controller_server')
        d('speed.param_name', 'FollowPath.desired_linear_vel')
        d('speed.cruise_min', 0.22)
        d('speed.cruise_max', 0.30)
        d('speed.narrow_speed', 0.20)
        d('speed.narrow_width_m', 0.85)        # passage plus étroit que ça => vitesse réduite
        d('speed.absolute_min', 0.20)          # zone morte base ~0.07 m/s, demande : jamais sous 0.20
        d('speed.scan_topic', '/scan_filtered')
        # ---- batterie (B) — LiFePO4 4S, tensions EN ROULAGE, à affiner avec le test d'autonomie
        d('battery.enabled', True)
        d('battery.tired_v', 11.5)
        d('battery.low_v', 11.0)
        d('battery.tired_speed_factor', 0.85)
        d('battery.tired_pause_factor', 1.6)
        d('battery.rest_waypoint', '')         # '' = premier waypoint du fichier
        # ---- sécurités
        d('safety.max_scan_age_s', 1.0)
        d('safety.max_loc_std_m', 0.8)
        d('lidar.start_service', '/start_motor')
        d('lidar.stop_service', '/stop_motor')
        d('lidar.stop_at_end', True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.P = {n: g(n) for n in self._parameters}
        P = self.P
        seed = int(P['random_seed'])
        self.rng = random.Random(None if seed < 0 else seed)
        self.pan_limit = math.radians(float(P['head.pan_limit_deg']))
        self.travel_limit = min(self.pan_limit, math.radians(float(P['head.travel_limit_deg'])))
        self.pause_limit = min(self.pan_limit, math.radians(float(P['pause.limit_deg'])))
        self.look_around_on = bool(P['pause.look_around'])
        self.head_vmax = math.radians(float(P['head.max_speed_deg_s']))
        self.tilt = float(P['head.tilt_level_rad'])
        self.wps = self._load_waypoints(P['waypoints_file'])
        if len(self.wps) < 2:
            raise RuntimeError(f'Il faut au moins 2 waypoints ({P["waypoints_file"]})')

        self.cbg = ReentrantCallbackGroup()
        self._stop = threading.Event()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

        # état capteurs
        self.js_pan = None
        self.js_t = 0.0
        self.odom_t = 0.0
        self.odom_xy = None
        self.dist_total = 0.0
        self.scan_t = 0.0
        self.scan = None
        self.plan = None
        self.plan_t = 0.0
        self.loc_std = None
        self.amcl_t = 0.0
        self.user_initpose = 0
        self.batt = deque(maxlen=60)
        self.batt_state = 'ok'

        # état tête
        self.head_mode = 'hold'                # hold / travel / move
        self.head_cmd = 0.0                    # dernière consigne de pan envoyée
        self.head_hold_pos = 0.0
        self.mv = None                         # mouvement min-jerk en cours (p0, p1, t0, T)
        self.head_pub_t = 0.0
        self.head_lock = threading.Lock()
        self._head_bad_since = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_head = self.create_publisher(JointState, P['head.command_topic'], 10)
        self.create_subscription(JointState, P['head.joint_states_topic'], self._js_cb, 20, callback_group=self.cbg)
        self.create_subscription(Odometry, P['odom_topic'], self._odom_cb, 20, callback_group=self.cbg)
        self.create_subscription(LaserScan, P['speed.scan_topic'], self._scan_cb, qos_profile_sensor_data,
                                 callback_group=self.cbg)
        self.create_subscription(Path, '/plan', self._plan_cb, 5, callback_group=self.cbg)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, latched,
                                 callback_group=self.cbg)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._initpose_cb, 10,
                                 callback_group=self.cbg)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag_cb, 10, callback_group=self.cbg)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cbg)
        self.cli_param = self.create_client(SetParameters, P['speed.controller_node'] + '/set_parameters',
                                            callback_group=self.cbg)
        self.cli_clear_l = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap',
                                              callback_group=self.cbg)
        self.cli_clear_g = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap',
                                              callback_group=self.cbg)
        self.cli_lidar_on = self.create_client(Empty, P['lidar.start_service'], callback_group=self.cbg)
        self.cli_lidar_off = self.create_client(Empty, P['lidar.stop_service'], callback_group=self.cbg)
        self.create_timer(1.0 / float(P['head.rate_hz']), self._head_tick, callback_group=self.cbg)

        self.goal_handle = None
        self.speed_sent = None
        self.speed_sent_t = 0.0
        log_dir = os.path.expanduser(P['log_dir'])
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, datetime.now().strftime('autonome_%Y%m%d_%H%M%S.csv'))
        self.csv_f = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['heure', 'trajet', 'tour', 'depart', 'arrivee', 'resultat', 'duree_s', 'distance_m',
                             'vitesse_croisiere', 'batterie_v', 'etat_batterie', 'pause_s'])
        self.get_logger().info(
            f'Néo autonome : {len(self.wps)} waypoints, pauses {P["pause.min_s"]:.0f}-{P["pause.max_s"]:.0f} s, '
            f'tête ±{math.degrees(self.travel_limit):.0f}° en roulant, '
            f'{"±%.0f° en pause" % math.degrees(self.pause_limit) if self.look_around_on else "fixe en pause"}, '
            f'{P["head.max_speed_deg_s"]:.0f}°/s max, '
            f'vitesse {P["speed.cruise_min"]:.2f}-{P["speed.cruise_max"]:.2f} m/s — journal {self.csv_path}')

    # ================================================================== chargement
    def _load_waypoints(self, path):
        with open(os.path.expanduser(path), 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        self.map_frame = data.get('frame_id', self.P['map_frame'])
        return [{'name': str(w.get('name', f'WP{i + 1}')), 'x': float(w['x']), 'y': float(w['y']),
                 'last': -1e9, 'visits': 0}
                for i, w in enumerate(data.get('waypoints', []) or [])]

    # ================================================================== callbacks
    def _js_cb(self, m):
        for n, p in zip(m.name, m.position):
            if n == self.P['head.pan_joint']:
                self.js_pan = p
                self.js_t = time.monotonic()

    def _odom_cb(self, m):
        p = m.pose.pose.position
        if self.odom_xy is not None:
            dd = math.hypot(p.x - self.odom_xy[0], p.y - self.odom_xy[1])
            if dd < 0.5:
                self.dist_total += dd
        self.odom_xy = (p.x, p.y)
        self.odom_t = time.monotonic()

    def _scan_cb(self, m):
        self.scan = m
        self.scan_t = time.monotonic()

    def _plan_cb(self, m):
        if m.poses:
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
            self.plan_t = time.monotonic()

    def _amcl_cb(self, m):
        c = m.pose.covariance
        self.loc_std = math.sqrt(max(c[0], c[7], 0.0))
        self.amcl_t = time.monotonic()

    def _initpose_cb(self, _m):
        self.user_initpose += 1

    def _diag_cb(self, m):
        for st in m.status:
            if 'Battery' in st.name:
                for kv in st.values:
                    if kv.key == 'Voltage':
                        try:
                            self.batt.append(float(kv.value))
                        except ValueError:
                            pass

    # ================================================================== outils
    def request_stop(self):
        self._stop.set()

    def _sleep(self, s):
        if self._stop.wait(max(0.0, s)):
            raise Stop()

    @staticmethod
    def _wait_future(fut, timeout):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.02)
        return True

    def _call(self, cli, req, label, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'Service indisponible : {label}')
            return False
        return self._wait_future(cli.call_async(req), timeout)

    def robot_pose(self):
        try:
            tr = self.tf.lookup_transform(self.map_frame, self.P['base_frame'], Time())
            t = tr.transform.translation
            return t.x, t.y, yaw_of(tr.transform.rotation)
        except Exception:
            return None

    def bearing_to(self, x, y, limit=None):
        """Angle (rad) du point (x, y) vu depuis le robot, limité à l'amplitude de la tête."""
        p = self.robot_pose()
        if p is None:
            return 0.0
        b = wrap(math.atan2(y - p[1], x - p[0]) - p[2])
        lim = self.pan_limit if limit is None else limit
        return max(-lim, min(lim, b))

    def battery_v(self):
        return statistics.median(self.batt) if len(self.batt) >= 5 else None

    def update_battery_state(self):
        if not self.P['battery.enabled']:
            return
        v = self.battery_v()
        if v is None:
            return
        old = self.batt_state
        if v < float(self.P['battery.low_v']):
            self.batt_state = 'low'
        elif v < float(self.P['battery.tired_v']) and self.batt_state == 'ok':
            self.batt_state = 'tired'                      # hystérésis : on ne redevient pas « ok »
        if self.batt_state != old:
            self.get_logger().warn(
                {'tired': f'Batterie {v:.2f} V : Néo est fatigué — plus lent, pauses plus longues',
                 'low': f'BATTERIE FAIBLE {v:.2f} V : Néo rejoint son point de repos'}[self.batt_state])

    # ================================================================== tête
    def _publish_head(self, pan, vel):
        """vel = vitesse du servo pour CETTE consigne. En flux à 20 Hz, elle vaut l'écart / période :
        le servo arrive pile à la consigne suivante => mouvement continu, sans à-coups.
        (Jamais 0 : pour un AX-12, Moving_Speed = 0 signifie vitesse MAXIMALE.)"""
        vel = max(0.05, min(float(vel), self.head_vmax * 1.5))
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [self.P['head.pan_joint'], self.P['head.tilt_joint']]
        m.position = [float(pan), float(self.tilt)]
        m.velocity = [vel, 0.3]
        self.pub_head.publish(m)
        self.head_cmd = pan
        self.head_pub_t = time.monotonic()

    def head_move(self, target, speed=None, wait=True):
        """Mouvement min-jerk de la position actuelle vers target (rad), vitesse de pointe <= speed."""
        target = max(-self.pan_limit, min(self.pan_limit, target))
        vpk = min(speed or self.head_vmax, self.head_vmax)
        with self.head_lock:
            p0 = self.head_cmd
            T = max(0.15, 1.875 * abs(target - p0) / max(vpk, 1e-3))   # pic min-jerk = 1.875 Δ/T
            self.mv = (p0, target, time.monotonic(), T)
            self.head_mode = 'move'
        if wait:
            self._sleep(T)
        return T

    def head_travel(self):
        with self.head_lock:
            self.head_mode = 'travel'

    def head_hold(self, pan):
        with self.head_lock:
            self.head_hold_pos = pan
            self.head_mode = 'hold'

    def _travel_target(self):
        """Pan désiré en roulant : angle du point du chemin situé à lookahead_m devant le robot."""
        if self.plan is None or time.monotonic() - self.plan_t > 3.0:
            return 0.0
        p = self.robot_pose()
        if p is None:
            return 0.0
        x, y, yaw = p
        path = self.plan
        i0 = min(range(len(path)), key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) < float(self.P['head.near_goal_m']):
            return 0.0
        acc, tx, ty = 0.0, gx, gy
        for i in range(i0, len(path) - 1):
            acc += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            if acc >= float(self.P['head.lookahead_m']):
                tx, ty = path[i + 1]
                break
        b = wrap(math.atan2(ty - y, tx - x) - yaw)
        return max(-self.travel_limit, min(self.travel_limit, float(self.P['head.gain']) * b))

    def _head_tick(self):
        now = time.monotonic()
        dt = 1.0 / float(self.P['head.rate_hz'])
        with self.head_lock:
            mode, cmd = self.head_mode, self.head_cmd
            if mode == 'move' and self.mv is not None:
                p0, p1, t0, T = self.mv
                s = (now - t0) / T
                pan = p0 + (p1 - p0) * min_jerk(s)
                if s >= 1.0:
                    self.head_mode, self.head_hold_pos, self.mv = 'hold', p1, None
            elif mode == 'travel':
                want = self._travel_target()
                a = dt / (float(self.P['head.smoothing_s']) + dt)
                step = max(-self.head_vmax * dt, min(self.head_vmax * dt, (want - cmd) * a))
                pan = cmd + step
            else:
                pan = self.head_hold_pos
        moving = abs(pan - cmd) > math.radians(0.2)
        if moving:
            self._publish_head(pan, abs(pan - cmd) / dt * 1.15)   # vitesse ajustée : servo en mouvement continu
        elif now - self.head_pub_t > 1.0:              # au repos : 1 consigne/s (évite l'auto torque off)
            self._publish_head(pan, 0.3)

    def head_ok(self, persist_s=0.0):
        """Vrai si la tête suit sa consigne. persist_s > 0 : faux seulement si l'écart dure."""
        now = time.monotonic()
        if self.js_pan is None or now - self.js_t > 1.0:
            return False
        good = abs(self.js_pan - self.head_cmd) < math.radians(float(self.P['head.max_error_deg']))
        if good:
            self._head_bad_since = None
            return True
        if self._head_bad_since is None:
            self._head_bad_since = now
        return now - self._head_bad_since < persist_s

    # ================================================================== pause « regarde autour »
    def look_around(self, duration, came_from, next_wp):
        t_end = time.monotonic() + duration
        if not self.look_around_on:                        # tête immobile, de face, pendant la pause
            self.head_move(0.0, speed=self.head_vmax * 0.5)
            self._sleep(max(0.0, t_end - time.monotonic()))
            return
        lim = self.pause_limit
        look_next = float(self.P['pause.look_next_s'])
        min_sacc = math.radians(float(self.P['pause.min_saccade_deg']))
        micro = math.radians(float(self.P['pause.micro_deg']))
        self._sleep(0.4)                                   # petit temps d'arrêt avant de regarder
        while time.monotonic() < t_end - look_next - 0.5:
            if came_from is not None and self.rng.random() < float(self.P['pause.look_back_prob']):
                tgt = self.bearing_to(came_from['x'], came_from['y'], lim)   # regard en arrière
            else:
                for _ in range(10):
                    tgt = self.rng.uniform(-lim, lim)
                    if abs(tgt - self.head_cmd) >= min_sacc:
                        break
            self.head_move(tgt)
            hold = self.rng.uniform(float(self.P['pause.fix_min_s']), float(self.P['pause.fix_max_s']))
            hold = min(hold, max(0.3, t_end - look_next - time.monotonic()))
            if self.rng.random() < 0.5 and hold > 1.0:     # petit ajustement pendant la fixation
                self._sleep(hold * 0.55)
                self.head_move(max(-lim, min(lim, self.head_cmd + self.rng.uniform(-micro, micro))),
                               speed=self.head_vmax * 0.3)
                self._sleep(max(0.0, hold * 0.45 - 0.3))
            else:
                self._sleep(hold)
        # anticipation : il regarde vers sa prochaine destination avant de repartir
        self.head_move(self.bearing_to(next_wp['x'], next_wp['y'], lim))
        self._sleep(max(0.3, t_end - time.monotonic()))

    # ================================================================== allure (A, B)
    def set_speed(self, v, force=False):
        v = max(float(self.P['speed.absolute_min']), v)
        now = time.monotonic()
        if not force and self.speed_sent is not None and (abs(v - self.speed_sent) < 0.01 or now - self.speed_sent_t < 0.5):
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name=self.P['speed.param_name'],
                                   value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))]
        if self.cli_param.service_is_ready():
            self.cli_param.call_async(req)
            self.speed_sent, self.speed_sent_t = v, now

    def narrow_passage(self):
        """Vrai si des obstacles encadrent le robot des deux côtés à moins de narrow_width_m (devant, 0-0.8 m)."""
        m = self.scan
        if m is None or time.monotonic() - self.scan_t > 1.0:
            return False
        try:
            tr = self.tf.lookup_transform(self.P['base_frame'], m.header.frame_id, Time())
        except Exception:
            return False
        tx, ty = tr.transform.translation.x, tr.transform.translation.y
        yaw = yaw_of(tr.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        left = right = 9.0
        for i in range(0, len(m.ranges), 2):
            r = m.ranges[i]
            if not (m.range_min < r < 2.0):
                continue
            a = m.angle_min + i * m.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x, y = tx + c * lx - s * ly, ty + s * lx + c * ly
            if 0.0 < x < 0.8:
                if y > 0:
                    left = min(left, y)
                else:
                    right = min(right, -y)
        return left + right < float(self.P['speed.narrow_width_m'])

    # ================================================================== navigation
    def choose_next(self, *exclude):
        """Tirage pondéré : jamais le waypoint actuel ; plus il n'a pas été visité, plus il a de chances."""
        now = time.monotonic()
        cands = [w for w in self.wps if all(w is not e for e in exclude)] or \
            [w for w in self.wps if w is not exclude[0]]
        weights = [1.0 + min(600.0, now - w['last']) / 60.0 for w in cands]
        return self.rng.choices(cands, weights=weights, k=1)[0]

    def navigate(self, wp, cruise):
        """Retourne 'success', 'failed', 'timeout'. Gère allure et sécurités pendant le trajet."""
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = wp['x'], wp['y']
        p = self.robot_pose()
        yaw = math.atan2(wp['y'] - p[1], wp['x'] - p[0]) if p else 0.0   # orientation sans importance (tolérance 180°)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action navigate_to_pose indisponible')
            return 'failed'
        self.set_speed(cruise, force=True)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        fut = self.nav.send_goal_async(goal)
        if not self._wait_future(fut, 10.0) or not fut.result().accepted:
            return 'failed'
        self.goal_handle = fut.result()
        res_fut = self.goal_handle.get_result_async()
        self.head_travel()
        t0 = time.monotonic()
        narrow = False
        try:
            while not res_fut.done():
                self._sleep(0.25)
                now = time.monotonic()
                if now - t0 > float(self.P['leg_timeout_s']):
                    self.cancel_goal()
                    return 'timeout'
                if now - self.scan_t > float(self.P['safety.max_scan_age_s']):
                    self.get_logger().warn('Lidar muet : arrêt du trajet')
                    self.cancel_goal()
                    return 'failed'
                if self.loc_std is not None and self.loc_std > float(self.P['safety.max_loc_std_m']):
                    self.get_logger().warn(f'Localisation incertaine (σ {self.loc_std:.2f} m) : arrêt du trajet')
                    self.cancel_goal()
                    return 'failed'
                if not self.head_ok(persist_s=1.5) and now - t0 > 2.0:
                    self.get_logger().warn('Tête : consigne non suivie (servo ? câble ?)', throttle_duration_sec=10.0)
                # allure : passages étroits + fatigue
                n = self.narrow_passage()
                if n != narrow:
                    narrow = n
                    self.get_logger().info('Passage étroit : ralenti' if n else 'Passage dégagé : allure normale')
                v = min(cruise, float(self.P['speed.narrow_speed'])) if narrow else cruise
                self.set_speed(v)
        finally:
            self.head_hold(self.head_cmd)
        st = res_fut.result().status
        self.goal_handle = None
        return 'success' if st == GoalStatus.STATUS_SUCCEEDED else 'failed'

    def cancel_goal(self):
        gh = self.goal_handle
        if gh is not None:
            try:
                self._wait_future(gh.cancel_goal_async(), 3.0)
            except Exception:
                pass
        self.goal_handle = None

    # ================================================================== démarrage
    def startup(self):
        log = self.get_logger()
        t0 = time.monotonic()
        while True:
            base = time.monotonic() - self.odom_t < 2.0
            head = time.monotonic() - self.js_t < 2.0
            if base and head:
                break
            if int(time.monotonic() - t0) % 10 == 0:
                manque = [n for n, ok in (('base (odom)', base), ('tête (/joint_states)', head)) if not ok]
                log.warn(f'En attente de : {", ".join(manque)} — lance qbo_startup', throttle_duration_sec=9.0)
            self._sleep(0.5)
        # tête au centre, tilt horizontal
        self.head_cmd = self.js_pan if self.js_pan is not None else 0.0
        self.head_move(0.0, speed=self.head_vmax * 0.6)
        self._sleep(1.0)
        if not self.head_ok():
            raise RuntimeError(f'tête : consigne non atteinte (pan mesuré '
                               f'{math.degrees(self.js_pan or 0):+.1f}°) — servo ou câble ?')
        log.info('Tête au centre, tilt horizontal')
        self._call(self.cli_lidar_on, Empty.Request(), 'start_motor')
        t0 = time.monotonic()
        while time.monotonic() - self.scan_t > 0.5:
            if time.monotonic() - t0 > 15.0:
                raise RuntimeError('lidar : aucun scan')
            self._sleep(0.2)
        log.warn('>>> Place Néo dans RViz avec "2D Pose Estimate" (le scan rouge doit coller aux murs)')
        n0, t0 = self.user_initpose, time.monotonic()
        while self.user_initpose == n0:
            if time.monotonic() - t0 > float(self.P['initial_pose_timeout_s']):
                raise RuntimeError('pas de 2D Pose Estimate')
            self._sleep(0.2)
        self._sleep(2.0)
        self._call(self.cli_clear_l, ClearEntireCostmap.Request(), 'clear local')
        self._call(self.cli_clear_g, ClearEntireCostmap.Request(), 'clear global')
        log.info('Localisé : Néo part se promener')

    # ================================================================== boucle principale
    def run(self):
        log = self.get_logger()
        reason = 'arrêt demandé'
        legs = ok = fails = consecutive = 0
        t_start = None
        try:
            self.startup()
            t_start = time.monotonic()
            p = self.robot_pose()
            here = min(self.wps, key=lambda w: math.hypot(w['x'] - p[0], w['y'] - p[1])) if p else None
            came_from = None
            nxt = self.choose_next(here)
            # premier départ : il regarde vers sa destination
            if self.look_around_on:
                self.head_move(self.bearing_to(nxt['x'], nxt['y'], self.pause_limit))
                self._sleep(0.8)
            while True:
                dur_min = float(self.P['duration_min'])
                if dur_min > 0 and time.monotonic() - t_start > dur_min * 60.0:
                    reason = f'durée atteinte ({dur_min:.0f} min)'
                    break
                self.update_battery_state()
                if self.batt_state == 'low':
                    reason = self.go_rest(here)
                    break
                tired = self.batt_state == 'tired'
                cruise = self.rng.uniform(float(self.P['speed.cruise_min']), float(self.P['speed.cruise_max']))
                if tired:
                    cruise *= float(self.P['battery.tired_speed_factor'])
                cruise = max(float(self.P['speed.absolute_min']), cruise)
                legs += 1
                lap = (legs - 1) // len(self.wps) + 1
                src = here['name'] if here else '?'
                target = nxt
                log.info(f'--- Trajet {legs} (tour {lap}) : {src} -> {nxt["name"]} --- allure {cruise:.2f} m/s')
                t0, d0 = time.monotonic(), self.dist_total
                res = self.navigate(nxt, cruise)
                dur, dist = time.monotonic() - t0, self.dist_total - d0
                pause = 0.0
                if res == 'success':
                    ok += 1
                    consecutive = 0
                    nxt['last'] = time.monotonic()
                    nxt['visits'] += 1
                    log.info(f'Arrivé à {nxt["name"]} en {dur:.0f} s, {dist:.1f} m')
                    came_from, here = here, nxt
                    following = self.choose_next(here)
                    pause = self.rng.uniform(float(self.P['pause.min_s']), float(self.P['pause.max_s']))
                    if tired:
                        pause *= float(self.P['battery.tired_pause_factor'])
                    log.info(f'Pause {pause:.1f} s : Néo regarde autour de lui (prochaine destination : '
                             f'{following["name"]})')
                    self.look_around(pause, came_from, following)
                    nxt = following
                else:
                    fails += 1
                    consecutive += 1
                    log.warn(f'Trajet vers {nxt["name"]} : {res} ({consecutive} échec(s) consécutif(s))')
                    if consecutive >= int(self.P['max_consecutive_failures']):
                        reason = f'{consecutive} échecs consécutifs'
                        break
                    self._call(self.cli_clear_l, ClearEntireCostmap.Request(), 'clear local')
                    self._call(self.cli_clear_g, ClearEntireCostmap.Request(), 'clear global')
                    self._sleep(3.0)
                    nxt = self.choose_next(nxt, here)      # autre destination
                    if self.look_around_on:
                        self.head_move(self.bearing_to(nxt['x'], nxt['y'], self.pause_limit))
                v = self.battery_v()
                self.csv_w.writerow([datetime.now().strftime('%H:%M:%S'), legs, lap, src, target['name'],
                                     res, f'{dur:.1f}',
                                     f'{dist:.2f}', f'{cruise:.2f}', f'{v:.2f}' if v else '', self.batt_state,
                                     f'{pause:.1f}'])
                self.csv_f.flush()
        except Stop:
            reason = 'arrêt demandé (Ctrl+C)'
        except Exception as e:  # noqa: BLE001
            reason = f'ARRÊT SÉCURISÉ : {e}'
            log.error(reason)
        finally:
            self.cancel_goal()
            try:
                self._stop.clear()
                self.head_move(0.0, speed=self.head_vmax * 0.6, wait=False)
                time.sleep(1.2)
            except Exception:
                pass
            if self.P['lidar.stop_at_end']:
                self._call(self.cli_lidar_off, Empty.Request(), 'stop_motor', 2.0)
            el = (time.monotonic() - t_start) / 60.0 if t_start else 0.0
            log.info(f'Néo autonome — fin : {reason}')
            log.info(f'Durée {el:.1f} min | trajets {legs} (réussis {ok}, échoués {fails}) | '
                     f'distance {self.dist_total:.0f} m | batterie {self.battery_v() or 0:.2f} V')
            log.info(f'Détail : {self.csv_path}')
            self.csv_f.close()

    def go_rest(self, here):
        name = self.P['battery.rest_waypoint'] or self.wps[0]['name']
        rest = next((w for w in self.wps if w['name'] == name), self.wps[0])
        if here is not rest:
            self.get_logger().warn(f'Retour au point de repos {rest["name"]}')
            if self.look_around_on:
                self.head_move(self.bearing_to(rest['x'], rest['y'], self.pause_limit))
            self.navigate(rest, float(self.P['speed.absolute_min']))
        self.head_move(0.0)
        return f'batterie faible ({self.battery_v() or 0:.2f} V) : Néo au repos à {rest["name"]}'


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)

    def _sigterm(_s, _f):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        node = NeoAutonomous()
    except Exception as e:  # noqa: BLE001
        rclpy.logging.get_logger('neo_autonomous').fatal(str(e))
        rclpy.try_shutdown()
        return
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=node.run, name='autonomie', daemon=True)
    th.start()
    try:
        while rclpy.ok() and th.is_alive():
            ex.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C : arrêt en cours (tête au centre, lidar arrêté)...')
    finally:
        node.request_stop()
        t_end = time.monotonic() + 10.0
        while th.is_alive() and time.monotonic() < t_end:
            try:
                ex.spin_once(timeout_sec=0.1)
            except Exception:
                break
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
