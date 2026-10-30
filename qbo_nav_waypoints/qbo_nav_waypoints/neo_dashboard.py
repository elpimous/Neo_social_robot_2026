#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
neo_dashboard.py — Tableau de bord graphique de Néo (test d'autonomie batterie)
===============================================================================
Auteur : Vincent FOUCAULT

Serveur web intégré (aucune dépendance) : ouvrir depuis le PC ROG
    http://192.168.1.23:8080

Affiche en direct : batterie (tension, pente, temps restant estimé, courbe),
position sur la carte avec la trace colorée par la tension et le scan laser,
vitesse, distance, trajets/tours, CPU par cœur, GPU, températures, puissance
du module Orin, RAM, Wi-Fi, journal des événements.

Enregistre toutes les `period_s` secondes une ligne CSV avec fsync() :
    ~/.ros/qbo_nav_test/autonomie_AAAAMMJJ_HHMMSS.csv
=> si Néo s'éteint batterie vide, tout est sur le disque jusqu'à la dernière seconde.
La page du navigateur garde les dernières valeurs et affiche l'heure de la coupure.

Revoir un test après recharge :
    ros2 run qbo_nav_waypoints neo_dashboard --ros-args -p replay:=last
"""

import base64
import csv
import glob
import json
import math
import os
import re
import socket
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import yaml

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None

COLS = ['heure', 't_s', 'batt_v', 'batt_vf', 'charge', 'ext_power', 'pc_on', 'x', 'y', 'yaw_deg',
        'v_cmd', 'v', 'w', 'dist_m', 'legs', 'laps', 'wps', 'cpu', 'gpu', 'ram_mb', 'temp_max',
        'temp_cpu', 'temp_gpu', 'p_w', 'e_wh', 'wifi_dbm', 'scan_hz', 'loc_std']
NUM_COLS = set(COLS) - {'heure', 'charge', 'ext_power', 'pc_on'}


def read(path, default=None):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return default


def share(pkg, *parts):
    try:
        return os.path.join(get_package_share_directory(pkg), *parts)
    except Exception:
        return ''


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '192.168.1.23'


# =============================================================================
#  Carte (map.yaml + PGM) -> grille 0 inconnu / 1 libre / 2 occupé
# =============================================================================
def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(yaml_path), img)
    with open(img, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise ValueError('seules les cartes PGM binaires (P5) sont gérées')
    tokens, pos = [], 2
    while len(tokens) < 3:                     # largeur, hauteur, maxval (commentaires # ignorés)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(int(data[start:pos]))
    pos += 1
    w, h, maxval = tokens
    pix = data[pos:pos + w * h]
    neg = int(meta.get('negate', 0))
    occ_t, free_t = float(meta.get('occupied_thresh', 0.65)), float(meta.get('free_thresh', 0.25))
    out = bytearray(w * h)
    for i, v in enumerate(pix):
        p = (v / maxval) if neg else ((maxval - v) / maxval)
        out[i] = 2 if p > occ_t else (1 if p < free_t else 0)
    return {'w': w, 'h': h, 'res': float(meta['resolution']),
            'origin': [float(meta['origin'][0]), float(meta['origin'][1])],
            'cells': base64.b64encode(bytes(out)).decode('ascii')}


# =============================================================================
#  Système Jetson
# =============================================================================
class SysStats:

    def __init__(self):
        self.prev = None
        self.thermal = [(read(z + '/type', os.path.basename(z)), z + '/temp')
                        for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*'))]
        self.gpu_path = next((p for p in ['/sys/devices/platform/gpu.0/load', '/sys/devices/gpu.0/load']
                              + glob.glob('/sys/devices/platform/*gpu*/load')
                              + glob.glob('/sys/devices/platform/bus@0/*gpu*/load')
                              if os.path.exists(p)), None)
        self.rails = []
        for hw in glob.glob('/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*'):
            for lab in sorted(glob.glob(hw + '/in*_label')):
                idx = lab.split('in')[-1].split('_')[0]
                v, c = f'{hw}/in{idx}_input', f'{hw}/curr{idx}_input'
                if os.path.exists(v) and os.path.exists(c):
                    self.rails.append((read(lab, f'in{idx}'), v, c))

    def cpu(self):
        cores = []
        for line in (read('/proc/stat', '') or '').splitlines():
            if line.startswith('cpu') and line[3:4].isdigit():
                f = [int(x) for x in line.split()[1:]]
                idle = f[3] + (f[4] if len(f) > 4 else 0)
                cores.append((sum(f), idle))
        out = []
        if self.prev and len(self.prev) == len(cores):
            for (t1, i1), (t0, i0) in zip(cores, self.prev):
                dt = t1 - t0
                out.append(round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0)
        self.prev = cores
        return out

    def gpu(self):
        v = read(self.gpu_path) if self.gpu_path else None
        return round(int(v) / 10.0, 1) if v and v.isdigit() else None

    def temps(self):
        t = {}
        for name, p in self.thermal:
            v = read(p)
            if v and v.lstrip('-').isdigit() and int(v) > -40000:
                t[name.replace('-thermal', '')] = round(int(v) / 1000.0, 1)
        return t

    def power(self):
        rails, total = {}, None
        for name, vf, cf in self.rails:
            v, c = read(vf), read(cf)
            if v and c and v.isdigit() and c.isdigit():
                w = int(v) * int(c) / 1e6
                rails[name] = round(w, 2)
                if 'VDD_IN' in name.upper():
                    total = w                      # entrée du module = total (les autres en font partie)
        if total is None and rails:
            total = sum(rails.values())
        return (round(total, 2) if total is not None else None), rails

    @staticmethod
    def mem():
        info = {}
        for line in (read('/proc/meminfo', '') or '').splitlines():
            k, _, v = line.partition(':')
            if v.split():
                info[k] = int(v.split()[0]) // 1024
        return (info.get('MemTotal', 0) - info.get('MemAvailable', 0), info.get('MemTotal', 0),
                info.get('SwapTotal', 0) - info.get('SwapFree', 0))

    @staticmethod
    def wifi():
        lines = (read('/proc/net/wireless', '') or '').splitlines()[2:]
        if not lines:
            return None, None
        f = lines[0].split()
        try:
            return f[0].rstrip(':'), float(f[3].rstrip('.'))
        except (IndexError, ValueError):
            return f[0].rstrip(':'), None


# =============================================================================
#  Nœud
# =============================================================================
class NeoDashboard(Node):

    def __init__(self):
        super().__init__('neo_dashboard')
        d = self.declare_parameter
        d('port', 8080)
        d('period_s', 2.0)
        d('log_dir', '~/.ros/qbo_nav_test')
        d('replay', '')                                    # '' = direct, 'last' ou chemin CSV
        d('map', share('qbo_navigation', 'map', 'map.yaml'))
        d('waypoints', share('qbo_nav_waypoints', 'config', 'waypoints.yaml'))
        d('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        d('cmd_vel_topic', '/qbo_arduqbo/base_ctrl/cmd_vel')
        d('base_frame', 'base_footprint')
        d('laser_frame', 'laser')
        d('cutoff_v', 10.5)                                # LiFePO4 4S : 2.6 V/cellule
        d('knee_v', 12.0)                                  # coude de décharge (3.0 V/cellule)
        d('slope_window_min', 10.0)
        d('camera_topic', '/inverted_eye_image')
        d('camera_fps', 5.0)                               # images/s envoyées au navigateur
        d('camera_width', 480)                             # largeur max du flux (px)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port = int(g('port'))
        self.period = float(g('period_s'))
        self.log_dir = os.path.expanduser(g('log_dir'))
        self.base_frame, self.laser_frame = g('base_frame'), g('laser_frame')
        self.cutoff, self.knee = float(g('cutoff_v')), float(g('knee_v'))
        self.slope_win = float(g('slope_window_min')) * 60.0
        self.cam_topic = g('camera_topic')
        self.cam_fps = max(0.5, float(g('camera_fps')))
        self.cam_w = int(g('camera_width'))
        self.cam_msg = None
        self.cam_t = 0.0
        self.cam_times = deque(maxlen=30)
        self.cam_jpg = None
        self.cam_jpg_t = -1.0
        self.cam_lock = threading.Lock()
        os.makedirs(self.log_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.cols = COLS
        self.rows = []                                     # historique (listes alignées sur COLS)
        self.events = deque(maxlen=80)
        self.ev_seen = {}
        self.map = self._load_map(g('map'))
        self.wps = self._load_wps(g('waypoints'))
        self.ip = local_ip()

        replay = str(g('replay')).strip()
        self.mode = 'replay' if replay else 'live'
        if self.mode == 'replay':
            self._load_replay(replay)
        else:
            self._init_live(g)

        self._start_http()
        url = f'http://{self.ip}:{self.port}'
        self.get_logger().info(f'Tableau de bord : {url}   ({"REPLAY " + self.replay_file if self.mode == "replay" else "direct"})')
        print(f'\n\033[1;36m  >>> Ouvre dans le navigateur du PC : {url}\033[0m\n', flush=True)

    # ------------------------------------------------------------------ chargement
    def _load_map(self, path):
        try:
            m = load_map(path)
            self.get_logger().info(f'Carte : {path} ({m["w"]}x{m["h"]} @ {m["res"]} m)')
            return m
        except Exception as e:
            self.get_logger().warn(f'Carte non chargée ({path}) : {e}')
            return None

    def _load_wps(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return [{'name': str(w.get('name', f'WP{i + 1}')), 'x': float(w['x']), 'y': float(w['y'])}
                    for i, w in enumerate(data.get('waypoints', []) or [])]
        except Exception as e:
            self.get_logger().warn(f'Waypoints non chargés ({path}) : {e}')
            return []

    def _load_replay(self, replay):
        if replay == 'last':
            files = sorted(glob.glob(os.path.join(self.log_dir, 'autonomie_*.csv')), key=os.path.getmtime)
            if not files:
                raise SystemExit('Aucun fichier autonomie_*.csv dans ' + self.log_dir)
            replay = files[-1]
        self.replay_file = os.path.expanduser(replay)
        with open(self.replay_file, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                row = []
                for c in COLS:
                    v = r.get(c, '')
                    if c in NUM_COLS:
                        try:
                            v = float(v)
                        except (TypeError, ValueError):
                            v = None
                    row.append(v)
                self.rows.append(row)
        self.csv_path = self.replay_file
        self.live = {}
        self._add_event('INFO', 'dashboard', f'Replay de {os.path.basename(self.replay_file)} : {len(self.rows)} points')

    def _init_live(self, g):
        self.sys = SysStats()
        self.sys.cpu()
        self.t0 = time.monotonic()
        self.start_wall = datetime.now()
        self.batt_raw = deque(maxlen=60)                   # (t, V)
        self.batt = {}
        self.pose = None
        self.scan_pts = []
        self.scan_times = deque(maxlen=30)
        self.last_scan = None
        self.odom_prev = None
        self.dist = 0.0
        self.v = self.w = 0.0
        self.v_cmd = 0.0
        self.cmd_t = 0.0
        self.legs = self.laps = self.wps_passed = 0
        self.last_wp = ''
        self.loc_std = None
        self.e_wh = 0.0
        self.last_p_t = None
        self.live = {}

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag_cb, 10)
        self.create_subscription(Odometry, g('odom_topic'), self._odom_cb, 20)
        self.create_subscription(Twist, g('cmd_vel_topic'), self._cmd_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, latched)
        self.create_subscription(Log, '/rosout', self._rosout_cb, QoSProfile(depth=200))
        if cv2 is not None:
            self.create_subscription(Image, self.cam_topic, self._cam_cb, qos_profile_sensor_data)
        else:
            self._add_event('WARN', 'dashboard', 'OpenCV absent : pas de retour caméra')

        stamp = self.start_wall.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.log_dir, f'autonomie_{stamp}.csv')
        self.csv_f = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(COLS)
        self._sync()

        self.create_timer(0.2, self._pose_tick)
        self.create_timer(0.5, self._scan_tick)
        self.create_timer(self.period, self._sample)
        self.create_timer(10.0, self._console)
        self._add_event('INFO', 'dashboard', f'Enregistrement : {self.csv_path}')

    # ------------------------------------------------------------------ callbacks
    def _diag_cb(self, msg):
        for st in msg.status:
            if 'Battery' not in st.name:
                continue
            kv = {k.key: k.value for k in st.values}
            try:
                v = float(kv.get('Voltage', ''))
            except ValueError:
                continue
            with self.lock:
                self.batt_raw.append((time.monotonic(), v))
                self.batt.update({
                    'v': v,
                    'charge': kv.get('Charge Mode Description', ''),
                    'ext_power': kv.get('External Power', ''),
                    'pc_on': kv.get('Power PC', ''),
                    'est_runtime': kv.get('Estimated Runtime'),
                    'est_power': kv.get('Estimated Power'),
                    'summary': st.message,
                })

    def _odom_cb(self, m):
        p = m.pose.pose.position
        if self.odom_prev is not None:
            d = math.hypot(p.x - self.odom_prev[0], p.y - self.odom_prev[1])
            if d < 0.5:
                self.dist += d
        self.odom_prev = (p.x, p.y)
        self.v, self.w = m.twist.twist.linear.x, m.twist.twist.angular.z

    def _cmd_cb(self, m):
        self.v_cmd, self.cmd_t = m.linear.x, time.monotonic()

    def _scan_cb(self, m):
        self.scan_times.append(time.monotonic())
        self.last_scan = m

    def _amcl_cb(self, m):
        c = m.pose.covariance
        self.loc_std = math.sqrt(max(c[0], c[7], 0.0))

    def _rosout_cb(self, m):
        name = m.name.split('.')[-1]
        if name == self.get_name():
            return
        txt = m.msg
        if name == 'waypoint_mission':
            mt = re.search(r'Trajet (\d+) \(tour (\d+)\)', txt)
            if mt:
                self.legs, self.laps = int(mt.group(1)), int(mt.group(2))
            mp = re.search(r'(Passé par|Arrivé à) (\S+)', txt)
            if mp:
                self.wps_passed += 1
                self.last_wp = mp.group(2).rstrip(',')
            if mp or m.level >= 30 or 'Fin' in txt or 'Mission' in txt:
                self._add_event(LEVEL.get(m.level, 'INFO'), name, txt)
        elif m.level >= 30 and 'collision ahead' not in txt and 'tick rate' not in txt:
            self._add_event(LEVEL.get(m.level, 'WARN'), name, txt)

    def _add_event(self, level, node, txt):
        key = (node, txt[:60])
        now = time.monotonic()
        if level != 'INFO' and now - self.ev_seen.get(key, -1e9) < 30.0:
            return
        self.ev_seen[key] = now
        with self.lock:
            self.events.append({'t': datetime.now().strftime('%H:%M:%S'), 'lvl': level, 'node': node, 'msg': txt})

    # ------------------------------------------------------------------ caméra
    def _cam_cb(self, m):
        now = time.monotonic()
        self.cam_times.append(now)
        self.cam_msg, self.cam_t = m, now          # simple référence : encodage seulement à la demande

    def cam_jpeg(self):
        """JPEG de la dernière image (mis en cache : un seul encodage par image, quel que soit le nb de clients)."""
        with self.cam_lock:
            m, t = self.cam_msg, self.cam_t
            if m is None:
                return None
            if t == self.cam_jpg_t:
                return self.cam_jpg
            try:
                enc = m.encoding.lower()
                ch = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'bgra8': 4, 'rgba8': 4}.get(enc)
                if ch is None:
                    return self.cam_jpg
                a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * ch]
                img = a.reshape(m.height, m.width, ch) if ch > 1 else a.reshape(m.height, m.width)
                if enc == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif enc == 'rgba8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                elif enc == 'bgra8':
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                if m.width > self.cam_w:
                    img = cv2.resize(img, (self.cam_w, int(m.height * self.cam_w / m.width)),
                                     interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self.cam_jpg, self.cam_jpg_t = buf.tobytes(), t
                    self.cam_res = f'{m.width}x{m.height}'
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Image caméra illisible : {e}', throttle_duration_sec=30.0)
            return self.cam_jpg

    def cam_info(self):
        now = time.monotonic()
        ts = [t for t in self.cam_times if now - t < 3.0]
        hz = round((len(ts) - 1) / (ts[-1] - ts[0]), 1) if len(ts) > 1 and ts[-1] > ts[0] else None
        return {'topic': self.cam_topic, 'hz': hz, 'res': getattr(self, 'cam_res', None),
                'age': (round(now - self.cam_t, 1) if self.cam_msg is not None else None),
                'stream_fps': self.cam_fps, 'available': cv2 is not None and self.mode == 'live'}

    # ------------------------------------------------------------------ TF
    def _lookup(self, target, source):
        try:
            tr = self.tf.lookup_transform(target, source, Time())
            t = tr.transform.translation
            return t.x, t.y, yaw_of(tr.transform.rotation)
        except Exception:
            return None

    def _pose_tick(self):
        p = self._lookup('map', self.base_frame)
        frame = 'map'
        if p is None:
            p, frame = self._lookup('odom', self.base_frame), 'odom'
        with self.lock:
            self.pose = None if p is None else {'x': p[0], 'y': p[1], 'yaw': p[2], 'frame': frame}

    def _scan_tick(self):
        m = self.last_scan
        if m is None:
            return
        tf = self._lookup('map', m.header.frame_id or self.laser_frame)
        if tf is None:
            return
        tx, ty, tyaw = tf
        c, s = math.cos(tyaw), math.sin(tyaw)
        pts = []
        step = max(1, len(m.ranges) // 240)
        for i in range(0, len(m.ranges), step):
            r = m.ranges[i]
            if not (m.range_min < r < min(m.range_max, 12.0)):
                continue
            a = m.angle_min + i * m.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append([round(tx + c * lx - s * ly, 3), round(ty + s * lx + c * ly, 3)])
        with self.lock:
            self.scan_pts = pts

    # ------------------------------------------------------------------ batterie
    def _batt_filtered(self):
        now = time.monotonic()
        vals = [v for t, v in self.batt_raw if now - t < 30.0]
        return statistics.median(vals) if vals else None

    def _batt_stats(self, rows):
        """Pente (mV/min) sur la fenêtre glissante et temps restant jusqu'au seuil de coupure."""
        i_t, i_vf = COLS.index('t_s'), COLS.index('batt_vf')
        pts = [(r[i_t], r[i_vf]) for r in rows if r[i_vf] is not None]
        if len(pts) < 10:
            return None, None, None
        t_end = pts[-1][0]
        win = [(t, v) for t, v in pts if t_end - t <= self.slope_win]
        v0 = pts[0][1]
        if len(win) < 10 or win[-1][0] - win[0][0] < 120.0:
            return v0, None, None
        n = len(win)
        mt = sum(t for t, _ in win) / n
        mv = sum(v for _, v in win) / n
        den = sum((t - mt) ** 2 for t, _ in win)
        slope = sum((t - mt) * (v - mv) for t, v in win) / den if den > 0 else 0.0   # V/s
        eta = None
        if slope < -1e-6:
            eta = max(0.0, (win[-1][1] - self.cutoff) / -slope / 60.0)
        return v0, slope * 60000.0, eta                                             # mV/min, min

    # ------------------------------------------------------------------ échantillon
    def _sync(self):
        try:
            self.csv_f.flush()
            os.fsync(self.csv_f.fileno())
        except Exception:
            pass

    def _sample(self):
        now = time.monotonic()
        cores = self.sys.cpu()
        cpu = round(sum(cores) / len(cores), 1) if cores else None
        gpu = self.sys.gpu()
        temps = self.sys.temps()
        p_w, rails = self.sys.power()
        if p_w is not None and self.last_p_t is not None:
            self.e_wh += p_w * (now - self.last_p_t) / 3600.0
        self.last_p_t = now
        ram, ram_tot, swap = self.sys.mem()
        wif, wdbm = self.sys.wifi()
        scan_hz = None
        st = [t for t in self.scan_times if now - t < 3.0]
        if len(st) > 1:
            scan_hz = round((len(st) - 1) / (st[-1] - st[0]), 1)
        t_cpu = next((v for k, v in temps.items() if 'cpu' in k.lower()), None)
        t_gpu = next((v for k, v in temps.items() if 'gpu' in k.lower()), None)
        t_max = max(temps.values()) if temps else None
        v_cmd = self.v_cmd if now - self.cmd_t < 1.0 else 0.0

        with self.lock:
            vf = self._batt_filtered()
            b = dict(self.batt)
            pose = dict(self.pose) if self.pose else None
            row = [datetime.now().strftime('%H:%M:%S'), round(now - self.t0, 1),
                   b.get('v'), (round(vf, 3) if vf is not None else None),
                   b.get('charge', ''), b.get('ext_power', ''), b.get('pc_on', ''),
                   (round(pose['x'], 3) if pose else None), (round(pose['y'], 3) if pose else None),
                   (round(math.degrees(pose['yaw']), 1) if pose else None),
                   round(v_cmd, 3), round(self.v, 3), round(self.w, 3), round(self.dist, 2),
                   self.legs, self.laps, self.wps_passed, cpu, gpu, ram, t_max, t_cpu, t_gpu,
                   p_w, round(self.e_wh, 3), wdbm, scan_hz,
                   (round(self.loc_std, 3) if self.loc_std is not None else None)]
            self.rows.append(row)
            v0, slope, eta = self._batt_stats(self.rows)
            self.live = {
                'batt': {**b, 'vf': vf, 'v0': v0, 'slope': slope, 'eta': eta},
                'sys': {'cpu': cpu, 'cores': cores, 'gpu': gpu, 'temps': temps, 'tmax': t_max,
                        'p_w': p_w, 'rails': rails, 'e_wh': round(self.e_wh, 2),
                        'ram': ram, 'ram_tot': ram_tot, 'swap': swap, 'wifi': wif, 'wifi_dbm': wdbm,
                        'load': (read('/proc/loadavg', '0') or '0').split()[:3],
                        'uptime_s': float((read('/proc/uptime', '0') or '0').split()[0]),
                        'disk_gb': round(os.statvfs(self.log_dir).f_bavail
                                         * os.statvfs(self.log_dir).f_frsize / 1e9, 1)},
                'motion': {'v': self.v, 'w': self.w, 'v_cmd': v_cmd, 'dist': self.dist,
                           'legs': self.legs, 'laps': self.laps, 'wps': self.wps_passed,
                           'last_wp': self.last_wp, 'loc_std': self.loc_std, 'scan_hz': scan_hz},
            }
        self.csv_w.writerow(['' if x is None else x for x in row])
        self._sync()

    def _console(self):
        lv = self.live
        if not lv:
            return
        b, m, s = lv['batt'], lv['motion'], lv['sys']
        el = time.monotonic() - self.t0
        vf = f'{b["vf"]:.2f} V' if b.get('vf') else '? V'
        sl = f'{b["slope"]:+.1f} mV/min' if b.get('slope') is not None else 'pente…'
        eta = f'~{int(b["eta"] // 60)}h{int(b["eta"] % 60):02d} avant {self.cutoff} V' if b.get('eta') else ''
        pose = self.pose
        pos = f'({pose["x"]:+.2f}, {pose["y"]:+.2f})' if pose else '(?)'
        print(f'\033[36m[{int(el // 3600)}:{int(el % 3600 // 60):02d}:{int(el % 60):02d}]\033[0m '
              f'\033[1;33m⚡ {vf}\033[0m {sl} {eta} | 📍 {pos} | 🛞 {m["v"]:.2f} m/s {m["dist"]:.0f} m | '
              f'WP {m["wps"]} tour {m["laps"]} | CPU {s["cpu"] or 0:.0f}% GPU {s["gpu"] or 0:.0f}% '
              f'{s["tmax"] or 0:.0f}°C {s["p_w"] or 0:.1f} W', flush=True)

    # ------------------------------------------------------------------ API
    def api_live(self):
        with self.lock:
            out = {'mode': self.mode, 'server_time': datetime.now().strftime('%H:%M:%S'),
                   'cutoff': self.cutoff, 'knee': self.knee, 'csv': self.csv_path,
                   'events': list(self.events), 'n_rows': len(self.rows)}
            if self.mode == 'live':
                out.update(self.live)
                out['pose'] = self.pose
                out['scan'] = self.scan_pts
                out['cam'] = self.cam_info()
                out['elapsed'] = time.monotonic() - self.t0
                out['start'] = self.start_wall.strftime('%d/%m/%Y %H:%M:%S')
            else:
                v0, slope, eta = self._batt_stats(self.rows)
                last = dict(zip(COLS, self.rows[-1])) if self.rows else {}
                out['batt'] = {'v': last.get('batt_v'), 'vf': last.get('batt_vf'), 'v0': v0, 'slope': slope,
                               'eta': None, 'charge': last.get('charge'), 'ext_power': last.get('ext_power'),
                               'pc_on': last.get('pc_on')}
                out['elapsed'] = last.get('t_s', 0)
                out['start'] = os.path.basename(self.replay_file)
                if last.get('x') is not None:
                    out['pose'] = {'x': last['x'], 'y': last['y'], 'yaw': math.radians(last.get('yaw_deg') or 0),
                                   'frame': 'map'}
        return out

    def api_history(self, since):
        with self.lock:
            return {'cols': COLS, 'since': since, 'rows': self.rows[since:], 'total': len(self.rows)}

    def _start_http(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype='application/json', extra=None):
                data = body if isinstance(body, bytes) else body.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _mjpeg(self):
                """Flux MJPEG : le navigateur l'affiche directement dans une balise <img>."""
                if node.mode != 'live':
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=neoframe')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last = None
                period = 1.0 / node.cam_fps
                while rclpy.ok():
                    t0 = time.monotonic()
                    jpg = node.cam_jpeg()
                    if jpg is not None and jpg is not last:
                        self.wfile.write(b'--neoframe\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                        self.wfile.flush()
                        last = jpg
                    time.sleep(max(0.02, period - (time.monotonic() - t0)))

            def do_GET(self):
                u = urlparse(self.path)
                try:
                    if u.path in ('/', '/index.html'):
                        self._send(PAGE, 'text/html; charset=utf-8')
                    elif u.path == '/api/live':
                        self._send(json.dumps(node.api_live()))
                    elif u.path == '/api/history':
                        since = int(parse_qs(u.query).get('since', ['0'])[0])
                        self._send(json.dumps(node.api_history(since)))
                    elif u.path == '/api/static':
                        self._send(json.dumps({'map': node.map, 'wps': node.wps}))
                    elif u.path == '/cam.mjpg':
                        self._mjpeg()
                    elif u.path == '/cam.jpg':
                        jpg = node.cam_jpeg() if node.mode == 'live' else None
                        if jpg:
                            self._send(jpg, 'image/jpeg')
                        else:
                            self.send_error(404)
                    elif u.path == '/api/csv':
                        with open(node.csv_path, 'rb') as f:
                            self._send(f.read(), 'text/csv', {
                                'Content-Disposition': f'attachment; filename="{os.path.basename(node.csv_path)}"'})
                    else:
                        self.send_error(404)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.httpd = ThreadingHTTPServer(('0.0.0.0', self.port), H)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        if self.mode == 'live':
            self._sync()
            self.csv_f.close()


LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


# =============================================================================
#  Page web (autonome, aucun accès Internet nécessaire)
# =============================================================================
PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Néo · Autonomie</title>
<style>
:root{--bg:#07090d;--panel:#0d121a;--line:#1b2533;--txt:#d6e2f0;--dim:#6f8399;--acc:#33e0c8;
--ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--blue:#60a5fa;--mag:#e879f9;
--mono:ui-monospace,"JetBrains Mono","Fira Code",Menlo,Consolas,monospace}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--txt);
font:14px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
body{background-image:radial-gradient(circle at 20% -10%,#0f2a33 0,transparent 45%),
linear-gradient(var(--line) 1px,transparent 1px),linear-gradient(90deg,var(--line) 1px,transparent 1px);
background-size:auto,40px 40px,40px 40px;background-attachment:fixed}
header{display:flex;align-items:center;gap:16px;padding:12px 20px;border-bottom:1px solid var(--line);
background:rgba(7,9,13,.85);backdrop-filter:blur(6px);position:sticky;top:0;z-index:5;flex-wrap:wrap}
.logo{font:700 18px var(--mono);letter-spacing:.18em;color:var(--acc)}
.logo small{color:var(--dim);font-weight:400;letter-spacing:.1em;margin-left:8px}
.pill{font:600 12px var(--mono);padding:4px 10px;border-radius:99px;border:1px solid currentColor;letter-spacing:.08em}
.pill.live{color:var(--ok)}.pill.live::before{content:"● ";animation:blink 1.2s infinite}
.pill.off{color:var(--bad)}.pill.replay{color:var(--mag)}
@keyframes blink{50%{opacity:.2}}
.hstat{font:13px var(--mono);color:var(--dim)}.hstat b{color:var(--txt);font-weight:600}
.sp{flex:1}a.btn{font:12px var(--mono);color:var(--acc);text-decoration:none;border:1px solid var(--line);
padding:5px 10px;border-radius:6px}a.btn:hover{border-color:var(--acc)}
main{display:grid;gap:14px;padding:14px 20px 30px;grid-template-columns:repeat(12,1fr)}
.card{background:linear-gradient(180deg,rgba(255,255,255,.02),transparent),var(--panel);
border:1px solid var(--line);border-radius:10px;padding:14px 16px;min-width:0;position:relative}
.card h2{margin:0 0 10px;font:600 11px var(--mono);letter-spacing:.16em;text-transform:uppercase;color:var(--dim)}
.c3{grid-column:span 3}.c4{grid-column:span 4}.c5{grid-column:span 5}.c6{grid-column:span 6}
.c7{grid-column:span 7}.c8{grid-column:span 8}.c12{grid-column:span 12}
@media(max-width:1100px){.c3,.c4,.c5,.c6,.c7,.c8{grid-column:span 12}}
.big{font:700 54px/1 var(--mono);letter-spacing:-.02em}.big small{font-size:20px;color:var(--dim);margin-left:4px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font:13px var(--mono);margin-top:10px}
.kv span:nth-child(odd){color:var(--dim)}.kv span:nth-child(even){text-align:right}
.gauge{height:18px;border-radius:4px;background:#0a0f16;border:1px solid var(--line);margin-top:14px;
position:relative;overflow:hidden}
.gauge i{position:absolute;inset:0 auto 0 0;background:linear-gradient(90deg,var(--bad),var(--warn) 30%,var(--ok));
transition:width .8s}
.gauge b{position:absolute;top:-2px;bottom:-2px;width:2px;background:var(--txt);opacity:.6}
.glab{display:flex;justify-content:space-between;font:11px var(--mono);color:var(--dim);margin-top:4px}
canvas{width:100%;display:block}
.chart{height:230px}.mapc{height:520px}
.cam{position:relative;background:#05070a;border:1px solid var(--line);border-radius:8px;overflow:hidden;aspect-ratio:4/3}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.cam .none{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font:12px var(--mono);color:var(--dim)}
.cam .rec{position:absolute;top:10px;left:38px;font:600 11px var(--mono);color:var(--bad);letter-spacing:.1em}
.cam .rec::before{content:"● ";animation:blink 1.2s infinite}
.cam .hud{position:absolute;bottom:10px;right:38px;font:11px var(--mono);color:var(--acc);text-shadow:0 0 4px #000}
.cam::after{content:"";position:absolute;inset:0;pointer-events:none;
background:linear-gradient(var(--acc),var(--acc)) left 12px top 12px/18px 2px no-repeat,linear-gradient(var(--acc),var(--acc)) left 12px top 12px/2px 18px no-repeat,
linear-gradient(var(--acc),var(--acc)) right 12px bottom 12px/18px 2px no-repeat,linear-gradient(var(--acc),var(--acc)) right 12px bottom 12px/2px 18px no-repeat;opacity:.7}
.tiles{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
.tile{background:#0a0f16;border:1px solid var(--line);border-radius:8px;padding:10px 12px}
.tile .l{font:11px var(--mono);color:var(--dim);letter-spacing:.08em;text-transform:uppercase}
.tile .v{font:700 26px/1.2 var(--mono)}.tile .v small{font-size:13px;color:var(--dim);font-weight:400}
.cores{display:flex;gap:4px;align-items:flex-end;height:70px;margin-top:6px}
.cores div{flex:1;background:#0a0f16;border:1px solid var(--line);border-radius:3px;position:relative;height:100%}
.cores div i{position:absolute;left:0;right:0;bottom:0;background:var(--acc);opacity:.85;transition:height .6s}
.cores div span{position:absolute;bottom:-16px;left:0;right:0;text-align:center;font:10px var(--mono);color:var(--dim)}
.bars{display:grid;gap:8px;margin-top:22px}
.bar{display:grid;grid-template-columns:90px 1fr 70px;gap:8px;align-items:center;font:12px var(--mono)}
.bar>div{height:8px;background:#0a0f16;border:1px solid var(--line);border-radius:4px;overflow:hidden}
.bar>div i{display:block;height:100%;background:var(--blue);transition:width .6s}
.bar span:last-child{text-align:right}
#ev{font:12px/1.55 var(--mono);max-height:260px;overflow:auto}
#ev div{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;border-bottom:1px dashed #141c27;padding:2px 0}
#ev .t{color:var(--dim)}#ev .WARN{color:var(--warn)}#ev .ERROR,#ev .FATAL{color:var(--bad)}#ev .ok{color:var(--ok)}
.ovl{position:absolute;top:12px;right:16px;font:11px var(--mono);color:var(--dim);text-align:right}
.legend{display:flex;gap:14px;font:11px var(--mono);color:var(--dim);margin-top:6px;flex-wrap:wrap}
.legend i{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:5px}
#lost{display:none;position:fixed;bottom:18px;left:50%;transform:translateX(-50%);z-index:9;
background:#2a0d10;border:1px solid var(--bad);color:var(--bad);padding:10px 18px;border-radius:8px;
font:600 14px var(--mono);box-shadow:0 0 40px rgba(248,113,113,.25)}
</style></head><body>
<header>
  <div class="logo">NÉO<small>// TEST D'AUTONOMIE LiFePO4</small></div>
  <span id="mode" class="pill live">EN DIRECT</span>
  <span class="hstat">début <b id="start">–</b></span>
  <span class="hstat">écoulé <b id="elapsed">–</b></span>
  <span class="hstat">heure Néo <b id="clock">–</b></span>
  <span class="sp"></span>
  <a class="btn" href="/api/csv">⇩ CSV</a>
</header>
<main>
  <section class="card c4">
    <h2>Batterie</h2>
    <div class="big" id="bv">–<small>V</small></div>
    <div class="gauge"><i id="gfill" style="width:0"></i><b id="gknee"></b></div>
    <div class="glab"><span id="gcut">coupure</span><span>coude 12.0</span><span id="gstart">départ</span></div>
    <div class="kv">
      <span>pente</span><span id="bslope">–</span>
      <span>restant estimé</span><span id="beta">–</span>
      <span>ΔV depuis départ</span><span id="bdv">–</span>
      <span>brute / filtrée</span><span id="braw">–</span>
      <span>mode</span><span id="bcharge">–</span>
      <span>secteur / PC</span><span id="bext">–</span>
      <span>énergie Jetson</span><span id="ewh">–</span>
    </div>
  </section>
  <section class="card c8">
    <h2>Courbe de décharge</h2>
    <div class="ovl" id="chov"></div>
    <canvas id="cbat" class="chart"></canvas>
    <div class="legend"><span><i style="background:var(--warn)"></i>tension filtrée (médiane 30 s)</span>
    <span><i style="background:#4b5a6d"></i>brute</span><span><i style="background:var(--bad)"></i>coupure</span>
    <span><i style="background:var(--mag)"></i>coude 3.0 V/cellule</span></div>
  </section>

  <section class="card c4">
    <h2>Position</h2>
    <div class="ovl" id="mapov"></div>
    <canvas id="cmap" class="mapc"></canvas>
    <div class="legend"><span><i style="background:var(--ok)"></i>trace : batterie pleine</span>
    <span><i style="background:var(--bad)"></i>vide</span><span><i style="background:#ff4d6d"></i>scan laser</span>
    <span><i style="background:var(--acc)"></i>waypoints</span></div>
  </section>
  <section class="card c4">
    <h2>Caméra œil</h2>
    <div class="ovl" id="camov"></div>
    <div class="cam"><img id="camimg" alt=""><div class="none" id="camnone">en attente d'image…</div>
      <span class="rec" id="camrec" style="display:none">LIVE</span><span class="hud" id="camhud"></span></div>
    <div class="kv" id="camk"></div>
  </section>
  <section class="card c4">
    <h2>Mouvement</h2>
    <div class="tiles">
      <div class="tile"><div class="l">vitesse</div><div class="v" id="mv">–<small> m/s</small></div></div>
      <div class="tile"><div class="l">rotation</div><div class="v" id="mw">–<small> rad/s</small></div></div>
      <div class="tile"><div class="l">distance</div><div class="v" id="md">–<small> m</small></div></div>
      <div class="tile"><div class="l">vitesse moy.</div><div class="v" id="mavg">–<small> m/s</small></div></div>
      <div class="tile"><div class="l">waypoints</div><div class="v" id="mwp">–</div></div>
      <div class="tile"><div class="l">tours</div><div class="v" id="mlap">–</div></div>
      <div class="tile"><div class="l">localisation σ</div><div class="v" id="mloc">–<small> cm</small></div></div>
      <div class="tile"><div class="l">lidar</div><div class="v" id="mscan">–<small> Hz</small></div></div>
    </div>
    <canvas id="cspd" style="height:120px;margin-top:12px"></canvas>
  </section>

  <section class="card c4">
    <h2>Jetson Orin NX · CPU par cœur</h2>
    <div class="ovl" id="sysov"></div>
    <div class="cores" id="cores"></div>
    <div class="bars">
      <div class="bar"><span>GPU</span><div><i id="bgpu"></i></div><span id="tgpu">–</span></div>
      <div class="bar"><span>RAM</span><div><i id="bram"></i></div><span id="tram">–</span></div>
      <div class="bar"><span>Wi-Fi</span><div><i id="bwifi"></i></div><span id="twifi">–</span></div>
    </div>
    <div class="kv" id="sysk"></div>
  </section>
  <section class="card c4">
    <h2>Puissance &amp; températures</h2>
    <canvas id="cpow" class="chart"></canvas>
    <div class="legend"><span><i style="background:var(--blue)"></i>puissance module (W)</span>
    <span><i style="background:var(--bad)"></i>température max (°C)</span></div>
  </section>
  <section class="card c4">
    <h2>Journal</h2>
    <div id="ev"></div>
  </section>
</main>
<div id="lost"></div>
<script>
const $=id=>document.getElementById(id);
let H={cols:[],rows:[]},C={},ST=null,LIVE=null,lastOk=Date.now(),lostAt=null,CUT=10.5,KNEE=12.0;
const fmt=(v,d=2)=>v==null||isNaN(v)?'–':(+v).toFixed(d);
const dur=s=>{if(s==null)return'–';s=Math.max(0,s|0);return`${(s/3600|0)}:${String((s%3600/60)|0).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`};
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function col(name){return C[name]}
function series(name){const i=col(name);return H.rows.map(r=>r[i])}

function setupCanvas(cv){const r=cv.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
 if(cv.width!==Math.round(r.width*dpr)||cv.height!==Math.round(r.height*dpr)){cv.width=Math.round(r.width*dpr);cv.height=Math.round(r.height*dpr)}
 const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,r.width,r.height);return[g,r.width,r.height]}

function chart(cv,xs,sets,o={}){
 const[g,W,Hh]=setupCanvas(cv);const L=46,R=o.right?46:12,T=10,B=24,w=W-L-R,h=Hh-T-B;
 g.font='11px '+css('--mono');g.fillStyle=css('--dim');g.strokeStyle='#1b2533';g.lineWidth=1;
 const xv=xs.filter(v=>v!=null);if(xv.length<2){g.fillText('en attente de données…',L+10,T+h/2);return}
 const x0=xv[0],x1=Math.max(xv[xv.length-1],x0+60);const X=v=>L+(v-x0)/(x1-x0)*w;
 const axes={};
 for(const s of sets){const ax=s.axis||'l';const v=s.ys.filter(y=>y!=null&&!isNaN(y));
  if(!axes[ax])axes[ax]={min:Infinity,max:-Infinity};
  if(v.length){axes[ax].min=Math.min(axes[ax].min,...v);axes[ax].max=Math.max(axes[ax].max,...v)}}
 for(const ax in axes){const a=axes[ax],f=o['fix_'+ax];if(f){a.min=Math.min(a.min,f[0]);a.max=Math.max(a.max,f[1])}
  if(!isFinite(a.min)){a.min=0;a.max=1}if(a.max-a.min<1e-6){a.max+=.5;a.min-=.5}
  const p=(a.max-a.min)*.08;a.min-=p;a.max+=p}
 const Y=(v,ax='l')=>T+h-(v-axes[ax].min)/(axes[ax].max-axes[ax].min)*h;
 for(let i=0;i<=4;i++){const yy=T+h*i/4;g.beginPath();g.moveTo(L,yy);g.lineTo(L+w,yy);g.stroke();
  const a=axes.l;if(a)g.fillText((a.max-(a.max-a.min)*i/4).toFixed(o.dl??1),4,yy+4);
  const b=axes.r;if(b&&o.right)g.fillText((b.max-(b.max-b.min)*i/4).toFixed(o.dr??0),L+w+6,yy+4)}
 const span=x1-x0,step=[60,300,600,1800,3600,7200].find(s=>span/s<=8)||7200;
 for(let t=Math.ceil(x0/step)*step;t<=x1;t+=step){const xx=X(t);g.beginPath();g.moveTo(xx,T);g.lineTo(xx,T+h);g.stroke();
  g.fillText(step>=3600?`${t/3600}h`:`${(t/3600|0)}h${String((t%3600/60)|0).padStart(2,'0')}`,xx-14,T+h+16)}
 for(const hl of(o.hl||[])){const yy=Y(hl.y);if(yy<T||yy>T+h)continue;g.save();g.setLineDash([6,4]);g.strokeStyle=hl.c;
  g.beginPath();g.moveTo(L,yy);g.lineTo(L+w,yy);g.stroke();g.restore();g.fillStyle=hl.c;g.textAlign='right';g.fillText(hl.t,L+w-4,yy-4);g.textAlign='left';g.fillStyle=css('--dim')}
 for(const s of sets){g.strokeStyle=s.c;g.lineWidth=s.w||1.6;g.beginPath();let pen=false;
  let px=xs,py=s.ys;
  if(!s.dots&&xs.length>w*1.5){const bx=[],by=[];let cx=null,sx=0,sy=0,n=0;   // moyenne par pixel (courbes longues)
   for(let i=0;i<xs.length;i++){const v=s.ys[i];if(v==null||isNaN(v)||xs[i]==null)continue;const c=Math.round(X(xs[i]));
    if(c!==cx&&n){bx.push(sx/n);by.push(sy/n);sx=sy=n=0}cx=c;sx+=xs[i];sy+=v;n++}
   if(n){bx.push(sx/n);by.push(sy/n)}px=bx;py=by}
  for(let i=0;i<px.length;i++){const v=py[i];if(v==null||isNaN(v)||px[i]==null){pen=false;continue}
   const xx=X(px[i]),yy=Y(v,s.axis||'l');if(s.dots){g.fillStyle=s.c;g.fillRect(xx-.75,yy-.75,1.5,1.5);continue}
   pen?g.lineTo(xx,yy):g.moveTo(xx,yy);pen=true}
  if(!s.dots)g.stroke()}
}

function battColor(v){const v0=(LIVE&&LIVE.batt&&LIVE.batt.v0)||13.4;let t=(v-CUT)/Math.max(.5,v0-CUT);t=Math.max(0,Math.min(1,t));
 const h=t*130;return`hsl(${h},85%,55%)`}

let mapImg=null;
function buildMap(){const m=ST&&ST.map;if(!m)return;const raw=atob(m.cells);const c=document.createElement('canvas');
 c.width=m.w;c.height=m.h;const g=c.getContext('2d');const id=g.createImageData(m.w,m.h);
 for(let r=0;r<m.h;r++)for(let q=0;q<m.w;q++){const src=r*m.w+q;const dst=src*4;const v=raw.charCodeAt(src);
  const p=v===2?[51,224,200,255]:v===1?[20,28,40,255]:[9,12,17,255];id.data.set(p,dst)}
 g.putImageData(id,0,0);mapImg=c}

function drawMap(){const cv=$('cmap');const[g,W,Hh]=setupCanvas(cv);const m=ST&&ST.map;
 if(!m){g.fillStyle=css('--dim');g.fillText('carte indisponible',20,30);return}
 const sx=W/(m.w*m.res),sy=Hh/(m.h*m.res),s=Math.min(sx,sy)*.97;
 const ox=(W-m.w*m.res*s)/2,oy=(Hh-m.h*m.res*s)/2;
 const P=(x,y)=>[ox+(x-m.origin[0])*s,Hh-oy-(y-m.origin[1])*s];
 g.imageSmoothingEnabled=false;if(mapImg)g.drawImage(mapImg,ox,oy,m.w*m.res*s,m.h*m.res*s);
 // grille 1 m
 g.strokeStyle='rgba(51,224,200,.06)';g.lineWidth=1;
 for(let x=Math.ceil(m.origin[0]);x<m.origin[0]+m.w*m.res;x++){const[a]=P(x,0);g.beginPath();g.moveTo(a,oy);g.lineTo(a,Hh-oy);g.stroke()}
 for(let y=Math.ceil(m.origin[1]);y<m.origin[1]+m.h*m.res;y++){const[,b]=P(0,y);g.beginPath();g.moveTo(ox,b);g.lineTo(W-ox,b);g.stroke()}
 // trace
 const xs=series('x'),ys=series('y'),vs=series('batt_vf');g.lineWidth=2;g.lineCap='round';
 for(let i=1;i<xs.length;i++){if(xs[i]==null||xs[i-1]==null)continue;const a=P(xs[i-1],ys[i-1]),b=P(xs[i],ys[i]);
  if(Math.hypot(a[0]-b[0],a[1]-b[1])>80)continue;g.strokeStyle=battColor(vs[i]??vs[i-1]??13);g.globalAlpha=.55;
  g.beginPath();g.moveTo(...a);g.lineTo(...b);g.stroke()}g.globalAlpha=1;
 // waypoints
 for(const w of(ST.wps||[])){const[a,b]=P(w.x,w.y);g.strokeStyle=css('--acc');g.lineWidth=1.5;g.setLineDash([3,3]);
  g.beginPath();g.arc(a,b,.45*s,0,7);g.stroke();g.setLineDash([]);g.fillStyle=css('--acc');g.beginPath();g.arc(a,b,3,0,7);g.fill();
  g.font='bold 12px '+css('--mono');g.fillText(w.name,a+8,b-8)}
 // scan
 if(LIVE&&LIVE.scan){g.fillStyle='#ff4d6d';for(const[x,y]of LIVE.scan){const[a,b]=P(x,y);g.fillRect(a-1.2,b-1.2,2.4,2.4)}}
 // robot
 const p=LIVE&&LIVE.pose;if(p){const[a,b]=P(p.x,p.y);const r=.2*s;g.save();g.translate(a,b);g.rotate(-p.yaw);
  g.shadowColor=css('--acc');g.shadowBlur=18;g.fillStyle='rgba(51,224,200,.18)';g.strokeStyle=css('--acc');g.lineWidth=2;
  g.beginPath();g.arc(0,0,r,0,7);g.fill();g.stroke();g.beginPath();g.moveTo(r*1.8,0);g.lineTo(-r*.5,r*.7);g.lineTo(-r*.5,-r*.7);g.closePath();
  g.fillStyle=css('--acc');g.fill();g.restore();
  $('mapov').textContent=`x ${fmt(p.x)} · y ${fmt(p.y)} m · ${fmt(p.yaw*57.2958,0)}° (${p.frame})`}
}

function p0(){const p=LIVE&&LIVE.pose;const b=LIVE&&LIVE.batt;return(p?`x ${fmt(p.x)} y ${fmt(p.y)} · `:'')+(b&&(b.vf??b.v)!=null?`${fmt(b.vf??b.v)} V`:'')}
function setBar(id,pct){$(id).style.width=Math.max(0,Math.min(100,pct||0))+'%'}

function render(){
 if(!LIVE)return;const L=LIVE,b=L.batt||{},m=L.motion||{},s=L.sys||{};CUT=L.cutoff;KNEE=L.knee;
 $('start').textContent=L.start||'–';$('elapsed').textContent=dur(L.elapsed);$('clock').textContent=L.server_time;
 const mode=$('mode');
 if(L.mode==='replay'){mode.className='pill replay';mode.textContent='REPLAY'}
 else if(lostAt){mode.className='pill off';mode.textContent='HORS LIGNE'}else{mode.className='pill live';mode.textContent='EN DIRECT'}
 const v=b.vf??b.v;$('bv').innerHTML=`${fmt(v)}<small>V</small>`;
 $('bv').style.color=v==null?css('--txt'):v<=KNEE?css('--bad'):v<KNEE+.6?css('--warn'):css('--ok');
 const v0=b.v0||13.4;setBar('gfill',(v-CUT)/(v0-CUT)*100);$('gknee').style.left=((KNEE-CUT)/(v0-CUT)*100)+'%';
 $('gcut').textContent=`coupure ${CUT}`;$('gstart').textContent=`départ ${fmt(v0)}`;
 $('bslope').textContent=b.slope==null?'calcul (2 min)…':`${fmt(b.slope,1)} mV/min`;
 $('beta').textContent=b.eta==null?(L.mode==='replay'?'—':'–'):`${dur(b.eta*60)} (linéaire*)`;
 $('bdv').textContent=b.v0==null||v==null?'–':`${fmt(v-b.v0,2)} V`;
 $('braw').textContent=`${fmt(b.v,1)} / ${fmt(b.vf,2)} V`;
 $('bcharge').textContent=b.charge||'–';$('bext').textContent=`${b.ext_power||'–'} / ${b.pc_on||'–'}`;
 $('ewh').textContent=s.e_wh!=null?`${fmt(s.e_wh,1)} Wh`:'–';
 $('mv').innerHTML=`${fmt(m.v)}<small> m/s</small>`;$('mw').innerHTML=`${fmt(m.w)}<small> rad/s</small>`;
 $('md').innerHTML=`${fmt(m.dist,0)}<small> m</small>`;$('mavg').innerHTML=`${fmt(L.elapsed>0?m.dist/L.elapsed:null)}<small> m/s</small>`;
 $('mwp').innerHTML=`${m.wps??'–'}<small> ${m.last_wp||''}</small>`;$('mlap').innerHTML=`${m.laps??'–'}<small> trajet ${m.legs??'–'}</small>`;
 $('mloc').innerHTML=`${m.loc_std==null?'–':fmt(m.loc_std*100,0)}<small> cm</small>`;$('mscan').innerHTML=`${fmt(m.scan_hz,1)}<small> Hz</small>`;
 // système
 const cs=$('cores');const cores=s.cores||[];if(cs.children.length!==cores.length){cs.innerHTML=cores.map((_,i)=>`<div><i></i><span>${i}</span></div>`).join('')}
 cores.forEach((c,i)=>{const e=cs.children[i].firstChild;e.style.height=c+'%';e.style.background=c>85?css('--bad'):c>60?css('--warn'):css('--acc')});
 setBar('bgpu',s.gpu);$('tgpu').textContent=s.gpu==null?'n/a':fmt(s.gpu,0)+' %';
 setBar('bram',s.ram/s.ram_tot*100);$('tram').textContent=s.ram?`${(s.ram/1024).toFixed(1)}/${(s.ram_tot/1024).toFixed(0)} Go`:'–';
 setBar('bwifi',s.wifi_dbm==null?0:(s.wifi_dbm+90)/60*100);$('twifi').textContent=s.wifi_dbm==null?'–':`${fmt(s.wifi_dbm,0)} dBm`;
 $('sysov').textContent=s.cpu==null?'':`moy. ${fmt(s.cpu,0)} %`;
 const t=s.temps||{};const tk=Object.keys(t).filter(k=>/cpu|gpu|soc|tj|cv/i.test(k)).slice(0,6);
 $('sysk').innerHTML=[['puissance',s.p_w==null?'n/a':fmt(s.p_w,1)+' W'],...tk.map(k=>[k,fmt(t[k],1)+' °C']),
  ['charge moy.',(s.load||[]).join(' ')],['swap',s.swap!=null?s.swap+' Mo':'–'],['uptime',dur(s.uptime_s)],['disque libre',s.disk_gb!=null?s.disk_gb+' Go':'–']]
  .map(([a,c])=>`<span>${a}</span><span>${c}</span>`).join('');
 // caméra
 const cm=L.cam||{};const img=$('camimg');
 if(L.mode!=='live'){$('camnone').textContent='pas de caméra en replay';img.style.display='none'}
 else if(cm.available===false){$('camnone').textContent='OpenCV absent sur Néo'}
 else{if(!img.dataset.on){img.dataset.on=1;img.onload=()=>{$('camnone').style.display='none';$('camrec').style.display=''};
   img.onerror=()=>{delete img.dataset.on;$('camrec').style.display='none'};img.src='/cam.mjpg?'+Date.now()}
  const stale=cm.age==null||cm.age>2;$('camrec').style.display=stale?'none':'';
  if(cm.age==null){$('camnone').style.display='';$('camnone').textContent=`aucune image sur ${cm.topic||'?'}`}
  $('camhud').textContent=`${p0()}`}
 $('camov').textContent=cm.topic||'';
 $('camk').innerHTML=[['source',cm.hz==null?'–':fmt(cm.hz,1)+' i/s'],['flux navigateur',cm.stream_fps?cm.stream_fps+' i/s':'–'],
  ['résolution',cm.res||'–'],['âge image',cm.age==null?'–':fmt(cm.age,1)+' s']].map(([a,c])=>`<span>${a}</span><span>${c}</span>`).join('');
 // journal
 $('ev').innerHTML=(L.events||[]).slice().reverse().map(e=>{const ok=/Passé par|Arrivé/.test(e.msg)?' ok':'';
  return`<div class="${e.lvl}${ok}"><span class="t">${e.t}</span> ${e.node==='waypoint_mission'?'':'['+e.node+'] '}${e.msg.replace(/</g,'&lt;')}</div>`}).join('');
 drawAll()}

function drawAll(){
 const t=series('t_s');
 chart($('cbat'),t,[{ys:series('batt_v'),c:'#4b5a6d',dots:true},{ys:series('batt_vf'),c:css('--warn'),w:2.2}],
  {dl:2,fix_l:[CUT,Math.max(KNEE+.3,CUT+1)],hl:[{y:CUT,c:css('--bad'),t:`coupure ${CUT}`},{y:KNEE,c:css('--mag'),t:`coude ${KNEE}`}]});
 const n=t.length,last=n?t[n-1]:0;
 $('chov').textContent=n?`${n} points · ${dur(last)}`:'';
 chart($('cpow'),t,[{ys:series('p_w'),c:css('--blue')},{ys:series('temp_max'),c:css('--bad'),axis:'r'}],{right:true,dl:1,dr:0});
 const k=t.findIndex(x=>x>=last-300);const sl=a=>a.slice(Math.max(0,k));
 chart($('cspd'),sl(t),[{ys:sl(series('v_cmd')),c:'#4b5a6d'},{ys:sl(series('v')),c:css('--acc'),w:1.8}],{dl:2,fix_l:[0,.35]});
 drawMap()}

async function poll(){
 try{const r=await fetch('/api/live',{cache:'no-store'});LIVE=await r.json();lastOk=Date.now();
  if(lostAt){lostAt=null;$('lost').style.display='none'}render()}
 catch(e){if(!lostAt&&LIVE&&LIVE.mode==='live'){lostAt=new Date();const el=$('lost');el.style.display='block';
  el.textContent=`⚠ CONNEXION PERDUE à ${lostAt.toLocaleTimeString('fr-FR')} — dernière tension ${fmt(LIVE.batt&&(LIVE.batt.vf??LIVE.batt.v))} V après ${dur(LIVE.elapsed)}`;render()}}
}
async function pollHist(){
 try{const r=await fetch('/api/history?since='+H.rows.length,{cache:'no-store'});const d=await r.json();
  if(!H.cols.length){H.cols=d.cols;d.cols.forEach((c,i)=>C[c]=i)}
  if(d.since===H.rows.length)H.rows.push(...d.rows);if(LIVE)drawAll()}catch(e){}
}
(async()=>{try{ST=await(await fetch('/api/static')).json();buildMap()}catch(e){}
 await pollHist();await poll();setInterval(poll,1000);setInterval(pollHist,4000);
 window.addEventListener('resize',drawAll)})();
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = NeoDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
