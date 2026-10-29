#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
neo_monitor.py — « boîte noire » de Néo pendant les tests de navigation
=======================================================================
Auteur : Vincent FOUCAULT

Écrit toutes les `period_s` secondes une ligne d'état dans
~/.ros/qbo_nav_test/blackbox_AAAAMMJJ_HHMMSS.log, avec fsync() après CHAQUE
ligne : si le Jetson s'éteint ou gèle, les dernières secondes avant la panne
sont sur le disque.

Chaque ligne contient :
  - batterie (V, depuis /diagnostics « Battery Status »)
  - charge CPU (1 min), RAM utilisée/disponible, swap
  - températures (CPU, GPU, SoC, max des zones thermiques)
  - puissance consommée par le module Orin (capteurs INA3221, si lisibles)
  - Wi-Fi : qualité / niveau du signal (/proc/net/wireless)
  - vitesse commandée (cmd_vel) et vitesse mesurée (odom)
  - âge du dernier scan lidar

Toutes les 30 s : les 5 processus les plus gourmands (CPU et RAM).
En continu : les nouveaux messages noyau (dmesg) contenant des mots-clés de
panne (oom, throttl, over-current, voltage, usb disconnect, wlan, error...).

Après un plantage :  tail -60 ~/.ros/qbo_nav_test/blackbox_*.log
"""

import datetime
import glob
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

DMESG_KEYS = ('oom', 'out of memory', 'killed process', 'throttl', 'over-current', 'overcurrent',
              'under-voltage', 'undervolt', 'voltage', 'brown', 'usb', 'disconnect', 'wlan', 'wifi',
              'iwlwifi', 'rtl', 'mt76', 'link is down', 'thermal', 'shutdown', 'error', 'fail', 'hung',
              'watchdog', 'soctherm', 'oc alarm')


def read(path, default=None):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return default


class NeoMonitor(Node):

    def __init__(self):
        super().__init__('neo_monitor')
        self.declare_parameter('period_s', 2.0)
        self.declare_parameter('log_dir', '~/.ros/qbo_nav_test')
        self.declare_parameter('cmd_vel_topic', '/qbo_arduqbo/base_ctrl/cmd_vel')
        self.declare_parameter('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.period = float(g('period_s'))
        log_dir = os.path.expanduser(g('log_dir'))
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f'blackbox_{stamp}.log')
        self.f = open(self.path, 'a', buffering=1, encoding='utf-8')

        self.battery_v = None
        self.cmd = (0.0, 0.0)
        self.cmd_t = 0.0
        self.odom = (0.0, 0.0)
        self.scan_t = 0.0
        self.last_top = 0.0
        self.thermal = self._thermal_zones()
        self.power = self._power_sensors()

        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag_cb, 10)
        self.create_subscription(Twist, g('cmd_vel_topic'), self._cmd_cb, 10)
        self.create_subscription(Odometry, g('odom_topic'), self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_timer(self.period, self._tick)

        self._write('=== NEO BLACKBOX démarrée — ' + datetime.datetime.now().isoformat(timespec='seconds')
                    + f' — uptime {self._uptime()} ===')
        self._write('# heure | batt V | load1 | RAM utilisée/dispo Mo | swap Mo | temp °C (max zone) '
                    '| puissance W | wifi qualité/niveau | cmd v,w | odom v,w | âge scan s')
        self._write('# zones thermiques : ' + ', '.join(n for n, _ in self.thermal))
        self._write('# capteurs puissance : ' + (', '.join(n for n, _, _ in self.power) or 'aucun lisible'))
        threading.Thread(target=self._dmesg_follow, daemon=True).start()
        self.get_logger().info(f'Boîte noire : {self.path}')

    # ---------------------------------------------------------------- callbacks
    def _diag_cb(self, msg):
        for st in msg.status:
            if 'Battery' in st.name:
                for kv in st.values:
                    if kv.key == 'Voltage':
                        try:
                            self.battery_v = float(kv.value)
                        except ValueError:
                            pass

    def _cmd_cb(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.monotonic()

    def _odom_cb(self, m):
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _scan_cb(self, _m):
        self.scan_t = time.monotonic()

    # ---------------------------------------------------------------- système
    @staticmethod
    def _uptime():
        u = read('/proc/uptime', '0').split()[0]
        return f'{float(u) / 60:.1f} min'

    @staticmethod
    def _thermal_zones():
        zones = []
        for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
            name = read(z + '/type', os.path.basename(z))
            zones.append((name, z + '/temp'))
        return zones

    @staticmethod
    def _power_sensors():
        """INA3221 du module Orin : (nom du rail, fichier tension mV, fichier courant mA)."""
        out = []
        for hw in glob.glob('/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*'):
            for lab in sorted(glob.glob(hw + '/in*_label')):
                idx = lab.split('in')[-1].split('_')[0]
                v, c = f'{hw}/in{idx}_input', f'{hw}/curr{idx}_input'
                if os.path.exists(v) and os.path.exists(c):
                    out.append((read(lab, f'in{idx}'), v, c))
        return out

    def _mem(self):
        info = {}
        for line in (read('/proc/meminfo', '') or '').splitlines():
            k, _, v = line.partition(':')
            info[k] = int(v.split()[0]) // 1024 if v.split() else 0
        used = info.get('MemTotal', 0) - info.get('MemAvailable', 0)
        swap = info.get('SwapTotal', 0) - info.get('SwapFree', 0)
        return used, info.get('MemAvailable', 0), swap

    def _temps(self):
        vals = []
        for name, p in self.thermal:
            t = read(p)
            if t and t.lstrip('-').isdigit():
                vals.append((name, int(t) / 1000.0))
        keep = [f'{n.replace("-thermal", "")}={t:.0f}' for n, t in vals
                if any(k in n.lower() for k in ('cpu', 'gpu', 'soc', 'tj'))]
        tmax = max((t for _, t in vals), default=0.0)
        return ' '.join(keep[:4]), tmax

    def _power_w(self):
        tot = 0.0
        parts = []
        for name, vf, cf in self.power:
            v, c = read(vf), read(cf)
            if v and c and v.isdigit() and c.isdigit():
                w = int(v) * int(c) / 1e6
                tot += w
                parts.append(f'{name}={w:.1f}')
        return (f'{tot:.1f} ({" ".join(parts)})' if parts else 'n/a')

    @staticmethod
    def _wifi():
        lines = (read('/proc/net/wireless', '') or '').splitlines()[2:]
        if not lines:
            return 'pas de wifi'
        f = lines[0].split()
        return f'{f[0].rstrip(":")} q={f[2].rstrip(".")} lvl={f[3].rstrip(".")}dBm'

    def _top(self):
        try:
            cpu = subprocess.run(['ps', '-eo', 'pid,pcpu,pmem,rss,comm', '--sort=-pcpu'],
                                 capture_output=True, text=True, timeout=3).stdout.splitlines()[1:6]
            mem = subprocess.run(['ps', '-eo', 'pid,pcpu,pmem,rss,comm', '--sort=-rss'],
                                 capture_output=True, text=True, timeout=3).stdout.splitlines()[1:6]
            self._write('  TOP CPU : ' + ' | '.join(' '.join(l.split()) for l in cpu))
            self._write('  TOP RAM : ' + ' | '.join(' '.join(l.split()) for l in mem))
        except Exception as e:  # noqa: BLE001
            self._write(f'  (ps indisponible : {e})')

    def _dmesg_follow(self):
        try:
            p = subprocess.Popen(['dmesg', '-w', '-T', '--level=emerg,alert,crit,err,warn,notice'],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except Exception:
            self._write('  (dmesg indisponible : lancer « sudo sysctl kernel.dmesg_restrict=0 »)')
            return
        first = time.monotonic()
        for line in p.stdout:
            if time.monotonic() - first < 3.0:      # ignore l'historique relu au démarrage
                continue
            if any(k in line.lower() for k in DMESG_KEYS):
                self._write('  KERNEL : ' + line.strip())

    # ---------------------------------------------------------------- écriture
    def _write(self, text):
        try:
            self.f.write(text + '\n')
            self.f.flush()
            os.fsync(self.f.fileno())   # sur le disque MAINTENANT (survit à une coupure)
        except Exception:
            pass

    def _battery_alert(self, now):
        """Alerte batterie sans spam : 1 message par palier de 0.2 V (ou toutes les 2 min).
        La tension en charge oscille de ±0.1 V : on suit le minimum, pas la valeur brute."""
        v = self.battery_v
        if v is None:
            return
        self.batt_min = min(getattr(self, 'batt_min', v), v)
        if self.batt_min >= 12.0:
            return
        last_v = getattr(self, 'batt_warn_v', None)
        last_t = getattr(self, 'batt_warn_t', 0.0)
        if last_v is None or self.batt_min <= last_v - 0.2 or now - last_t > 120.0:
            self.batt_warn_v, self.batt_warn_t = self.batt_min, now
            if self.batt_min < 11.0:
                self.get_logger().error(f'BATTERIE CRITIQUE : {self.batt_min:.1f} V — ARRÊTER ET RECHARGER')
            else:
                self.get_logger().warn(f'BATTERIE BASSE : {self.batt_min:.1f} V')

    def _tick(self):
        now = time.monotonic()
        used, avail, swap = self._mem()
        temps, tmax = self._temps()
        load = (read('/proc/loadavg', '0') or '0').split()[0]
        batt = f'{self.battery_v:.2f}' if self.battery_v is not None else '?'
        cmd = self.cmd if now - self.cmd_t < 1.0 else (0.0, 0.0)
        scan_age = f'{now - self.scan_t:.1f}' if self.scan_t else '∞'
        self._write(f'{datetime.datetime.now().strftime("%H:%M:%S")} | {batt} V | load {load} | '
                    f'RAM {used}/{avail} | swap {swap} | {temps} max={tmax:.0f} | P {self._power_w()} | '
                    f'{self._wifi()} | cmd {cmd[0]:+.2f},{cmd[1]:+.2f} | odom {self.odom[0]:+.2f},'
                    f'{self.odom[1]:+.2f} | scan {scan_age}')
        self._battery_alert(now)
        if tmax > 85.0:
            self.get_logger().warn(f'TEMPÉRATURE ÉLEVÉE : {tmax:.0f} °C')
        if now - self.last_top > 30.0:
            self.last_top = now
            self._top()


def main(args=None):
    rclpy.init(args=args)
    node = NeoMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write('=== arrêt propre de la boîte noire ===')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
