#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_trace.py — Suivi en direct des waypoints et des erreurs pendant que Néo roule
=================================================================================
Auteur : Vincent FOUCAULT

Écoute /rosout (les logs de TOUS les nœuds) et affiche :
  - tous les messages de la mission (waypoint_mission) : trajets, passages, échecs ;
  - les WARN / ERROR / FATAL de tous les autres nœuds (Nav2, lidar, base...) ;
  - les manœuvres de dégagement (spin, backup, wait).
Les messages répétitifs (ex. « collision ahead ») sont regroupés : 1 ligne / 5 s
avec le nombre de répétitions.

Tout est aussi écrit dans ~/.ros/qbo_nav_test/trace_AAAAMMJJ_HHMMSS.log

  ros2 run qbo_nav_waypoints nav_trace
  ros2 run qbo_nav_waypoints nav_trace --ros-args -p warnings:=false   # erreurs seulement
"""

import os
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log

LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
COLORS = {20: '\033[0m', 30: '\033[33m', 40: '\033[31m', 50: '\033[1;31m'}
GREEN = '\033[32m'
CYAN = '\033[36m'
RESET = '\033[0m'
RECOVERY_KEYS = ('Running spin', 'Running backup', 'Running wait', 'Spin', 'BackUp',
                 'clear', 'Clearing')


class NavTrace(Node):

    def __init__(self):
        super().__init__('nav_trace')
        self.declare_parameter('warnings', True)
        self.declare_parameter('repeat_window_s', 5.0)
        self.show_warn = bool(self.get_parameter('warnings').value)
        self.window = float(self.get_parameter('repeat_window_s').value)
        self.last = {}            # (nœud, texte) -> [dernier affichage, répétitions masquées]
        self.counts = {'wp': 0, 'warn': 0, 'error': 0}

        out_dir = os.path.expanduser('~/.ros/qbo_nav_test')
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, datetime.now().strftime('trace_%Y%m%d_%H%M%S.log'))
        self.file = open(self.path, 'a', encoding='utf-8', buffering=1)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Log, '/rosout', self._cb, qos)
        print(f'{CYAN}=== Trace navigation Néo — fichier : {self.path} — Ctrl+C pour quitter ==={RESET}',
              flush=True)

    def _keep(self, msg):
        if msg.name == self.get_name():
            return False
        if 'waypoint_mission' in msg.name:
            return True
        if msg.level >= 40:
            return True
        if any(k in msg.msg for k in RECOVERY_KEYS) and msg.level >= 20:
            return True
        return self.show_warn and msg.level >= 30

    def _cb(self, msg):
        if not self._keep(msg):
            return
        now = time.monotonic()
        key = (msg.name, msg.msg[:80])
        entry = self.last.get(key)
        if entry and now - entry[0] < self.window:
            entry[1] += 1
            return
        repeats = entry[1] if entry else 0
        self.last[key] = [now, 0]

        stamp = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec * 1e-9).strftime('%H:%M:%S')
        lvl = LEVELS.get(msg.level, str(msg.level))
        node = msg.name.split('.')[-1]
        suffix = f'  (x{repeats + 1} en {self.window:.0f} s)' if repeats else ''
        line = f'[{stamp}] {lvl:5s} {node}: {msg.msg}{suffix}'

        color = COLORS.get(msg.level, RESET)
        if 'waypoint_mission' in msg.name:
            if 'Passé par' in msg.msg or 'Arrivé' in msg.msg:
                color = GREEN
                self.counts['wp'] += 1
            elif msg.level < 30:
                color = CYAN
        if msg.level == 30:
            self.counts['warn'] += 1
        elif msg.level >= 40:
            self.counts['error'] += 1

        print(f'{color}{line}{RESET}', flush=True)
        self.file.write(line + '\n')

    def close(self):
        resume = (f'=== Fin de trace : {self.counts["wp"]} waypoints atteints, '
                  f'{self.counts["warn"]} WARN, {self.counts["error"]} ERROR ===')
        print(f'{CYAN}{resume}{RESET}', flush=True)
        self.file.write(resume + '\n')
        self.file.close()


def main(args=None):
    rclpy.init(args=args)
    node = NavTrace()
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
