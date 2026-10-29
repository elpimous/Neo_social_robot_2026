#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speed_trace.py — d'où vient l'effet accordéon ?
================================================
Auteur : Vincent FOUCAULT

Enregistre à 20 Hz, pendant la navigation, la vitesse linéaire à CHAQUE étage :

  contrôleur RPP (/cmd_vel_nav) -> lissage (/cmd_vel_smoothed)
  -> collision_monitor (/qbo_arduqbo/base_ctrl/cmd_vel) -> base réelle (odom)

puis indique quel étage fait chuter la vitesse.

  ros2 run qbo_nav_waypoints speed_trace            (pendant que Néo roule, 30 s)
  ros2 run qbo_nav_waypoints speed_trace --ros-args -p duration_s:=60
"""

import datetime
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SpeedTrace(Node):
    def __init__(self):
        super().__init__('speed_trace')
        self.declare_parameter('duration_s', 30.0)
        self.duration = float(self.get_parameter('duration_s').value)
        self.v = {'rpp': 0.0, 'lisse': 0.0, 'base_cmd': 0.0, 'odom': 0.0}
        self.w = {'rpp': 0.0}
        self.last_xy = None
        self.last_t = None
        self.rows = []
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self._tw('rpp', m), 20)
        self.create_subscription(Twist, '/cmd_vel_smoothed', lambda m: self._tw('lisse', m), 20)
        self.create_subscription(Twist, '/qbo_arduqbo/base_ctrl/cmd_vel', lambda m: self._tw('base_cmd', m), 20)
        self.create_subscription(Odometry, '/qbo_arduqbo/base_ctrl/odom', self._odom, 20)
        self.t0 = time.monotonic()
        self.create_timer(0.05, self._sample)

    def _tw(self, k, m):
        self.v[k] = m.linear.x
        if k == 'rpp':
            self.w['rpp'] = m.angular.z

    def _odom(self, m):
        # vitesse réelle calculée sur la position (twist.linear.x de la base n'a pas de signe)
        p = m.pose.pose.position
        t = time.monotonic()
        if self.last_xy is not None and t - self.last_t > 0.04:
            self.v['odom'] = math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1]) / (t - self.last_t)
            self.last_xy, self.last_t = (p.x, p.y), t
        elif self.last_xy is None:
            self.last_xy, self.last_t = (p.x, p.y), t

    def _sample(self):
        t = time.monotonic() - self.t0
        self.rows.append((t, self.v['rpp'], self.w['rpp'], self.v['lisse'], self.v['base_cmd'], self.v['odom']))
        if t >= self.duration:
            raise SystemExit


def analyse(rows):
    moving = [r for r in rows if r[1] > 0.05]
    if len(moving) < 20:
        return ['Pas assez de données en mouvement : lance l\'outil PENDANT que Néo roule.']
    out = []
    names = ['RPP', 'lissage', 'collision_monitor', 'base réelle']
    for i, n in zip((1, 3, 4, 5), names):
        vals = [r[i] for r in moving]
        mean = sum(vals) / len(vals)
        std = math.sqrt(sum((x - mean) ** 2 for x in vals) / len(vals))
        dips = sum(1 for a, b in zip(vals, vals[1:]) if b < a - 0.05)
        out.append(f'{n:18s} moyenne {mean:.2f} m/s  écart-type {std:.3f}  chutes>5cm/s : {dips}')
    # Étage responsable : plus forte augmentation de l'écart-type
    stds = []
    for i in (1, 3, 4, 5):
        vals = [r[i] for r in moving]
        m = sum(vals) / len(vals)
        stds.append(math.sqrt(sum((x - m) ** 2 for x in vals) / len(vals)))
    jumps = [stds[0]] + [stds[k] - stds[k - 1] for k in range(1, 4)]
    worst = max(range(4), key=lambda k: jumps[k])
    out.append(f'=> les variations naissent surtout dans : {names[worst].upper()}')
    return out


def main(args=None):
    rclpy.init(args=args)
    node = SpeedTrace()
    print(f'Enregistrement {node.duration:.0f} s (Néo doit rouler)...')
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    d = os.path.expanduser('~/.ros/qbo_nav_test')
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f'speed_trace_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
    with open(path, 'w') as f:
        f.write('t_s,rpp_v,rpp_w,lisse_v,base_cmd_v,odom_v\n')
        for r in node.rows:
            f.write(','.join(f'{x:.3f}' for x in r) + '\n')
    print('\n'.join(analyse(node.rows)))
    print(f'CSV : {path}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
