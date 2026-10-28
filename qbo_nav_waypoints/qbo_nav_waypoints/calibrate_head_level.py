#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_head_level.py — Trouver le tilt qui met le RPLidar à l'horizontale
=============================================================================
Auteur : Vincent FOUCAULT

Pose un niveau à bulle (ou l'appli niveau d'un téléphone) sur le dessus du
RPLidar, puis ajuste le tilt au clavier jusqu'à l'horizontale parfaite.
Le pan est maintenu à 0 (face).

  +  / -   : tilt +0.5° / -0.5°
  ++ / --  : tilt +2°   / -2°
  p        : afficher la position mesurée (/joint_states)
  q        : quitter et afficher la valeur à reporter

  ros2 run qbo_nav_waypoints calibrate_head_level

La valeur trouvée se passe au launch :  tilt_level_rad:=<valeur>
(ou se met en défaut dans le launch / config).
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Calib(Node):
    def __init__(self):
        super().__init__('calibrate_head_level')
        self.declare_parameter('command_topic', '/cmd_joints')
        self.declare_parameter('pan_joint', 'head_pan_joint')
        self.declare_parameter('tilt_joint', 'head_tilt_joint')
        self.declare_parameter('start_tilt_rad', 0.0)
        self.pan_j = self.get_parameter('pan_joint').value
        self.tilt_j = self.get_parameter('tilt_joint').value
        self.target = float(self.get_parameter('start_tilt_rad').value)
        self.meas = {}
        self.pub = self.create_publisher(JointState, self.get_parameter('command_topic').value, 10)
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.create_timer(5.0, lambda: self.send(False))   # garde le couple actif

    def _js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.meas[n] = p

    def send(self, with_speed=True):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [self.pan_j, self.tilt_j]
        m.position = [0.0, self.target]
        if with_speed:
            m.velocity = [0.3, 0.3]
        self.pub.publish(m)

    def status(self):
        t = self.meas.get(self.tilt_j)
        p = self.meas.get(self.pan_j)
        mt = f'{t:+.4f} rad ({math.degrees(t):+.2f}°)' if t is not None else '?'
        mp = f'{math.degrees(p):+.1f}°' if p is not None else '?'
        return (f'consigne tilt {self.target:+.4f} rad ({math.degrees(self.target):+.2f}°) | '
                f'mesuré tilt {mt} | pan {mp}')


def main(args=None):
    rclpy.init(args=args)
    node = Calib()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    node.send()
    steps = {'+': 0.5, '-': -0.5, '++': 2.0, '--': -2.0}
    print(__doc__.split('\n\n')[1])
    print('Commandes : + - ++ -- p q')
    try:
        while rclpy.ok():
            cmd = input('> ').strip()
            if cmd == 'q':
                break
            if cmd in steps:
                node.target += math.radians(steps[cmd])
                node.target = max(math.radians(-30), min(math.radians(20), node.target))  # limites yaml
                node.send()
                time.sleep(0.6)
            print(node.status())
    except (KeyboardInterrupt, EOFError):
        pass
    t = node.meas.get(node.tilt_j, node.target)
    print('\n=== RÉSULTAT ===')
    print(f'tilt_level_rad:={t:.4f}   ({math.degrees(t):+.2f}°, valeur MESURÉE par le servo)')
    print('Lance le test avec :')
    print(f'  ros2 launch qbo_nav_waypoints test_moving_laser_on_waypoints.launch.py tilt_level_rad:={t:.4f}')
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
