#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
head_lock.py — Verrouillage permanent de la tête de Néo (lidar au sommet)
=========================================================================
Auteur : Vincent FOUCAULT

Indépendant de la mission : la tête est verrouillée dès le lancement, même
pendant l'enregistrement des waypoints ou si la mission s'arrête.

  * envoie pan = 0, tilt = tilt_level_rad sur /cmd_joints (qbo_dynamixel)
    avec une vitesse de ralliement au premier envoi ;
  * renvoie la consigne toutes les `republish_s` (sans vitesse) : empêche
    l'auto torque OFF de qbo_dynamixel (20 s sans commande) ;
  * surveille /joint_states : si la tête s'écarte de plus de `drift_tol_rad`,
    renvoie immédiatement la consigne (avec vitesse) et le signale.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HeadLock(Node):

    def __init__(self):
        super().__init__('head_lock')
        d = self.declare_parameter
        d('command_topic', '/cmd_joints')
        d('joint_states_topic', '/joint_states')
        d('pan_joint', 'head_pan_joint')
        d('tilt_joint', 'head_tilt_joint')
        d('pan_rad', 0.0)
        d('tilt_level_rad', 0.0)
        d('speed_rad_s', 0.6)
        d('republish_s', 5.0)
        d('drift_tol_rad', 0.035)
        d('status_period_s', 30.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pan_j = g('pan_joint')
        self.tilt_j = g('tilt_joint')
        self.target = {self.pan_j: float(g('pan_rad')), self.tilt_j: float(g('tilt_level_rad'))}
        self.speed = float(g('speed_rad_s'))
        self.drift = float(g('drift_tol_rad'))
        self.status_period = float(g('status_period_s'))

        self.meas = {}
        self.locked = False
        self.last_fix = 0.0
        self.last_status = 0.0

        self.pub = self.create_publisher(JointState, g('command_topic'), 10)
        self.create_subscription(JointState, g('joint_states_topic'), self._js_cb, 20)
        self.create_timer(float(g('republish_s')), lambda: self.send(False))
        self.create_timer(0.2, self._check)
        self.send(True)
        self.get_logger().info(
            f'Verrou tête : pan {math.degrees(self.target[self.pan_j]):.1f}°, '
            f'tilt {math.degrees(self.target[self.tilt_j]):.2f}° (tilt_level_rad)')

    def send(self, with_speed):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [self.pan_j, self.tilt_j]
        m.position = [self.target[self.pan_j], self.target[self.tilt_j]]
        if with_speed:
            m.velocity = [self.speed, self.speed]
        self.pub.publish(m)

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in self.target:
                self.meas[n] = p

    def _check(self):
        if len(self.meas) < 2:
            return
        err = max(abs(self.meas[n] - t) for n, t in self.target.items())
        now = time.monotonic()
        if err <= self.drift:
            if not self.locked:
                self.get_logger().info(f'Tête VERROUILLÉE (erreur {math.degrees(err):.1f}°)')
                self.locked = True
        else:
            if self.locked:
                self.get_logger().warn(f'Tête hors position ({math.degrees(err):.1f}°) : reverrouillage')
                self.locked = False
            if now - self.last_fix > 1.0:
                self.send(True)
                self.last_fix = now
        if now - self.last_status > self.status_period:
            self.last_status = now
            self.get_logger().info(
                f'tête : pan {math.degrees(self.meas[self.pan_j]):+.1f}°  '
                f'tilt {math.degrees(self.meas[self.tilt_j]):+.1f}°  '
                f'({"verrouillée" if self.locked else "EN MOUVEMENT"})')


def main(args=None):
    rclpy.init(args=args)
    node = HeadLock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
