#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
head_laser_tf.py — TF dynamique du RPLidar monté sur la tête de Néo
===================================================================
Auteur : Vincent FOUCAULT

Le RPLidar A2 est fixé au sommet de la tête (Ø ~20 cm), elle-même portée par
les servos pan (head_pan_joint) et tilt (head_tilt_joint). La pose réelle du
plan laser dépend donc des angles mesurés des servos : ce nœud publie

    base_link -> head_pan_link   (rotation pan autour de z, au centre de la tête)
    head_pan_link -> head_tilt_link (rotation tilt autour de y)
    head_tilt_link -> laser      (fixe : au-dessus du centre de la tête, yaw de montage)

à partir de /joint_states. Le paramètre tilt_level_rad est la valeur du joint
tilt pour laquelle le plan laser est HORIZONTAL : à cette valeur, le laser a
un pitch nul ; tout écart apparaît dans la TF (AMCL, costmaps, contrôle de
niveau de la mission voient la vraie inclinaison).

Si /joint_states s'arrête (> stale_timeout_s), la TF n'est plus publiée :
Nav2 s'arrête faute de transformée => comportement sûr.

Mesures à faire sur Néo (mètre ruban) :
  head_center_x / head_center_z : centre de la tête (axes pan/tilt) dans base_link
  laser_above_center            : du centre de la tête au plan de mesure du lidar
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class HeadLaserTF(Node):

    def __init__(self):
        super().__init__('head_laser_tf')
        d = self.declare_parameter
        d('base_frame', 'base_link')
        d('laser_frame', 'laser')
        d('pan_joint', 'head_pan_joint')
        d('tilt_joint', 'head_tilt_joint')
        d('joint_states_topic', '/joint_states')
        d('head_center_x', 0.10)       # m, en avant de base_link
        d('head_center_y', 0.0)
        d('head_center_z', 0.34)       # m, au-dessus de base_link (base_link = sol + 0.02)
        d('laser_above_center', 0.13)  # m : rayon tête (0.10) + hauteur plan de mesure A2 (~0.03)
        d('laser_x_offset', 0.0)       # m, si le lidar n'est pas centré sur la tête
        d('laser_yaw', math.pi)        # montage du lidar (repris de qbo_navigation)
        d('tilt_level_rad', 0.0)       # valeur du joint tilt pour laquelle le laser est horizontal
        d('pan_sign', 1.0)
        d('tilt_sign', 1.0)            # -1 si tilt positif = tête qui monte (pitch ROS + = nez vers le bas)
        d('rate_hz', 50.0)
        d('stale_timeout_s', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base = g('base_frame')
        self.laser = g('laser_frame')
        self.pan_j = g('pan_joint')
        self.tilt_j = g('tilt_joint')
        self.cx, self.cy, self.cz = float(g('head_center_x')), float(g('head_center_y')), float(g('head_center_z'))
        self.l_up = float(g('laser_above_center'))
        self.l_x = float(g('laser_x_offset'))
        self.l_yaw = float(g('laser_yaw'))
        self.tilt_level = float(g('tilt_level_rad'))
        self.pan_sign = float(g('pan_sign'))
        self.tilt_sign = float(g('tilt_sign'))
        self.stale = float(g('stale_timeout_s'))

        self.pan = None
        self.tilt = None
        self.last_js = 0.0
        self._warned = False

        self.br = TransformBroadcaster(self)
        self.create_subscription(JointState, g('joint_states_topic'), self._js_cb, 20)
        self.create_timer(1.0 / max(float(g('rate_hz')), 1.0), self._publish)
        self.get_logger().info(
            f'TF tête->laser : centre tête ({self.cx:.2f}, {self.cy:.2f}, {self.cz:.2f}) m, '
            f'laser +{self.l_up:.2f} m, tilt horizontal = {math.degrees(self.tilt_level):.1f}°')

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n == self.pan_j:
                self.pan = p
            elif n == self.tilt_j:
                self.tilt = p
        if self.pan is not None and self.tilt is not None:
            self.last_js = time.monotonic()

    def _tf(self, stamp, parent, child, xyz, rpy):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        q = quat_from_rpy(*rpy)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def _publish(self):
        if self.last_js == 0.0 or time.monotonic() - self.last_js > self.stale:
            if not self._warned:
                self.get_logger().warn('Pas de /joint_states tête : TF laser NON publiée (sécurité)')
                self._warned = True
            return
        if self._warned:
            self.get_logger().info('/joint_states tête reçu : TF laser publiée')
            self._warned = False
        stamp = self.get_clock().now().to_msg()
        pan = self.pan_sign * self.pan
        pitch = self.tilt_sign * (self.tilt - self.tilt_level)
        self.br.sendTransform([
            self._tf(stamp, self.base, 'head_pan_link', (self.cx, self.cy, self.cz), (0.0, 0.0, pan)),
            self._tf(stamp, 'head_pan_link', 'head_tilt_link', (0.0, 0.0, 0.0), (0.0, pitch, 0.0)),
            self._tf(stamp, 'head_tilt_link', self.laser, (self.l_x, 0.0, self.l_up), (0.0, 0.0, self.l_yaw)),
        ])


def main(args=None):
    rclpy.init(args=args)
    node = HeadLaserTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
