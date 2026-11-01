#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_range_filter.py — nettoie et REDRESSE le scan du lidar monté sur la tête
=============================================================================
Auteur : Vincent FOUCAULT

1. Points trop proches (< min_range : support, câbles du lidar) => +inf.
   Le collision_monitor de Nav2 Humble n'a pas de filtre de distance : ces
   points provoquaient un effet accordéon.

2. Redressement (« deskew ») quand la tête tourne : un tour de lidar dure
   0.1 s ; si la tête tourne à 45°/s pendant ce temps, les derniers points du
   tour sont décalés de 4.5° par rapport aux premiers (18 cm à 2 m !). AMCL et
   les costmaps transforment tout le tour avec UNE seule position de tête (celle
   de l'horodatage du scan) => murs dédoublés, obstacles fantômes près du robot
   (« Starting point in lethal space »).
   Ici chaque point est remis à l'angle qu'il aurait eu si la tête n'avait pas
   bougé depuis le début du tour, grâce à l'historique de /joint_states.

   Ordre temporel des points (paramètre time_order) :
     'reversed' : driver Slamtec rplidar_ros (ranges[n-1-i] = i-ème mesure)
     'forward'  : ranges[i] = i-ème mesure
     'off'      : pas de redressement

/scan  ->  /scan_filtered   (utilisé par AMCL, les costmaps et le collision_monitor)
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__('scan_range_filter')
        d = self.declare_parameter
        d('min_range', 0.25)
        d('input', '/scan')
        d('output', '/scan_filtered')
        d('time_order', 'reversed')
        d('pan_joint', 'head_pan_joint')
        d('pan_sign', 1.0)
        d('joint_states_topic', '/joint_states')
        d('min_shift_deg', 0.2)            # en dessous : scan publié tel quel
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.min_range = float(g('min_range'))
        self.order = str(g('time_order')).lower()
        self.pan_joint = g('pan_joint')
        self.pan_sign = float(g('pan_sign'))
        self.min_shift = math.radians(float(g('min_shift_deg')))
        self.hist = deque(maxlen=120)      # (t, pan) ~ 4 s à 30 Hz
        self.n_deskew = 0
        self.pub = self.create_publisher(LaserScan, g('output'), qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('input'), self._cb, qos_profile_sensor_data)
        if self.order != 'off':
            self.create_subscription(JointState, g('joint_states_topic'), self._js_cb, 50)
        self.create_timer(60.0, self._report)
        self.get_logger().info(f'Filtre scan : min {self.min_range:.2f} m, redressement tête : {self.order}')

    def _js_cb(self, m):
        for n, p in zip(m.name, m.position):
            if n == self.pan_joint:
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if t <= 0.0:                                   # message non horodaté
                    t = self.get_clock().now().nanoseconds * 1e-9
                if not self.hist or t > self.hist[-1][0]:
                    self.hist.append((t, self.pan_sign * p))
                return

    def _report(self):
        if self.n_deskew:
            self.get_logger().info(f'{self.n_deskew} scans redressés (tête en mouvement) cette minute')
        self.n_deskew = 0

    def _deskew(self, msg, r):
        n = r.size
        if self.order == 'off' or len(self.hist) < 2 or n < 10 or msg.angle_increment == 0.0:
            return r
        dt = msg.time_increment if msg.time_increment > 0.0 else (msg.scan_time / (n - 1) if msg.scan_time > 0 else 0.0)
        if dt <= 0.0:
            return r
        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        h = np.array(self.hist)
        if t0 < h[0, 0] or t0 > h[-1, 0] + 0.5:                 # historique hors fenêtre
            return r
        k = np.arange(n)
        if self.order == 'reversed':
            k = (n - 1) - k                                     # rang temporel de chaque index
        pan = np.interp(t0 + k * dt, h[:, 0], h[:, 1])
        shift = pan - np.interp(t0, h[:, 0], h[:, 1])           # rotation de la tête depuis le début du tour
        if np.max(np.abs(shift)) < self.min_shift:
            return r
        full_turn = abs(msg.angle_increment * n) > 2.0 * math.pi - 2.0 * abs(msg.angle_increment)
        idx = np.rint(np.arange(n) + shift / msg.angle_increment).astype(int)
        if full_turn:
            idx %= n
        ok = (idx >= 0) & (idx < n) & np.isfinite(r)
        out = np.full(n, np.inf, dtype=np.float32)
        np.minimum.at(out, idx[ok], r[ok])
        self.n_deskew += 1
        return out

    def _cb(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        r[r < self.min_range] = np.inf
        r2 = self._deskew(msg, r)
        if r2 is not r:
            msg.intensities = []                                # plus alignées après redressement
        msg.ranges = r2.tolist()
        if msg.range_min < self.min_range:
            msg.range_min = self.min_range
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
