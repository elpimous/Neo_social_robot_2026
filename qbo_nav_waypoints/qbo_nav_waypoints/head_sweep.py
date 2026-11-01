#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
head_sweep.py — balayage de la tête (pan) pour vérifier le redressement du scan
================================================================================
Auteur : Vincent FOUCAULT

Robot IMMOBILE, navigation lancée (lidar + TF), RViz ouvert sur le scan rouge.
La tête balaie de gauche à droite à vitesse constante. Si le redressement est
correct, les murs du scan restent nets et collés à la carte pendant le balayage.

  ros2 run qbo_nav_waypoints head_sweep                       # ±40° à 45°/s, 30 s
  ros2 run qbo_nav_waypoints head_sweep --ros-args -p amplitude_deg:=50.0 -p speed_deg_s:=60.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main(args=None):
    rclpy.init(args=args)
    n = Node('head_sweep')
    amp = math.radians(n.declare_parameter('amplitude_deg', 40.0).value)
    spd = math.radians(n.declare_parameter('speed_deg_s', 45.0).value)
    dur = float(n.declare_parameter('duration_s', 30.0).value)
    tilt = float(n.declare_parameter('tilt_rad', 0.0).value)
    pub = n.create_publisher(JointState, '/cmd_joints', 10)
    period = 4.0 * amp / spd                      # aller-retour complet
    n.get_logger().info(f'Balayage ±{math.degrees(amp):.0f}° à {math.degrees(spd):.0f}°/s pendant {dur:.0f} s')

    def send(p):
        m = JointState()
        m.header.stamp = n.get_clock().now().to_msg()
        m.name = ['head_pan_joint', 'head_tilt_joint']
        m.position = [p, tilt]
        m.velocity = [spd * 1.3, spd * 1.3]
        pub.publish(m)

    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < dur:
            ph = ((time.monotonic() - t0) % period) / period          # onde triangle = vitesse constante
            p = amp * (4 * ph - 1 if ph < 0.5 else 3 - 4 * ph)
            send(p)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            send(0.0)
            time.sleep(0.05)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
