#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_range_filter.py — supprime les points lidar trop proches (support, câbles)
=============================================================================
Auteur : Vincent FOUCAULT

Le RPLidar, fixé au sommet de la tête, voit par moments son propre support et
ses câbles à moins de 25 cm. Les costmaps les ignorent (obstacle_min_range),
mais le collision_monitor de Nav2 Humble n'a pas de filtre de distance : ces
points tombaient dans sa zone « ralentir » et le robot alternait
ralentissement / accélération (effet accordéon).

/scan  ->  [ranges < min_range  =>  +inf]  ->  /scan_filtered
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__('scan_range_filter')
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter('input', '/scan')
        self.declare_parameter('output', '/scan_filtered')
        self.min_range = float(self.get_parameter('min_range').value)
        self.pub = self.create_publisher(LaserScan, self.get_parameter('output').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input').value, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        mr = self.min_range
        msg.ranges = [math.inf if r < mr else r for r in msg.ranges]
        if msg.range_min < mr:
            msg.range_min = mr
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
