#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_waypoints.py — Capture des waypoints de Néo sur la carte
================================================================
Auteur : Vincent FOUCAULT

Deux modes (paramètre ROS "mode") :

  click  (défaut) : dans RViz, outil "Publish Point", clique N points sur la
                    carte. L'orientation de chaque waypoint pointe vers le
                    suivant (le dernier vers le premier) => trajets naturels.
                    (Ne PAS utiliser "2D Goal Pose" : Nav2 partirait.)

  robot           : place Néo physiquement (téléop) sur chaque waypoint et
                    appuie sur Entrée : la pose AMCL (avec orientation réelle)
                    est enregistrée. Nécessite la localisation active.

Exemples :
  ros2 run qbo_nav_waypoints record_waypoints
  ros2 run qbo_nav_waypoints record_waypoints --ros-args -p mode:=robot -p count:=5 \
      -p output:=~/qbo_ws/src/qbo_nav_waypoints/config/waypoints.yaml
"""

import math
import os
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('record_waypoints')
        self.declare_parameter('count', 5)
        self.declare_parameter('mode', 'click')
        self.declare_parameter('output', '~/qbo_waypoints.yaml')
        self.count = int(self.get_parameter('count').value)
        self.mode = self.get_parameter('mode').value
        self.output = os.path.expanduser(self.get_parameter('output').value)
        self.points = []
        self.amcl = None
        self.done = threading.Event()

        if self.mode == 'robot':
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, qos)
            self.get_logger().info(f'Mode ROBOT : place Néo sur chaque waypoint puis Entrée ({self.count} points)')
        else:
            self.create_subscription(PointStamped, '/clicked_point', self._click_cb, 10)
            self.get_logger().info(f'Mode CLICK : RViz > "Publish Point", clique {self.count} points dans l\'ordre')

    def _amcl_cb(self, msg):
        self.amcl = msg.pose.pose

    def _click_cb(self, msg):
        if self.done.is_set():
            return
        self.points.append({'x': msg.point.x, 'y': msg.point.y, 'yaw': None})
        self.get_logger().info(f'WP{len(self.points)} : x={msg.point.x:.2f}  y={msg.point.y:.2f}')
        if len(self.points) >= self.count:
            self.write()

    def capture_robot_pose(self):
        if self.amcl is None:
            self.get_logger().warn('Pas encore de /amcl_pose (localise Néo dans RViz)')
            return False
        p = self.amcl.position
        q = self.amcl.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.points.append({'x': p.x, 'y': p.y, 'yaw': yaw})
        self.get_logger().info(
            f'WP{len(self.points)} : x={p.x:.2f}  y={p.y:.2f}  yaw={math.degrees(yaw):.0f}°')
        return True

    def write(self):
        n = len(self.points)
        for i, pt in enumerate(self.points):
            if pt['yaw'] is None:   # orientation vers le waypoint suivant
                nxt = self.points[(i + 1) % n]
                pt['yaw'] = math.atan2(nxt['y'] - pt['y'], nxt['x'] - pt['x'])
        first = self.points[0]
        lines = [
            '# waypoints.yaml — généré par record_waypoints (Vincent FOUCAULT)',
            f'# mode : {self.mode}',
            'frame_id: map',
            '',
            '# Pose de départ (désactivée : "2D Pose Estimate" dans RViz au lancement).',
            '# Passe enabled à true si Néo démarre toujours au même endroit.',
            'initial_pose:',
            '  enabled: false',
            f'  x: {first["x"]:.3f}',
            f'  y: {first["y"]:.3f}',
            f'  yaw_deg: {math.degrees(first["yaw"]):.1f}',
            '',
            'waypoints:',
        ]
        for i, pt in enumerate(self.points):
            lines.append(f'  - {{name: "WP{i + 1}", x: {pt["x"]:.3f}, y: {pt["y"]:.3f}, '
                         f'yaw_deg: {math.degrees(pt["yaw"]):.1f}}}')
        os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
        with open(self.output, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
        self.get_logger().info(f'{n} waypoints écrits dans {self.output}')
        self.done.set()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        if node.mode == 'robot':
            while len(node.points) < node.count and rclpy.ok():
                input(f'>>> Néo en place sur WP{len(node.points) + 1} ? Entrée pour enregistrer... ')
                node.capture_robot_pose()
            if len(node.points) >= node.count:
                node.write()
        else:
            while not node.done.wait(0.2) and rclpy.ok():
                pass
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()          # arrêt propre du thread (plus de « terminate called »)
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
