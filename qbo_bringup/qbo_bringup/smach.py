#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

class SmachNode(Node):
    def __init__(self):
        super().__init__('smatch')
        self.get_logger().info('smatch: tous les services démarrés. Attente...')

        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            self.get_logger().info('smatch: arrêt demandé, fermeture de ROS 2...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmachNode()
    node.destroy_node()

if __name__ == '__main__':
    main()
