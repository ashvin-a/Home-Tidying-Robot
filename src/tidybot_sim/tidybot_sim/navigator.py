"""
navigator.py — Autonomous waypoint navigator with LiDAR obstacle avoidance.

PHASE 1: Stub (not launched yet).
PHASE 4: Full implementation.
"""

import rclpy
from rclpy.node import Node


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')
        self.get_logger().info('Navigator stub — full implementation in Phase 4.')


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
