"""
arm_controller.py — Arm joint control and pick-and-place pipeline.

PHASE 1: Stub (not launched yet).
PHASE 5: Full implementation.
"""

import rclpy
from rclpy.node import Node


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.get_logger().info('ArmController stub — full implementation in Phase 5.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
