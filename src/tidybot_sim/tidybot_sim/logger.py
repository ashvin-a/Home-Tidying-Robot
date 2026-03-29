"""
logger.py — Odometry + sensor data CSV logger.

PHASE 1: Stub node — just subscribes to /odom and prints.
PHASE 4: Will write full CSV log (sim_time, x, y, yaw, speed, lidar_min).

Teaches: basic ROS 2 node structure, subscriptions, parameters.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LoggerNode(Node):
    """
    A ROS 2 node is a process that communicates via topics, services, actions.
    Node.__init__ registers it with the ROS 2 graph under the given name.
    """

    def __init__(self):
        super().__init__('logger')

        # Subscriber: (message_type, topic_name, callback, queue_size)
        # Queue size 10: if we can't process messages fast enough, keep 10 buffered.
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )

        self._count = 0
        self.get_logger().info('Logger node started — subscribing to /odom')

    def _odom_callback(self, msg: Odometry):
        """Called every time a new /odom message arrives (~20Hz)."""
        self._count += 1
        # Print a summary every 40 messages (~every 2 seconds)
        if self._count % 40 == 0:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            self.get_logger().info(
                f'Odom #{self._count}: pos=({x:.2f}, {y:.2f}), '
                f'speed={vx:.3f} m/s'
            )


def main(args=None):
    """Entry point — called by 'ros2 run tidybot_sim logger'."""
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        # spin() blocks and processes callbacks until Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Logger shutting down. Received {node._count} odom messages.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
