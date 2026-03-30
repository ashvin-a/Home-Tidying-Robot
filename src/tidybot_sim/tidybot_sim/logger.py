"""
logger.py — Odometry + sensor data CSV logger.

PHASE 4: Full CSV logger.

Subscribes to:
  /odom             (nav_msgs/Odometry)   — position, orientation, velocity
  /scan             (sensor_msgs/LaserScan) — LiDAR min range + angle
  /tidybot/status   (std_msgs/String)     — navigator state machine status

Writes a CSV file (tidybot_log.csv) at 1 Hz with columns:
  sim_time_s, x, y, yaw_rad, linear_speed, angular_speed,
  lidar_min_m, lidar_min_angle_deg, room, status

Also prints a human-readable summary to the terminal every 5 seconds.
"""

import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# ── CONFIGURATION ────────────────────────────────────────────────────────────
CSV_FILENAME       = 'tidybot_log.csv'
LOG_RATE_HZ        = 1.0      # CSV write frequency
TERMINAL_RATE_HZ   = 0.2      # Terminal summary frequency (every 5s)
ROOM2_X_THRESHOLD  = 5.0      # x > this = Room 2 (matches navigator)


class LoggerNode(Node):
    """
    A ROS 2 node that logs robot telemetry to a CSV file and terminal.

    Uses a timer-based approach (not callback-driven) to control the logging
    rate. Topic callbacks just cache the latest message; the timer reads
    the cache and writes one CSV row per tick.
    """

    def __init__(self):
        super().__init__('logger')

        # ── Cached latest messages ───────────────────────────────────────
        self._odom: Odometry  = None
        self._scan: LaserScan = None
        self._status_str: str = ''

        # ── Subscribers (queue size 1: only keep latest) ─────────────────
        self.create_subscription(Odometry,  '/odom',           self._odom_cb,   1)
        self.create_subscription(LaserScan, '/scan',           self._scan_cb,   1)
        self.create_subscription(String,    '/tidybot/status', self._status_cb, 1)

        # ── CSV file setup ───────────────────────────────────────────────
        self._csv_path = CSV_FILENAME
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'sim_time_s', 'x', 'y', 'yaw_rad',
            'linear_speed', 'angular_speed',
            'lidar_min_m', 'lidar_min_angle_deg',
            'room', 'status',
        ])
        self._row_count = 0

        # ── Timers ───────────────────────────────────────────────────────
        self._log_timer = self.create_timer(1.0 / LOG_RATE_HZ, self._log_tick)
        self._terminal_timer = self.create_timer(
            1.0 / TERMINAL_RATE_HZ, self._terminal_summary
        )

        self.get_logger().info(
            f'Logger started — writing CSV to {self._csv_path}'
        )

    # ── Subscriber callbacks (just cache) ─────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _status_cb(self, msg: String):
        self._status_str = msg.data

    # ── CSV logging timer (1 Hz) ──────────────────────────────────────────

    def _log_tick(self):
        """Write one row to the CSV if odometry is available."""
        if self._odom is None:
            return

        # Extract pose
        x   = self._odom.pose.pose.position.x
        y   = self._odom.pose.pose.position.y
        yaw = self._quat_to_yaw(self._odom.pose.pose.orientation)

        # Extract velocities
        linear_speed  = self._odom.twist.twist.linear.x
        angular_speed = self._odom.twist.twist.angular.z

        # Extract LiDAR minimum
        lidar_min, lidar_angle_deg = self._get_lidar_min()

        # Determine room
        room = 'Room 2' if x > ROOM2_X_THRESHOLD else 'Room 1'

        # Simulation time
        now = self.get_clock().now()
        sim_time_s = now.nanoseconds / 1e9

        # Write row
        self._csv_writer.writerow([
            f'{sim_time_s:.3f}',
            f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}',
            f'{linear_speed:.4f}', f'{angular_speed:.4f}',
            f'{lidar_min:.3f}', f'{lidar_angle_deg:.1f}',
            room,
            self._status_str,
        ])
        self._csv_file.flush()
        self._row_count += 1

    # ── Terminal summary timer (every 5s) ─────────────────────────────────

    def _terminal_summary(self):
        """Print a human-readable summary to the terminal."""
        if self._odom is None:
            return

        x   = self._odom.pose.pose.position.x
        y   = self._odom.pose.pose.position.y
        vx  = self._odom.twist.twist.linear.x
        room = 'Room 2' if x > ROOM2_X_THRESHOLD else 'Room 1'
        lidar_min, _ = self._get_lidar_min()

        self.get_logger().info(
            f'[LOG #{self._row_count}] pos=({x:.2f}, {y:.2f}) | '
            f'speed={vx:.3f} m/s | lidar_min={lidar_min:.2f}m | '
            f'{room} | {self._status_str}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────

    def _get_lidar_min(self):
        """
        Return (min_range, angle_of_min_in_degrees) from the latest scan.

        Skips inf and nan readings. Returns (inf, 0.0) if no valid scan
        is available.
        """
        if self._scan is None:
            return float('inf'), 0.0

        min_range = float('inf')
        min_angle = 0.0
        angle = self._scan.angle_min

        for r in self._scan.ranges:
            if math.isfinite(r) and r < min_range:
                min_range = r
                min_angle = angle
            angle += self._scan.angle_increment

        return min_range, math.degrees(min_angle)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """
        Extract yaw from a quaternion (same formula as navigator.py).
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
        """
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _shutdown(self):
        """Close CSV and print final summary."""
        self.get_logger().info(
            f'Logger shutting down. '
            f'{self._row_count} rows written to {self._csv_path}'
        )
        self._csv_file.close()


def main(args=None):
    """Entry point — called by 'ros2 run tidybot_sim logger'."""
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
