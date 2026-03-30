"""
navigator.py — Autonomous waypoint navigator with LiDAR obstacle avoidance.

PHASE 1: Stub (not launched yet).
PHASE 4: Full implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32


# ── TUNING PARAMETERS ────────────────────────────────────────────────────────

# Waypoints in world frame (= odom frame at spawn)
# Designed to:
#   - Cover both rooms completely
#   - Stay in open corridors (clear of known furniture)
#   - Pass near all 6 pickup objects
#   - Return to collection box at the end
WAYPOINTS = [
    (0.5,  0.0),   # 0  start position (robot spawn)
    (2.5, -0.5),   # 1  Room 1 south-center (below obj_red)
    (3.5,  0.0),   # 2  Room 1 east (near obj_blue approach)
    (4.3,  0.0),   # 3  approach doorway
    (5.5,  0.0),   # 4  through doorway → Room 2 entry
    (6.5,  0.5),   # 5  Room 2 center-north
    (7.0,  1.5),   # 6  Room 2 north (passes near obj_yellow, obj_orange)
    (8.0,  0.0),   # 7  Room 2 east center
    (7.5, -1.5),   # 8  Room 2 south-east (passes near obj_purple)
    (6.5, -0.5),   # 9  Room 2 south return
    (5.5,  0.0),   # 10 back through doorway
    (3.0,  0.5),   # 11 Room 1 return path
    (1.5,  1.5),   # 12 near collection box
]

# Object world positions (used to trigger arm in Phase 5)
OBJECT_POSITIONS = {
    'obj_red':    (3.0,  1.0),
    'obj_green':  (1.5, -1.0),
    'obj_blue':   (4.0, -0.5),
    'obj_yellow': (6.0,  2.0),
    'obj_purple': (8.0, -1.5),
    'obj_orange': (7.0,  1.5),
}

WAYPOINT_RADIUS   = 0.35   # m — consider waypoint reached within this distance
HEADING_TOL       = 0.15   # rad (≈8.5°) — switch from ROTATING to DRIVING
MAX_LINEAR        = 0.28   # m/s — top forward speed
MAX_ANGULAR       = 0.80   # rad/s — top rotation speed
KP_HEADING        = 1.2    # proportional gain for heading correction
OBSTACLE_DIST     = 0.55   # m — if LiDAR reads < this ahead, avoid
FORWARD_HALF_ANG  = 0.52   # rad (≈30°) — the forward scan sector half-width
AVOID_DURATION    = 2.0    # s — how long to turn away from an obstacle
AVOID_TURN_RATE   = 0.50   # rad/s — rotation speed during avoidance
PICKUP_RADIUS     = 0.60   # m — trigger arm controller when within this distance
ROOM2_X_THRESHOLD = 5.0    # m — x > this = robot is in Room 2
STATUS_RATE_HZ    = 2.0    # Hz — how often to publish /tidybot/status


class State(enum.Enum):
    ROTATING = 'ROTATING'
    DRIVING  = 'DRIVING'
    AVOIDING = 'AVOIDING'
    PICKUP   = 'PICKUP'   # paused, arm controller is working (Phase 5)
    DONE     = 'DONE'


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
