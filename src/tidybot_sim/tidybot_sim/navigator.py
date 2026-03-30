"""
navigator.py — Autonomous waypoint navigator with LiDAR obstacle avoidance.

PHASE 4: Full implementation.

Algorithm overview:
  1. Pre-defined waypoints trace a path through Room 1, through the doorway,
     across Room 2, back through the doorway, and to the collection box.
  2. A simple state machine controls the robot:
       ROTATING  — spin in place to align heading with next waypoint
       DRIVING   — drive forward toward waypoint
       AVOIDING  — obstacle detected ahead, rotate left for fixed duration
       DONE      — all waypoints visited, stop
  3. LiDAR readings are checked every loop: if anything is < 0.5m in the
     forward ±30° sector, the robot enters AVOIDING state.

ROS 2 concepts used:
  - Node.create_timer(): periodic callback (like a control loop)
  - Node.create_subscription(): callback-driven topic subscriber
  - Node.create_publisher(): topic publisher
  - use_sim_time: True → all rclpy time calls use the /clock topic from Gazebo

Odometry frame note:
  Ignition Gazebo's DiffDrive plugin initializes the odom frame from the
  robot's world spawn pose (0.5, 0.0). So odom-reported positions match
  world coordinates directly. Waypoints are in world coordinates.
"""

import enum
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32, Bool


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
PICKUP_RADIUS     = 0.85   # m — trigger arm controller when within this distance
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

        # ── Publishers ──────────────────────────────────────────────────
        # Twist: {linear.x = forward speed m/s, angular.z = turn rate rad/s}
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Status string for graders / monitoring
        self.status_pub = self.create_publisher(String, '/tidybot/status', 10)

        # Arm trigger: publish object index when near a pickup object (Phase 5)
        self.arm_trigger_pub = self.create_publisher(Int32, '/arm/trigger_pickup', 10)

        # ── Subscribers ─────────────────────────────────────────────────
        # Queue size 1: we only care about the LATEST message, not a backlog.
        # If the callback can't keep up, older messages are dropped.
        self.create_subscription(Odometry,   '/odom',           self._odom_cb,         1)
        self.create_subscription(LaserScan,  '/scan',           self._scan_cb,         1)
        self.create_subscription(Bool, '/arm/pickup_done',  self._pickup_done_cb,  10)
        self.create_subscription(Bool, '/arm/deposit_done', self._deposit_done_cb, 10)

        # ── State ────────────────────────────────────────────────────────
        self.odom: Odometry  = None
        self.scan: LaserScan = None
        self.state           = State.ROTATING
        self.wp_idx          = 0       # index into WAYPOINTS
        self.avoid_start     = None    # rclpy Time when avoidance began
        self.objects_nearby  = set()   # objects we've already triggered
        self.rooms_visited   = set()
        self._dist_traveled  = 0.0
        self._last_pos       = None

        # Arm coordination (Phase 5)
        self.pickup_start    = None    # rclpy Time when PICKUP state began
        self.arm_busy        = False   # True from trigger → pickup_done received
        PICKUP_TIMEOUT       = 20.0    # s — resume if arm doesn't respond
        self._pickup_timeout = PICKUP_TIMEOUT

        # ── Control timer: runs the state machine at 10 Hz ───────────────
        # create_timer(period_seconds, callback)
        # With use_sim_time=True, the period is in sim time.
        self.ctrl_timer = self.create_timer(0.10, self._control_loop)

        # ── Status timer: publishes a human-readable status string ────────
        self.status_timer = self.create_timer(1.0 / STATUS_RATE_HZ, self._publish_status)

        self.get_logger().info(
            f'Navigator started. {len(WAYPOINTS)} waypoints loaded. '
            f'Room2 threshold at x={ROOM2_X_THRESHOLD}m'
        )

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        """Store latest odometry. Also accumulate distance traveled."""
        self.odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_pos is not None:
            dx = x - self._last_pos[0]
            dy = y - self._last_pos[1]
            self._dist_traveled += math.hypot(dx, dy)

        self._last_pos = (x, y)

        # Track which rooms the robot has visited
        if x > ROOM2_X_THRESHOLD:
            self.rooms_visited.add('Room 2')
        else:
            self.rooms_visited.add('Room 1')

    def _scan_cb(self, msg: LaserScan):
        """Store latest LaserScan."""
        self.scan = msg

    def _pickup_done_cb(self, msg: Bool):
        """
        arm_controller publishes True here when the pickup sequence is finished
        and the arm is raised to carry height. The navigator resumes driving.
        """
        if msg.data and self.state == State.PICKUP:
            self.get_logger().info(
                'Arm pickup complete — resuming navigation. '
                '(arm_busy stays True until deposit)'
            )
            # Do NOT clear arm_busy here — arm is still CARRYING the object.
            # We clear arm_busy in _deposit_done_cb when arm returns to IDLE.
            self.pickup_start = None
            self.state        = State.ROTATING

    def _deposit_done_cb(self, msg: Bool):
        """
        arm_controller publishes True here when it has deposited the object
        and returned to IDLE. Clears the arm_busy flag so the navigator can
        trigger the next pickup on a future waypoint.
        """
        if msg.data:
            self.get_logger().info('Arm deposit complete — arm_busy cleared.')
            self.arm_busy = False

    # ── Main control loop (10 Hz) ─────────────────────────────────────────────

    def _control_loop(self):
        """State machine tick. Called every 100ms."""

        # Wait until we have odometry (required for navigation).
        # LiDAR scan is optional — if unavailable, we skip obstacle avoidance
        # but still navigate by waypoints. This handles cases where the
        # sensor system fails to initialize (e.g. software rendering).
        if self.odom is None:
            return

        if self.state == State.DONE:
            self._publish_cmd(0.0, 0.0)
            return

        if self.state == State.PICKUP:
            self._publish_cmd(0.0, 0.0)
            # Safety timeout: if arm_controller doesn't publish pickup_done
            # within _pickup_timeout seconds, resume navigation anyway.
            if self._sim_elapsed_since(self.pickup_start) > self._pickup_timeout:
                self.get_logger().warn(
                    f'Arm pickup timed out after {self._pickup_timeout:.0f}s '
                    f'— resuming navigation. arm_busy stays True.'
                )
                # Keep arm_busy=True so we don't re-trigger while arm may
                # still be mid-sequence. Cleared only by deposit_done.
                self.pickup_start = None
                self.state        = State.ROTATING
            return

        # ── Current pose from odometry ──────────────────────────────────
        x      = self.odom.pose.pose.position.x
        y      = self.odom.pose.pose.position.y
        yaw    = self._quat_to_yaw(self.odom.pose.pose.orientation)

        # ── Target waypoint ─────────────────────────────────────────────
        wx, wy = WAYPOINTS[self.wp_idx]
        dx     = wx - x
        dy     = wy - y
        dist   = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_err    = self._wrap_angle(target_heading - yaw)

        # ── Check waypoint reached ───────────────────────────────────────
        if dist < WAYPOINT_RADIUS:
            self.get_logger().info(
                f'Waypoint {self.wp_idx + 1}/{len(WAYPOINTS)} reached: '
                f'({wx:.1f}, {wy:.1f}) | dist traveled: {self._dist_traveled:.1f}m'
            )
            self.wp_idx += 1
            if self.wp_idx >= len(WAYPOINTS):
                self.state = State.DONE
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f'Navigation COMPLETE. '
                    f'Rooms visited: {", ".join(sorted(self.rooms_visited))}. '
                    f'Total distance: {self._dist_traveled:.1f}m'
                )
                return
            # Re-enter ROTATING to align with the next waypoint
            self.state = State.ROTATING
            # Check for nearby pickup objects (Phase 5 hook)
            self._check_pickup_trigger(x, y)
            return

        # ── Obstacle avoidance ───────────────────────────────────────────
        # Priority: safety check runs every tick regardless of current state.
        # If we're already AVOIDING, check if the timer has expired.
        if self.state == State.AVOIDING:
            elapsed = self._sim_elapsed_since(self.avoid_start)
            if elapsed >= AVOID_DURATION:
                self.get_logger().info('Obstacle clear — resuming navigation.')
                self.state = State.ROTATING
            else:
                # Keep rotating
                self._publish_cmd(0.0, AVOID_TURN_RATE)
                return
        elif self._obstacle_ahead():
            # Transition to AVOIDING from ROTATING or DRIVING
            self.avoid_start = self.get_clock().now()
            self.state = State.AVOIDING
            self.get_logger().warn(
                f'Obstacle detected! Avoiding for {AVOID_DURATION}s.'
            )
            self._publish_cmd(0.0, AVOID_TURN_RATE)
            return

        # ── ROTATING: align heading ──────────────────────────────────────
        if self.state == State.ROTATING:
            if abs(heading_err) < HEADING_TOL:
                self.state = State.DRIVING
            else:
                # Turn toward waypoint
                # Clamp to ±MAX_ANGULAR for safety
                omega = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_HEADING * heading_err))
                self._publish_cmd(0.0, omega)
                return

        # ── DRIVING: move toward waypoint ───────────────────────────────
        if self.state == State.DRIVING:
            # Scale forward speed down when heading error is large.
            # This prevents the robot from drifting wide on turns.
            # At 0° error: full speed. At 45°+ error: stop (rely on ROTATING).
            speed_scale = max(0.0, 1.0 - abs(heading_err) / (math.pi / 4.0))
            linear_v    = MAX_LINEAR * speed_scale

            # Simultaneously correct heading while driving (gentler than stopping to rotate)
            angular_v = max(-MAX_ANGULAR * 0.5,
                            min(MAX_ANGULAR * 0.5, KP_HEADING * heading_err))

            self._publish_cmd(linear_v, angular_v)

            # If we drifted too far off course, go back to ROTATING
            if abs(heading_err) > math.pi / 3.0:
                self.state = State.ROTATING

    # ── Helper methods ────────────────────────────────────────────────────────

    def _obstacle_ahead(self) -> bool:
        """
        Returns True if any valid LiDAR reading in the forward ±30° sector
        is closer than OBSTACLE_DIST.

        LaserScan layout (360 samples, -π to +π):
          ranges[0]   = angle_min = -π  (directly behind)
          ranges[180] = angle = 0       (directly forward)
          ranges[270] = angle = +π/2    (left)
          ranges[90]  = angle = -π/2    (right)

        inf  = no obstacle within max_range (valid, no obstacle)
        nan  = sensor error (skip)
        """
        if self.scan is None:
            return False

        angle = self.scan.angle_min
        inc   = self.scan.angle_increment

        for r in self.scan.ranges:
            # Check if this angle is in the forward sector
            if -FORWARD_HALF_ANG <= angle <= FORWARD_HALF_ANG:
                # math.isfinite filters out both inf (no obstacle) and nan (error)
                if math.isfinite(r) and r < OBSTACLE_DIST:
                    return True
            angle += inc

        return False

    def _check_pickup_trigger(self, x: float, y: float):
        """
        Check if the robot is near any pickup object and trigger the arm
        controller if so. Enters PICKUP state so the navigator waits while
        the arm performs the pickup sequence.

        Guards:
          - arm_busy: True from trigger until arm signals pickup_done.
            Prevents triggering a second pickup while the arm is still
            carrying the first object (arm_controller ignores triggers
            when not IDLE, which would leave the navigator stuck forever).
          - objects_nearby: set of already-triggered objects, prevents
            re-triggering the same object.
        """
        if self.arm_busy:
            return  # arm is carrying or running a sequence — skip

        for name, (ox, oy) in OBJECT_POSITIONS.items():
            if name in self.objects_nearby:
                continue  # already handled
            if math.hypot(x - ox, y - oy) < PICKUP_RADIUS:
                self.objects_nearby.add(name)
                idx = list(OBJECT_POSITIONS.keys()).index(name)
                trig_msg = Int32()
                trig_msg.data = idx
                self.arm_trigger_pub.publish(trig_msg)
                # Mark arm as busy and record when we entered PICKUP
                self.arm_busy     = True
                self.pickup_start = self.get_clock().now()
                self.state        = State.PICKUP
                self.get_logger().info(
                    f'Near {name} at ({ox:.1f}, {oy:.1f}) — '
                    f'arm trigger sent (index {idx}). Navigator paused.'
                )
                return  # only trigger one object per waypoint

    def _publish_cmd(self, linear: float, angular: float):
        """Publish a Twist command to /cmd_vel."""
        cmd = Twist()
        cmd.linear.x  = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def _publish_status(self):
        """Publish a human-readable status string for monitoring."""
        if self.odom is None:
            return
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        room = 'Room 2' if x > ROOM2_X_THRESHOLD else 'Room 1'
        wp_str = f'{self.wp_idx}/{len(WAYPOINTS)}'
        msg = String()
        msg.data = (
            f'State:{self.state.value} | WP:{wp_str} | '
            f'Pos:({x:.1f},{y:.1f}) | {room} | '
            f'Dist:{self._dist_traveled:.1f}m'
        )
        self.status_pub.publish(msg)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """
        Extract yaw (rotation around Z axis) from a quaternion.

        ROS uses the ZYX Euler convention. For a robot moving in the XY plane,
        only yaw matters. The formula is:
          yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))

        This is a standard quaternion-to-Euler formula for the Z component.
        """
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """
        Wrap an angle to [-π, π].

        Why needed: if target_heading = 3.0 rad and current_yaw = -3.0 rad,
        the naive difference is 6.0 rad — but the robot only needs to turn
        0.28 rad (the short way around). atan2(sin, cos) handles wrapping
        correctly by exploiting the periodicity of sin/cos.
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def _sim_elapsed_since(self, start_time) -> float:
        """Return elapsed simulation seconds since start_time."""
        if start_time is None:
            return float('inf')
        delta = self.get_clock().now() - start_time
        return delta.nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)   # safety stop
        node.get_logger().info(
            f'Navigator shutting down. '
            f'Rooms visited: {", ".join(sorted(node.rooms_visited)) or "none"}. '
            f'Distance: {node._dist_traveled:.1f}m'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
