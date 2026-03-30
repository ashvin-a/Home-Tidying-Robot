"""
arm_controller.py — Pick-and-place arm controller for the home-tidying robot.

PHASE 5: Full pick-and-place implementation.

Algorithm overview:
  1. Waits for a trigger on /arm/trigger_pickup (Int32 object index).
  2. Executes a time-based pickup sequence:
       OPEN_GRIPPER  → open fingers wide before lowering
       LOWERING      → rotate shoulder down to floor level (θ ≈ π/2)
       GRIPPING      → close fingers to grasp object
       RAISING       → lift arm back to horizontal carry position
  3. Publishes /arm/pickup_done (Bool=True) once arm is raised.
     This signals the navigator to resume driving to the collection box.
  4. While CARRYING, monitors /odom. When near the collection box:
       DEPOSIT_LOWER → lower arm into the box
       DEPOSIT_OPEN  → open fingers to release
       DEPOSIT_RAISE → raise arm back to rest
  5. Returns to IDLE ready for the next object.

Joint reference:
  right_shoulder_joint  — revolute, Y-axis
    θ=0.00 : arm horizontal forward (rest / carry position)
    θ=1.47 : arm pointing near-vertical down (≈85°) for floor pickup
  right_elbow_joint     — revolute, Y-axis
    θ=0.00 : arm fully extended
    θ=0.30 : slight bend (clears torso during carry)
  right_wrist_joint     — revolute, Z-axis
    θ=0.00 : neutral orientation
  right_gripper_*_joint — prismatic  [lower=0, upper=0.04 m]
    0.00   : fully closed (grip)
    0.04   : fully open (release)
    Both fingers share the /arm/gripper_cmd topic.

ROS 2 topics:
  Subscribed:
    /arm/trigger_pickup  (std_msgs/Int32)    — object index to pick up
    /odom                (nav_msgs/Odometry) — robot position
  Published:
    /arm/shoulder_cmd    (std_msgs/Float64)  — shoulder position (rad)
    /arm/elbow_cmd       (std_msgs/Float64)  — elbow position (rad)
    /arm/wrist_cmd       (std_msgs/Float64)  — wrist position (rad)
    /arm/gripper_cmd     (std_msgs/Float64)  — gripper opening (m)
    /arm/pickup_done     (std_msgs/Bool)     — True when pickup complete
    /arm/status          (std_msgs/String)   — human-readable state
"""

import enum
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, Bool, String
from nav_msgs.msg import Odometry


# ── COLLECTION BOX ───────────────────────────────────────────────────────────
# From home.sdf: box_floor centred at (1.5, 2.0)
COLLECTION_BOX_X      = 1.5
COLLECTION_BOX_Y      = 2.0
COLLECTION_BOX_RADIUS = 0.90   # m — deposit when within this radius

# ── JOINT TARGETS ────────────────────────────────────────────────────────────
SHOULDER_REST   =  0.00   # rad — horizontal forward (carry / rest)
SHOULDER_DOWN   =  1.13   # rad — 95° down toward floor

ELBOW_REST      =  0.00   # rad — fully extended
ELBOW_BENT      =  0.30   # rad — slight bend for carry (clears torso)

WRIST_NEUTRAL   =  0.00   # rad

GRIPPER_OPEN    =  0.04   # m — fingers spread
GRIPPER_CLOSED  =  0.00   # m — fingers together (gripping)

# ── STAGE DURATIONS (seconds of sim time) ────────────────────────────────────
T_OPEN_GRIPPER  = 0.8
T_LOWERING      = 2.5
T_GRIPPING      = 1.2
T_RAISING       = 2.5
T_DEPOSIT_LOWER = 2.0
T_DEPOSIT_OPEN  = 0.8
T_DEPOSIT_RAISE = 2.0

STATUS_RATE_HZ  = 2.0    # Hz


class ArmState(enum.Enum):
    IDLE           = 'IDLE'
    OPEN_GRIPPER   = 'OPEN_GRIPPER'
    LOWERING       = 'LOWERING'
    GRIPPING       = 'GRIPPING'
    RAISING        = 'RAISING'
    CARRYING       = 'CARRYING'       # arm up, navigator drives to box
    DEPOSIT_LOWER  = 'DEPOSIT_LOWER'
    DEPOSIT_OPEN   = 'DEPOSIT_OPEN'
    DEPOSIT_RAISE  = 'DEPOSIT_RAISE'


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller')

        # ── Publishers ───────────────────────────────────────────────────
        self.shoulder_pub    = self.create_publisher(Float64, '/arm/shoulder_cmd',  10)
        self.elbow_pub       = self.create_publisher(Float64, '/arm/elbow_cmd',     10)
        self.wrist_pub       = self.create_publisher(Float64, '/arm/wrist_cmd',     10)
        self.gripper_pub     = self.create_publisher(Float64, '/arm/gripper_cmd',   10)
        self.pickup_done_pub  = self.create_publisher(Bool,   '/arm/pickup_done',   10)
        self.deposit_done_pub = self.create_publisher(Bool,   '/arm/deposit_done',  10)
        self.status_pub       = self.create_publisher(String, '/arm/status',        10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Int32,    '/arm/trigger_pickup', self._trigger_cb, 10)
        self.create_subscription(Odometry, '/odom',               self._odom_cb,    1)

        # ── State ────────────────────────────────────────────────────────
        self.arm_state      = ArmState.IDLE
        self.stage_start    = None     # rclpy Time when current stage began
        self.current_object = None     # index of object being handled
        self.objects_picked = []       # indices of objects successfully deposited
        self.robot_x        = None
        self.robot_y        = None

        # ── Timers ───────────────────────────────────────────────────────
        self.ctrl_timer   = self.create_timer(0.10, self._control_loop)
        self.status_timer = self.create_timer(1.0 / STATUS_RATE_HZ, self._publish_status)

        # Move to rest on startup
        self._publish_joints(SHOULDER_REST, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_CLOSED)
        self.get_logger().info('ArmController started — arm at rest.')

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _trigger_cb(self, msg: Int32):
        """Accept pickup trigger only when IDLE (ignore duplicates)."""
        if self.arm_state != ArmState.IDLE:
            return
        self.current_object = msg.data
        self.get_logger().info(
            f'Pickup trigger: object index {msg.data}. Starting sequence.'
        )
        self._transition(ArmState.OPEN_GRIPPER)

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # ── Control loop (10 Hz) ─────────────────────────────────────────────────

    def _control_loop(self):
        if self.arm_state == ArmState.IDLE:
            return

        elapsed = self._elapsed()

        if self.arm_state == ArmState.OPEN_GRIPPER:
            # Spread fingers wide before lowering so we don't knock the object
            self._publish_joints(SHOULDER_REST, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_OPEN)
            if elapsed >= T_OPEN_GRIPPER:
                self._transition(ArmState.LOWERING)

        elif self.arm_state == ArmState.LOWERING:
            # Swing arm down toward the floor
            self._publish_joints(SHOULDER_DOWN, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_OPEN)
            if elapsed >= T_LOWERING:
                self._transition(ArmState.GRIPPING)

        elif self.arm_state == ArmState.GRIPPING:
            # Close fingers to grasp object
            self._publish_joints(SHOULDER_DOWN, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_CLOSED)
            if elapsed >= T_GRIPPING:
                self._transition(ArmState.RAISING)

        elif self.arm_state == ArmState.RAISING:
            # Lift arm to carry height — elbow slightly bent keeps object clear of torso
            self._publish_joints(SHOULDER_REST, ELBOW_BENT, WRIST_NEUTRAL, GRIPPER_CLOSED)
            if elapsed >= T_RAISING:
                self.get_logger().info(
                    f'Object {self.current_object} picked up. '
                    f'Signalling navigator to resume.'
                )
                # Tell the navigator it can resume driving
                done_msg = Bool()
                done_msg.data = True
                self.pickup_done_pub.publish(done_msg)
                self._transition(ArmState.CARRYING)

        elif self.arm_state == ArmState.CARRYING:
            # Hold carry position; check proximity to collection box
            self._publish_joints(SHOULDER_REST, ELBOW_BENT, WRIST_NEUTRAL, GRIPPER_CLOSED)
            if self._near_collection_box():
                self.get_logger().info('Near collection box — depositing.')
                self._transition(ArmState.DEPOSIT_LOWER)

        elif self.arm_state == ArmState.DEPOSIT_LOWER:
            # Lower arm into the box
            self._publish_joints(SHOULDER_DOWN, ELBOW_BENT, WRIST_NEUTRAL, GRIPPER_CLOSED)
            if elapsed >= T_DEPOSIT_LOWER:
                self._transition(ArmState.DEPOSIT_OPEN)

        elif self.arm_state == ArmState.DEPOSIT_OPEN:
            # Release object
            self._publish_joints(SHOULDER_DOWN, ELBOW_BENT, WRIST_NEUTRAL, GRIPPER_OPEN)
            if elapsed >= T_DEPOSIT_OPEN:
                self.objects_picked.append(self.current_object)
                self.get_logger().info(
                    f'Object {self.current_object} deposited. '
                    f'Total collected: {len(self.objects_picked)}'
                )
                self._transition(ArmState.DEPOSIT_RAISE)

        elif self.arm_state == ArmState.DEPOSIT_RAISE:
            # Return to rest
            self._publish_joints(SHOULDER_REST, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_OPEN)
            if elapsed >= T_DEPOSIT_RAISE:
                self.current_object = None
                self.get_logger().info('Deposit complete — ready for next pickup.')
                # Signal navigator that arm is fully idle; arm_busy flag clears
                done_msg = Bool()
                done_msg.data = True
                self.deposit_done_pub.publish(done_msg)
                self._transition(ArmState.IDLE)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _transition(self, new_state: ArmState):
        self.arm_state   = new_state
        self.stage_start = self.get_clock().now()
        self.get_logger().info(f'Arm → {new_state.value}')

    def _elapsed(self) -> float:
        if self.stage_start is None:
            return float('inf')
        return (self.get_clock().now() - self.stage_start).nanoseconds / 1e9

    def _near_collection_box(self) -> bool:
        if self.robot_x is None:
            return False
        return math.hypot(
            self.robot_x - COLLECTION_BOX_X,
            self.robot_y - COLLECTION_BOX_Y,
        ) < COLLECTION_BOX_RADIUS

    def _publish_joints(self, shoulder, elbow, wrist, gripper):
        def f(v):
            m = Float64(); m.data = float(v); return m
        self.shoulder_pub.publish(f(shoulder))
        self.elbow_pub.publish(f(elbow))
        self.wrist_pub.publish(f(wrist))
        self.gripper_pub.publish(f(gripper))

    def _publish_status(self):
        holding = f'idx_{self.current_object}' if self.current_object is not None else 'none'
        msg = String()
        msg.data = (
            f'ArmState:{self.arm_state.value} | '
            f'Holding:{holding} | '
            f'Collected:{len(self.objects_picked)}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_joints(SHOULDER_REST, ELBOW_REST, WRIST_NEUTRAL, GRIPPER_OPEN)
        node.get_logger().info(
            f'ArmController shutting down. '
            f'Objects collected: {len(node.objects_picked)}'
        )
        node.destroy_node()
        rclpy.shutdown()
