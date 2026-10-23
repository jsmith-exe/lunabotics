import json
import time
from math import atan2, hypot, pi

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from qpl_autonomy.nav.costmap_check import footprint_blocked
from qpl_autonomy.nav.nav_excavation import NavExcavationSequence
from qpl_autonomy.nav.nav_deposition import NavDepositionSequence
from qpl_autonomy.navigation import Navigation

# Seconds after startup before a missing /clock counts as an error
CLOCK_GRACE_S = 5.0
# Local-EKF movement below this between samples isn't counted as driving, so pose jitter
# while standing still doesn't add up
DRIVE_STEP_M = 0.05
# How often the current candidate is re-checked against the costmap while driving to it
TARGET_CHECK_PERIOD_S = 1.0


def parse_poses(flat):
    # [x, y, yaw, x, y, yaw, ...] -> [(x, y, yaw), ...]
    if not flat or len(flat) % 3:
        raise ValueError(f"candidate list must be a non-empty flat list of x, y, yaw triples, got {list(flat)}")
    return [tuple(float(v) for v in flat[i:i + 3]) for i in range(0, len(flat), 3)]


class NavAutonomyNode(Node):
    """Nav2 based autonomy for US competition.

    Separate from the blind autonomy node (autonomy_node and its variants), which is left
    as it is as a backup. Started with qpl_nav, but sits IDLE until a command arrives.

    NAV_EXCAVATE / NAV_DEPOSIT drive to a map-frame waypoint with Nav2, then dig or dump
    standing still. AUTO (full autonomy) repeats NAV_EXCAVATE then NAV_DEPOSIT until STOP.
    """

    def __init__(self):
        super().__init__("nav_autonomy")

        # Parameters
        self.declare_parameter("autonomy.update_rate_hz", 5.0)
        update_rate_hz = self.get_parameter("autonomy.update_rate_hz").value

        self.declare_parameter("excavation.drum_spin_speed", 0.8)
        self.declare_parameter("excavation.drum_spin_direction", 1.0)
        self.declare_parameter("excavation.lowered_position_m", 0.032)
        self.declare_parameter("excavation.dig_duration_s", 30.0)
        self.declare_parameter("excavation.raised_position_m", 0.218)
        self.declare_parameter("excavation.position_tolerance_m", 0.005)
        excavation_config = {
            "drum_spin_speed": self.get_parameter("excavation.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("excavation.drum_spin_direction").value,
            "lowered_position_m": self.get_parameter("excavation.lowered_position_m").value,
            "dig_duration_s": self.get_parameter("excavation.dig_duration_s").value,
            "raised_position_m": self.get_parameter("excavation.raised_position_m").value,
            "position_tolerance_m": self.get_parameter("excavation.position_tolerance_m").value,
        }

        self.declare_parameter("deposition.raised_position_m", 0.218)
        self.declare_parameter("deposition.dump_duration_s", 30.0)
        self.declare_parameter("deposition.drum_spin_speed", 0.4)
        self.declare_parameter("deposition.drum_spin_direction", 1.0)
        self.declare_parameter("deposition.position_tolerance_m", 0.005)
        deposition_config = {
            "raised_position_m": self.get_parameter("deposition.raised_position_m").value,
            "dump_duration_s": self.get_parameter("deposition.dump_duration_s").value,
            "drum_spin_speed": self.get_parameter("deposition.drum_spin_speed").value,
            "drum_spin_direction": self.get_parameter("deposition.drum_spin_direction").value,
            "position_tolerance_m": self.get_parameter("deposition.position_tolerance_m").value,
        }

        # Candidate poses, map frame, flattened [x, y, yaw, x, y, yaw, ...]. The rover stands still at
        # one to dig or dump. Each leg takes the next candidate whose footprint is clear in the
        # global costmap, so a boulder on a dig spot or an old pit doesn't block the run.
        self.declare_parameter("navigation.excavation_candidates", [6.0, 1.4, 0.0, 7.4, 1.4, 0.0, 6.7, 2.4, 0.0, 5.4, 2.4, 0.0])
        self.declare_parameter("navigation.deposition_candidates", [8.14, 6.95, 1.5708, 8.14, 5.95, 1.5708])
        # Dumps go through the candidates once, then keep using the last one, so a later approach
        # never drives over an earlier pile
        self.declare_parameter("navigation.deposition_repeat_last", True)
        self.deposition_repeat_last = self.get_parameter("navigation.deposition_repeat_last").value
        # Footprint (half sizes, as nav_params.yaml) plus a margin, checked for lethal cells
        self.declare_parameter("navigation.footprint_half_length_m", 0.62)
        self.declare_parameter("navigation.footprint_half_width_m", 0.53)
        self.declare_parameter("navigation.candidate_clearance_m", 0.10)
        self.footprint_half_length_m = self.get_parameter("navigation.footprint_half_length_m").value
        self.footprint_half_width_m = self.get_parameter("navigation.footprint_half_width_m").value
        self.candidate_clearance_m = self.get_parameter("navigation.candidate_clearance_m").value
        self.declare_parameter("navigation.goal_timeout_s", 180.0)
        self.declare_parameter("navigation.max_retries", 2)
        self.declare_parameter("navigation.travel_lift_position_m", 0.218)
        self.travel_lift_position_m = self.get_parameter("navigation.travel_lift_position_m").value
        # Within this distance and heading of a waypoint the sequence starts, even if Nav2 is
        # still lining up. A small buffer so it doesn't keep shuffling to correct itself.
        self.declare_parameter("navigation.arrival_radius_m", 0.25)
        self.arrival_radius_m = self.get_parameter("navigation.arrival_radius_m").value
        self.declare_parameter("navigation.arrival_yaw_tolerance_rad", 0.3)
        self.arrival_yaw_tolerance_rad = self.get_parameter("navigation.arrival_yaw_tolerance_rad").value

        self.candidates = {
            "EXCAVATION": parse_poses(self.get_parameter("navigation.excavation_candidates").value),
            "DEPOSITION": parse_poses(self.get_parameter("navigation.deposition_candidates").value),
        }
        # Next candidate to try per zone, the current leg's candidate, and those found blocked on it
        self.next_candidate = {"EXCAVATION": 0, "DEPOSITION": 0}
        self.target_index = None
        self.blocked_candidates = set()
        self.retargets = 0
        self.last_target_check_s = 0.0
        self.global_costmap = None
        self.max_nav_retries = self.get_parameter("navigation.max_retries").value
        self.nav_retries = 0
        self.navigation = Navigation(self, self.get_parameter("navigation.goal_timeout_s").value)
        # A goal waiting for Nav2 to finish cancelling the previous one (see Navigation.ready)
        self.goal_pending = False

        # Localisation check for new commands: a recent AprilTag fix, not much driving since,
        # and the global EKF agreeing with it (it rejects fixes it thinks are outliers)
        self.declare_parameter("localisation.require_localised", True)
        self.declare_parameter("localisation.max_fix_age_s", 30.0)
        self.declare_parameter("localisation.max_drive_since_fix_m", 1.0)
        self.declare_parameter("localisation.max_fix_disagreement_m", 0.3)
        self.require_localised = self.get_parameter("localisation.require_localised").value
        self.max_fix_age_s = self.get_parameter("localisation.max_fix_age_s").value
        self.max_drive_since_fix_m = self.get_parameter("localisation.max_drive_since_fix_m").value
        self.max_fix_disagreement_m = self.get_parameter("localisation.max_fix_disagreement_m").value
        self.tag_fix = None  # (map x, map y, monotonic time)
        self.drive_since_fix = 0.0
        self.drive_anchor = None  # local-EKF (x, y) the drive distance was last measured from

        # Top-level autonomy state
        self.state = "IDLE"

        # AUTO: after each sequence completes, navigate to the other zone instead of idling
        self.auto_mode = False
        self.auto_cycle = 0

        # Command publishers, the same ones the blind autonomy node uses
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.drum_lift_pub = self.create_publisher(Float64, "/drum_lift_control/autonomy", 10)
        self.drum_spin_pub = self.create_publisher(Float64, "/drum_spin_control/autonomy", 10)

        # Feedback
        self.left_actuator_position = None
        self.right_actuator_position = None
        self.joint_state_subscription = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        # Map-frame pose from the global EKF, for the arrival radius
        self.map_x = None
        self.map_y = None
        self.map_yaw = None
        self.global_pose_subscription = self.create_subscription(Odometry, "/odometry/global", self.global_pose_callback, 10)
        # Every AprilTag fix (before apriltag_gate's rate limit), and the local EKF to measure driving since
        self.tag_subscription = self.create_subscription(PoseWithCovarianceStamped, "/apriltag/pose", self.tag_callback, 10)
        self.local_pose_subscription = self.create_subscription(Odometry, "/odometry/filtered", self.local_pose_callback, 10)
        # Global costmap (latched), to check candidates are clear
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.costmap_callback, latched)

        # Sequences
        self.excavation = NavExcavationSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=excavation_config,
        )
        self.deposition = NavDepositionSequence(
            cmd_vel_pub=self.cmd_vel_pub,
            drum_lift_pub=self.drum_lift_pub,
            drum_spin_pub=self.drum_spin_pub,
            config=deposition_config,
        )

        # Basestation command input, separate from the blind node's /autonomy/command
        self.command_subscription = self.create_subscription(
            String,
            "/nav_autonomy/command",
            self.command_callback,
            10,
        )

        # State for the HUD and teleop window (JSON, latched): what is running, the last event, and
        # the result of the last command (COMPLETE / FAILED / REFUSED / STOPPED), so the team
        # can announce completion or failure without touching anything
        self.state_pub = self.create_publisher(String, "/nav_autonomy/state", latched)
        self.mode = None  # AUTO / NAV_EXCAVATE / NAV_DEPOSIT while one is running
        self.result = None
        self.result_time = None  # monotonic time the result was set
        self.message = "Waiting for a command."
        self.last_state = None

        # Update the active FSM and refresh active commands at the configured rate
        self.sequence_timer = self.create_timer(
            1.0 / update_rate_hz,
            self.update_sequence,
        )

        # The timer above runs on ROS time: with use_sim_time and no /clock (qpl_nav_sim on the
        # rover) it never fires, and nothing would say why. This one runs on the steady clock.
        self.started_s = time.monotonic()
        self.time_ok = True
        self.time_message = ""
        self.health_timer = self.create_timer(1.0, self.check_health, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info("Nav autonomy started.")
        self.get_logger().info("State: IDLE")
        self.get_logger().info("Waiting for NAV_EXCAVATE, NAV_DEPOSIT, AUTO, or STOP command.")

    """Feedback callbacks"""
    def joint_state_callback(self, msg):
        # Store the latest linear actuator positions
        for name, position in zip(msg.name, msg.position):
            if name == "left_linear_actuator_joint":
                self.left_actuator_position = position
            elif name == "right_linear_actuator_joint":
                self.right_actuator_position = position

    def global_pose_callback(self, msg):
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.map_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def tag_callback(self, msg):
        p = msg.pose.pose.position
        self.tag_fix = (p.x, p.y, time.monotonic())
        self.drive_since_fix = 0.0

    def local_pose_callback(self, msg):
        p = msg.pose.pose.position
        if self.drive_anchor is None:
            self.drive_anchor = (p.x, p.y)
            return
        step = hypot(p.x - self.drive_anchor[0], p.y - self.drive_anchor[1])
        if step >= DRIVE_STEP_M:
            self.drive_since_fix += step
            self.drive_anchor = (p.x, p.y)

    def costmap_callback(self, msg):
        self.global_costmap = msg

    def localisation_status(self):
        # (localised, reason if not, fix age s or None, metres driven since the fix or None)
        if self.tag_fix is None:
            return False, "no AprilTag fix yet; turn the rover until a camera sees the tag", None, None
        fix_x, fix_y, fix_time = self.tag_fix
        age = time.monotonic() - fix_time
        drive = self.drive_since_fix
        if age > self.max_fix_age_s:
            return False, f"last AprilTag fix was {age:.0f} s ago", age, drive
        if drive > self.max_drive_since_fix_m:
            return False, f"driven {drive:.1f} m since the last AprilTag fix", age, drive
        if self.map_x is None:
            return False, "no pose from the global EKF", age, drive
        disagreement = hypot(self.map_x - fix_x, self.map_y - fix_y)
        if disagreement > self.max_fix_disagreement_m + drive:
            return False, f"global EKF is {disagreement:.2f} m from the last AprilTag fix", age, drive
        return True, "", age, drive

    """Basestation commands"""
    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command == "NAV_EXCAVATE":
            self.handle_navigate("EXCAVATION")
        elif command == "NAV_DEPOSIT":
            self.handle_navigate("DEPOSITION")
        elif command == "AUTO":
            self.handle_auto()
        elif command == "STOP":
            self.handle_stop()
        else:
            self.report(f"Unknown nav autonomy command: {command}", "warning")
        self.publish_state()

    def report(self, message, level="info", result=None):
        # Log an event and show it on /nav_autonomy/state; result marks how a command ended
        getattr(self.get_logger(), level)(message)
        self.message = message
        if result is not None:
            self.result = result
            self.result_time = time.monotonic()
            self.mode = None

    def can_start(self, command):
        # Whether a new command may start; reports why not
        if self.state != "IDLE":
            self.report(f"Cannot start {command} while in state: {self.state}", "warning")
            return False
        if not self.time_ok:
            self.report(f"Not starting {command}: {self.time_message}", "error", "REFUSED")
            return False
        if not self.navigation.is_available():
            self.report(f"Not starting {command}: Nav2 is not running (start it with qpl_nav, or qpl_nav_sim "
                        "against the sim).", "error", "REFUSED")
            return False
        if self.require_localised:
            localised, reason, _, _ = self.localisation_status()
            if not localised:
                self.report(f"Not starting {command}: not localised ({reason}).", "error", "REFUSED")
                return False
        return True

    def handle_navigate(self, target):
        # Drive to the target's waypoint with Nav2, then start its sequence
        command = "NAV_EXCAVATE" if target == "EXCAVATION" else "NAV_DEPOSIT"
        if not self.can_start(command):
            return

        self.mode = command
        self.result = None
        self.start_navigation(target)

    def handle_auto(self):
        # Loop excavation and deposition at their waypoints until STOP
        if not self.can_start("AUTO"):
            return

        self.auto_mode = True
        self.auto_cycle = 1
        self.mode = "AUTO"
        self.result = None
        self.report("AUTO started. Cycle 1.")
        self.start_navigation("EXCAVATION")

    def start_navigation(self, target):
        self.state = f"NAVIGATING_TO_{target}"
        self.nav_retries = 0
        self.retargets = 0
        self.blocked_candidates = set()
        self.target_index = self.choose_candidate(target)
        self.last_target_check_s = time.monotonic()
        self.send_navigation_goal(target)

    def send_navigation_goal(self, target):
        # Sent while Nav2 is still cancelling the last goal, Nav2 aborts it; update_navigation
        # sends it once the cancel has finished
        if not self.navigation.ready():
            self.goal_pending = True
            return
        self.goal_pending = False
        x, y, yaw = self.target_pose(target)
        self.report(
            f"Navigating to {target.lower()} candidate {self.target_index + 1}/{len(self.candidates[target])} "
            f"({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    """Candidates"""
    def target_pose(self, target):
        return self.candidates[target][self.target_index]

    def candidate_order(self, target):
        # Indexes to try, starting from the next unused one. Dumps stay on the last candidate
        # once they reach it (deposition_repeat_last); earlier ones are only a fallback.
        n = len(self.candidates[target])
        start = self.next_candidate[target]
        if target == "DEPOSITION" and self.deposition_repeat_last:
            start = min(start, n - 1)
        return [(start + i) % n for i in range(n)]

    def candidate_blocked(self, target, index):
        # Lethal cell under the candidate's footprint (plus clearance) in the global costmap.
        # Unknown space counts as clear.
        if self.global_costmap is None:
            return False
        x, y, yaw = self.candidates[target][index]
        return footprint_blocked(self.global_costmap, x, y, yaw, self.footprint_half_length_m,
                                 self.footprint_half_width_m, self.candidate_clearance_m)

    def choose_candidate(self, target):
        order = self.candidate_order(target)
        for index in order:
            if index not in self.blocked_candidates and not self.candidate_blocked(target, index):
                return index
        # Nothing looks clear: take the first not yet given up on, and let Nav2 try
        index = next((i for i in order if i not in self.blocked_candidates), order[0])
        self.get_logger().warning(f"No clear {target.lower()} candidate in the costmap. Trying candidate {index + 1} anyway.")
        return index

    def check_target(self, target):
        # Switch candidates if the current one has become blocked. Returns True if it switched.
        # A switch doesn't use up a retry; there are at most as many switches as candidates.
        now = time.monotonic()
        if now - self.last_target_check_s < TARGET_CHECK_PERIOD_S:
            return False
        self.last_target_check_s = now
        if self.retargets >= len(self.candidates[target]) or not self.candidate_blocked(target, self.target_index):
            return False
        self.blocked_candidates.add(self.target_index)
        index = self.choose_candidate(target)
        if index in self.blocked_candidates:
            return False
        self.report(
            f"{target.capitalize()} candidate {self.target_index + 1} is blocked in the costmap. "
            f"Switching to candidate {index + 1}.", "warning")
        self.retargets += 1
        self.target_index = index
        self.navigation.cancel()
        self.send_navigation_goal(target)
        return True

    def handle_stop(self):
        # Stop the active sequence or navigation in its current position
        if self.state == "IDLE":
            self.get_logger().info("Nav autonomy already idle.")
            return

        stopped_from = self.state
        self.get_logger().info(f"Stopping nav autonomy from state: {self.state}")
        self.auto_mode = False
        self.goal_pending = False
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)

        if self.state == "EXCAVATING":
            self.excavation.stop(actuator_positions)
        elif self.state == "DEPOSITING":
            self.deposition.stop(actuator_positions)
        elif self.state.startswith("NAVIGATING_TO_"):
            # Nav2 keeps driving its goal unless it's cancelled
            self.navigation.cancel()
            self.cmd_vel_pub.publish(Twist())

        self.state = "IDLE"
        self.report(f"Nav autonomy stopped from {stopped_from}. State: IDLE", result="STOPPED")

    """FSM update"""
    def update_sequence(self):
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)
        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.state.startswith("NAVIGATING_TO_"):
            self.update_navigation(self.state.removeprefix("NAVIGATING_TO_"))
        elif self.state == "EXCAVATING":
            self.excavation.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.excavation.is_complete():
                if self.auto_mode:
                    self.report("Excavation complete.")
                    self.start_navigation("DEPOSITION")
                else:
                    self.state = "IDLE"
                    self.report("Excavation complete. State: IDLE", result="COMPLETE")
        elif self.state == "DEPOSITING":
            self.deposition.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.deposition.is_complete():
                if self.auto_mode:
                    self.auto_cycle += 1
                    self.report(f"Deposition complete. AUTO cycle {self.auto_cycle}.")
                    self.start_navigation("EXCAVATION")
                else:
                    self.state = "IDLE"
                    self.report("Deposition complete. State: IDLE", result="COMPLETE")
        self.publish_state()

    def update_navigation(self, target):
        # Wait for Nav2 to reach the waypoint, retrying a failed goal before giving up.
        # Hold the bucket raised so it doesn't plough the regolith on the way.
        self.drum_lift_pub.publish(Float64(data=self.travel_lift_position_m))
        if self.goal_pending:
            # The last goal is still being cancelled; done/succeeded below still describe it
            self.send_navigation_goal(target)
            return
        self.navigation.update()
        if self.check_target(target):
            return

        # Close enough and facing the right way counts, even if Nav2 is still lining up or
        # has given up on the spot
        error = self.waypoint_error(target)
        if error is not None and error[0] <= self.arrival_radius_m and error[1] <= self.arrival_yaw_tolerance_rad and (
                not self.navigation.done or not self.navigation.succeeded):
            self.navigation.cancel()
            self.report(
                f"Within {error[0]:.2f} m and {error[1]:.2f} rad of {target.lower()} candidate "
                f"{self.target_index + 1}. Starting here.")
            self.start_sequence(target)
            return

        if not self.navigation.done:
            return

        if self.navigation.succeeded:
            if error is not None:
                self.report(
                    f"Reached {target.lower()} candidate {self.target_index + 1} "
                    f"({error[0]:.2f} m, {error[1]:.2f} rad off).")
            else:
                self.report(f"Reached {target.lower()} candidate {self.target_index + 1}.")
            self.start_sequence(target)
            return

        self.nav_retries += 1
        if self.nav_retries <= self.max_nav_retries:
            self.report(f"Navigation failed. Retry {self.nav_retries}/{self.max_nav_retries}.", "warning")
            self.send_navigation_goal(target)
        else:
            self.cmd_vel_pub.publish(Twist())
            self.auto_mode = False
            self.state = "IDLE"
            self.report(f"Navigation to the {target.lower()} zone failed after {self.max_nav_retries} retries. "
                        "State: IDLE", "error", "FAILED")

    def waypoint_error(self, target):
        # (distance, absolute heading error) from the current candidate, or None without a map pose
        if self.map_x is None or self.map_y is None or self.map_yaw is None:
            return None
        x, y, yaw = self.target_pose(target)
        yaw_error = abs((self.map_yaw - yaw + pi) % (2.0 * pi) - pi)
        return hypot(self.map_x - x, self.map_y - y), yaw_error

    def start_sequence(self, target):
        # Digging or dumping starts here, so the next leg to this zone moves on to another candidate
        self.next_candidate[target] = self.target_index + 1
        if target == "EXCAVATION":
            self.state = "EXCAVATING"
            self.excavation.start()
            self.report("Starting excavation sequence.")
        else:
            self.state = "DEPOSITING"
            self.deposition.start()
            self.report("Starting deposition sequence.")

    """Health (steady clock, so it runs even when ROS time is stuck)"""
    def check_clock(self):
        # (ok, message): is ROS time usable for this node's timers and Nav2's?
        if self.get_parameter("use_sim_time").value:
            if self.get_clock().now().nanoseconds > 0 or time.monotonic() - self.started_s < CLOCK_GRACE_S:
                return True, ""
            return False, ("use_sim_time is true but nothing publishes /clock. On the rover start Nav2 with "
                           "qpl_nav; qpl_nav_sim is for the sim.")
        if self.count_publishers("/clock") > 0:
            return False, "the sim is publishing /clock but this node runs on wall time. Start Nav2 with qpl_nav_sim."
        return True, ""

    def check_health(self):
        ok, message = self.check_clock()
        if (ok, message) != (self.time_ok, self.time_message):
            if ok:
                self.get_logger().info("Clock OK.")
            else:
                self.get_logger().error(f"Clock problem: {message}")
        self.time_ok, self.time_message = ok, message
        self.publish_state(heartbeat=True)

    """State topic"""
    def sequence_phase(self):
        # (sequence step name, seconds left in a timed dig/dump) or (None, None)
        if self.state not in ("EXCAVATING", "DEPOSITING"):
            return None, None
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.state == "EXCAVATING":
            sequence, start_s, duration_s = self.excavation, self.excavation.dig_start_s, self.excavation.dig_duration_s
        else:
            sequence, start_s, duration_s = self.deposition, self.deposition.dump_start_s, self.deposition.dump_duration_s
        remaining = None
        if sequence.state.name in ("DIG", "DUMP_REGOLITH") and start_s is not None:
            remaining = round(max(0.0, duration_s - (now_s - start_s)), 1)
        return sequence.state.name, remaining

    def publish_state(self, heartbeat=False):
        # Publish when something changed, and every second from the steady clock regardless
        localised, loc_reason, fix_age, drive = self.localisation_status()
        phase, remaining = self.sequence_phase()
        target = None
        if self.state != "IDLE" and self.target_index is not None:
            kind = "EXCAVATION" if self.state in ("NAVIGATING_TO_EXCAVATION", "EXCAVATING") else "DEPOSITION"
            x, y, yaw = self.candidates[kind][self.target_index]
            target = {"kind": kind, "index": self.target_index + 1, "count": len(self.candidates[kind]),
                      "x": x, "y": y, "yaw": yaw}
        state = {
            "state": self.state,
            "mode": self.mode,
            "cycle": self.auto_cycle if self.mode == "AUTO" else None,
            "phase": phase,
            "remaining_s": remaining,
            "target": target,
            "retries": self.nav_retries,
            "max_retries": self.max_nav_retries,
            "result": self.result,
            "result_age_s": None if self.result_time is None else int(time.monotonic() - self.result_time),
            "message": self.message,
            "localised": localised,
            "loc_reason": loc_reason,
            "fix_age_s": None if fix_age is None else int(fix_age),
            "drive_since_fix_m": None if drive is None else round(drive, 2),
            "time_ok": self.time_ok,
            "time_message": self.time_message,
        }
        if state == self.last_state and not heartbeat:
            return
        self.last_state = state
        self.state_pub.publish(String(data=json.dumps({**state, "stamp": round(time.time(), 2)})))

    def shutdown(self):
        # Without this Nav2 keeps driving the last goal after the node dies
        if self.state.startswith("NAVIGATING_TO_"):
            self.get_logger().info("Shutting down. Cancelling navigation goal.")
            self.navigation.cancel()


def main(args=None):
    # Python's own Ctrl-C handling, so the context is still up to send the Nav2 cancel
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = NavAutonomyNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        # Flush the cancel request before the node goes away
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
