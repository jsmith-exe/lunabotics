import json
import time
from math import atan2, cos, hypot, pi, sin

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from qpl_autonomy.nav.costmap_check import footprint_lethal_cells
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
# Clearance round the footprint (at the planner's heading) for a pose to count as unstuck
UNSTICK_MARGIN_M = 0.05


def load_lattice_headings(path):
    # Heading angles of a Smac lattice primitives file; 16 evenly spaced if it can't be read
    try:
        with open(path) as f:
            return list(json.load(f)["lattice_metadata"]["heading_angles"])
    except (OSError, ValueError, KeyError):
        return [k * pi / 8.0 for k in range(16)]


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
        self.declare_parameter("navigation.excavation_candidates",
                               [7.8, 1.5, 1.5708, 6.2, 1.5, 1.5708, 4.6, 1.5, 1.5708, 1.4, 1.5, 1.5708, 3.0, 1.5, 1.5708,
                                7.8, 2.4, 1.5708, 6.2, 2.4, 1.5708, 4.6, 2.4, 1.5708, 1.4, 2.4, 1.5708, 3.0, 2.4, 1.5708])
        self.declare_parameter("navigation.deposition_candidates", [8.0, 6.40, 1.5708])
        # Dumps go through the candidates once, then keep using the last one, so a later approach
        # never drives over an earlier pile
        self.declare_parameter("navigation.deposition_repeat_last", True)
        self.declare_parameter("navigation.deposition_approach_m", 1.5)
        self.deposition_approach_m = self.get_parameter("navigation.deposition_approach_m").value
        # The approach point is only somewhere to pass through: any heading, and a looser radius.
        # Held to the dump spot's yaw tolerance the rover orbited it by the east wall until the
        # goal timed out.
        self.declare_parameter("navigation.approach_radius_m", 0.4)
        self.approach_radius_m = self.get_parameter("navigation.approach_radius_m").value
        self.deposition_repeat_last = self.get_parameter("navigation.deposition_repeat_last").value
        # Footprint (half sizes, as nav_params.yaml) plus a margin, checked for lethal cells
        self.declare_parameter("navigation.footprint_half_length_m", 0.62)
        self.declare_parameter("navigation.footprint_half_width_m", 0.53)
        self.declare_parameter("navigation.candidate_clearance_m", 0.05)
        # A candidate is blocked with at least this many lethal cells under it, so isolated
        # specks of sensor noise don't rule out every spot
        self.declare_parameter("navigation.candidate_min_lethal_cells", 4)
        self.candidate_min_lethal_cells = self.get_parameter("navigation.candidate_min_lethal_cells").value
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
        # Within stall_radius_m but no 5 cm closer for stall_time_s: Nav2
        # is shuffling near the goal, so start there instead
        self.declare_parameter("navigation.stall_radius_m", 0.40)
        self.declare_parameter("navigation.stall_time_s", 10.0)
        self.stall_radius_m = self.get_parameter("navigation.stall_radius_m").value
        self.stall_time_s = self.get_parameter("navigation.stall_time_s").value
        self.best_error = None  # (closest distance this leg, time it was reached)

        self.candidates = {
            "EXCAVATION": parse_poses(self.get_parameter("navigation.excavation_candidates").value),
            "DEPOSITION": parse_poses(self.get_parameter("navigation.deposition_candidates").value),
        }
        # Next candidate to try per zone, the current leg's candidate, and those found blocked on it
        self.next_candidate = {"EXCAVATION": 0, "DEPOSITION": 0}
        self.target_index = None
        self.approach_stage = False  # driving to the point before the dump spot
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
        # Driving since the last autonomy run that started localised (None: no such run yet).
        # A run's EKF pose is still good for the next command unless the rover was driven by
        # hand, so NAV_EXCAVATE then NAV_DEPOSIT isn't refused for an old tag fix.
        self.drive_since_run = None
        self.drive_since_fix = 0.0
        self.drive_anchor = None  # local-EKF (x, y) the drive distance was last measured from

        # Re-localising in AUTO: after a dig, with no recent tag fix, turn on the spot until a
        # camera faces the AprilTag and wait for a fix before driving to the berm. Turns on the
        # spot and digging slip the skid-steer, and the berm is too far from the tag to correct
        # it there: in the sim the map pose was 0.4-0.6 m out by the berm, enough to hit walls.
        self.declare_parameter("relocalise.enabled", True)
        self.declare_parameter("relocalise.tag_xy", [3.32, 0.02])  # qpl_rover/config/arena/<arena>.yaml
        self.declare_parameter("relocalise.max_fix_age_s", 15.0)  # a fix newer than this is enough
        self.declare_parameter("relocalise.turn_speed", 0.4)
        self.declare_parameter("relocalise.heading_tolerance_rad", 0.15)
        self.declare_parameter("relocalise.timeout_s", 40.0)
        self.declare_parameter("relocalise.rear_camera_max_range_m", 2.5)
        self.declare_parameter("relocalise.max_range_m", 4.0)
        self.declare_parameter("relocalise.min_angle_from_wall_rad", 0.35)
        self.relocalise_enabled = self.get_parameter("relocalise.enabled").value
        self.tag_xy = list(self.get_parameter("relocalise.tag_xy").value)
        self.relocalise_max_fix_age_s = self.get_parameter("relocalise.max_fix_age_s").value
        self.relocalise_turn_speed = self.get_parameter("relocalise.turn_speed").value
        self.relocalise_tolerance = self.get_parameter("relocalise.heading_tolerance_rad").value
        self.relocalise_timeout_s = self.get_parameter("relocalise.timeout_s").value
        self.relocalise_rear_range_m = self.get_parameter("relocalise.rear_camera_max_range_m").value
        self.relocalise_max_range_m = self.get_parameter("relocalise.max_range_m").value
        self.relocalise_min_angle = self.get_parameter("relocalise.min_angle_from_wall_rad").value
        self.relocalise_start = None  # (monotonic start, tag fix time before, EKF x, EKF y)
        self.declare_parameter("relocalise.min_fixes", 5)
        self.relocalise_min_fixes = self.get_parameter("relocalise.min_fixes").value
        self.relocalise_fixes = 0  # fixes seen while re-localising

        # Leaving the berm (AUTO): after a dump, reverse back the way the rover drove in (along
        # the dump spot's heading) before heading for the next dig spot. It faces north by the east wall there, and in
        # the sim the turn round swept its corners into the wall, so Nav2 couldn't start the leg.
        self.declare_parameter("deposition.back_out_m", 1.0)
        self.declare_parameter("deposition.back_out_speed", 0.1)
        self.back_out_m = self.get_parameter("deposition.back_out_m").value
        self.back_out_speed = self.get_parameter("deposition.back_out_speed").value
        self.back_out_start = None  # (monotonic start, EKF x, EKF y, heading to straighten up to)

        # Before re-sending a failed goal, move a little clear first: with its footprint touching a
        # lethal cell (it grazed something, or the map pose is off by a wall) Nav2's planner finds
        # no first move and its recoveries all report a collision, so a plain retry fails at once.
        self.declare_parameter("navigation.unstick_m", 0.4)
        self.declare_parameter("navigation.unstick_speed", 0.08)
        self.unstick_m = self.get_parameter("navigation.unstick_m").value
        self.unstick_speed = self.get_parameter("navigation.unstick_speed").value
        self.unstick = None  # dict: kind "turn" (to yaw) or "drive" (direction, distance), start pose and time
        # The planner (Smac lattice) checks the start pose at its nearest primitive heading, so a
        # rover clear at its own heading can still be "in lethal space" for it. Its headings, from
        # the same primitives file as qpl_rover/config/nav_params.yaml.
        self.declare_parameter(
            "navigation.lattice_file",
            "/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json")
        self.lattice_headings = load_lattice_headings(self.get_parameter("navigation.lattice_file").value)

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
        if self.state == "RELOCALISING":
            self.relocalise_fixes += 1

    def local_pose_callback(self, msg):
        p = msg.pose.pose.position
        if self.drive_anchor is None:
            self.drive_anchor = (p.x, p.y)
            return
        step = hypot(p.x - self.drive_anchor[0], p.y - self.drive_anchor[1])
        if step >= DRIVE_STEP_M:
            self.drive_since_fix += step
            if self.drive_since_run is not None and self.state == "IDLE":
                self.drive_since_run += step
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
        if self.drive_since_run is not None and self.drive_since_run <= self.max_drive_since_fix_m:
            return True, "", age, drive
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
        # One call per severity: rclpy raises if a single logging call site changes severity
        logger = self.get_logger()
        if level == "error":
            logger.error(message)
        elif level == "warning":
            logger.warning(message)
        else:
            logger.info(message)
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
            self.drive_since_run = 0.0
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
        self.unstick = None
        self.retargets = 0
        self.blocked_candidates = set()
        self.target_index, _ = self.choose_candidate(target)
        self.best_error = None
        # Dumps go via a point deposition_approach_m short of the dump spot, on its heading, so the
        # rover arrives lined up instead of swinging round by the wall (its corners hit the east wall)
        x, y, _ = self.candidates[target][self.target_index]
        self.approach_stage = (target == "DEPOSITION" and self.deposition_approach_m > 0 and
                               (self.map_x is None or hypot(x - self.map_x, y - self.map_y) > self.deposition_approach_m + 0.3))
        self.last_target_check_s = time.monotonic()
        # Starting with the footprint on a lethal cell (the map pose moved under a remembered mark
        # when re-localising, or a mark grew as it was seen closer), the planner can't start
        if (self.global_costmap is not None and self.map_x is not None and self.map_yaw is not None
                and self.footprint_blocked(self.map_x, self.map_y, self.map_yaw)
                and self.start_unstick("before setting off")):
            return
        self.send_navigation_goal(target)

    def send_navigation_goal(self, target):
        # Sent while Nav2 is still cancelling the last goal, Nav2 aborts it; update_navigation
        # sends it once the cancel has finished
        if not self.navigation.ready():
            self.goal_pending = True
            return
        self.goal_pending = False
        x, y, yaw = self.target_pose(target)
        if self.map_x is not None and not (target == "DEPOSITION" and self.approach_stage):
            # Heading doesn't matter for digging or dumping (the drum is under the middle): arrive
            # facing the way the rover drives there (or as it is, when already close), so there's no
            # turn on the spot at the goal. It orbited goals it had to turn round at, and by the
            # berm, with the map pose ~0.4 m out that far from the AprilTag, a turn to the dump
            # spot's heading swept into the east wall and stalled until the goal timed out.
            # Only the berm approach point keeps its heading, so the rover arrives lined up.
            if hypot(x - self.map_x, y - self.map_y) > 0.3:
                yaw = atan2(y - self.map_y, x - self.map_x)
            elif self.map_yaw is not None:
                yaw = self.map_yaw
        self.report(
            f"Navigating to {target.lower()} candidate {self.target_index + 1}/{len(self.candidates[target])} "
            f"({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    """Candidates"""
    def target_pose(self, target):
        x, y, yaw = self.candidates[target][self.target_index]
        if target == "DEPOSITION" and self.approach_stage:
            d = self.deposition_approach_m
            return x - d * cos(yaw), y - d * sin(yaw), yaw
        return x, y, yaw

    def leg_arrived(self, target):
        # At the berm approach point: drive straight on to the dump spot. Otherwise start digging/dumping.
        if target == "DEPOSITION" and self.approach_stage:
            self.approach_stage = False
            self.best_error = None
            self.navigation.cancel()
            self.report("At the berm approach point. Driving straight on to the dump spot.")
            self.send_navigation_goal(target)
            return
        self.start_sequence(target)

    def candidate_order(self, target):
        # Indexes to try, starting from the next unused one. Dumps stay on the last candidate
        # once they reach it (deposition_repeat_last); earlier ones are only a fallback.
        n = len(self.candidates[target])
        start = self.next_candidate[target]
        if target == "DEPOSITION" and self.deposition_repeat_last:
            start = min(start, n - 1)
        return [(start + i) % n for i in range(n)]

    def candidate_lethal_cells(self, target, index):
        if self.global_costmap is None:
            return 0
        x, y, yaw = self.candidates[target][index]
        return footprint_lethal_cells(self.global_costmap, x, y, yaw, self.footprint_half_length_m,
                                      self.footprint_half_width_m, self.candidate_clearance_m)

    def candidate_blocked(self, target, index):
        # Lethal cell under the candidate's footprint (plus clearance) in the global costmap.
        # Unknown space counts as clear.
        if self.global_costmap is None:
            return False
        x, y, yaw = self.candidates[target][index]
        return self.candidate_lethal_cells(target, index) >= self.candidate_min_lethal_cells

    def choose_candidate(self, target):
        # (index, clear): the next clear candidate, or the least blocked one if none is clear
        order = self.candidate_order(target)
        for index in order:
            if index not in self.blocked_candidates and not self.candidate_blocked(target, index):
                return index, True
        index = min(order, key=lambda i: self.candidate_lethal_cells(target, i))
        self.report(f"No clear {target.lower()} candidate in the costmap. Trying candidate {index + 1}, "
                    "the least blocked.", "warning")
        return index, False

    def check_target(self, target):
        # Switch candidates if the current one has become blocked. Returns True if it switched.
        # A switch doesn't use up a retry; there are at most as many switches as candidates, and
        # none once no candidate is clear.
        now = time.monotonic()
        if now - self.last_target_check_s < TARGET_CHECK_PERIOD_S:
            return False
        self.last_target_check_s = now
        if self.retargets >= len(self.candidates[target]) or not self.candidate_blocked(target, self.target_index):
            return False
        self.blocked_candidates.add(self.target_index)
        index, clear = self.choose_candidate(target)
        if not clear:
            # Keep going to the current one and stop checking for this leg
            self.retargets = len(self.candidates[target])
            return False
        self.report(
            f"{target.capitalize()} candidate {self.target_index + 1} is blocked in the costmap. "
            f"Switching to candidate {index + 1}.", "warning")
        self.retargets += 1
        self.target_index = index
        self.best_error = None
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
        self.unstick = None
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)

        if self.state == "EXCAVATING":
            self.excavation.stop(actuator_positions)
        elif self.state == "DEPOSITING":
            self.deposition.stop(actuator_positions)
        elif self.state.startswith("NAVIGATING_TO_"):
            # Nav2 keeps driving its goal unless it's cancelled
            self.navigation.cancel()
            self.cmd_vel_pub.publish(Twist())
        elif self.state in ("RELOCALISING", "BACKING_OUT"):
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
                    self.start_relocalising()
                else:
                    self.state = "IDLE"
                    self.report("Excavation complete. State: IDLE", result="COMPLETE")
        elif self.state == "RELOCALISING":
            self.update_relocalising()
        elif self.state == "BACKING_OUT":
            self.update_back_out()
        elif self.state == "DEPOSITING":
            self.deposition.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.deposition.is_complete():
                if self.auto_mode:
                    self.auto_cycle += 1
                    self.report(f"Deposition complete. AUTO cycle {self.auto_cycle}.")
                    self.start_back_out()
                else:
                    self.state = "IDLE"
                    self.report("Deposition complete. State: IDLE", result="COMPLETE")
        self.publish_state()

    def update_navigation(self, target):
        # Wait for Nav2 to reach the waypoint, retrying a failed goal before giving up.
        # Hold the bucket raised so it doesn't plough the regolith on the way.
        self.drum_lift_pub.publish(Float64(data=self.travel_lift_position_m))
        if self.unstick is not None:
            self.update_unstick(target)
            return
        if self.goal_pending:
            # The last goal is still being cancelled; done/succeeded below still describe it
            self.send_navigation_goal(target)
            return
        self.navigation.update()
        if self.check_target(target):
            return

        # Close enough counts, even if Nav2 is still lining up or has given up on the spot
        error = self.waypoint_error(target)
        radius = self.approach_radius_m if target == "DEPOSITION" and self.approach_stage else self.arrival_radius_m
        if error is not None and error <= radius and (
                not self.navigation.done or not self.navigation.succeeded):
            self.navigation.cancel()
            self.report(
                f"Within {error:.2f} m of {target.lower()} candidate "
                f"{self.target_index + 1}. Starting here.")
            self.leg_arrived(target)
            return

        # Close but no longer getting closer: Nav2 is shuffling near the goal, so start here
        if error is not None and error <= self.stall_radius_m:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if self.best_error is None or error < self.best_error[0] - 0.05:
                self.best_error = (error, now_s)
            elif now_s - self.best_error[1] >= self.stall_time_s:
                self.navigation.cancel()
                self.report(
                    f"No closer than {error:.2f} m to {target.lower()} candidate {self.target_index + 1} "
                    f"for {self.stall_time_s:.0f} s. Starting here.")
                self.leg_arrived(target)
                return

        if not self.navigation.done:
            return

        if self.navigation.succeeded:
            if error is not None:
                self.report(
                    f"Reached {target.lower()} candidate {self.target_index + 1} "
                    f"({error:.2f} m off).")
            else:
                self.report(f"Reached {target.lower()} candidate {self.target_index + 1}.")
            self.leg_arrived(target)
            return

        self.nav_retries += 1
        if self.nav_retries <= self.max_nav_retries:
            self.report(f"Navigation failed. Retry {self.nav_retries}/{self.max_nav_retries}.", "warning")
            if target == "EXCAVATION":
                # Any dig spot will do: one Nav2 couldn't reach (no path fits the footprint past the
                # obstacles in the way) is left for this leg and the next clear one tried instead
                self.blocked_candidates.add(self.target_index)
                index, clear = self.choose_candidate(target)
                if clear and index != self.target_index:
                    self.report(f"Couldn't reach dig spot {self.target_index + 1}. Trying dig spot {index + 1}.", "warning")
                    self.target_index = index
                    self.best_error = None
            if not self.start_unstick():
                self.send_navigation_goal(target)
        else:
            self.cmd_vel_pub.publish(Twist())
            self.auto_mode = False
            self.state = "IDLE"
            self.report(f"Navigation to the {target.lower()} zone failed after {self.max_nav_retries} retries. "
                        "State: IDLE", "error", "FAILED")

    def move_blocked(self, forward=0.0, turn=0.0):
        # For the moves this node drives itself (re-localising turn, back-out, unstick): would
        # going `forward` m or turning `turn` rad from here put more lethal cells under the
        # footprint than there are now? The skid-steer slides when it turns on the spot (0.4 m
        # in the sim, into a boulder), so this is checked every step, not just at the start.
        if self.global_costmap is None or self.map_x is None or self.map_yaw is None:
            return False
        def cells(x, y, yaw):
            return footprint_lethal_cells(self.global_costmap, x, y, yaw, self.footprint_half_length_m,
                                          self.footprint_half_width_m)
        now = cells(self.map_x, self.map_y, self.map_yaw)
        ahead = cells(self.map_x + forward * cos(self.map_yaw), self.map_y + forward * sin(self.map_yaw),
                      self.map_yaw + turn)
        return ahead > now

    def planner_heading(self, yaw):
        # The lattice heading the planner would snap yaw to
        return min(self.lattice_headings, key=lambda h: abs((h - yaw + pi) % (2.0 * pi) - pi))

    def footprint_blocked(self, x, y, yaw):
        # Lethal cells under the footprint as the planner checks it: at its nearest primitive
        # heading, with a little margin for the map pose moving
        return footprint_lethal_cells(self.global_costmap, x, y, self.planner_heading(yaw), self.footprint_half_length_m,
                                      self.footprint_half_width_m, UNSTICK_MARGIN_M) > 0

    def start_unstick(self, why="before retrying"):
        # Get to a pose the planner can start from: first a small turn on the spot onto the
        # nearest primitive heading (it then checks the true footprint), else back (or forwards)
        # 1x, 1.75x or 2.5x unstick_m. False without a map pose or costmap, or with nowhere clear.
        if self.map_x is None or self.map_yaw is None or self.global_costmap is None or self.unstick_m <= 0.0:
            return False
        heading = self.planner_heading(self.map_yaw)
        off_heading = abs((heading - self.map_yaw + pi) % (2.0 * pi) - pi) > 0.05
        if off_heading and not self.footprint_blocked(self.map_x, self.map_y, heading):
            self.navigation.cancel()
            self.unstick = {"kind": "turn", "yaw": heading, "start": time.monotonic()}
            self.report(f"Turning onto heading {heading:.2f} to get clear {why}.")
            return True
        for scale in (1.0, 1.75, 2.5):
            distance = scale * self.unstick_m
            for direction in (-1.0, 1.0):
                x = self.map_x + direction * distance * cos(self.map_yaw)
                y = self.map_y + direction * distance * sin(self.map_yaw)
                if not self.footprint_blocked(x, y, self.map_yaw):
                    self.navigation.cancel()
                    self.unstick = {"kind": "drive", "direction": direction, "distance": distance,
                                    "x": self.map_x, "y": self.map_y, "start": time.monotonic()}
                    self.report(f"Moving {distance:.1f} m {'back' if direction < 0 else 'forwards'} "
                                f"to get clear {why}.")
                    return True
        return False

    def update_unstick(self, target):
        u = self.unstick
        cmd = Twist()
        if u["kind"] == "turn":
            error = (u["yaw"] - self.map_yaw + pi) % (2.0 * pi) - pi
            done = (abs(error) < 0.03 or time.monotonic() - u["start"] > 10.0
                    or self.move_blocked(turn=0.1 if error > 0 else -0.1))
            cmd.angular.z = max(-0.3, min(0.3, 2.0 * error))
        else:
            done = (hypot(self.map_x - u["x"], self.map_y - u["y"]) >= u["distance"]
                    or time.monotonic() - u["start"] > 2.0 * u["distance"] / self.unstick_speed
                    or self.move_blocked(forward=0.1 * u["direction"]))
            cmd.linear.x = u["direction"] * self.unstick_speed
        if done:
            self.cmd_vel_pub.publish(Twist())
            self.unstick = None
            self.send_navigation_goal(target)
            return
        self.cmd_vel_pub.publish(cmd)

    def waypoint_error(self, target):
        # Distance from the current waypoint, or None without a map pose. Any heading will do.
        if self.map_x is None or self.map_y is None:
            return None
        x, y, _ = self.target_pose(target)
        return hypot(self.map_x - x, self.map_y - y)

    """Backing out of the berm (AUTO, after a dump)"""
    def start_back_out(self):
        if self.back_out_m <= 0.0 or self.map_x is None or self.map_yaw is None:
            self.start_navigation("EXCAVATION")
            return
        # Straight back the way the rover drove in, holding its heading. Lethal cells where the
        # footprint ends up, so it doesn't reverse into anything
        yaw = self.map_yaw
        bx = self.map_x - self.back_out_m * cos(yaw)
        by = self.map_y - self.back_out_m * sin(yaw)
        if self.global_costmap is not None and footprint_lethal_cells(
                self.global_costmap, bx, by, yaw, self.footprint_half_length_m,
                self.footprint_half_width_m) >= self.candidate_min_lethal_cells:
            self.report("Not backing out of the berm: something is behind the rover.", "warning")
            self.start_navigation("EXCAVATION")
            return
        self.state = "BACKING_OUT"
        self.back_out_start = (time.monotonic(), self.map_x, self.map_y, yaw)
        self.report(f"Backing {self.back_out_m:.1f} m out of the berm.")

    def update_back_out(self):
        start_s, x0, y0, yaw = self.back_out_start
        moved = hypot(self.map_x - x0, self.map_y - y0)
        # Twice the time it should take, in case it's held up
        if (moved >= self.back_out_m or time.monotonic() - start_s > 2.0 * self.back_out_m / self.back_out_speed
                or self.move_blocked(forward=-0.1)):
            self.cmd_vel_pub.publish(Twist())
            self.start_navigation("EXCAVATION")
            return
        cmd = Twist()
        cmd.linear.x = -self.back_out_speed
        error = (yaw - self.map_yaw + pi) % (2.0 * pi) - pi
        cmd.angular.z = max(-0.3, min(0.3, 1.5 * error))
        self.cmd_vel_pub.publish(cmd)

    """Re-localising (AUTO, between a dig and the berm)"""
    def start_relocalising(self):
        age = None if self.tag_fix is None else time.monotonic() - self.tag_fix[2]
        if not self.relocalise_enabled or self.map_x is None or self.map_yaw is None or (
                age is not None and age <= self.relocalise_max_fix_age_s):
            self.start_navigation("DEPOSITION")
            return
        # The tag is on the y = 0 wall facing into the arena: too far, or seen too obliquely, the
        # cameras don't detect it, and turning would only waste the timeout
        dx, dy = self.map_x - self.tag_xy[0], self.map_y - self.tag_xy[1]
        if hypot(dx, dy) > self.relocalise_max_range_m or atan2(dy, abs(dx)) < self.relocalise_min_angle:
            self.report(f"Not re-localising here: the AprilTag is {hypot(dx, dy):.1f} m away at "
                        f"{atan2(dy, abs(dx)) * 180.0 / pi:.0f} deg to its wall, too far or oblique to detect.")
            self.start_navigation("DEPOSITION")
            return
        self.state = "RELOCALISING"
        self.relocalise_fixes = 0
        self.relocalise_start = (time.monotonic(), None if self.tag_fix is None else self.tag_fix[2],
                                 self.map_x, self.map_y)
        fix = "no AprilTag fix yet" if age is None else f"last AprilTag fix {age:.0f} s ago"
        self.report(f"Re-localising ({fix}): turning a camera to the AprilTag.")

    def update_relocalising(self):
        start_s, fix_before, x0, y0 = self.relocalise_start
        elapsed = time.monotonic() - start_s
        fresh = self.tag_fix is not None and self.tag_fix[2] != fix_before
        # Several fixes standing still, so the EKF averages out their noise (a single one from
        # ~4 m was up to 0.3 m out in the sim), then the EKF agreeing with the latest
        if (self.relocalise_fixes >= self.relocalise_min_fixes
                and hypot(self.map_x - self.tag_fix[0], self.map_y - self.tag_fix[1]) <= self.max_fix_disagreement_m):
            self.cmd_vel_pub.publish(Twist())
            self.report(f"Re-localised: the map pose moved {hypot(self.map_x - x0, self.map_y - y0):.2f} m.")
            self.start_navigation("DEPOSITION")
            return
        if elapsed > self.relocalise_timeout_s:
            self.cmd_vel_pub.publish(Twist())
            why = ("the global EKF isn't taking the AprilTag fix" if fresh else "no AprilTag fix")
            self.report(f"Re-localising timed out after {elapsed:.0f} s ({why}). Driving on.", "warning")
            self.start_navigation("DEPOSITION")
            return
        # Face the tag with the front camera, or the rear one (it looks backwards) if that's a
        # smaller turn and the tag is close enough for it
        bearing = atan2(self.tag_xy[1] - self.map_y, self.tag_xy[0] - self.map_x)
        near = hypot(self.tag_xy[0] - self.map_x, self.tag_xy[1] - self.map_y) <= self.relocalise_rear_range_m
        errors = [(bearing + offset - self.map_yaw + pi) % (2.0 * pi) - pi for offset in ((0.0, pi) if near else (0.0,))]
        error = min(errors, key=abs)
        cmd = Twist()
        if abs(error) > self.relocalise_tolerance:
            if self.move_blocked(turn=0.15 if error > 0 else -0.15):
                self.cmd_vel_pub.publish(Twist())
                self.report("Stopped turning to the AprilTag: an obstacle is in the way. Driving on.", "warning")
                self.start_navigation("DEPOSITION")
                return
            cmd.angular.z = self.relocalise_turn_speed if error > 0 else -self.relocalise_turn_speed
        self.cmd_vel_pub.publish(cmd)

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
        if self.state not in ("IDLE", "RELOCALISING", "BACKING_OUT") and self.target_index is not None:
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
