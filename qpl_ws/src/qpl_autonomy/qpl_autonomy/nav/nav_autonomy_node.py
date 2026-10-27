import json
import time
from math import atan2, cos, hypot, pi, sin

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from qpl_autonomy.nav.costmap_check import footprint_lethal_cells
from qpl_autonomy.nav.route import path_clearance, widest_route
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
# How often the widest way round is recomputed while following it
ROUTE_REFRESH_S = 3.0
# Clearance round the footprint (at the planner's heading) for a pose to count as unstuck
UNSTICK_MARGIN_M = 0.05
# AUTO_TRENCH: longest turn to line up with the trench, how far along it the rover steers for,
# and the half width of the corridor the depth filters don't mark craters in
ALIGN_TIMEOUT_S = 20.0
TRENCH_LOOKAHEAD_M = 0.5
TRENCH_CORRIDOR_HALF_WIDTH_M = 0.8


def load_lattice_headings(path):
    # Heading angles of a Smac lattice primitives file; 16 evenly spaced if it can't be read
    try:
        with open(path) as f:
            return list(json.load(f)["lattice_metadata"]["heading_angles"])
    except (OSError, ValueError, KeyError):
        return [k * pi / 8.0 for k in range(16)]


def wrap_angle(angle):
    # angle in [-pi, pi)
    return (angle + pi) % (2.0 * pi) - pi


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
    AUTO_TRENCH (high risk) is AUTO with one fixed trench instead of the dig spots: the rover
    reverses along it with the bucket down every cycle.
    """

    def __init__(self):
        super().__init__("nav_autonomy")

        # Parameters. Defaults match config/nav/nav_autonomy.yaml, which documents each one.
        update_rate_hz = self.param("autonomy.update_rate_hz", 5.0)
        excavation_config = {key: self.param(f"excavation.{key}", default) for key, default in {
            "drum_spin_speed": 0.8,
            "drum_spin_direction": 1.0,
            "lowered_position_m": 0.032,
            "dig_duration_s": 30.0,
            "raised_position_m": 0.218,
            "position_tolerance_m": 0.005,
        }.items()}
        deposition_config = {key: self.param(f"deposition.{key}", default) for key, default in {
            "raised_position_m": 0.218,
            "dump_duration_s": 60.0,
            "drum_spin_speed": 0.4,
            "drum_spin_direction": -1.0,
            "position_tolerance_m": 0.005,
        }.items()}

        # Candidate poses, map frame, flattened [x, y, yaw, x, y, yaw, ...]. The rover stands still at
        # one to dig or dump. Each leg takes the next candidate whose footprint is clear in the
        # global costmap, so a boulder on a dig spot or an old pit doesn't block the run.
        self.candidates = {
            "EXCAVATION": parse_poses(self.param("navigation.excavation_candidates", [
                6.9, 1.5, 1.5708, 5.5, 1.5, 1.5708, 4.1, 1.5, 1.5708, 1.4, 1.5, 1.5708, 2.7, 1.5, 1.5708,
                6.9, 2.4, 1.5708, 5.5, 2.4, 1.5708, 4.1, 2.4, 1.5708, 1.4, 2.4, 1.5708, 2.7, 2.4, 1.5708])),
            "DEPOSITION": parse_poses(self.param("navigation.deposition_candidates", [8.0, 6.40, 1.5708])),
        }
        self.dig_candidates = self.candidates["EXCAVATION"]
        # AUTO_TRENCH: every cycle the rover drives to trench.start_xy, turns to face away from
        # trench.end_xy and reverses there with the bucket down, digging all the way (it can't dig
        # driving forwards). Its one excavation "candidate" is the start, facing away from the end.
        start = list(self.param("trench.start_xy", [4.4, 1.5]))
        end = list(self.param("trench.end_xy", [6.9, 1.5]))
        self.trench_start = (start[0], start[1], atan2(start[1] - end[1], start[0] - end[0]))
        self.trench_end = (end[0], end[1])
        self.trench_speed = self.param("trench.speed", 0.05)
        self.trench_align_tolerance = self.param("trench.align_tolerance_rad", 0.1)
        self.trench_mode = False
        self.align_start_s = None
        self.trench_drive_start_s = None
        # Footprint (half sizes, as nav_params.yaml) plus a margin, checked for lethal cells
        self.footprint_half_length_m = self.param("navigation.footprint_half_length_m", 0.62)
        self.footprint_half_width_m = self.param("navigation.footprint_half_width_m", 0.53)
        self.candidate_clearance_m = self.param("navigation.candidate_clearance_m", 0.05)
        # A candidate is blocked with at least this many lethal cells under it, so isolated
        # specks of sensor noise don't rule out every spot
        self.candidate_min_lethal_cells = self.param("navigation.candidate_min_lethal_cells", 4)
        goal_timeout_s = self.param("navigation.goal_timeout_s", 180.0)
        self.max_nav_retries = self.param("navigation.max_retries", 3)
        self.travel_lift_position_m = self.param("navigation.travel_lift_position_m", 0.218)
        # Within this distance of a waypoint the sequence starts, even if Nav2 is still lining
        # up. A small buffer so it doesn't keep shuffling to correct itself.
        self.arrival_radius_m = self.param("navigation.arrival_radius_m", 0.25)
        # Within stall_radius_m but no 5 cm closer for stall_time_s: Nav2
        # is shuffling near the goal, so start there instead
        self.stall_radius_m = self.param("navigation.stall_radius_m", 0.40)
        self.stall_time_s = self.param("navigation.stall_time_s", 10.0)
        self.best_error = None  # (closest distance this leg, time it was reached)

        # Next candidate to try per zone, the current leg's candidate, and those found blocked on it
        self.next_candidate = {"EXCAVATION": 0, "DEPOSITION": 0}
        self.target_index = None
        self.blocked_candidates = set()
        self.retargets = 0
        self.last_target_check_s = 0.0
        self.global_costmap = None
        self.nav_retries = 0
        self.navigation = Navigation(self, goal_timeout_s)
        # A goal waiting for Nav2 to finish cancelling the previous one (see Navigation.ready)
        self.goal_pending = False

        # Localisation check for new commands: a recent AprilTag fix, not much driving since,
        # and the global EKF agreeing with it (it rejects fixes it thinks are outliers)
        self.require_localised = self.param("localisation.require_localised", True)
        self.max_fix_age_s = self.param("localisation.max_fix_age_s", 30.0)
        self.max_drive_since_fix_m = self.param("localisation.max_drive_since_fix_m", 1.0)
        self.max_fix_disagreement_m = self.param("localisation.max_fix_disagreement_m", 0.3)
        self.tag_fix = None  # (map x, map y, monotonic time)
        # Driving since the last autonomy run that started localised (None: no such run yet).
        # A run's EKF pose is still good for the next command unless the rover was driven by
        # hand, so NAV_EXCAVATE then NAV_DEPOSIT isn't refused for an old tag fix.
        self.drive_since_run = None
        self.drive_since_fix = 0.0
        self.drive_anchor = None  # local-EKF (x, y) the drive distance was last measured from

        # Re-localising in AUTO: after a dig, turn on the spot until a camera faces the AprilTag
        # and wait for fixes before driving to the berm. Turns on the spot and digging slip the
        # skid-steer, and the berm is too far from the tag to correct it there: in the sim the
        # map pose was 0.4-0.6 m out by the berm, enough to hit walls.
        self.relocalise_enabled = self.param("relocalise.enabled", True)
        self.tag_xy = list(self.param("relocalise.tag_xy", [3.32, 0.02]))  # qpl_rover/config/arena/<arena>.yaml
        self.relocalise_max_fix_age_s = self.param("relocalise.max_fix_age_s", 0.0)  # a fix newer than this is enough
        self.relocalise_min_fixes = self.param("relocalise.min_fixes", 5)
        self.relocalise_turn_speed = self.param("relocalise.turn_speed", 0.4)
        self.relocalise_tolerance = self.param("relocalise.heading_tolerance_rad", 0.15)
        self.relocalise_timeout_s = self.param("relocalise.timeout_s", 25.0)
        self.relocalise_max_range_m = self.param("relocalise.max_range_m", 4.4)
        self.relocalise_min_angle = self.param("relocalise.min_angle_from_wall_rad", 0.35)
        self.relocalise_rear_range_m = self.param("relocalise.rear_camera_max_range_m", 2.5)
        self.relocalise_turn_clearance_m = self.param("relocalise.turn_clearance_m", 1.25)
        self.relocalise_start = None  # (monotonic start, tag fix time before, EKF x, EKF y)
        self.relocalise_fixes = 0  # fixes seen while re-localising

        # Leaving the berm (AUTO): after a dump, reverse straight back the way the rover drove in
        # before heading for the next dig spot. It faces the east wall there, and in the sim the
        # turn round swept its corners into the wall, so Nav2 couldn't start the leg.
        self.back_out_m = self.param("deposition.back_out_m", 1.0)
        self.back_out_speed = self.param("deposition.back_out_speed", 0.1)
        self.back_out_start = None  # (monotonic start, EKF x, EKF y, heading to hold)

        # Before re-sending a failed goal, move a little clear first: with its footprint touching a
        # lethal cell (it grazed something, or the map pose is off by a wall) Nav2's planner finds
        # no first move and its recoveries all report a collision, so a plain retry fails at once.
        self.unstick_m = self.param("navigation.unstick_m", 0.4)
        self.unstick_speed = self.param("navigation.unstick_speed", 0.08)
        self.unstick = None  # dict: kind "turn" (to yaw) or "drive" (direction, distance), start pose and time
        # Each leg (and each retry, from where the rover is) goes via waypoints along the widest way
        # round in the global costmap (qpl_autonomy/nav/route.py), not the planner's shortest way
        # that fits, which in a cluster of craters was often a gap the rover then grazed or stuck in
        self.route_min_clearance_m = self.param("navigation.route_min_clearance_m", 0.55)
        self.route_spacing_m = self.param("navigation.route_waypoint_spacing_m", 1.0)
        self.route = []  # map-frame (x, y) waypoints still to pass on this leg
        self.route_path = []  # the route cell by cell, to check it's still clear
        self.route_checked_s = 0.0  # monotonic time the route was last recomputed
        # Tight mode, after a failed leg: the depth filters shrink the margins round obstacles
        # (qpl_rover depth_obstacle_filter.yaml *_tight_m) and Nav2 slows down, so the rover can
        # squeeze through a gap the full clearance closes. Off again at the next leg.
        self.tight_speed_percent = self.param("navigation.tight_speed_percent", 50.0)
        self.tight = False
        # The planner (Smac lattice) checks the start pose at its nearest primitive heading, so a
        # rover clear at its own heading can still be "in lethal space" for it. Its headings, from
        # the same primitives file as qpl_rover/config/nav_params.yaml.
        self.lattice_headings = load_lattice_headings(self.param(
            "navigation.lattice_file",
            "/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json"))

        # Top-level autonomy state
        self.state = "IDLE"

        # AUTO: after each sequence completes, navigate to the other zone instead of idling
        self.auto_mode = False
        self.auto_cycle = 0

        # Command publishers, the same ones the blind autonomy node uses
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        tight_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.tight_pub = self.create_publisher(Bool, "/nav_autonomy/tight", tight_qos)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.tight_pub.publish(Bool(data=False))
        # The trench's corridor while AUTO_TRENCH runs (empty otherwise): the depth filters don't
        # mark craters in it, or the trench dug last cycle would block the way back to its start
        self.trench_corridor_pub = self.create_publisher(Float64MultiArray, "/nav_autonomy/trench_corridor", tight_qos)
        self.trench_corridor_pub.publish(Float64MultiArray())
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
        self.get_logger().info("Waiting for NAV_EXCAVATE, NAV_DEPOSIT, AUTO, AUTO_TRENCH, or STOP command.")

    def param(self, name, default):
        # Declare a parameter and return its value
        return self.declare_parameter(name, default).value

    def footprint_cells(self, x, y, yaw, margin=0.0):
        # Lethal cells in the global costmap under the rover's footprint (grown by margin) at a pose
        return footprint_lethal_cells(self.global_costmap, x, y, yaw, self.footprint_half_length_m,
                                      self.footprint_half_width_m, margin)

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
        elif command == "AUTO_TRENCH":
            self.handle_auto(trench=True)
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
        self.set_trench(False)
        self.start_navigation(target)

    def handle_auto(self, trench=False):
        # Loop excavation and deposition at their waypoints (or the trench) until STOP
        mode = "AUTO_TRENCH" if trench else "AUTO"
        if not self.can_start(mode):
            return

        self.auto_mode = True
        self.auto_cycle = 1
        self.mode = mode
        self.result = None
        self.set_trench(trench)
        self.report(f"{mode} started. Cycle 1.")
        self.start_navigation("EXCAVATION")

    def set_trench(self, on):
        # Dig the trench (AUTO_TRENCH) or at the dig spots, and tell the depth filters where it is
        self.trench_mode = on
        self.candidates["EXCAVATION"] = [self.trench_start] if on else self.dig_candidates
        corridor = Float64MultiArray()
        if on:
            (sx, sy, _), (ex, ey) = self.trench_start, self.trench_end
            m = TRENCH_CORRIDOR_HALF_WIDTH_M
            corridor.data = [min(sx, ex) - m, max(sx, ex) + m, min(sy, ey) - m, max(sy, ey) + m]
        self.trench_corridor_pub.publish(corridor)

    def start_navigation(self, target):
        self.state = f"NAVIGATING_TO_{target}"
        self.nav_retries = 0
        self.set_tight(False)
        self.unstick = None
        self.route, self.route_path = [], []
        self.retargets = 0
        self.blocked_candidates = set()
        self.target_index, _ = self.choose_candidate(target)
        self.best_error = None
        self.last_target_check_s = time.monotonic()
        self.plan_route(target)
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
        if self.route:
            # Facing on along the route (the next waypoint, or the candidate), so no turn at this one
            x, y = self.route[0]
            nx, ny = self.route[1] if len(self.route) > 1 else self.candidates[target][self.target_index][:2]
            yaw = atan2(ny - y, nx - x)
            self.report(f"Navigating via waypoint ({x:.2f}, {y:.2f}), {len(self.route)} left to "
                        f"{target.lower()} candidate {self.target_index + 1}.")
            self.navigation.navigate_to(x, y, yaw)
            return
        x, y, yaw = self.candidates[target][self.target_index]
        if self.map_x is not None:
            # Heading doesn't matter for digging or dumping (the drum is under the middle): arrive
            # facing the way the rover drives there (or as it is, when already close), so there's no
            # turn on the spot at the goal. It orbited goals it had to turn round at, and by the
            # berm, with the map pose ~0.4 m out that far from the AprilTag, a turn to the dump
            # spot's heading swept into the east wall and stalled until the goal timed out.
            if hypot(x - self.map_x, y - self.map_y) > 0.3:
                yaw = atan2(y - self.map_y, x - self.map_x)
            elif self.map_yaw is not None:
                yaw = self.map_yaw
        self.report(
            f"Navigating to {target.lower()} candidate {self.target_index + 1}/{len(self.candidates[target])} "
            f"({x:.2f}, {y:.2f}, yaw {yaw:.2f}).")
        self.navigation.navigate_to(x, y, yaw)

    """Candidates"""
    def candidate_order(self, target):
        # Indexes to try, starting from the next unused one
        n = len(self.candidates[target])
        start = self.next_candidate[target]
        return [(start + i) % n for i in range(n)]

    def candidate_lethal_cells(self, target, index):
        if self.global_costmap is None:
            return 0
        x, y, yaw = self.candidates[target][index]
        return self.footprint_cells(x, y, yaw, self.candidate_clearance_m)

    def candidate_blocked(self, target, index):
        # Enough lethal cells under the candidate's footprint (plus clearance) in the global
        # costmap. Unknown space counts as clear, and so does everything without a costmap yet.
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
        self.set_tight(False)
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)

        if self.state == "EXCAVATING":
            self.excavation.stop(actuator_positions)
        elif self.state == "DEPOSITING":
            self.deposition.stop(actuator_positions)
        elif self.state.startswith("NAVIGATING_TO_"):
            # Nav2 keeps driving its goal unless it's cancelled
            self.navigation.cancel()
            self.cmd_vel_pub.publish(Twist())
        elif self.state in ("RELOCALISING", "BACKING_OUT", "ALIGNING"):
            self.cmd_vel_pub.publish(Twist())
        self.set_trench(False)

        self.state = "IDLE"
        self.report(f"Nav autonomy stopped from {stopped_from}. State: IDLE", result="STOPPED")

    """FSM update"""
    def update_sequence(self):
        actuator_positions = (self.left_actuator_position, self.right_actuator_position)
        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.state.startswith("NAVIGATING_TO_"):
            self.update_navigation(self.state.removeprefix("NAVIGATING_TO_"))
        elif self.state == "ALIGNING":
            self.update_aligning()
        elif self.state == "EXCAVATING":
            self.excavation.update(actuator_positions=actuator_positions, now_s=now_s)
            if self.excavation.driven and self.excavation.state.name == "DIG":
                self.update_trench()
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
        if not self.route and self.tight and time.monotonic() - self.route_checked_s >= ROUTE_REFRESH_S:
            # Tight mode came on with no route: once the smaller margins are in the costmap there
            # may be one
            if self.plan_route(target, quiet=True) and self.route:
                self.report("A way round fits with the smaller margins; following it.")
                self.navigation.cancel()
                self.send_navigation_goal(target)
                return
        if self.route and time.monotonic() - self.route_checked_s >= ROUTE_REFRESH_S:
            # Obstacles keep appearing as the cameras get closer: a gap the route went through can
            # close, and following it the rover got wedged. Recompute from here and switch if the
            # way round has moved.
            old = self.route
            # Keep the current route while it's still wide enough, unless a new one is clearly
            # wider: two similar routes swapped every refresh and the rover dithered between them
            # (along the part still ahead, up to the last waypoint: the ends can be close to
            # obstacles, a dig spot by a boulder)
            current = self.route_ahead_clearance(old[-1])
            old_path = self.route_path
            new_clearance = self.plan_route(target, quiet=True)
            if (self.route and (hypot(self.route[0][0] - old[0][0], self.route[0][1] - old[0][1]) <= 0.5
                                or (current >= self.route_min_clearance_m and new_clearance < current + 0.1))):
                self.route, self.route_path = old, old_path  # keep the waypoint Nav2 is driving to
            else:
                self.report(f"The widest way round has changed ({current:.2f} m clear ahead on the old one, "
                            f"{new_clearance:.2f} m on the new one); following the new one.")
                self.navigation.cancel()
                self.send_navigation_goal(target)
                return
        if self.route and self.map_x is not None:
            # Passing a waypoint (or Nav2 reaching it): on to the next, or the candidate itself
            x, y = self.route[0]
            if hypot(x - self.map_x, y - self.map_y) <= 0.6 or (self.navigation.done and self.navigation.succeeded):
                self.route.pop(0)
                self.navigation.cancel()
                self.send_navigation_goal(target)
                return
        if self.check_target(target):
            return

        # Close enough counts, even if Nav2 is still lining up or has given up on the spot
        error = self.waypoint_error(target)
        if error is not None and error <= self.arrival_radius_m and (
                not self.navigation.done or not self.navigation.succeeded):
            self.navigation.cancel()
            self.report(
                f"Within {error:.2f} m of {target.lower()} candidate "
                f"{self.target_index + 1}. Starting here.")
            self.start_sequence(target)
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
                self.start_sequence(target)
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
            self.start_sequence(target)
            return

        self.nav_retries += 1
        if self.nav_retries <= self.max_nav_retries:
            self.report(f"Navigation failed. Retry {self.nav_retries}/{self.max_nav_retries}.", "warning")
            if not self.tight:
                self.set_tight(True)
                self.report("Squeezing through: smaller margins round obstacles and half speed for the rest of "
                            "this leg.", "warning")
            if target == "EXCAVATION":
                # Any dig spot will do: one Nav2 couldn't reach (no path fits the footprint past the
                # obstacles in the way) is left for this leg and the next clear one tried instead
                self.blocked_candidates.add(self.target_index)
                index, clear = self.choose_candidate(target)
                if clear and index != self.target_index:
                    self.report(f"Couldn't reach dig spot {self.target_index + 1}. Trying dig spot {index + 1}.", "warning")
                    self.target_index = index
                    self.best_error = None
            self.plan_route(target)
            if not self.start_unstick():
                self.send_navigation_goal(target)
        else:
            self.cmd_vel_pub.publish(Twist())
            self.set_tight(False)
            self.set_trench(False)
            self.auto_mode = False
            self.state = "IDLE"
            self.report(f"Navigation to the {target.lower()} zone failed after {self.max_nav_retries} retries. "
                        "State: IDLE", "error", "FAILED")

    def nearest_lethal_m(self):
        # Distance from the rover's centre to the nearest lethal cell in the global costmap, or None
        if self.global_costmap is None or self.map_x is None:
            return None
        info = self.global_costmap.info
        cells = np.array(self.global_costmap.data, dtype=np.int16).reshape(info.height, info.width)
        rows, cols = np.nonzero(cells == 100)
        if len(rows) == 0:
            return None
        xs = info.origin.position.x + (cols + 0.5) * info.resolution
        ys = info.origin.position.y + (rows + 0.5) * info.resolution
        return float(np.min(np.hypot(xs - self.map_x, ys - self.map_y)))

    def move_blocked(self, forward=0.0, turn=0.0):
        # For the moves this node drives itself (re-localising turn, back-out, unstick): would
        # going `forward` m or turning `turn` rad from here put more lethal cells under the
        # footprint than there are now? The skid-steer slides when it turns on the spot (0.4 m
        # in the sim, into a boulder), so this is checked every step, not just at the start.
        if self.global_costmap is None or self.map_x is None or self.map_yaw is None:
            return False
        now = self.footprint_cells(self.map_x, self.map_y, self.map_yaw)
        ahead = self.footprint_cells(self.map_x + forward * cos(self.map_yaw), self.map_y + forward * sin(self.map_yaw),
                      self.map_yaw + turn)
        return ahead > now

    def planner_heading(self, yaw):
        # The lattice heading the planner would snap yaw to
        return min(self.lattice_headings, key=lambda h: abs(wrap_angle(h - yaw)))

    def footprint_blocked(self, x, y, yaw):
        # Lethal cells under the footprint as the planner checks it: at its nearest primitive
        # heading, with a little margin for the map pose moving
        return self.footprint_cells(x, y, self.planner_heading(yaw), UNSTICK_MARGIN_M) > 0

    def plan_route(self, target, quiet=False):
        # The widest way round to the candidate, as waypoints for the rest of this leg
        self.route_checked_s = time.monotonic()
        if self.global_costmap is None or self.map_x is None:
            return 0.0
        x, y, _ = self.candidates[target][self.target_index]
        waypoints, clearance, path = widest_route(self.global_costmap, (self.map_x, self.map_y), (x, y),
                                                  self.route_min_clearance_m, self.route_spacing_m)
        self.route, self.route_path = waypoints, path
        if waypoints and not quiet:
            self.report(f"Taking the widest way round: {len(waypoints)} waypoints, {clearance:.2f} m clear of "
                        "obstacles.")
        return clearance

    def route_ahead_clearance(self, last_waypoint):
        # Clearance along the stored route from its point nearest the rover to its last waypoint
        path = self.route_path
        if len(path) < 2:
            return float("inf")
        near = min(range(len(path)), key=lambda i: hypot(path[i][0] - self.map_x, path[i][1] - self.map_y))
        end = min(range(len(path)), key=lambda i: hypot(path[i][0] - last_waypoint[0], path[i][1] - last_waypoint[1]))
        return path_clearance(self.global_costmap, path[near:end + 1]) if end > near else float("inf")

    def set_tight(self, tight):
        if tight == self.tight:
            return
        self.tight = tight
        self.tight_pub.publish(Bool(data=tight))
        limit = SpeedLimit()
        limit.percentage = True
        limit.speed_limit = self.tight_speed_percent if tight else 0.0  # 0: no limit
        self.speed_limit_pub.publish(limit)

    def start_unstick(self, why="before retrying"):
        # Get to a pose the planner can start from: first a small turn on the spot onto the
        # nearest primitive heading (it then checks the true footprint), else back (or forwards)
        # 1x, 1.75x or 2.5x unstick_m. False without a map pose or costmap, or with nowhere clear.
        if self.map_x is None or self.map_yaw is None or self.global_costmap is None or self.unstick_m <= 0.0:
            return False
        heading = self.planner_heading(self.map_yaw)
        off_heading = abs(wrap_angle(heading - self.map_yaw)) > 0.05
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
            error = wrap_angle(u["yaw"] - self.map_yaw)
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
        x, y, _ = self.candidates[target][self.target_index]
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
        if (self.global_costmap is not None
                and self.footprint_cells(bx, by, yaw) >= self.candidate_min_lethal_cells):
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
        error = wrap_angle(yaw - self.map_yaw)
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
        nearest = self.nearest_lethal_m()
        if nearest is not None and nearest < self.relocalise_turn_clearance_m:
            self.report(f"Not turning to the AprilTag to re-localise: an obstacle is {nearest:.1f} m away, and "
                        "turning on the spot the rover can slide into it.")
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
        errors = [wrap_angle(bearing + offset - self.map_yaw) for offset in ((0.0, pi) if near else (0.0,))]
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

    """Trench (AUTO_TRENCH)"""
    def start_aligning(self):
        self.state = "ALIGNING"
        self.align_start_s = time.monotonic()
        self.report("At the trench start: turning to face away from its end.")

    def update_aligning(self):
        # Turn on the spot to the trench heading, then dig. Stops short rather than turn into
        # something (see move_blocked), and digs from wherever it got to.
        error = wrap_angle(self.trench_start[2] - self.map_yaw)
        blocked = self.move_blocked(turn=0.15 if error > 0 else -0.15)
        if abs(error) <= self.trench_align_tolerance or blocked or time.monotonic() - self.align_start_s > ALIGN_TIMEOUT_S:
            self.cmd_vel_pub.publish(Twist())
            if abs(error) > self.trench_align_tolerance:
                self.report(f"Couldn't line up with the trench ({abs(error) * 180.0 / pi:.0f} deg off"
                            f"{': an obstacle is in the way' if blocked else ''}). Digging from here.", "warning")
            self.state = "EXCAVATING"
            self.trench_drive_start_s = None
            self.excavation.start(driven=True)
            self.report("Digging the trench: bucket down, reversing to its end.")
            return
        cmd = Twist()
        cmd.angular.z = max(-self.relocalise_turn_speed, min(self.relocalise_turn_speed, 1.5 * error))
        if abs(cmd.angular.z) < 0.15:
            cmd.angular.z = 0.15 if error > 0 else -0.15  # too slow and the skid-steer doesn't turn
        self.cmd_vel_pub.publish(cmd)

    def update_trench(self):
        # Reverse along the trench with the bucket down, steering back onto its line, until past
        # its end, held up, or about to reverse into something
        (sx, sy, _), (ex, ey) = self.trench_start, self.trench_end
        length = hypot(ex - sx, ey - sy)
        ux, uy = (ex - sx) / length, (ey - sy) / length
        along = (self.map_x - sx) * ux + (self.map_y - sy) * uy
        if self.trench_drive_start_s is None:
            self.trench_drive_start_s = time.monotonic()
        elapsed = time.monotonic() - self.trench_drive_start_s
        why = None
        if along >= length:
            why = f"Reached the end of the trench ({length:.1f} m)."
        elif elapsed > 2.0 * length / self.trench_speed:
            why = f"Trench timed out after {elapsed:.0f} s, {max(along, 0.0):.1f} of {length:.1f} m along."
        elif self.move_blocked(forward=-0.1):
            why = f"Stopped the trench {max(along, 0.0):.1f} of {length:.1f} m along: something is behind the rover."
        if why:
            self.cmd_vel_pub.publish(Twist())
            self.report(why, "info" if along >= length else "warning")
            self.excavation.finish_dig()
            return
        # Face directly away from a point a little further along the line, so reversing heads for it
        ahead = min(length, max(along, 0.0) + TRENCH_LOOKAHEAD_M)
        tx, ty = sx + ahead * ux, sy + ahead * uy
        error = wrap_angle(atan2(self.map_y - ty, self.map_x - tx) - self.map_yaw)
        cmd = Twist()
        cmd.linear.x = -self.trench_speed
        cmd.angular.z = max(-0.3, min(0.3, 1.5 * error))
        self.cmd_vel_pub.publish(cmd)

    def trench_left_m(self):
        (sx, sy, _), (ex, ey) = self.trench_start, self.trench_end
        length = hypot(ex - sx, ey - sy)
        along = ((self.map_x - sx) * (ex - sx) + (self.map_y - sy) * (ey - sy)) / length
        return max(0.0, length - along)

    def start_sequence(self, target):
        # Digging or dumping starts here, so the next leg to this zone moves on to another candidate
        self.next_candidate[target] = self.target_index + 1
        if target == "EXCAVATION" and self.trench_mode:
            self.start_aligning()
        elif target == "EXCAVATION":
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
        if sequence is self.excavation and sequence.driven:
            return ("TRENCH" if sequence.state.name == "DIG" else sequence.state.name), None
        if sequence.state.name in ("DIG", "DUMP_REGOLITH") and start_s is not None:
            remaining = round(max(0.0, duration_s - (now_s - start_s)), 1)
        return sequence.state.name, remaining

    def publish_state(self, heartbeat=False):
        # Publish when something changed, and every second from the steady clock regardless
        localised, loc_reason, fix_age, drive = self.localisation_status()
        phase, remaining = self.sequence_phase()
        target = None
        if self.state not in ("IDLE", "RELOCALISING", "BACKING_OUT") and self.target_index is not None:
            kind = "EXCAVATION" if self.state in ("NAVIGATING_TO_EXCAVATION", "ALIGNING", "EXCAVATING") else "DEPOSITION"
            x, y, yaw = self.candidates[kind][self.target_index]
            target = {"kind": kind, "index": self.target_index + 1, "count": len(self.candidates[kind]),
                      "x": x, "y": y, "yaw": yaw}
        state = {
            "state": self.state,
            "mode": self.mode,
            "cycle": self.auto_cycle if self.mode in ("AUTO", "AUTO_TRENCH") else None,
            "phase": phase,
            "remaining_s": remaining,
            "trench_left_m": (round(self.trench_left_m(), 1) if phase == "TRENCH" and self.map_x is not None
                              else None),
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
