#!/usr/bin/env python3
"""

Mission Flow:
    TAKEOFF → LAP → SCAN → [LOCALIZE → DELIVER → DEPLOY] x N → SCAN → RTL

The scan always runs to the end of its plan, including after the last payload
is delivered, so the stitch covers the whole region rather than stopping
wherever the final object happened to be.

Each detected object triggers the sequence:
    1. Stop drone (LOITER)
    2. Localize object using camera intrinsics
    3. Fly to localized object
    4. Drop the payload: plate release + pulsed clamp (see payload.py)
    5. Resume scanning from where we left off

Stitching runs during RTL - the stitching node listens for "return" state.

FUTURE CHANGES:
    1. localizer must respond to get_object_locations service when state is "localize":
       - Must return localized GPS coordinates when called
       - First location in list should be the most recent detection
    2. stitching node should collect its own frames
    

"""

# Imports
import json
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandLong, ParamSetV2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterValue, ParameterType
from bv_msgs.srv import LocalizeObject
from bv_msgs.msg import ConfirmedDetection, ObjectLocations
from .mission_logger import MissionLogger
from .approval_gate import ApprovalGate
from .mission_config import (
    expand_lap_route,
    mission_config_path,
    select_takeoff_waypoint,
)
from .scan_plan import load_scan_plan
from .payload import (
    DropSequence,
    actuator_params,
    altitude_mismatch_warning,
    load_payload_config,
    payload_for_class,
)

# Mission configuration
# Drop sequence tick. The fastest clamp toggle is 100 ms, so 20 ms keeps each
# toggle within a fifth of its interval; the 0.5 s main timer is far too coarse.
DEPLOY_TICK_SEC = 0.02
CLASS_NAMES = ("person", "tent")


# State constants
STATE_TAKEOFF  = "takeoff"
STATE_LAP      = "lap"
STATE_SCAN_TRANSIT = "scan_transit"
STATE_SCAN     = "scan"
STATE_LOCALIZE = "localize"
STATE_DELIVER  = "deliver"
STATE_DEPLOY   = "deploy"
STATE_RTL      = "return"

# Floor for Approval_timeout_sec. Once localization succeeds, this is the only
# exit while waiting for an operator verdict. Clamping at the config boundary
# keeps a dead ground station from leaving the aircraft in an indefinite loiter.
MIN_APPROVAL_TIMEOUT_SEC = 10.0
LOCALIZATION_TIMEOUT_SEC = 15.0


def confirmation_rejection_reason(message, scan_started_ns, handled_ids):
    """Return why a confirmation cannot start work, or ``None`` if fresh."""
    if message.class_id < 0 or not message.detection_id:
        return 'invalid'
    if message.detection_id in handled_ids:
        return 'duplicate'
    stamp_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    if stamp_ns < scan_started_ns:
        return 'stale'
    return None


# Mavlink constants
MAV_CMD_NAV_WAYPOINT = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3


# Mission runner class
class MissionRunner(Node):
    """
    Main mission orchestrator node.
    
    Manages the finite state machine that controls the drone through
    all phases of the mission.
    """

    # Initialization
    def __init__(self):
        super().__init__('mission_runner')
        
        # Load configuration from YAML
        self.load_config_from_yaml()
        self.log = MissionLogger('mission')

        # Initialize state variables
        self.init_state_variables()
        
        # Set up ROS interfaces (publishers, subscribers, service clients)
        self.setup_ros_interfaces()
        
        # Wait for GPS fix to establish home position
        self.wait_for_gps_fix()
        
        # Wait for required services to become available
        self.wait_for_services()

        # Put the payload servos at hold/rest before arming. PX4 outputs a
        # peripheral's disarmed PWM until armed, then this stored command.
        self.send_payload_rest_positions()

        # Start the mission
        self.get_logger().info("=" * 50)
        self.get_logger().info("MISSION STARTING")
        self.get_logger().info(f"Objects to find: {self.num_objects_to_find}")
        self.get_logger().info("=" * 50)
        self.log.event('STARTUP',
            f"num_objects={self.num_objects_to_find}, "
            f"scan_wps={len(self.scan_waypoints)}, "
            f"lap_wps={len(self.lap_waypoints)}, "
            f"scan_transit_vel={self.scan_transit_velocity}m/s, "
            f"scan_vel={self.scan_velocity}m/s, "
            f"deliver_vel={self.deliver_velocity}m/s, "
            f"rtl_vel={self.rtl_velocity}m/s, "
            f"scan_tol={self.scan_tolerance}m, "
            f"deliver_tol={self.deliver_tolerance}m")

        self.enter_takeoff_state()
        
        # Start main timer loop
        self.main_timer = self.create_timer(0.5, self.main_timer_callback)

    def load_config_from_yaml(self):
        """Load waypoints and parameters from the selected mission config."""
        config_path = mission_config_path()
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Waypoint lists
        lap_route = config.get('points', [])
        lap_count = int(config.get('lap_count', 1))
        self.lap_waypoints = expand_lap_route(lap_route, lap_count)
        scan_plan = load_scan_plan()
        self.scan_waypoints = scan_plan.waypoints
        self.takeoff_waypoint = select_takeoff_waypoint(
            self.lap_waypoints, self.scan_waypoints)
        
        # Velocity parameters (m/s)
        self.lap_velocity = config.get('Lap_velocity', 5.0)
        self.scan_transit_velocity = config.get(
            'Scan_transit_velocity', self.lap_velocity)
        self.scan_velocity = config.get('Scan_velocity', 2.5)
        self.takeoff_velocity = config.get(
            'Takeoff_velocity', self.scan_velocity)
        self.deliver_velocity = config.get('Deliver_velocity', 5.0)
        self.rtl_velocity = config.get('RTL_velocity', self.scan_velocity)
        
        # Waypoint acceptance tolerance (meters)
        self.lap_tolerance = config.get('Lap_tolerance', 2.0)
        self.scan_tolerance = config.get('Scan_tolerance', 1.0)
        self.deliver_tolerance = config.get('Deliver_tolerance', 1.0)

        # Human-in-the-loop approval gate
        self.approval_required = bool(config.get('Approval_required', False))
        configured_timeout = float(config.get('Approval_timeout_sec', 180.0))
        self.approval_timeout_sec = max(
            MIN_APPROVAL_TIMEOUT_SEC, configured_timeout)
        if self.approval_timeout_sec != configured_timeout:
            self.get_logger().warn(
                f"Approval_timeout_sec={configured_timeout:g}s raised to "
                f"{self.approval_timeout_sec:g}s: the approval timeout is the "
                f"only exit from the localize state, so it cannot be disabled "
                f"or set arbitrarily short")

        # Payload release. None when payload.enabled is false. Raises on any
        # position the flight controller's PWM range cannot reach, so a bad
        # config stops the mission on the ground rather than at the drop.
        self.payload = load_payload_config(config)
        if self.payload is None:
            self.get_logger().info(
                "Payload DISABLED - DEPLOY will skip the drop")
        else:
            self._log_payload_config(self.payload)
            warning = altitude_mismatch_warning(
                self.payload, scan_plan.altitude_m)
            if warning:
                self.get_logger().warn(warning)

        # Required mission parameters
        if 'num_objects' not in config:
            raise ValueError("mission config is missing required key: num_objects")
        self.num_objects_to_find = int(config['num_objects'])
        
        self.get_logger().info(f"Loaded {len(self.lap_waypoints)} lap waypoints")
        self.get_logger().info(
            f"Prepared {len(self.scan_waypoints)} scan waypoints across "
            f"{scan_plan.row_count} row(s): capture_spacing="
            f"{scan_plan.capture_spacing_m:.2f}m, "
            f"row_spacing={scan_plan.row_spacing_m:.2f}m"
        )

    def init_state_variables(self):
        """Initialize all state tracking variables."""
        # Current FSM state
        self.current_state = STATE_TAKEOFF
        
        # Transition lock to prevent double-transitions
        self.is_transitioning = False
        
        # Mission progress
        self.objects_delivered_count = 0
        self.in_auto_mission = False
        self.rtl_completed = False
        
        # Waypoint tracking
        self.expected_final_waypoint_index = None
        self.active_waypoint_list = []
        self.desired_velocity = self.lap_velocity
        self.last_waypoint_reached = None  # Track in case event fires during transition
        self.last_processed_waypoint = -1  # Prevent duplicate waypoint processing
        
        # Scan resume tracking
        self.scan_waypoint_index_on_detection = 0
        self.last_reached_scan_waypoint = -1
        self.loiter_resume_coords = None  # (lat, lon, alt) to return to after delivery
        self.scan_resume_waypoint_offset = 0  # Offset when loiter point is prepended

        # Continuous GPS tracking
        self.current_lat = None
        self.current_lon = None
        
        # Current target for delivery (set by localization)
        self.current_target_coords = None  # (lat, lon, alt)
        self.current_target_class_id = None
        
        # Confirmed detection class from filtering_node (set on object detection)
        self.confirmed_detection_class_id = -1
        self.confirmed_detection_id = ''
        self.confirmed_detection_coords = None
        self.handled_confirmation_ids = set()
        self.scan_started_ns = 0
        self.scan_route_pending = False
        self._pending_scan_confirmation = None
        self._localize_retry_timer = None
        self._localization_timeout_timer = None
        self._localization_request_id = 0
        self.localization_retry_count = 0

        # Operator approval gate; None when flying fully autonomously
        self.approval_gate = None

        # Active payload drop; None outside DEPLOY
        self._drop_sequence = None
        self._drop_started = 0.0
        self._drop_last_sent = None
        self._deploy_timer = None
        
        # Home position (set by GPS)
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        self.has_armed = False

    def setup_ros_interfaces(self):
        """Set up all ROS publishers, subscribers, and service clients."""
        
        
        # QoS Profiles
        
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        
        best_effort_qos = QoSProfile(depth=10)
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        
        # Publishers
        
        # Mission state - consumed by stitching.py and vision nodes
        self.mission_state_pub = self.create_publisher(
            String,
            '/mission_state',
            qos_profile=transient_local_qos
        )
        self.path_progress_pub = self.create_publisher(
            String,
            '/path_progress',
            qos_profile=transient_local_qos
        )
        self.deployed_object_pub = self.create_publisher(
            ObjectLocations,
            '/deployed_object_locations',
            qos_profile=reliable_qos
        )
        
        
        # Subscribers
        
        # GPS for home position
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.on_gps_received,
            qos_profile=best_effort_qos
        )
        
        # Waypoint reached notifications
        self.waypoint_reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.on_waypoint_reached,
            qos_profile=reliable_qos
        )
        
        # Vehicle state (flight mode)
        self.vehicle_state_sub = self.create_subscription(
            MavState,
            '/mavros/state',
            self.on_vehicle_state_changed,
            qos_profile=reliable_qos
        )
        
        # Object detection trigger from filtering_node (confirmed M-of-N detection)
        # Carries a unique ID and the scan-time candidate position so delayed
        # confirmations and same-class detections can be disambiguated.
        self.object_detected_sub = self.create_subscription(
            ConfirmedDetection,
            '/global_obj_dets',
            self.on_object_detected,
            qos_profile=reliable_qos
        )

        # Locations the operator rejected, so filtering_node stops
        # re-confirming the same false positive.
        self.rejected_object_pub = self.create_publisher(
            ObjectLocations,
            '/rejected_object_locations',
            qos_profile=reliable_qos
        )

        
        # Service Clients
        
        self.waypoint_push_client = self.create_client(
            WaypointPush, 
            '/mavros/mission/push'
        )
        self.arm_client = self.create_client(
            CommandBool, 
            '/mavros/cmd/arming'
        )
        self.set_mode_client = self.create_client(
            SetMode, 
            '/mavros/set_mode'
        )
        self.command_client = self.create_client(
            CommandLong, 
            '/mavros/cmd/command'
        )
        self.param_set_client = self.create_client(
            ParamSetV2, 
            '/mavros/param/set'
        )
        self.localize_object_client = self.create_client(
            LocalizeObject,
            'localize_object'
        )

        if self.approval_required:
            self.approval_gate = ApprovalGate(
                self, self.approval_timeout_sec, self.log,
                on_callback_error=self._on_approval_callback_failed)
            self.get_logger().info(
                f"Human-in-the-loop approval ENABLED "
                f"(timeout={self.approval_timeout_sec:.0f}s, fails open)")
        else:
            self.get_logger().info(
                "Human-in-the-loop approval disabled - flying autonomously")

    def wait_for_gps_fix(self):
        """Block until we receive a GPS fix for the home position."""
        self.get_logger().info("Waiting for GPS fix...")
        
        while rclpy.ok() and self.home_lat is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Keep GPS subscription alive for continuous position tracking
        
        self.get_logger().info(
            f"Home position set: lat={self.home_lat:.6f}, "
            f"lon={self.home_lon:.6f}, alt={self.home_alt:.2f}m"
        )

    def wait_for_services(self):
        """Wait for all required MAVROS services to become available."""
        services = [
            (self.waypoint_push_client, '/mavros/mission/push'),
            (self.arm_client, '/mavros/cmd/arming'),
            (self.set_mode_client, '/mavros/set_mode'),
            (self.command_client, '/mavros/cmd/command'),
            (self.param_set_client, '/mavros/param/set'),
            (self.localize_object_client, 'localize_object'),
        ]
        
        for client, name in services:
            has_logged_wait = False
            while not client.wait_for_service(timeout_sec=1.0):
                if not has_logged_wait:
                    self.get_logger().info(f"Waiting for service: {name}")
                    has_logged_wait = True
        
        self.get_logger().info("All services available")

    # State machine core
    def handle_state_completion(self):
        """
        Called when the current state's objective is complete.
        Determines and initiates the appropriate next state.
        """
        self.get_logger().info(f"State '{self.current_state}' complete")
        
        if self.current_state == STATE_TAKEOFF:
            if self.lap_waypoints:
                self.enter_lap_state()
            else:
                self.enter_scan_state()
            
        elif self.current_state == STATE_LAP:
            self.enter_scan_transit_state()

        elif self.current_state == STATE_SCAN_TRANSIT:
            self.activate_scan_state()
            
        elif self.current_state == STATE_SCAN:
            # Scan waypoints finished - proceed to RTL
            self.get_logger().info("Scan complete")
            self.log.event('SCAN_COMPLETE',
                f"objects_delivered={self.objects_delivered_count}/{self.num_objects_to_find}, action=rtl")
            self.enter_rtl_state()
            
        elif self.current_state == STATE_LOCALIZE:
            # Handled by service callback, not waypoint completion
            pass
            
        elif self.current_state == STATE_DELIVER:
            self.enter_deploy_state()
            
        elif self.current_state == STATE_DEPLOY:
            # Handled by timer, not waypoint completion
            pass
            
    def publish_mission_state(self):
        """Publish current state to /mission_state topic."""
        msg = String()
        msg.data = self.current_state
        self.mission_state_pub.publish(msg)

    def publish_path_progress(self, phase, completed, total):
        """Publish absolute lap/scan progress despite MAVROS mission renumbering."""
        completed = max(0, min(int(completed), int(total)))
        target = completed + 1 if completed < total else None
        route = self.lap_waypoints if phase == 'lap' else self.scan_waypoints
        segment_start = None
        segment_end = None
        if target is not None and completed < len(route):
            segment_end = [float(route[completed][0]), float(route[completed][1])]
            if completed > 0:
                segment_start = [
                    float(route[completed - 1][0]),
                    float(route[completed - 1][1]),
                ]
            elif self.current_lat is not None and self.current_lon is not None:
                segment_start = [float(self.current_lat), float(self.current_lon)]
        msg = String()
        msg.data = json.dumps({
            'phase': phase,
            'completed': completed,
            'target': target,
            'total': int(total),
            'segment_start': segment_start,
            'segment_end': segment_end,
        }, separators=(',', ':'))
        self.path_progress_pub.publish(msg)

    # State entry methods
    def enter_takeoff_state(self):
        """
        Arm and fly to the first waypoint of the enabled mission route.

        A mission with laps flies to its first lap waypoint. A zero-lap mission
        flies directly to the first generated scan waypoint.
        """
        self.current_state = STATE_TAKEOFF
        self.is_transitioning = True
        self.desired_velocity = (
            self.lap_velocity if self.lap_waypoints else self.takeoff_velocity)
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: TAKEOFF")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'takeoff')

        if self.takeoff_waypoint is None:
            self.get_logger().error("No lap or scan waypoint defined for takeoff!")
            return

        takeoff_tolerance = (
            self.lap_tolerance if self.lap_waypoints else self.scan_tolerance)
        route_name = 'lap' if self.lap_waypoints else 'scan'
        self.get_logger().info(
            f"Takeoff destination is the first {route_name} waypoint")
        route = self.lap_waypoints if self.lap_waypoints else self.scan_waypoints
        self.publish_path_progress(route_name, 0, len(route))
        
        self.active_waypoint_list = self.build_waypoint_list(
            [self.takeoff_waypoint],
            takeoff_tolerance
        )
        self.expected_final_waypoint_index = 0
        self.push_mission_to_autopilot()

    def enter_lap_state(self):
        """
        Fly a reconnaissance lap around the competition field.
        Vision nodes are idle or passively observing.
        """
        self.current_state = STATE_LAP
        self.is_transitioning = True
        self.desired_velocity = self.lap_velocity
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: LAP")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'takeoff -> lap')
        self.publish_path_progress('lap', 0, len(self.lap_waypoints))

        self.active_waypoint_list = self.build_waypoint_list(
            self.lap_waypoints,
            self.lap_tolerance,
            pass_through_ratio=1.0  # Fly through without stopping
        )
        self.expected_final_waypoint_index = len(self.active_waypoint_list) - 1
        self.push_mission_to_autopilot()

    def enter_scan_state(self):
        """
        Fly the scan pattern while vision node actively detects objects.
        Frames are saved for later stitching.
        Will be interrupted by on_object_detected() callback.
        """
        self.current_state = STATE_SCAN
        self.scan_route_pending = True
        self._pending_scan_confirmation = None
        self.is_transitioning = True
        self.scan_started_ns = self.get_clock().now().nanoseconds
        self.publish_mission_state()
        self.desired_velocity = self.scan_velocity
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: SCAN")
        self.get_logger().info(
            f"Resuming from waypoint index: {self.scan_waypoint_index_on_detection}"
        )
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', f"-> scan (resume from wp {self.scan_waypoint_index_on_detection})")
        self.publish_path_progress(
            'scan', self.scan_waypoint_index_on_detection,
            len(self.scan_waypoints))

        # Get remaining scan waypoints from where we left off
        remaining_scan_points = self.scan_waypoints[self.scan_waypoint_index_on_detection:]

        # If resuming after delivery, fly back to loiter position first
        if self.loiter_resume_coords is not None:
            self.get_logger().info(
                f"Prepending loiter return point: "
                f"lat={self.loiter_resume_coords[0]:.6f}, lon={self.loiter_resume_coords[1]:.6f}"
            )
            remaining_scan_points = [self.loiter_resume_coords] + remaining_scan_points
            self.scan_resume_waypoint_offset = 1
            self.loiter_resume_coords = None  # Clear after use
        else:
            self.scan_resume_waypoint_offset = 0

        if not remaining_scan_points:
            self.scan_route_pending = False
            self.get_logger().info("No remaining scan waypoints")
            self.handle_state_completion()
            return

        self.active_waypoint_list = self.build_waypoint_list(
            remaining_scan_points,
            self.scan_tolerance,
            pass_through_ratio=1.0  # Continuous flight for scanning
        )
        self.expected_final_waypoint_index = len(self.active_waypoint_list) - 1
        # -1 means this newly uploaded segment has not reached an actual scan
        # waypoint yet.  A prepended delivery-return point also maps to -1.
        self.last_reached_scan_waypoint = -1
        self.push_mission_to_autopilot()

    def enter_scan_transit_state(self):
        """Fly the complete scan route at transit speed until its first waypoint."""
        self.current_state = STATE_SCAN_TRANSIT
        self.scan_route_pending = False
        self._pending_scan_confirmation = None
        self.is_transitioning = True
        self.desired_velocity = self.scan_transit_velocity
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        self.scan_resume_waypoint_offset = 0

        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: SCAN TRANSIT")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'lap -> scan_transit')
        self.publish_mission_state()
        self.publish_path_progress('scan', 0, len(self.scan_waypoints))

        self.active_waypoint_list = self.build_waypoint_list(
            self.scan_waypoints,
            self.scan_tolerance,
            pass_through_ratio=1.0,
        )
        self.expected_final_waypoint_index = 0
        self.last_reached_scan_waypoint = -1
        self.push_mission_to_autopilot()

    def activate_scan_state(self):
        """Begin scanning at waypoint zero without replacing the active route."""
        self.current_state = STATE_SCAN
        self.scan_route_pending = False
        self._pending_scan_confirmation = None
        self.scan_started_ns = self.get_clock().now().nanoseconds
        self.is_transitioning = False
        self.desired_velocity = self.scan_velocity
        self.expected_final_waypoint_index = len(self.active_waypoint_list) - 1
        self.last_reached_scan_waypoint = 0
        self.last_processed_waypoint = 0

        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: SCAN")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'scan_transit -> scan')
        self.publish_path_progress('scan', 1, len(self.scan_waypoints))
        self.publish_mission_state()
        self.set_velocity(self.scan_velocity)

    def _resume_scan_in_place(self, reason):
        """Continue toward the next scan endpoint without retracing the row."""
        self.scan_waypoint_index_on_detection += max(
            0, self.last_reached_scan_waypoint + 1)
        self.loiter_resume_coords = None
        self.get_logger().info(
            f"Resuming scan in place toward waypoint "
            f"{self.scan_waypoint_index_on_detection} ({reason})")
        self.log.event(
            'SCAN_RESUME',
            f"from_current_position=true, "
            f"toward_wp={self.scan_waypoint_index_on_detection}/"
            f"{len(self.scan_waypoints)}, reason={reason}")
        self.enter_scan_state()

    def enter_localize_state(self):
        """
        Hold position while the localizer computes the object's GPS coordinates
        using camera intrinsics and drone pose.
        """
        self.current_state = STATE_LOCALIZE
        self.scan_route_pending = False
        self._pending_scan_confirmation = None
        self.is_transitioning = True
        self.localization_retry_count = 0
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: LOCALIZE")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'scan -> localize')

        # Publish state so vision node knows to keep pipeline running
        self.publish_mission_state()

        if self._localization_timeout_timer is not None:
            self._localization_timeout_timer.cancel()
        self._localization_timeout_timer = self.create_timer(
            LOCALIZATION_TIMEOUT_SEC, self._on_localization_timeout)
        
        # Request object location from vision_node via localize_object service
        self.request_localization_from_vision()

    def enter_deliver_state(self):
        """
        Fly to the localized object position for payload delivery.
        """
        # Cancel any pending localization retry timer
        if self._localize_retry_timer is not None:
            self._localize_retry_timer.cancel()
            self._localize_retry_timer = None
        if self._localization_timeout_timer is not None:
            self._localization_timeout_timer.cancel()
            self._localization_timeout_timer = None

        self.current_state = STATE_DELIVER
        self.publish_mission_state()
        self.is_transitioning = True
        self.desired_velocity = self.deliver_velocity
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: DELIVER")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'localize -> deliver')

        if self.current_target_coords is None:
            self.get_logger().error("No target coordinates available!")
            self.enter_rtl_state()
            return
        
        lat, lon, alt = self.current_target_coords
        cls_name = CLASS_NAMES[self.current_target_class_id] if self.current_target_class_id is not None and 0 <= self.current_target_class_id < len(CLASS_NAMES) else 'unknown'
        self.get_logger().info(f"Flying to {cls_name}: lat={lat:.6f}, lon={lon:.6f}")
        self.log.event('DELIVER_TARGET',
            f"lat={lat:.6f}, lon={lon:.6f}, class={cls_name}({self.current_target_class_id})")

        self.active_waypoint_list = self.build_waypoint_list(
            [self.current_target_coords],
            self.deliver_tolerance
        )
        self.expected_final_waypoint_index = 0
        self.push_mission_to_autopilot()

    def enter_deploy_state(self):
        """
        Drop the payload for this target while holding over it.

        Runs the plate + clamp sequence from payload.py on its own fast timer;
        on_deploy_complete resumes the scan once the sequence ends.
        """
        self.current_state = STATE_DEPLOY
        self.publish_mission_state()
        self.is_transitioning = False  # No waypoint transition for deploy

        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: DEPLOY")
        self.get_logger().info(
            f"Deploying payload {self.objects_delivered_count + 1}/{self.num_objects_to_find}"
        )
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', 'deliver -> deploy')

        if self.payload is None:
            self.get_logger().info("Payload disabled - skipping drop")
            self.log.event('DEPLOY_SKIPPED', 'reason=payload_disabled')
            self.on_deploy_complete()
            return

        payload = payload_for_class(self.current_target_class_id)
        if payload is None:
            # Guessing would drop the wrong object on the wrong target.
            self.get_logger().error(
                f"No payload for class {self.current_target_class_id} - "
                f"skipping drop")
            self.log.event(
                'DEPLOY_SKIPPED',
                f"reason=unknown_class, class={self.current_target_class_id}")
            self.on_deploy_complete()
            return

        self._drop_sequence = DropSequence(self.payload, payload)
        self._drop_started = time.monotonic()
        self._drop_last_sent = None
        self.get_logger().info(
            f"[DEPLOY] dropping {payload}: plate -> "
            f"{self._drop_sequence.plate_us:g} us, clamp pulsing "
            f"{self._drop_sequence.clamped_us:g}/{self.payload.unclamped_us:g} us "
            f"for {self.payload.total_duration_s:.2f}s")
        self.log.event(
            'DEPLOY_START',
            f"payload={payload}, plate_us={self._drop_sequence.plate_us:g}, "
            f"clamped_us={self._drop_sequence.clamped_us:g}, "
            f"duration={self.payload.total_duration_s:.2f}s")

        # First command now; the timer takes it from there.
        self._on_deploy_tick()
        if self._drop_sequence is not None:
            self._deploy_timer = self.create_timer(
                DEPLOY_TICK_SEC, self._on_deploy_tick)

    def _on_deploy_tick(self):
        """Advance the drop: command any change, finish when it ends.

        Timing follows the clock, never command replies: a slow or failed
        command is logged by on_payload_command_complete, but the drop always
        ends on schedule and the mission moves on.
        """
        sequence = self._drop_sequence
        if sequence is None:
            return
        if self.current_state != STATE_DEPLOY:
            # RTL or another interruption already took over.
            self._stop_drop()
            return

        plate_us, clamp_us, done = sequence.positions_at(
            time.monotonic() - self._drop_started)
        if (plate_us, clamp_us) != self._drop_last_sent:
            # Always both servos: every clamp toggle re-asserts the plate's
            # release, so one lost packet cannot leave the payload held.
            self.send_payload_actuators(plate_us, clamp_us)
            self._drop_last_sent = (plate_us, clamp_us)

        if done:
            self._stop_drop()
            self.get_logger().info("[DEPLOY] drop sequence complete")
            self.log.event('DEPLOY_DONE', f"payload={sequence.payload}")
            self.on_deploy_complete()

    def _stop_drop(self):
        """Cancel the drop timer and forget the active sequence."""
        self._drop_sequence = None
        timer = self._deploy_timer
        self._deploy_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def enter_rtl_state(self, command_mode=True):
        """
        Stop mission work and return; skip the command if RTL is already observed.
        """
        if self.approval_gate is not None and self.approval_gate.is_pending():
            self.approval_gate.cancel('rtl')

        # Interrupted mid-drop: stop pulsing and leave the clamp open.
        if getattr(self, '_drop_sequence', None) is not None:
            self._stop_drop()
            self.send_payload_actuators(clamp_us=self.payload.unclamped_us)
            self.log.event('DEPLOY_ABORTED', 'reason=rtl')

        for name in ('_localize_timer', '_localize_retry_timer',
                     '_localization_timeout_timer', '_velocity_delay_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                setattr(self, name, None)

        self.current_state = STATE_RTL
        self.scan_route_pending = False
        self._pending_scan_confirmation = None
        self.in_auto_mission = False
        self.desired_velocity = self.rtl_velocity
        self.publish_mission_state()
        self.is_transitioning = True
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: RTL")
        self.get_logger().info("-" * 40)
        self.log.event('STATE_CHANGE', '-> rtl')
        self.set_velocity(self.rtl_velocity)

        if command_mode:
            self.set_flight_mode("AUTO.RTL")

    # Mavros utilities
    def build_waypoint_list(self, points, tolerance, pass_through_ratio=0.0):
        """
        Build a list of MAVROS Waypoint messages from coordinate tuples.
        
        Args:
            points: List of (lat, lon, alt) tuples
            tolerance: Waypoint acceptance radius in meters
            pass_through_ratio: 0.0 = stop at waypoint, 1.0 = fly through
            
        Returns:
            List of Waypoint messages
        """
        waypoints = []
        
        # Determine hold time at each waypoint based on state
        if self.current_state == STATE_SCAN:
            hold_time = 0.0  # Make this 1.0 for the drone to stop at waypoints
        elif self.current_state == STATE_DELIVER:
            hold_time = 1.0  # Pause before deploy
        else:
            hold_time = 0.0  # No pause
        
        for i, (lat, lon, alt) in enumerate(points):
            wp = Waypoint()
            wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = MAV_CMD_NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.param1 = hold_time                      # Hold time (seconds)
            wp.param2 = tolerance                      # Acceptance radius
            wp.param3 = tolerance * pass_through_ratio # Pass-through radius
            wp.param4 = float('nan')                   # Yaw (NaN = auto)
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            waypoints.append(wp)
        
        return waypoints

    def push_mission_to_autopilot(self):
        """Upload the current waypoint list to the autopilot."""
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = self.active_waypoint_list
        
        self.get_logger().info(
            f"Pushing {len(self.active_waypoint_list)} waypoint(s) to autopilot..."
        )
        
        future = self.waypoint_push_client.call_async(request)
        future.add_done_callback(self.on_waypoint_push_complete)

    def arm_vehicle(self):
        """Arm the vehicle."""
        request = CommandBool.Request()
        request.value = True
        
        self.get_logger().info("Arming vehicle...")
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.on_arm_complete)

    def set_flight_mode(self, mode):
        """
        Set the vehicle's flight mode.
        
        Args:
            mode: Mode string (e.g., "AUTO.MISSION", "AUTO.RTL", "AUTO.LOITER")
        """
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        
        self.get_logger().info(f"Setting flight mode: {mode}")
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(
            lambda completed, requested_mode=mode:
                self.on_set_mode_complete(completed, requested_mode)
        )

    def set_velocity(self, speed):
        """
        Set the vehicle's maximum horizontal velocity.
        
        Args:
            speed: Velocity in m/s
        """
        request = ParamSetV2.Request()
        request.force_set = False
        request.param_id = 'MPC_XY_VEL_ALL'
        
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = float(speed)
        request.value = param_value
        
        future = self.param_set_client.call_async(request)
        future.add_done_callback(self.on_velocity_set_complete)

    def send_payload_actuators(self, plate_us=None, clamp_us=None):
        """
        Command the payload servos through PX4's MAV_CMD_DO_SET_ACTUATOR.

        Positions are pulse widths in microseconds; a servo left as None is
        not changed.

        Sent as a broadcast so MAVROS does not wait for an ACK. MAVROS refuses
        a command while the previous one of the same type awaits its ACK (up
        to a 5 s timeout), so one lost ACK would otherwise freeze the clamp
        mid-drop. PX4 accepts broadcast commands (target 0/0).
        """
        command, params = actuator_params(self.payload, plate_us, clamp_us)
        request = CommandLong.Request()
        request.broadcast = True
        request.command = command
        request.confirmation = 0
        (request.param1, request.param2, request.param3, request.param4,
         request.param5, request.param6, request.param7) = params
        future = self.command_client.call_async(request)
        future.add_done_callback(self.on_payload_command_complete)

    def send_payload_rest_positions(self):
        """Plate to hold, clamp open. No-op with the payload disabled."""
        if self.payload is None:
            return
        self.send_payload_actuators(
            self.payload.plate_hold_us, self.payload.unclamped_us)

    def on_payload_command_complete(self, future):
        """Log a payload command MAVROS could not send. Never raises."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - a callback must not raise
            self.get_logger().warn(f"Payload servo command failed: {exc!r}")
            return
        if not response.success:
            self.get_logger().warn(
                f"Payload servo command rejected (result={response.result})")

    def _log_payload_config(self, payload):
        """Log every configured servo position."""
        self.get_logger().info(
            f"Payload ENABLED: plate=actuator set "
            f"{payload.plate.actuator_set}, clamp=actuator set "
            f"{payload.clamp.actuator_set}, drop={payload.drop_ft:g}ft over "
            f"{payload.total_duration_s:.2f}s")
        for servo, name, pulse in payload.positions():
            self.get_logger().info(f"  {servo.name} {name}: {pulse:g} us")

    def request_localization_from_vision(self):
        """Request object localization from vision node."""
        request = LocalizeObject.Request()
        request.target_class_id = self.confirmed_detection_class_id
        request.detection_id = self.confirmed_detection_id
        if self.confirmed_detection_coords is not None:
            request.candidate_latitude = float(self.confirmed_detection_coords[0])
            request.candidate_longitude = float(self.confirmed_detection_coords[1])
        # Only pay for the annotated crop when a human will actually look at it.
        # vision_node cannot see Approval_required, so the flag rides the request:
        # with the gate disabled the localize service does exactly the work it
        # did before this feature existed.
        request.want_crop = self.approval_gate is not None
        target_name = (
            CLASS_NAMES[request.target_class_id]
            if 0 <= request.target_class_id < len(CLASS_NAMES)
            else 'any object'
        )
        
        self.get_logger().info(
            f"Requesting localization from vision node for {target_name}..."
        )
        
        future = self.localize_object_client.call_async(request)
        self._localization_request_id += 1
        request_id = self._localization_request_id
        future.add_done_callback(
            lambda completed, expected_id=request_id:
                self.on_vision_localization_complete(completed, expected_id)
        )

    def _on_localization_timeout(self):
        """Abandon a localization request that never produced a response."""
        timer = self._localization_timeout_timer
        if timer is None:
            return
        timer.cancel()
        self._localization_timeout_timer = None
        if self.current_state != STATE_LOCALIZE:
            return

        # Invalidate the outstanding callback so a very late response cannot
        # affect a later localization attempt.
        self._localization_request_id += 1
        if self._localize_retry_timer is not None:
            self._localize_retry_timer.cancel()
            self._localize_retry_timer = None

        self.get_logger().warn(
            f"Localization produced no response within "
            f"{LOCALIZATION_TIMEOUT_SEC:g} seconds - "
            "abandoning this object and resuming scan"
        )
        self.log.event(
            'LOCALIZE_TIMEOUT',
            f'timeout={LOCALIZATION_TIMEOUT_SEC:g}s, '
            'action=abandon_target_resume_scan')
        self.current_target_coords = None
        self.current_target_class_id = None
        self.confirmed_detection_class_id = -1
        self.confirmed_detection_id = ''
        self.confirmed_detection_coords = None
        self._resume_scan_in_place('localization_timeout')

    # Deploy completion
    def on_deploy_complete(self):
        """Called when the servo deploy sequence finishes."""
        if self.current_target_coords is not None:
            lat, lon, _ = self.current_target_coords
            deployed_msg = ObjectLocations()
            deployed_msg.latitude = float(lat)
            deployed_msg.longitude = float(lon)
            deployed_msg.class_id = int(self.current_target_class_id) if self.current_target_class_id is not None else -1
            self.deployed_object_pub.publish(deployed_msg)
            cls_name = CLASS_NAMES[int(deployed_msg.class_id)] if 0 <= deployed_msg.class_id < len(CLASS_NAMES) else 'unknown'
            actual_pos = (self.current_lat, self.current_lon) if self.current_lat is not None else (lat, lon)
            self.log.deploy_complete(
                class_id=deployed_msg.class_id,
                class_name=cls_name,
                target_pos=(lat, lon),
                actual_pos=actual_pos,
                delivered_count=self.objects_delivered_count + 1,
                total=self.num_objects_to_find
            )
            self.log.event('DEPLOYED_LOCATION',
                f"lat={lat:.6f}, lon={lon:.6f}, class={cls_name}({deployed_msg.class_id}), "
                f"ignore_radius=0.0001deg")
            self.get_logger().info(
                f"Published deployed object location: lat={lat:.6f}, lon={lon:.6f}, "
                f"class={cls_name}({deployed_msg.class_id})"
            )

        self.objects_delivered_count += 1
        
        self.get_logger().info(
            f"Payload {self.objects_delivered_count}/{self.num_objects_to_find} delivered!"
        )
        
        # Clear current target
        self.current_target_coords = None
        self.current_target_class_id = None
        self.confirmed_detection_class_id = -1
        self.confirmed_detection_id = ''
        self.confirmed_detection_coords = None
        
        if self.objects_delivered_count >= self.num_objects_to_find:
            self.get_logger().info("All payloads delivered!")

        # Resume the scan whether or not every payload is gone. Returning the
        # moment the last one lands leaves the rest of the scan region unflown,
        # and the panorama is built from the frames captured along it — so the
        # stitch would be missing whatever the aircraft had not reached yet.
        # This still ends the mission on its own: an exhausted plan completes
        # out of STATE_SCAN into RTL, which is what triggers stitching.
        self.scan_waypoint_index_on_detection = (
            self.scan_waypoint_index_on_detection + self.last_reached_scan_waypoint + 1
        )

        # Wrap only while objects are still outstanding. Once everything has
        # been delivered the index is deliberately left past the end, so the
        # scan finishes instead of sweeping the region a second time.
        if (self.scan_waypoint_index_on_detection >= len(self.scan_waypoints)
                and self.objects_delivered_count < self.num_objects_to_find):
            self.scan_waypoint_index_on_detection = 0

        self.get_logger().info(
            f"Resuming scan from waypoint {self.scan_waypoint_index_on_detection}"
        )
        self.log.event('SCAN_RESUME',
            f"from_wp={self.scan_waypoint_index_on_detection}/{len(self.scan_waypoints)}")
        self.enter_scan_state()

    # Callbacks - service responses
    def on_waypoint_push_complete(self, future):
        """Callback when waypoint push to autopilot completes."""
        if self.current_state == STATE_RTL:
            return
        response = future.result()
        
        if not response.success:
            self.get_logger().error(
                f"Waypoint push failed! Only {response.wp_transfered} transferred."
            )
            self.is_transitioning = False
            if self.current_state == STATE_SCAN:
                self.scan_route_pending = False
                self._pending_scan_confirmation = None
            return
        
        self.get_logger().info("Waypoints uploaded successfully")
        if not self.has_armed:
            self.arm_vehicle()
            self.has_armed = True
        else:
            self.set_flight_mode("AUTO.MISSION")

    def on_arm_complete(self, future):
        """Callback when arming completes."""
        if self.current_state == STATE_RTL:
            return
        response = future.result()
        
        if not response.success:
            self.get_logger().error("Arming failed!")
            self.is_transitioning = False
            return
        
        self.get_logger().info("Vehicle armed")
        self.set_flight_mode("AUTO.MISSION")

    def on_set_mode_complete(self, future, requested_mode=None):
        """Callback when mode change completes."""
        response = future.result()
        
        if not response.mode_sent:
            self.get_logger().error("Mode change failed!")
            self.is_transitioning = False
            if self.current_state == STATE_SCAN:
                self.scan_route_pending = False
                self._pending_scan_confirmation = None
            return
        
        if self.current_state == STATE_RTL:
            return

        self.get_logger().info("Flight mode set successfully")

        # A successful LOITER response only confirms that braking has begun.
        # Keep the detection transition locked until the stabilization timer
        # advances the mission from scan to localize.
        if requested_mode == "AUTO.LOITER":
            return

        scan_route_ready = (
            requested_mode == "AUTO.MISSION"
            and self.current_state == STATE_SCAN
            and self.scan_route_pending
        )
        if scan_route_ready:
            self.scan_route_pending = False

        self.in_auto_mission = True
        self.is_transitioning = False
        
        # Set velocity (delayed to let the mode change settle)
        def set_velocity_once():
            if self.current_state == STATE_RTL:
                return
            if hasattr(self, '_velocity_delay_timer') and self._velocity_delay_timer is not None:
                self._velocity_delay_timer.cancel()
                self._velocity_delay_timer = None
            self.set_velocity(self.desired_velocity)
        if hasattr(self, '_velocity_delay_timer') and self._velocity_delay_timer is not None:
            self._velocity_delay_timer.cancel()
        self._velocity_delay_timer = self.create_timer(1.0, set_velocity_once)
        
        # Check if we missed a waypoint completion during transition
        # (This can happen if waypoints are very close together)
        missed_completion = (
            self.last_waypoint_reached is not None and
            self.last_waypoint_reached == self.expected_final_waypoint_index
        )
        
        # Reset before potentially triggering another transition
        self.last_waypoint_reached = None
        
        if missed_completion:
            self.get_logger().warn(
                "Detected missed waypoint completion during transition - processing now"
            )
            self.handle_state_completion()

        if scan_route_ready:
            self._drain_pending_scan_confirmation()

    def _drain_pending_scan_confirmation(self):
        """Handle one confirmation retained while the scan route activated."""
        message = self._pending_scan_confirmation
        self._pending_scan_confirmation = None
        if (message is None or self.current_state != STATE_SCAN
                or self.is_transitioning):
            return
        self.get_logger().info(
            f"Processing confirmation {message.detection_id} retained during "
            "scan route activation"
        )
        self.on_object_detected(message)

    def on_velocity_set_complete(self, future):
        """Callback when velocity parameter change completes."""
        response = future.result()
        
        if response.success:
            self.get_logger().info(
                f"Velocity set to {response.value.double_value:.2f} m/s"
            )
        else:
            self.get_logger().warn("Failed to set velocity parameter")

    def on_vision_localization_complete(self, future, request_id=None):
        """Callback when vision node localization service returns."""
        if self.current_state != STATE_LOCALIZE:
            return
        if (request_id is not None
                and request_id != self._localization_request_id):
            return
        
        response = future.result()
        
        if not response.success:
            # Increment retry counter and decide whether to continue or abandon
            self.localization_retry_count += 1
            if self.localization_retry_count >= 5:
                self.get_logger().warn(
                    "Localization failed 5 times - abandoning this object and resuming scan"
                )
                cls_name = CLASS_NAMES[self.confirmed_detection_class_id] if 0 <= self.confirmed_detection_class_id < len(CLASS_NAMES) else 'unknown'
                self.log.event('LOCALIZE_ABANDONED',
                    f"class={cls_name}({self.confirmed_detection_class_id}), attempts=5, resuming_scan")
                # Stop any pending retry timer
                if self._localize_retry_timer is not None:
                    self._localize_retry_timer.cancel()
                    self._localize_retry_timer = None
                if self._localization_timeout_timer is not None:
                    self._localization_timeout_timer.cancel()
                    self._localization_timeout_timer = None
                # Clear current target/confirmation and resume scanning
                self.current_target_coords = None
                self.current_target_class_id = None
                self.confirmed_detection_class_id = -1
                self.confirmed_detection_id = ''
                self.confirmed_detection_coords = None
                self._resume_scan_in_place('localization_failed')
                return

            self.get_logger().warn(
                f"Localization failed (attempt {self.localization_retry_count}/5) - retrying in 1s..."
            )
            # Cancel any existing retry timer before creating a new one
            if self._localize_retry_timer is not None:
                self._localize_retry_timer.cancel()
            def retry_once():
                if self._localize_retry_timer is not None:
                    self._localize_retry_timer.cancel()
                    self._localize_retry_timer = None
                if self.current_state == STATE_LOCALIZE:
                    self.request_localization_from_vision()
            self._localize_retry_timer = self.create_timer(1.0, retry_once)
            return
        
        # Successful localization - reset retry counter and clean up any retry timer
        self.localization_retry_count = 0
        if self._localize_retry_timer is not None:
            self._localize_retry_timer.cancel()
            self._localize_retry_timer = None
        if self._localization_timeout_timer is not None:
            self._localization_timeout_timer.cancel()
            self._localization_timeout_timer = None

        self.current_target_coords = (response.latitude, response.longitude, response.altitude)
        self.current_target_class_id = int(response.class_id)
        cls_name = (
            CLASS_NAMES[self.current_target_class_id]
            if 0 <= self.current_target_class_id < len(CLASS_NAMES)
            else 'unknown'
        )
        
        self.get_logger().info(
            f"Localized {cls_name} at: lat={response.latitude:.6f}, lon={response.longitude:.6f}, "
            f"alt={response.altitude:.2f}m"
        )
        
        # Proceed to delivery, or hold for an operator verdict first.
        if self.approval_gate is None:
            self.enter_deliver_state()
            return

        crop = bytes(response.annotated_crop.data)
        if not crop:
            self.get_logger().warn(
                "Localization returned no annotated crop - the operator will "
                "have to judge without an image")

        self.approval_gate.request(
            class_id=self.current_target_class_id,
            lat=response.latitude,
            lon=response.longitude,
            alt=response.altitude,
            confidence=response.confidence,
            drone_lat=self.current_lat if self.current_lat is not None else 0.0,
            drone_lon=self.current_lon if self.current_lon is not None else 0.0,
            annotated_crop=crop,
            on_approve=self._on_approval_granted,
            on_reject=self._on_approval_rejected,
        )

    def _on_approval_granted(self):
        """Operator approved, or the timeout expired. Deliver as normal."""
        if self.current_state != STATE_LOCALIZE:
            # The FSM moved on (RTL, abandon). Ignore a late verdict.
            self.get_logger().warn(
                f"Approval arrived in state '{self.current_state}' - ignoring")
            return
        self.enter_deliver_state()

    def _on_approval_rejected(self, lat, lon, class_id, reason):
        """Operator rejected. Suppress this spot for this class, resume scan."""
        if self.current_state != STATE_LOCALIZE:
            self.get_logger().warn(
                f"Rejection arrived in state '{self.current_state}' - ignoring")
            return

        rejected_msg = ObjectLocations()
        rejected_msg.latitude = float(lat)
        rejected_msg.longitude = float(lon)
        rejected_msg.class_id = int(class_id)
        self.rejected_object_pub.publish(rejected_msg)

        cls_name = (CLASS_NAMES[int(class_id)]
                    if 0 <= int(class_id) < len(CLASS_NAMES) else 'unknown')
        self.get_logger().info(
            f"Operator rejected {cls_name} at lat={lat:.6f}, lon={lon:.6f}"
            + (f" ({reason})" if reason else "") + " - resuming scan")

        # Same cleanup as the abandon-after-5-failures path.
        self.current_target_coords = None
        self.current_target_class_id = None
        self.confirmed_detection_class_id = -1
        self.confirmed_detection_id = ''
        self.confirmed_detection_coords = None
        self._resume_scan_in_place('operator_rejected')

    def _on_approval_callback_failed(self, name, exc):
        """A verdict callback raised. Get the FSM moving again.

        The gate clears its pending and destroys its timer before running a
        callback, so a half-completed transition leaves nothing armed to fire.
        Without this the aircraft holds in STATE_LOCALIZE until a battery
        failsafe. Abandon this target and resume the scan — the same tail the
        reject path and the abandon-after-5-failures path already use.

        Deliberately no retry: degraded-but-flying beats stranded.
        """
        if self.current_state != STATE_LOCALIZE:
            self.get_logger().warn(
                f"Approval callback '{name}' failed in state "
                f"'{self.current_state}' - no recovery needed: {exc!r}")
            return

        self.get_logger().error(
            f"Approval callback '{name}' failed ({exc!r}) - abandoning this "
            f"target and resuming scan so the aircraft keeps flying")
        self.log.event('APPROVAL_RECOVERY',
                       f"callback={name}, action=abandon_target_resume_scan")

        self.current_target_coords = None
        self.current_target_class_id = None
        self.confirmed_detection_class_id = -1
        self.confirmed_detection_id = ''
        self.confirmed_detection_coords = None
        self._resume_scan_in_place('approval_callback_failed')

    # Callbacks - topic subscriptions
    def on_gps_received(self, msg):
        """Callback for GPS position - tracks current position continuously."""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.log.event('HOME_SET',
                f"lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}")

    def on_waypoint_reached(self, msg):
        """Callback when a waypoint is reached."""
        waypoint_index = msg.wp_seq
        
        # Ignore duplicate waypoint messages (MAVROS publishes repeatedly)
        if waypoint_index == self.last_processed_waypoint and not self.is_transitioning:
            return
        
        # Always store the last reached waypoint (even during transitions)
        self.last_waypoint_reached = waypoint_index
        
        # Track scan progress for resume functionality
        # Subtract offset if loiter return point was prepended to the waypoint list
        if self.current_state == STATE_SCAN:
            self.last_reached_scan_waypoint = waypoint_index - self.scan_resume_waypoint_offset

        if self.current_state == STATE_LAP:
            self.publish_path_progress(
                'lap', waypoint_index + 1, len(self.lap_waypoints))
        elif self.current_state == STATE_SCAN:
            reached_in_segment = waypoint_index - self.scan_resume_waypoint_offset
            completed = self.scan_waypoint_index_on_detection
            if reached_in_segment >= 0:
                completed += reached_in_segment + 1
            self.publish_path_progress('scan', completed, len(self.scan_waypoints))

        # Don't process completion if we're mid-transition
        if self.is_transitioning:
            self.get_logger().debug(
                f"Waypoint {waypoint_index} reached during transition - stored for later"
            )
            return
        
        # Mark as processed
        self.last_processed_waypoint = waypoint_index
        
        self.get_logger().info(
            f"Reached waypoint {waypoint_index} (state={self.current_state})"
        )
        is_final = (waypoint_index == self.expected_final_waypoint_index)
        self.log.event('WP_REACHED',
            f"wp={waypoint_index}, state={self.current_state}, final={is_final}")

        # Check if this was the final waypoint
        if waypoint_index == self.expected_final_waypoint_index:
            self.handle_state_completion()

    def on_vehicle_state_changed(self, msg):
        """Callback when vehicle state (flight mode) changes."""
        if msg.mode == 'AUTO.RTL' and self.current_state != STATE_RTL:
            self.get_logger().info("External RTL detected - stopping mission actions")
            self.enter_rtl_state(command_mode=False)

        if self.current_state == STATE_RTL and not msg.armed and not self.rtl_completed:
            self.rtl_completed = True
            self.get_logger().info("RETURN COMPLETE - DISARMED")

    def on_object_detected(self, msg: ConfirmedDetection):
        """
        Callback when filtering_node confirms an object in its M-of-N window.
        The message carries the semantic class, candidate position, and a
        unique ID minted when filtering confirmed it.
        Stops the drone, waits for stabilization, then transitions to localization.
        """
        rejection_reason = confirmation_rejection_reason(
            msg, self.scan_started_ns, self.handled_confirmation_ids)
        if rejection_reason == 'invalid':
            return
        if rejection_reason == 'duplicate':
            self.get_logger().warn(
                f"Ignoring duplicate confirmation {msg.detection_id}")
            return
        if rejection_reason == 'stale':
            self.get_logger().warn(
                f"Ignoring stale confirmation {msg.detection_id} from a prior scan")
            return
        
        if self.current_state != STATE_SCAN:
            return  # Ignore detections outside of scan phase
        
        if self.is_transitioning:
            if (self.scan_route_pending
                    and self._pending_scan_confirmation is None):
                self._pending_scan_confirmation = msg
                self.get_logger().info(
                    f"Retaining confirmation {msg.detection_id} until the "
                    "scan route is active"
                )
            return  # Already handling a transition

        target_name = (
            CLASS_NAMES[int(msg.class_id)]
            if 0 <= int(msg.class_id) < len(CLASS_NAMES)
            else "unknown"
        )
        
        self.get_logger().info(
            f"OBJECT CONFIRMED! (#{self.objects_delivered_count + 1}) "
            f"Target={target_name}({int(msg.class_id)}). Stopping to localize..."
        )
        
        self.handled_confirmation_ids.add(msg.detection_id)
        # Preserve the exact scan-time candidate so localization can select
        # the same instance when a frame contains multiple boxes of this class.
        self.confirmed_detection_class_id = int(msg.class_id)
        self.confirmed_detection_id = msg.detection_id
        self.confirmed_detection_coords = (msg.latitude, msg.longitude)
        cls_name = (
            CLASS_NAMES[int(msg.class_id)]
            if 0 <= int(msg.class_id) < len(CLASS_NAMES)
            else 'unknown'
        )
        self.log.event('OBJECT_DETECTED',
            f"id={msg.detection_id}, class={cls_name}({msg.class_id}), "
            f"candidate=({msg.latitude:.6f},{msg.longitude:.6f}), "
            f"delivered_so_far={self.objects_delivered_count}/{self.num_objects_to_find}")

        # Mark as transitioning to prevent duplicate triggers
        self.is_transitioning = True
        
        # Save current scan progress for later resume
        # (last_reached_scan_waypoint is updated by on_waypoint_reached)

        # Save drone's current position so we can return here after delivery
        if self.current_lat is not None and self.current_lon is not None:
            scan_alt = self.scan_waypoints[0][2]  # Use scan altitude
            self.loiter_resume_coords = (self.current_lat, self.current_lon, scan_alt)
            self.log.event('LOITER_SAVED',
                f"lat={self.current_lat:.6f}, lon={self.current_lon:.6f}, "
                f"scan_wp_progress={self.last_reached_scan_waypoint}/{len(self.scan_waypoints)}")
            self.get_logger().info(
                f"Saved loiter position: lat={self.current_lat:.6f}, lon={self.current_lon:.6f}"
            )

        # Command drone to hold position
        self.set_flight_mode("AUTO.LOITER")
        
        # Wait 1 second for drone to stabilize, then transition to localize
        # Use a one-shot timer pattern: store reference and cancel after firing
        def localize_once():
            if self.current_state != STATE_SCAN:
                return
            if hasattr(self, '_localize_timer') and self._localize_timer is not None:
                self._localize_timer.cancel()
                self._localize_timer = None
            else:
                return  # Timer was already consumed/cancelled — orphan fire, skip
            self.enter_localize_state()
        
        # Cancel any existing localize timer to prevent orphaned timers
        if hasattr(self, '_localize_timer') and self._localize_timer is not None:
            self._localize_timer.cancel()
        self._localize_timer = self.create_timer(1.0, localize_once)

    # Main timer
    def main_timer_callback(self):
        """
        Main timer callback - runs every 0.5 seconds.
        Publishes state; the payload drop runs on its own DEPLOY_TICK_SEC timer.
        """
        self.publish_mission_state()


# Main entry point
def main(args=None):
    rclpy.init(args=args)
    
    node = MissionRunner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user")
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
