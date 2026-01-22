#!/usr/bin/env python3
"""

Mission Flow:
    TAKEOFF → LAP → SCAN → [LOCALIZE → DELIVER → DEPLOY] x N → RTL

Each detected object triggers the sequence:
    1. Stop drone (LOITER)
    2. Localize object using camera intrinsics
    3. Fly to localized object (stitching runs in background)
    4. Deploy payload via servo sequence
    5. Wait for stitching to complete
    6. Resume scanning from where we left off

FUTURE CHANGES:
    1. if no detections are found the drone stays at the last waypoint instead of restarting scan region
       i believe its because px4 doesnt restart and thinks mission is complete
    2. localizer  localizer livesMust respond to get_object_locations service when state is "localize":
    3. get_object_locations topic.
       - Must return localized GPS coordinates when called
       - First location in list should be the most recent detection
    4. stithcing node should collect its own frames
    

"""

# Imports
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandLong, ParamSetV2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int8
from rcl_interfaces.msg import ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory

from bv_msgs.srv import GetObjectLocations, TriggerStitching
from bv_msgs.msg import ObjectDetections


# Mission configuration
NUM_OBJECTS_TO_FIND = 4          # Total number of objects to detect and deliver to
DEPLOY_SERVO_CYCLE_TIME = 5.0    # Seconds per servo state during payload deploy


# State constants
STATE_TAKEOFF  = "takeoff"
STATE_LAP      = "lap"
STATE_SCAN     = "scan"
STATE_LOCALIZE = "localize"
STATE_DELIVER  = "stitching"
STATE_DEPLOY   = "deploy"
STATE_RTL      = "return"


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
        
        # Initialize state variables
        self.init_state_variables()
        
        # Set up ROS interfaces (publishers, subscribers, service clients)
        self.setup_ros_interfaces()
        
        # Wait for GPS fix to establish home position
        self.wait_for_gps_fix()
        
        # Wait for required services to become available
        self.wait_for_services()
        
        # Start the mission
        self.get_logger().info("=" * 50)
        self.get_logger().info("MISSION STARTING")
        self.get_logger().info(f"Objects to find: {self.num_objects_to_find}")
        self.get_logger().info("=" * 50)
        
        self.enter_takeoff_state()
        
        # Start main timer loop
        self.main_timer = self.create_timer(0.5, self.main_timer_callback)

    def load_config_from_yaml(self):
        """Load waypoints and parameters from mission_params.yaml."""
        config_path = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'mission_params.yaml'
        )
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Waypoint lists
        self.lap_waypoints = config.get('points', [])
        self.scan_waypoints = config.get('scan_points', [])
        
        # Velocity parameters (m/s)
        self.lap_velocity = config.get('Lap_velocity', 5.0)
        self.scan_velocity = config.get('Scan_velocity', 2.5)
        self.deliver_velocity = config.get('Deliver_velocity', 5.0)
        
        # Waypoint acceptance tolerance (meters)
        self.lap_tolerance = config.get('Lap_tolerance', 2.0)
        self.scan_tolerance = config.get('Scan_tolerance', 1.0)
        self.deliver_tolerance = config.get('Deliver_tolerance', 1.0)
        
        # Servo PWM configuration for payload deployment
        self.default_servo_pwms = config.get('Default_servo_pwms', [1500, 1500, 1500, 1500])
        self.deploy_initial_pwms = config.get('Deliver_initial_pwms', [1600, 1600, 1600, 1600])
        self.deploy_second_pwms = config.get('Deliver_second_pwms', [1400, 1400, 1400, 1400])
        
        # Number of objects (can be overridden in YAML)
        self.num_objects_to_find = config.get('num_objects', NUM_OBJECTS_TO_FIND)
        
        self.get_logger().info(f"Loaded {len(self.lap_waypoints)} lap waypoints")
        self.get_logger().info(f"Loaded {len(self.scan_waypoints)} scan waypoints")

    def init_state_variables(self):
        """Initialize all state tracking variables."""
        # Current FSM state
        self.current_state = STATE_TAKEOFF
        
        # Transition lock to prevent double-transitions
        self.is_transitioning = False
        
        # Mission progress
        self.objects_delivered_count = 0
        self.in_auto_mission = False
        
        # Waypoint tracking
        self.expected_final_waypoint_index = None
        self.active_waypoint_list = []
        self.desired_velocity = self.lap_velocity
        self.last_waypoint_reached = None  # Track in case event fires during transition
        self.last_processed_waypoint = -1  # Prevent duplicate waypoint processing
        
        # Scan resume tracking
        self.scan_waypoint_index_on_detection = 0
        self.last_reached_scan_waypoint = 0
        
        # Current target for delivery (set by localization)
        self.current_target_coords = None  # (lat, lon, alt)
        
        # Deploy state machine
        self.deploy_servo_state = 0  # 0=extend, 1=retract, 2=idle
        self.deploy_state_start_time = 0.0
        
        # Home position (set by GPS)
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        
        # Pending scan resume data (set after deploy, used after stitching)
        self.pending_scan_waypoint_index = None

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
        
        # Object detection trigger from filtering_node
        self.object_detected_sub = self.create_subscription(
            ObjectDetections,
            '/obj_dets',
            self.on_object_detected,
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
        self.get_object_locations_client = self.create_client(
            GetObjectLocations,
            'get_object_locations'
        )
        self.trigger_stitching_client = self.create_client(
            TriggerStitching,
            'trigger_stitching'
        )

    def wait_for_gps_fix(self):
        """Block until we receive a GPS fix for the home position."""
        self.get_logger().info("Waiting for GPS fix...")
        
        while rclpy.ok() and self.home_lat is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Unsubscribe from GPS after getting home position
        self.destroy_subscription(self.gps_sub)
        
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
            (self.get_object_locations_client, 'get_object_locations'),
            (self.trigger_stitching_client, 'trigger_stitching'),
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service: {name}")
        
        self.get_logger().info("All services available")

    # State machine core
    def handle_state_completion(self):
        """
        Called when the current state's objective is complete.
        Determines and initiates the appropriate next state.
        """
        self.get_logger().info(f"State '{self.current_state}' complete")
        
        if self.current_state == STATE_TAKEOFF:
            self.enter_lap_state()
            
        elif self.current_state == STATE_LAP:
            self.enter_scan_state()
            
        elif self.current_state == STATE_SCAN:
            # Scan waypoints finished without detection - restart or finish
            self.get_logger().info("Scan complete - no new detections")
            
            if self.objects_delivered_count >= self.num_objects_to_find:
                self.get_logger().info("All objects found and delivered!")
                self.enter_rtl_state()
            else:
                # Restart scan to continue looking
                self.get_logger().info(
                    f"Only {self.objects_delivered_count}/{self.num_objects_to_find} "
                    "objects found. Restarting scan..."
                )
                self.scan_waypoint_index_on_detection = 0
                self.enter_scan_state()
            
        elif self.current_state == STATE_LOCALIZE:
            # Handled by service callback, not waypoint completion
            pass
            
        elif self.current_state == STATE_DELIVER:
            self.enter_deploy_state()
            
        elif self.current_state == STATE_DEPLOY:
            # Handled by timer, not waypoint completion
            pass
            
        elif self.current_state == STATE_RTL:
            self.get_logger().info("=" * 50)
            self.get_logger().info("MISSION COMPLETE - LANDED")
            self.get_logger().info("=" * 50)

    def publish_mission_state(self):
        """Publish current state to /mission_state topic."""
        msg = String()
        msg.data = self.current_state
        self.mission_state_pub.publish(msg)

    # State entry methods
    def enter_takeoff_state(self):
        """
        Initial state - arm the vehicle and fly to the first lap waypoint.
        This effectively performs the takeoff.
        """
        self.current_state = STATE_TAKEOFF
        self.is_transitioning = True
        self.desired_velocity = self.lap_velocity
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: TAKEOFF")
        self.get_logger().info("-" * 40)
        
        # Use first lap point as takeoff destination
        if not self.lap_waypoints:
            self.get_logger().error("No lap waypoints defined!")
            return
        
        self.active_waypoint_list = self.build_waypoint_list(
            [self.lap_waypoints[0]],
            self.lap_tolerance
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
        self.is_transitioning = True
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
        
        # Get remaining scan waypoints from where we left off
        remaining_scan_points = self.scan_waypoints[self.scan_waypoint_index_on_detection:]
        
        if not remaining_scan_points:
            self.get_logger().info("No remaining scan waypoints")
            self.handle_state_completion()
            return
        
        self.active_waypoint_list = self.build_waypoint_list(
            remaining_scan_points,
            self.scan_tolerance,
            pass_through_ratio=1.0  # Continuous flight for scanning
        )
        self.expected_final_waypoint_index = len(self.active_waypoint_list) - 1
        self.last_reached_scan_waypoint = 0  # Reset for this segment
        self.push_mission_to_autopilot()

    def enter_localize_state(self):
        """
        Hold position while the localizer computes the object's GPS coordinates
        using camera intrinsics and drone pose.
        """
        self.current_state = STATE_LOCALIZE
        self.is_transitioning = True
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: LOCALIZE")
        self.get_logger().info("-" * 40)
        
        # Request object location from filtering_node
        self.request_object_location()

    def enter_deliver_state(self):
        """
        Fly to the localized object position for payload delivery.
        
        Note: This state publishes "stitching" to /mission_state, which
        activates stitching.py to create a panorama from frames saved
        during scan - this happens in the background while flying.
        """
        self.current_state = STATE_DELIVER  # Publishes "stitching" to activate stitching.py
        self.is_transitioning = True
        self.desired_velocity = self.deliver_velocity
        
        # Reset waypoint tracking for new state
        self.last_waypoint_reached = None
        self.last_processed_waypoint = -1
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: DELIVER")
        self.get_logger().info("(Stitching runs in background)")
        self.get_logger().info("-" * 40)
        
        if self.current_target_coords is None:
            self.get_logger().error("No target coordinates available!")
            self.enter_rtl_state()
            return
        
        lat, lon, alt = self.current_target_coords
        self.get_logger().info(f"Flying to target: lat={lat:.6f}, lon={lon:.6f}")
        
        self.active_waypoint_list = self.build_waypoint_list(
            [self.current_target_coords],
            self.deliver_tolerance
        )
        self.expected_final_waypoint_index = 0
        self.push_mission_to_autopilot()

    def enter_deploy_state(self):
        """
        Execute the servo sequence to release the payload.
        Controlled by timer, not waypoints.
        """
        self.current_state = STATE_DEPLOY
        self.is_transitioning = False  # No waypoint transition for deploy
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: DEPLOY")
        self.get_logger().info(
            f"Deploying payload {self.objects_delivered_count + 1}/{self.num_objects_to_find}"
        )
        self.get_logger().info("-" * 40)
        
        # Initialize servo state machine
        self.deploy_servo_state = 0
        self.deploy_state_start_time = time.monotonic()
        
        # Execute first servo step immediately
        self.execute_deploy_servo_step()

    def enter_rtl_state(self):
        """
        Return to launch and land.
        """
        self.current_state = STATE_RTL
        self.is_transitioning = True
        
        self.get_logger().info("-" * 40)
        self.get_logger().info("ENTERING STATE: RTL")
        self.get_logger().info("-" * 40)
        
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
        future.add_done_callback(self.on_set_mode_complete)

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

    def set_servo_pwm(self, servo_channel, pwm_value):
        """
        Set a servo's PWM value by adjusting its MIN/MAX parameters.
        
        Args:
            servo_channel: Servo channel number (1-4)
            pwm_value: PWM value in microseconds
        """
        min_param = f'PWM_MAIN_MIN{servo_channel}'
        max_param = f'PWM_MAIN_MAX{servo_channel}'
        
        # Set min parameter
        req_min = ParamSetV2.Request()
        req_min.force_set = False
        req_min.param_id = min_param
        pv_min = ParameterValue()
        pv_min.type = ParameterType.PARAMETER_INTEGER
        pv_min.integer_value = pwm_value - 1
        req_min.value = pv_min
        
        # Set max parameter
        req_max = ParamSetV2.Request()
        req_max.force_set = False
        req_max.param_id = max_param
        pv_max = ParameterValue()
        pv_max.type = ParameterType.PARAMETER_INTEGER
        pv_max.integer_value = pwm_value + 1
        req_max.value = pv_max
        
        self.param_set_client.call_async(req_min)
        self.param_set_client.call_async(req_max)

    def reset_all_servos_to_default(self):
        """Reset all servos to their default PWM positions."""
        for channel, pwm in enumerate(self.default_servo_pwms, start=1):
            self.set_servo_pwm(channel, pwm)

    def request_object_location(self):
        """Request object GPS location from the filtering node."""
        request = GetObjectLocations.Request()
        
        self.get_logger().info("Requesting object location from filtering node...")
        
        future = self.get_object_locations_client.call_async(request)
        future.add_done_callback(self.on_object_location_received)

    def request_stitching_complete(self):
        """Request stitching service and wait for completion before resuming scan."""
        request = TriggerStitching.Request()
        
        self.get_logger().info("Waiting for stitching to complete...")
        
        future = self.trigger_stitching_client.call_async(request)
        future.add_done_callback(self.on_stitching_complete)

    # Deploy servo logic
    def execute_deploy_servo_step(self):
        """
        Execute the current step in the servo deploy sequence.
        Uses the servo channel corresponding to the current object number.
        """
        # Determine which servo to use based on object count (cycles 1-4)
        servo_index = self.objects_delivered_count % 4
        servo_channel = servo_index + 1
        
        initial_pwm = self.deploy_initial_pwms[servo_index]
        second_pwm = self.deploy_second_pwms[servo_index]
        default_pwm = self.default_servo_pwms[servo_index]
        
        if self.deploy_servo_state == 0:
            # Extend
            self.get_logger().info(f"[DEPLOY] CH{servo_channel} <- {initial_pwm} (extend)")
            self.set_servo_pwm(servo_channel, initial_pwm)
            
        elif self.deploy_servo_state == 1:
            # Retract
            self.get_logger().info(f"[DEPLOY] CH{servo_channel} <- {second_pwm} (retract)")
            self.set_servo_pwm(servo_channel, second_pwm)
            
        elif self.deploy_servo_state == 2:
            # Return to idle
            self.get_logger().info(f"[DEPLOY] CH{servo_channel} <- {default_pwm} (idle)")
            self.set_servo_pwm(servo_channel, default_pwm)

    def on_deploy_complete(self):
        """Called when the servo deploy sequence finishes."""
        self.objects_delivered_count += 1
        
        self.get_logger().info(
            f"Payload {self.objects_delivered_count}/{self.num_objects_to_find} delivered!"
        )
        
        # Clear current target
        self.current_target_coords = None
        
        if self.objects_delivered_count >= self.num_objects_to_find:
            # All objects delivered - mission complete
            self.get_logger().info("All payloads delivered!")
            self.enter_rtl_state()
        else:
            # Calculate and store the scan resume index for after stitching completes
            self.pending_scan_waypoint_index = (
                self.scan_waypoint_index_on_detection + self.last_reached_scan_waypoint + 1
            )
            
            # Wrap around if we've passed the end
            if self.pending_scan_waypoint_index >= len(self.scan_waypoints):
                self.pending_scan_waypoint_index = 0
            
            # Wait for stitching to complete before resuming scan
            self.request_stitching_complete()

    def on_stitching_complete(self, future):
        """Callback when stitching service returns - now safe to resume scanning."""
        response = future.result()
        
        if response.success:
            self.get_logger().info("Stitching completed successfully")
        else:
            self.get_logger().warn("Stitching reported failure, continuing anyway...")
        
        # Now resume scanning from the stored waypoint index
        self.scan_waypoint_index_on_detection = self.pending_scan_waypoint_index
        self.pending_scan_waypoint_index = None
        
        self.get_logger().info(
            f"Resuming scan from waypoint {self.scan_waypoint_index_on_detection}"
        )
        self.enter_scan_state()

    # Callbacks - service responses
    def on_waypoint_push_complete(self, future):
        """Callback when waypoint push to autopilot completes."""
        response = future.result()
        
        if not response.success:
            self.get_logger().error(
                f"Waypoint push failed! Only {response.wp_transfered} transferred."
            )
            self.is_transitioning = False
            return
        
        self.get_logger().info("Waypoints uploaded successfully")
        self.arm_vehicle()

    def on_arm_complete(self, future):
        """Callback when arming completes."""
        response = future.result()
        
        if not response.success:
            self.get_logger().error("Arming failed!")
            self.is_transitioning = False
            return
        
        self.get_logger().info("Vehicle armed")
        self.set_flight_mode("AUTO.MISSION")

    def on_set_mode_complete(self, future):
        """Callback when mode change completes."""
        response = future.result()
        
        if not response.mode_sent:
            self.get_logger().error("Mode change failed!")
            self.is_transitioning = False
            return
        
        self.get_logger().info("Flight mode set successfully")
        self.in_auto_mission = True
        self.is_transitioning = False
        
        # Reset servos and set velocity
        self.reset_all_servos_to_default()
        self.set_velocity(self.desired_velocity)
        
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

    def on_velocity_set_complete(self, future):
        """Callback when velocity parameter change completes."""
        response = future.result()
        
        if response.success:
            self.get_logger().info(
                f"Velocity set to {response.value.double_value:.2f} m/s"
            )
        else:
            self.get_logger().warn("Failed to set velocity parameter")

    def on_object_location_received(self, future):
        """Callback when object location service returns."""
        if self.current_state != STATE_LOCALIZE:
            return
        
        response = future.result()
        locations = response.locations
        
        if not locations:
            self.get_logger().warn("No object location returned - retrying in 1s...")
            self.create_timer(1.0, lambda: self.request_object_location())
            return
        
        # Take the first (most recent) localized object
        loc = locations[0]
        self.current_target_coords = (loc.latitude, loc.longitude, loc.altitude)
        
        self.get_logger().info(
            f"Object localized at: lat={loc.latitude:.6f}, lon={loc.longitude:.6f}, "
            f"alt={loc.altitude:.2f}m"
        )
        
        # Proceed to delivery (stitching runs in background during this phase)
        self.enter_deliver_state()

    # Callbacks - topic subscriptions
    def on_gps_received(self, msg):
        """Callback for GPS position - used only for initial home position."""
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude

    def on_waypoint_reached(self, msg):
        """Callback when a waypoint is reached."""
        waypoint_index = msg.wp_seq
        
        # Ignore duplicate waypoint messages (MAVROS publishes repeatedly)
        if waypoint_index == self.last_processed_waypoint and not self.is_transitioning:
            return
        
        # Always store the last reached waypoint (even during transitions)
        self.last_waypoint_reached = waypoint_index
        
        # Track scan progress for resume functionality
        if self.current_state == STATE_SCAN:
            self.last_reached_scan_waypoint = waypoint_index
        
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
        
        # Check if this was the final waypoint
        if waypoint_index == self.expected_final_waypoint_index:
            self.handle_state_completion()

    def on_vehicle_state_changed(self, msg):
        """Callback when vehicle state (flight mode) changes."""
        # Detect when RTL completes
        if (self.in_auto_mission and 
            msg.mode != 'AUTO.MISSION' and 
            self.current_state == STATE_RTL):
            
            self.get_logger().info(f"Flight mode changed to: {msg.mode}")
            self.in_auto_mission = False
            self.handle_state_completion()

    def on_object_detected(self, msg):
        """
        Callback when filtering_node publishes detections during scan.
        Immediately stops the drone and transitions to localization.
        """
        # Check if there are any detections in the message
        if not msg.detections:
            return
        
        if self.current_state != STATE_SCAN:
            return  # Ignore detections outside of scan phase
        
        if self.is_transitioning:
            return  # Already handling a transition
        
        self.get_logger().info(
            f"OBJECT DETECTED! (#{self.objects_delivered_count + 1}, "
            f"{len(msg.detections)} detection(s)) - Stopping to localize..."
        )
        
        # Save current scan progress for later resume
        # (last_reached_scan_waypoint is updated by on_waypoint_reached)
        
        # Command drone to hold position
        self.set_flight_mode("AUTO.LOITER")
        
        # Brief delay to ensure loiter is set, then transition
        self.create_timer(0.5, lambda: self.enter_localize_state())

    # Main timer
    def main_timer_callback(self):
        """
        Main timer callback - runs every 0.5 seconds.
        Handles state publishing and deploy servo timing.
        """
        # Always publish current state
        self.publish_mission_state()
        
        # Handle deploy servo timing
        if self.current_state == STATE_DEPLOY:
            elapsed = time.monotonic() - self.deploy_state_start_time
            
            if elapsed >= DEPLOY_SERVO_CYCLE_TIME:
                # Advance to next servo state
                self.deploy_servo_state += 1
                self.deploy_state_start_time = time.monotonic()
                
                if self.deploy_servo_state > 2:
                    # Deploy sequence complete
                    self.on_deploy_complete()
                else:
                    self.execute_deploy_servo_step()


# Main entry point
def main(args=None):
    rclpy.init(args=args)
    
    node = MissionRunner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()