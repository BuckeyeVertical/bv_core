#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached, ParamValue
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, ParamSet
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# MAVLink constants
MAV_CMD_NAV_WAYPOINT          = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

        # FSM control variables
        self.transition_in_progress = False
        self.expected_final_wp = None

        # Load all parameters from mission_params.yaml
        mission_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'mission_params.yaml'
        )
        with open(mission_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.points         = cfg.get('points', [])
        self.scan_points    = cfg.get('scan_points', [])
        self.deliver_points = cfg.get('deliver_points', [])
        self.stitch_points  = cfg.get('stitch_points', [])

        # Velocity and tolerance parameters from yaml
        self.lap_velocity      = cfg.get('Lap_velocity', 5.0)
        self.lap_tolerance     = cfg.get('Lap_tolerance', 10.0)
        self.scan_velocity     = cfg.get('Scan_velocity', 3.0)
        self.scan_tolerance    = cfg.get('Scan_tolerance', 1.0)
        self.stitch_velocity   = cfg.get('Stitch_velocity', 5.0)
        self.stitch_tolerance  = cfg.get('Stitch_tolerance', 1.0)
        self.deliver_tolerance = cfg.get('Deliver_tolerance', 1.0)

        # Old/Manual RTL code
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        gps_qos = QoSProfile(depth=10)
        gps_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos_profile=gps_qos
        )
        self.get_logger().info('Waiting for first GPS fix to set home position…')
        while rclpy.ok() and self.home_lat is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Manual/Old RTL code
        self.destroy_subscription(self.gps_sub)
        self.get_logger().info(
            f'Home set → lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}'
        )

        # Create MAVROS service clients
        self.cli_push = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_arm  = self.create_client(CommandBool,   '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode,       '/mavros/set_mode')

        reach_qos = QoSProfile(depth=10)
        reach_qos.reliability = ReliabilityPolicy.RELIABLE
        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.reached_cb,
            qos_profile=reach_qos
        )

        for client, name in [
            (self.cli_push, '/mavros/mission/push'),
            (self.cli_arm,  '/mavros/cmd/arming'),
            (self.cli_mode, '/mavros/set_mode'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}…')

        # Subscribe to /mavros/state to detect AUTO.MISSION → LOITER/HOLD
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_sub = self.create_subscription(
            MavState,
            '/mavros/state',
            self.state_cb,
            qos_profile=state_qos
        )

        mission_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,       # No lost updates
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # "Latch" the last message for late joiners
            history=HistoryPolicy.KEEP_LAST,              # Only keep the most recent
            depth=1                                        # …since you only need one message
        )
        self.mission_state_pub = self.create_publisher(
            String,
            '/mission_state',
            qos_profile=mission_state_qos
        )

        self.in_auto_mission = False

        # FSM state variables
        self.state = 'lap'
        self.lap_count = 1
        self.deliver_index = 0

        # Start the first lap
        self.start_lap()

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.state
        self.mission_state_pub.publish(msg)
    
    # Set cruise speed parameter
    def set_cruise_speed(self, speed):
        """Set MPC_XY_CRUISE parameter for cruise speed"""
        try:
            cli_param = self.create_client(ParamSet, '/mavros/param/set')
            if not cli_param.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('Param service not available, skipping speed setting')
                return
                
            req = ParamSet.Request()
            req.param_id = 'MPC_XY_CRUISE'
            req.value = ParamValue()
            req.value.integer = 0
            req.value.real = float(speed)
            
            future = cli_param.call_async(req)
            # Don't wait for response to avoid blocking
            self.get_logger().info(f'Setting cruise speed to {speed} m/s')
        except Exception as e:
            self.get_logger().warn(f'Could not set speed parameter: {e}')
    
    # Convert a list of [lat, lon, alt] triplets into mavros_msgs/Waypoint messages
    def build_waypoints(self, waypoint_list, tolerance, pass_through_ratio=0.0):
        """
        Build waypoints with specified tolerance and pass-through settings.
        pass_through_ratio: 0.0 = stop at waypoint, 1.0 = pass through at same radius as tolerance
        """
        wp_list = []
        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = Waypoint()
            wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command      = MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)
            wp.autocontinue = True
            wp.param1       = 0.0  # Hold time in seconds
            wp.param2       = tolerance  # Acceptance radius in meters
            wp.param3       = tolerance * pass_through_ratio  # Pass-through radius
            wp.param4       = float('nan')  # Yaw angle
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = alt
            wp_list.append(wp)
        return wp_list


    # Lap state
    def start_lap(self):
        self.state = 'lap'
        self.transition_in_progress = True
        self.get_logger().info(f'Starting lap {self.lap_count} …')
        
        # Try to set cruise speed
        self.set_cruise_speed(self.lap_velocity)
        
        # Build waypoints
        self.wp_list = self.build_waypoints(self.points, self.lap_tolerance, pass_through_ratio=1.0)
        
        # Expected final waypoint
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()

    # Stitching state
    def start_stitching(self):
        self.state = 'stitching'
        self.transition_in_progress = True
        self.get_logger().info(f'Starting stitching state …')
        
        # Try to set cruise speed
        self.set_cruise_speed(self.stitch_velocity)
        
        # Build waypoints
        self.wp_list = self.build_waypoints(self.stitch_points, self.stitch_tolerance, pass_through_ratio=0.0)
        
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()
    
    # Scan state
    def start_scan(self):
        self.state = 'scan'
        self.transition_in_progress = True
        self.get_logger().info('Starting scan state …')
        
        # Try to set cruise speed
        self.set_cruise_speed(self.scan_velocity)
        
        # Build waypoints
        self.wp_list = self.build_waypoints(self.scan_points, self.scan_tolerance, pass_through_ratio=1.0)
        
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()
    
    # Deliver state
    def start_deliver(self):
        self.state = 'deliver'
        self.transition_in_progress = True
        idx = self.deliver_index
        lat, lon, alt = self.deliver_points[idx]
        self.get_logger().info(f'Delivering to point {idx + 1} …')
        
        # Try to set cruise speed (using lap velocity)
        self.set_cruise_speed(self.lap_velocity)
        
        # Build single delivery waypoint
        self.wp_list = self.build_waypoints([(lat, lon, alt)], self.deliver_tolerance, pass_through_ratio=0.0)
        
        self.expected_final_wp = 0  # Only one waypoint
        self.push_mission()
    
    # RTL Code
    def return_to_rtl(self):
        self.state = 'return'
        self.transition_in_progress = True
        self.get_logger().info('Switching to AUTO.RTL …')
        mode_req = SetMode.Request()
        mode_req.base_mode   = 0
        mode_req.custom_mode = 'AUTO.RTL'
        future = self.cli_mode.call_async(mode_req)
        future.add_done_callback(self.mode_cb)

    
    def handle_mission_completion(self):
        """Central FSM transition logic - called when any state completes"""
        self.get_logger().info(f'State "{self.state}" complete!')
        
        if self.state == 'lap':
            if self.lap_count == 1:
                self.start_stitching()
            else:
                self.start_deliver()
                
        elif self.state == 'stitching':
            self.start_scan()
            
        elif self.state == 'scan':
            self.start_deliver()
            
        elif self.state == 'deliver':
            self.deliver_index += 1
            self.lap_count += 1
            self.get_logger().info(f"Completed delivery {self.deliver_index}/{len(self.deliver_points)}")
            
            if self.deliver_index < len(self.deliver_points):
                self.get_logger().info("Going back to lap")
                self.start_lap()
            else:
                self.return_to_rtl()
                
        elif self.state == 'return':
            self.get_logger().info('Mission complete - landed!')

    
    def push_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = self.wp_list

        self.get_logger().info(
            f'Pushing {len(self.wp_list)} waypoint(s) for state="{self.state}" …'
        )
        future = self.cli_push.call_async(req)
        future.add_done_callback(self.push_cb)

    def push_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(
                f"Mission push failed → success={resp.success}, "
                f"wp_transferred={resp.wp_transfered}"
            )
            self.transition_in_progress = False
            return
        self.get_logger().info(
            f"Mission pushed ✓ → transferred {resp.wp_transfered} waypoint(s) → Arming vehicle …"
        )
        self.arm_vehicle()

    def arm_vehicle(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.cli_arm.call_async(arm_req)
        future.add_done_callback(self.arm_cb)

    def arm_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error('Arming FAILED')
            self.transition_in_progress = False
            return
        self.get_logger().info('Armed ✓ → Setting mode to AUTO.MISSION …')
        self.switch_to_mission()

    def switch_to_mission(self):
        mode_req = SetMode.Request()
        mode_req.base_mode   = 0
        mode_req.custom_mode = 'AUTO.MISSION'
        future = self.cli_mode.call_async(mode_req)
        future.add_done_callback(self.mode_cb)

    def mode_cb(self, future):
        resp = future.result()
        if not resp.mode_sent:
            self.get_logger().error('Failed to set mode')
            self.transition_in_progress = False
        else:
            self.get_logger().info(f'Mode set ✓ → {"RTL" if self.state=="return" else "flying"} …')
            if self.state != 'return':
                self.in_auto_mission = True
            # Mission is now active, allow waypoint processing
            self.transition_in_progress = False

    
    def reached_cb(self, msg: WaypointReached):
        # Ignore all waypoints during state transitions
        if self.transition_in_progress:
            return
        
        idx = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {idx} (state={self.state})')
        
        # Check if this is the final waypoint we're expecting
        if idx == self.expected_final_wp:
            self.handle_mission_completion()

    
    def state_cb(self, msg: MavState):
        if self.in_auto_mission and msg.mode != 'AUTO.MISSION' and self.deliver_index == len(self.deliver_points):
            self.get_logger().info(f'Mission ended, detected PX4 mode={msg.mode}')
            self.in_auto_mission = False

    def gps_cb(self, msg: NavSatFix):
        if self.home_lat is None and self.home_lon is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.get_logger().info(
                f'GPS fixed: home_lat={self.home_lat:.6f}, '
                f'home_lon={self.home_lon:.6f}, home_alt={self.home_alt:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()