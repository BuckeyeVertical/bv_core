#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached, CommandCode
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandLong, ParamSetV2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int8

from rcl_interfaces.msg import ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory
import time

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

        # Velocity and tolerance parameters
        self.lap_velocity      = cfg.get('Lap_velocity', 5.0)
        self.lap_tolerance     = cfg.get('Lap_tolerance', 3.0)
        self.scan_velocity     = cfg.get('Scan_velocity', 3.0)
        self.scan_tolerance    = cfg.get('Scan_tolerance', 1.0)
        self.stitch_velocity   = cfg.get('Stitch_velocity', 5.0)
        self.stitch_tolerance  = cfg.get('Stitch_tolerance', 1.0)
        self.deliver_tolerance = cfg.get('Deliver_tolerance', 1.0)

        # Home position vars
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
        self.get_logger().info('Waiting for first GPS fix…')
        while rclpy.ok() and self.home_lat is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(self.gps_sub)
        self.get_logger().info(
            f'Home set → lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}'
        )

        # MAVROS service clients
        self.cli_push  = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_arm   = self.create_client(CommandBool,   '/mavros/cmd/arming')
        self.cli_mode  = self.create_client(SetMode,       '/mavros/set_mode')
        self.cli_cmd   = self.create_client(CommandLong,    '/mavros/cmd/command')
        self.cli_param = self.create_client(ParamSetV2,    '/mavros/param/set')

        # Wait for services
        for client, name in [
            (self.cli_push,  '/mavros/mission/push'),
            (self.cli_arm,   '/mavros/cmd/arming'),
            (self.cli_mode,  '/mavros/set_mode'),
            (self.cli_cmd,   '/mavros/cmd/command'),
            (self.cli_param, '/mavros/param/set'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}…')

        # Subscribers
        reach_qos = QoSProfile(depth=10)
        reach_qos.reliability = ReliabilityPolicy.RELIABLE
        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.reached_cb,
            qos_profile=reach_qos
        )
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_sub = self.create_subscription(
            MavState,
            '/mavros/state',
            self.state_cb,
            qos_profile=state_qos
        )

        # Mission state publisher
        mission_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.mission_state_pub = self.create_publisher(
            String,
            '/mission_state',
            qos_profile=mission_state_qos
        )

        self.queue_state_sub = self.create_subscription(
            Int8,
            '/queue_state',
            self.handle_queue_state,
            qos_profile=mission_state_qos
        )

        # FSM variables
        self.state = 'lap'
        self.lap_count = 1
        self.deliver_index = 0
        self.in_auto_mission = False
        self.queue_state = 0
        self.maybe_deliver = False

        # Kick off
        self.start_lap()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def handle_queue_state(self, msg: Int8):
        if msg.data == 0:
            self.get_logger().info("Waiting for processing to complete")
        self.queue_state = msg.data

    def timer_callback(self):
        msg = String()
        msg.data = self.state
        self.mission_state_pub.publish(msg)

        if self.queue_state == 1 and self.maybe_deliver:
            self.get_logger().info("Starting deliver")
            self.start_deliver()
            self.maybe_deliver = False

    def build_waypoints(self, waypoint_list, tolerance, pass_through_ratio=0.0):
        wp_list = []
        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = Waypoint()
            wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command      = MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)
            wp.autocontinue = True
            wp.param1       = 1.0
            wp.param2       = tolerance
            wp.param3       = tolerance * pass_through_ratio
            wp.param4       = float('nan')
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = alt
            wp_list.append(wp)
        return wp_list

    def set_servo(self, servo_id, value):
        """Set a servo to a specific value."""
        req = CommandLong.Request()
        req.command = CommandCode.CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = servo_id
        req.param2 = value
        self.cli_cmd.call_async(req).add_done_callback(self.cmd_cb)

    # State starters (no speed items)
    def start_lap(self):
        self.state = 'lap'
        self.transition_in_progress = True
        self.desired_speed = self.lap_velocity
        self.get_logger().info(f'Starting lap {self.lap_count}…')
        self.wp_list = self.build_waypoints(self.points, self.lap_tolerance, 1.0)
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()

    def start_stitching(self):
        self.state = 'stitching'
        self.transition_in_progress = True
        self.desired_speed = self.stitch_velocity
        self.get_logger().info('Starting stitching…')
        self.wp_list = self.build_waypoints(self.stitch_points, self.stitch_tolerance)
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()

    def start_scan(self):
        self.state = 'scan'
        self.transition_in_progress = True
        self.desired_speed = self.scan_velocity
        self.get_logger().info('Starting scan…')
        self.wp_list = self.build_waypoints(self.scan_points, self.scan_tolerance, 1.0)
        self.expected_final_wp = len(self.wp_list) - 1
        self.push_mission()

    def start_deliver(self):
        self.state = 'deliver'
        self.transition_in_progress = True
        self.desired_speed = self.lap_velocity
        idx = self.deliver_index
        lat, lon, alt = self.deliver_points[idx]
        self.get_logger().info(f'Delivering to point {idx+1}…')
        self.wp_list = self.build_waypoints([(lat, lon, alt)], self.deliver_tolerance)
        self.expected_final_wp = 0
        self.push_mission()

    def return_to_rtl(self):
        self.state = 'return'
        self.transition_in_progress = True
        self.get_logger().info('Switching to AUTO.RTL…')
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'AUTO.RTL'
        self.cli_mode.call_async(req).add_done_callback(self.mode_cb)

    def handle_mission_completion(self):
        self.get_logger().info(f'State "{self.state}" complete!')
        if self.state == 'lap':
            if self.lap_count == 1:
                self.start_stitching()
            else:
                self.start_deliver()
        elif self.state == 'stitching':
            self.start_scan()
        elif self.state == 'scan':
            self.maybe_deliver = True
        elif self.state == 'deliver':
            self.deliver_index += 1
            self.lap_count += 1
            if self.deliver_index < len(self.deliver_points):
                self.start_lap()
            else:
                self.return_to_rtl()
        elif self.state == 'return':
            self.get_logger().info('Mission complete - landed!')

    # Mission push / arm / mode callbacks
    def push_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = self.wp_list
        self.get_logger().info(f'Pushing {len(self.wp_list)} waypoint(s)…')
        self.cli_push.call_async(req).add_done_callback(self.push_cb)

    def push_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'Push failed: transferred {resp.wp_transfered}')
            self.transition_in_progress = False
            return
        self.get_logger().info('Mission pushed ✓ → Arming…')
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.cli_arm.call_async(arm_req).add_done_callback(self.arm_cb)

    def arm_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error('Arming FAILED')
            self.transition_in_progress = False
            return
        self.get_logger().info('Armed ✓ → Setting AUTO.MISSION…')
        mode_req = SetMode.Request()
        mode_req.base_mode = 0
        mode_req.custom_mode = 'AUTO.MISSION'
        self.cli_mode.call_async(mode_req).add_done_callback(self.mode_cb)

    def mode_cb(self, future):
        resp = future.result()
        if not resp.mode_sent:
            self.get_logger().error('Mode change failed')
            self.transition_in_progress = False
            return
        self.get_logger().info('Mode set ✓ → flying…')
        self.in_auto_mission = True
        self.transition_in_progress = False
        # Update PX4 speed limit parameter
        self.set_mpc_xy_vel_all(self.desired_speed)

    def set_mpc_xy_vel_all(self, speed):
        req = ParamSetV2.Request()
        req.force_set = False
        req.param_id  = 'MPC_XY_VEL_ALL'
        pv = ParameterValue()
        pv.type            = ParameterType.PARAMETER_DOUBLE
        pv.double_value    = float(speed)
        req.value          = pv
        self.cli_param.call_async(req).add_done_callback(self.param_cb)

    def param_cb(self, future):
        resp = future.result()
        if resp.success:
            val = resp.value.double_value
            self.get_logger().info(f'MPC_XY_VEL_ALL set to {val:.2f} m/s')
        else:
            self.get_logger().error('Failed to set MPC_XY_VEL_ALL')

    def reached_cb(self, msg: WaypointReached):
        if self.transition_in_progress:
            return
        idx = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {idx} (state={self.state})')
        if idx == self.expected_final_wp:
            self.handle_mission_completion()

    def state_cb(self, msg: MavState):
        if self.in_auto_mission and msg.mode != 'AUTO.MISSION' and self.deliver_index == len(self.deliver_points):
            self.get_logger().info(f'Mission ended, px4 mode={msg.mode}')
            self.in_auto_mission = False

    def gps_cb(self, msg: NavSatFix):
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.get_logger().info(
                f'GPS fixed: lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
