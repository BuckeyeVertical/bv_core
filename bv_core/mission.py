#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from sensor_msgs.msg import NavSatFix

from ament_index_python.packages import get_package_share_directory

# MAVLink constants
MAV_CMD_NAV_WAYPOINT          = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

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

        # Scalar parameters
        self.max_vel       = cfg.get('max_velocity', 1.0)
        self.time_to_max   = cfg.get('time_to_max', 1.0)
        self.wp_tol        = cfg.get('waypoint_tolerance', 1.0)

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

        self.in_auto_mission = False

        # FSM state variables
        self.state = 'lap'
        self.lap_count = 1
        self.deliver_index = 0

        # Start the first lap
        self.start_lap()


    
    # Convert a list of [lat, lon, alt] triplets into mavros_msgs/Waypoint messages
    
    def build_waypoints(self, waypoint_list):
        wp_list = []
        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = Waypoint()
            wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command      = MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)
            wp.autocontinue = True
            wp.param1       = 0.0
            wp.param2       = 0.0
            wp.param3       = 0.0
            wp.param4       = float('nan')
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = alt
            wp_list.append(wp)
        return wp_list


    
    # Lap state
    
    def start_lap(self):
        self.state = 'lap'
        self.get_logger().info(f'Starting lap {self.lap_count} …')
        self.wp_list = self.build_waypoints(self.points)
        self.push_mission()


    
    # Scan state
    
    def start_scan(self):
        self.state = 'scan'
        self.get_logger().info('Starting scan state …')
        self.wp_list = self.build_waypoints(self.scan_points)
        self.push_mission()


    
    # Deliver state
    
    def start_deliver(self):
        self.state = 'deliver'
        idx = self.deliver_index
        lat, lon, alt = self.deliver_points[idx]
        self.get_logger().info(f'Delivering to point {idx + 1} …')

        wp = Waypoint()
        wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command      = MAV_CMD_NAV_WAYPOINT
        wp.is_current   = True
        wp.autocontinue = True
        wp.param1       = 0.0
        wp.param2       = 0.0
        wp.param3       = 0.0
        wp.param4       = float('nan')
        wp.x_lat        = lat
        wp.y_long       = lon
        wp.z_alt        = alt

        self.wp_list = [wp]
        self.push_mission()


    
    # RTL Code
    
    def return_to_rtl(self):
        self.state = 'return'
        self.get_logger().info('Switching to AUTO.RTL …')
        mode_req = SetMode.Request()
        mode_req.base_mode   = 0
        mode_req.custom_mode = 'AUTO.RTL'
        future = self.cli_mode.call_async(mode_req)
        future.add_done_callback(self.mode_cb)


    
    # Push the current self.wp_list to /mavros/mission/push, then arms and switches to AUTO.MISSION.
    
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
            self.get_logger().error(f'Mission push failed (result={resp.result})')
            return
        self.get_logger().info('Mission pushed ✓ → Arming vehicle …')
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
        else:
            self.get_logger().info(f'Mode set ✓ → {"RTL" if self.state=="return" else "flying"} …')
            if self.state != 'return':
                self.in_auto_mission = True


    #
    # Called every time PX4 publishes `/mavros/mission/reached`
    #
    def reached_cb(self, msg: WaypointReached):
        idx = msg.wp_seq
        last_idx = len(self.wp_list) - 1
        self.get_logger().info(f'Reached waypoint index: {idx}')

        if idx == last_idx:
            if self.state == 'lap':
                # if statement to check lap number
                if self.lap_count == 1:
                    pass
                else:
                    self.start_deliver()
            elif self.state == 'scan':
                self.start_deliver()
            elif self.state == 'deliver':
                self.deliver_index += 1
                self.lap_count += 1
                if self.deliver_index < len(self.deliver_points):
                    self.start_lap()
                else:
                    self.return_to_rtl()
            elif self.state == 'return':
                self.get_logger().info('RTL in progress or landed.')


    #
    # End segment check
    #
    def state_cb(self, msg: MavState):
        if self.in_auto_mission and msg.mode != 'AUTO.MISSION':
            self.get_logger().info(f'Mission ended, detected PX4 mode={msg.mode}')
            self.in_auto_mission = False

            if self.state == 'lap' and self.lap_count == 1:
                self.start_scan()


    #
    # Capture the first NavSatFix as “home”
    #
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
