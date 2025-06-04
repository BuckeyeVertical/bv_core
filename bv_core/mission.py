#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from std_msgs.msg import UInt16
import math

# MAVLink constants
MAV_CMD_NAV_WAYPOINT          = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

        # 1) Create clients for mission push, arming, and mode switching
        self.cli_push = self.create_client(WaypointPush,   '/mavros/mission/push')
        self.cli_arm  = self.create_client(CommandBool,     '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode,         '/mavros/set_mode')

        # 2) Subscribe to '/mavros/mission/reached' with BEST_EFFORT QoS
        be_qos = QoSProfile(depth=10)
        be_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.reached_sub = self.create_subscription(
            UInt16,
            '/mavros/mission/reached',
            self.reached_cb,
            qos_profile=be_qos
        )

        # Wait for all three services to become available
        for client, name in [
            (self.cli_push, '/mavros/mission/push'),
            (self.cli_arm,  '/mavros/cmd/arming'),
            (self.cli_mode, '/mavros/set_mode')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}…')

        # 3) Build a 5-point square mission at 10 m altitude
        home_lat = 47.397742   # SITL default home latitude
        home_lon = 8.545594    # SITL default home longitude
        side_m   = 15.0        # 15-meter square side length
        # Approximate 15 m in degrees: 1° ≈ 111 km → 15 m ≈ 0.000135°
        delta_deg = side_m / 111000.0

        self.wp_list = []

        # WP0: home → climb to 10 m (index=0, is_current=True)
        wp0 = Waypoint()
        wp0.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp0.command      = MAV_CMD_NAV_WAYPOINT
        wp0.is_current   = True
        wp0.autocontinue = True
        wp0.param1 = 0.0; wp0.param2 = 0.0; wp0.param3 = 0.0
        wp0.param4 = float('nan')      # “unset” yaw → use MPC_YAW_MODE
        wp0.x_lat  = home_lat
        wp0.y_long = home_lon
        wp0.z_alt  = 10.0
        self.wp_list.append(wp0)

        # WP1: 15 m north of home
        wp1 = Waypoint()
        wp1.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp1.command      = MAV_CMD_NAV_WAYPOINT
        wp1.is_current   = False
        wp1.autocontinue = True
        wp1.param1 = 0.0; wp1.param2 = 0.0; wp1.param3 = 0.0
        wp1.param4 = float('nan')
        wp1.x_lat  = home_lat + delta_deg
        wp1.y_long = home_lon
        wp1.z_alt  = 10.0
        self.wp_list.append(wp1)

        # WP2: 15 m northeast (home_lat + delta, home_lon + delta)
        wp2 = Waypoint()
        wp2.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp2.command      = MAV_CMD_NAV_WAYPOINT
        wp2.is_current   = False
        wp2.autocontinue = True
        wp2.param1 = 0.0; wp2.param2 = 0.0; wp2.param3 = 0.0
        wp2.param4 = float('nan')
        wp2.x_lat  = home_lat + delta_deg
        wp2.y_long = home_lon + delta_deg
        wp2.z_alt  = 10.0
        self.wp_list.append(wp2)

        # WP3: 15 m east of home (home_lat, home_lon + delta)
        wp3 = Waypoint()
        wp3.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp3.command      = MAV_CMD_NAV_WAYPOINT
        wp3.is_current   = False
        wp3.autocontinue = True
        wp3.param1 = 0.0; wp3.param2 = 0.0; wp3.param3 = 0.0
        wp3.param4 = float('nan')
        wp3.x_lat  = home_lat
        wp3.y_long = home_lon + delta_deg
        wp3.z_alt  = 10.0
        self.wp_list.append(wp3)

        # WP4: return to home at 10 m
        wp4 = Waypoint()
        wp4.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp4.command      = MAV_CMD_NAV_WAYPOINT
        wp4.is_current   = False
        wp4.autocontinue = True
        wp4.param1 = 0.0; wp4.param2 = 0.0; wp4.param3 = 0.0
        wp4.param4 = float('nan')
        wp4.x_lat  = home_lat
        wp4.y_long = home_lon
        wp4.z_alt  = 10.0
        self.wp_list.append(wp4)

        # 4) Push → arm → set to AUTO.MISSION
        self.push_mission()

    def push_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = self.wp_list

        self.get_logger().info(f'Pushing {len(self.wp_list)} waypoints…')
        future = self.cli_push.call_async(req)
        future.add_done_callback(self.push_cb)

    def push_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'Mission push failed (result={resp.result})')
            return
        self.get_logger().info('Mission pushed ✓ → arming vehicle…')
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
        self.get_logger().info('Armed ✓ → switching to AUTO.MISSION…')
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
            self.get_logger().error('Failed to set AUTO.MISSION')
        else:
            self.get_logger().info('Mode set → PX4 will now fly the square smoothly.')
        # Keep node alive to continue logging reached indices

    def reached_cb(self, msg: UInt16):
        self.get_logger().info(f'Reached waypoint index: {msg.data}')


def main(args=None):
    # Before running, ensure PX4’s yaw mode is set:
    # ros2 param set /px4 MPC_YAW_MODE 0
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
