#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import relevant MAVROS message/service types
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool

MAV_CMD_NAV_WAYPOINT       = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

        # 1) Create clients for push, arm, and set_mode
        self.cli_push = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_arm  = self.create_client(CommandBool,    '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode,        '/mavros/set_mode')

        # Wait for all services
        for cli, name in [(self.cli_push, '/mavros/mission/push'),
                          (self.cli_arm,  '/mavros/cmd/arming'),
                          (self.cli_mode, '/mavros/set_mode')]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}…')

        # 2) Build a simple 2-waypoint mission (modify as needed)
        self.wp_list = []
        wp0 = Waypoint()
        wp0.frame       = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp0.command     = MAV_CMD_NAV_WAYPOINT
        wp0.is_current  = True
        wp0.autocontinue= True
        wp0.param1 = 0.0; wp0.param2 = 0.0; wp0.param3 = 0.0; wp0.param4 = 0.0
        wp0.x_lat  = 47.397742  # example lat (SITL home)
        wp0.y_long = 8.545594   # example lon 
        wp0.z_alt  = 10.0       # climb to 10 m
        self.wp_list.append(wp0)

        wp1 = Waypoint()
        wp1.frame       = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp1.command     = MAV_CMD_NAV_WAYPOINT
        wp1.is_current  = False
        wp1.autocontinue= True
        wp1.param1 = 0.0; wp1.param2 = 0.0; wp1.param3 = 0.0; wp1.param4 = 0.0
        wp1.x_lat  = 47.397842  # ~11 m north of home
        wp1.y_long = 8.545594
        wp1.z_alt  = 10.0
        self.wp_list.append(wp1)

        # 3) Chain the calls: push → arm → set_mode
        self.push_mission()

    def push_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = self.wp_list

        self.get_logger().info(f'Pushing {len(self.wp_list)} WPs…')
        f = self.cli_push.call_async(req)
        f.add_done_callback(self.push_cb)

    def push_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'Push failed (result={resp.result})')
            rclpy.shutdown()
            return
        self.get_logger().info('Mission pushed ✓ → now arming…')
        self.arm_vehicle()

    def arm_vehicle(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        f = self.cli_arm.call_async(arm_req)
        f.add_done_callback(self.arm_cb)

    def arm_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error('Arming FAILED')
            rclpy.shutdown()
            return
        self.get_logger().info('Armed ✓ → switching to AUTO.MISSION…')
        self.switch_to_mission()

    def switch_to_mission(self):
        mode_req = SetMode.Request()
        mode_req.base_mode   = 0
        mode_req.custom_mode = 'AUTO.MISSION'
        f = self.cli_mode.call_async(mode_req)
        f.add_done_callback(self.mode_cb)

    def mode_cb(self, future):
        resp = future.result()
        if not resp.mode_sent:
            self.get_logger().error('Failed to set AUTO.MISSION')
        else:
            self.get_logger().info('Mode set → PX4 will start mission now.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
