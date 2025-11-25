#!/usr/bin/env python3

import os
import yaml
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import Waypoint, State as MavState, WaypointReached, CommandCode
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandLong, ParamSetV2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int8

from rcl_interfaces.msg import ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory
from bv_msgs.srv import GetObjectLocations

# MAVLink constants
MAV_CMD_NAV_WAYPOINT = 16
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

        self.lap_points = cfg.get('lap_points', [])
        self.scan_points = cfg.get('scan_points', [])
        self.deliver_points = cfg.get('deliver_points', [])
        self.stitch_points = cfg.get('stitch_points', [])

        # Velocity and tolerance parameters
        self.lap_velocity = cfg.get('Lap_velocity', 5.0)
        self.lap_tolerance = cfg.get('Lap_tolerance', 3.0)
        self.scan_velocity = cfg.get('Scan_velocity', 3.0)
        self.scan_tolerance = cfg.get('Scan_tolerance', 1.0)
        self.stitch_velocity = cfg.get('Stitch_velocity', 5.0)
        self.stitch_tolerance = cfg.get('Stitch_tolerance', 1.0)
        self.deliver_tolerance = cfg.get('Deliver_tolerance', 1.0)

        # Servo PWM configuration parameters
        self.default_servo_pwms = cfg.get(
            'Default_servo_pwms',   [1500, 1500, 1500, 1500])
        self.deliver_initial_pwms = cfg.get(
            'Deliver_initial_pwms', [1600, 1600, 1600, 1600])
        self.deliver_second_pwms = cfg.get(
            'Deliver_second_pwms',  [1400, 1400, 1400, 1400])

        # Home position vars
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        # Subscribe to GPS for home position
        gps_qos = QoSProfile(depth=10)
        gps_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos_profile=gps_qos
        )

        # Get object location service
        self.cli_get_locs = self.create_client(
            GetObjectLocations,
            'get_object_locations'
        )
        while not self.cli_get_locs.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_object_locations service…')

        # MAVROS service clients
        self.cli_push = self.create_client(
            WaypointPush, '/mavros/mission/push')
        self.cli_arm = self.create_client(CommandBool,   '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode,       '/mavros/set_mode')
        self.cli_cmd = self.create_client(
            CommandLong,    '/mavros/cmd/command')
        self.cli_param = self.create_client(ParamSetV2,    '/mavros/param/set')

        # Wait for MAVROS services
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

        # Mission state publisher and queue state subscriber
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
            self.queue_state_callback,
            qos_profile=mission_state_qos
        )

        self.winch_state = 2          # 0-deploy, 1-retract, 2-idle
        self.last_winch_change = time.monotonic()

        # FSM variables
        self.state = 'lap'
        self.lap_count = 1
        self.deliver_index = 0
        self.in_auto_mission = False
        self.queue_state = 0
        self.try_deliver = False
        self.awaiting_object_locs = True
        self.async_block = False
        self.cli_queue = []
        self.retry_timer = None  # Helper for safe retries

        self.completed_deliver_build = False

    def use_cli(self, path, req, cb):
        # if self.async_block:
            # self.cli_queue.append((path, req, cb))
            # return
        # self.async_block = True
        fut = path.call_async(req)
        # def safe_cb(future):
        #     try:
        #         cb(future)
        #     finally:
        #         self.async_block = False
        fut.add_done_callback(cb)

    # def check_cli_queue(self):
    #     if self.async_block:
    #         return
    #     if not self.cli_queue:
    #         return
    #     command = self.cli_queue.pop(0)
    #     path = command[0]
    #     req = command[1]
    #     cb = command[2]
    #     self.async_block = True
    #     fut = path.call_async(req)
    #     def safe_cb(future):
    #         try:
    #             cb(future)
    #         finally:
    #             self.async_block = False
    #     fut.add_done_callback(safe_cb)


    def queue_state_callback(self, msg: Int8):
        if msg.data == 0:
            self.get_logger().info("Waiting for processing to complete", once=True)
        self.queue_state = msg.data

    def timer_callback(self):
        # Process one item from the async queue if available
        # self.check_cli_queue()

        msg = String()
        msg.data = self.state
        self.mission_state_pub.publish(msg)
        # print(f"-> \n {self.queue_state} \n {self.scan_done} \n {self.transition_in_progress} \n <-")
        # if self.queue_state == 1 and self.scan_done and not self.transition_in_progress:
        if self.try_deliver and not self.transition_in_progress:
            self.get_logger().info("Starting deliver")
            self.try_deliver = False
            self.start_deliver()
            if self.completed_deliver_build is True:
                self.try_deliver = False

        # --- FIX APPLIED HERE ---
        if self.state in ('deploy',):             # run only while we are in DEPLOY
            now = time.monotonic()
            if now - self.last_winch_change >= 5.0:    # 5-second cadence
                # cycle 0 ➜ 1 ➜ 2 ➜ 0 …
                self.winch_state = (self.winch_state + 1) % 3
                self.last_winch_change = now

                # console feedback so we can watch it in SITL
                self.get_logger().info(
                    f'[DEPLOY] Winch state → {self.winch_state}')

                # command the servo (real HW) or just print (SITL)
                # INDENTATION FIXED: This now only runs once every 5 seconds
                self.deploy()

    def build_waypoints(self, waypoint_list, tolerance, pass_through_ratio=0.0):
        wp_list = []
        if self.state == 'deliver':
            time_delay = 1.0
        elif self.state in ('scan', 'stitching'):
            time_delay = 1.0
        else:
            time_delay = 0.0

        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = Waypoint()
            wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = MAV_CMD_NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.param1 = time_delay
            wp.param2 = tolerance
            wp.param3 = tolerance*pass_through_ratio
            wp.param4 = float('nan')
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            wp_list.append(wp)
        return wp_list

    def set_servo_pwm(self, servo_num, pwm_value):
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_num)  # servo channel
        req.param2 = float(pwm_value)  # PWM (e.g., 1500)
        self.use_cli(self.cli_cmd, req, self.dummy_cb)

    def dummy_cb(self, future):
        return

    def start_lap(self):
        self.state = 'lap'
        self.transition_in_progress = True
        self.desired_speed = self.lap_velocity
        self.get_logger().info(f'Starting lap {self.lap_count}…')
        self.wp_list = self.build_waypoints(
            self.lap_points, self.lap_tolerance, 1.0)
        self.expected_final_wp = len(self.wp_list)-1
        self.push_mission()

    def start_stitching(self):
        self.state = 'stitching'
        self.transition_in_progress = True
        self.desired_speed = self.stitch_velocity
        self.get_logger().info('Starting stitching…')
        self.wp_list = self.build_waypoints(
            self.stitch_points, self.stitch_tolerance)
        self.expected_final_wp = len(self.wp_list)-1
        self.push_mission()

    def start_scan(self):
        self.state = 'scan'
        self.transition_in_progress = True
        self.desired_speed = self.scan_velocity
        self.get_logger().info('Starting scan…')
        self.wp_list = self.build_waypoints(
            self.scan_points, self.scan_tolerance, 1.0)
        self.expected_final_wp = len(self.wp_list)-1
        self.push_mission()

    def start_deliver(self):
        self.state = 'deliver'
        self.transition_in_progress = True
        self.desired_speed = self.lap_velocity

        if self.awaiting_object_locs:
            self.get_logger().info('Waiting for object locations')
            self.deliver_points = []
            self.request_object_locations()
        else:
            pt = self.deliver_index
            lat, lon, alt = self.deliver_points[pt]
            self.get_logger().info(f'Delivering to point {pt+1}...')
            self.wp_list = self.build_waypoints(
                [(lat, lon, alt)], self.deliver_tolerance)
            self.expected_final_wp = len(self.wp_list)-1
            self.push_mission()
            self.state = 'deliver'
            self.completed_deliver_build = True

    def return_to_rtl(self):
        self.state = 'return'
        self.transition_in_progress = True
        self.get_logger().info('Switching to AUTO.RTL…')
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'AUTO.RTL'
        self.use_cli(self.cli_mode, req, self.mode_cb)

    def request_object_locations(self):
        req = GetObjectLocations.Request()
        self.use_cli(self.cli_get_locs, req, self.object_locs_cb)

    def object_locs_cb(self, future):
        resp = future.result()
        locs = resp.locations
        if not locs:
            # FIX APPLIED: Removed recursive create_timer which caused exponential spam.
            # We now just log a warning.
            self.get_logger().warn('No drop points received. Retrying in 3s via safe timer...')

            # Use a safe, non-recursive check or single-shot
            if self.retry_timer is not None:
                self.retry_timer.cancel()
            self.retry_timer = self.create_timer(3.0, self.retry_loc_req)
            return

        # If we got data, cancel any pending retries
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

        if self.state != 'deliver':
            self.deliver_points = [
                (loc.latitude, loc.longitude, self.home_alt or 0.0) for loc in locs]
            return
        self.awaiting_object_locs = False
        self.deliver_points = [
            (loc.latitude, loc.longitude, self.home_alt or 0.0) for loc in locs]
        self.deliver_index = 0
        self.get_logger().info(
            f'Received {len(self.deliver_points)} drop point(s) → pushing deliver mission'
        )
        # Re-trigger deliver build now that we have points
        self.start_deliver()

    def retry_loc_req(self):
        # One-shot retry callback
        self.get_logger().info("Retrying object location request...")
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.request_object_locations()

    def deploy(self):
        # determine servo channel based on lap_count
        idx = (self.lap_count - 1) % 4
        channel = idx + 1
        init_pwm, second_pwm, default_pwm = (
            self.deliver_initial_pwms[idx],
            self.deliver_second_pwms[idx],
            self.default_servo_pwms[idx],
        )

        if self.winch_state == 0:
            self.get_logger().info(f'[DEPLOY] CH{channel} ← {init_pwm}')
            self.set_servo_pwm(channel, init_pwm)
        elif self.winch_state == 1:
            self.get_logger().info(f'[DEPLOY] CH{channel} ← {second_pwm}')
            self.set_servo_pwm(channel, second_pwm)
        elif self.winch_state == 2:
            self.get_logger().info(
                f'[DEPLOY] CH{channel} ← {default_pwm} (idle)')
            self.set_servo_pwm(channel, default_pwm)

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
            self.try_deliver = True
        elif self.state == 'deliver':
            self.state = 'deploy'
        elif self.state == 'deploy':
            self.deliver_index += 1
            self.lap_count += 1
            if self.deliver_index < len(self.deliver_points):
                self.start_lap()
            else:
                self.return_to_rtl()
        elif self.state == 'return':
            self.get_logger().info('Mission complete - landed!')

    def push_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = self.wp_list
        self.get_logger().info(f'Pushing {len(self.wp_list)} waypoint(s)…')
        self.use_cli(self.cli_push, req, self.push_cb)

    def push_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(
                f'Push failed: transferred {resp.wp_transfered}')
            self.transition_in_progress = False
            return

        self.get_logger().info('Mission pushed ✓ -> setting mode')
        mode_req = SetMode.Request()
        mode_req.base_mode = 0
        mode_req.custom_mode = 'AUTO.MISSION'
        self.use_cli(self.cli_mode, mode_req, self.mode_cb)

    def mode_cb(self, future):
        resp = future.result()
        if not resp.mode_sent:
            self.get_logger().error('Mode change failed')
            self.transition_in_progress = False
            return
        self.get_logger().info('Mode set ✓ → flying...')
        self.in_auto_mission = True
        self.transition_in_progress = False
        # Reset all servos to default on takeoff
        for i, pwm in enumerate(self.default_servo_pwms, start=1):
            self.set_servo_pwm(i, pwm)
        self.set_mpc_xy_vel_all(self.desired_speed)

    def arm_cb(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'Arming FAILED {resp} trying again')
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.use_cli(self.cli_arm, arm_req, self.arm_cb)
            self.transition_in_progress = False
            return

        self.get_logger().info('Armed')
        # STARTING POINT
        self.start_lap()

    def set_mpc_xy_vel_all(self, speed):
        req = ParamSetV2.Request()
        req.force_set = False
        req.param_id = 'MPC_XY_VEL_ALL'
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(speed)
        req.value = pv
        self.use_cli(self.cli_param, req, self.param_cb)

    def param_cb(self, future):
        resp = future.result()
        if resp.success:
            self.get_logger().info(
                f'MPC_XY_VEL_ALL set to {resp.value.double_value:.2f} m/s')
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
                f'GPS fixed: lat={self.home_lat:.6f}, lon={self.home_lon:.6f}, alt={self.home_alt:.2f}')
            self.get_logger().info(f"ARMING")
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.use_cli(self.cli_arm, arm_req, self.arm_cb)
            self.timer = self.create_timer(0.5, self.timer_callback)


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
