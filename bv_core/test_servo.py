#!/usr/bin/env python3
"""Bench tool for the payload servos, in the same degrees as the mission config.

    ros2 run bv_core test_servo slider 130      # one servo to an angle
    ros2 run bv_core test_servo brake 107
    ros2 run bv_core test_servo rest            # slider hold + brake rest
    ros2 run bv_core test_servo drop bottle     # full drop sequence (or beacon)

Reads the payload block from the mission config selected by BV_MISSION_CONFIG
(default real_params.yaml), ignoring payload.enabled and the startup range check:
finding a position the flight controller can reach is what this tool is for.

PX4 only drives peripheral outputs while armed or pre-armed. On the bench,
remove the propellers and either arm or set COM_PREARM_MODE = 2 (Always).
See docs/HITL/payload.md.
"""

import sys
import time

import rclpy
import yaml
from mavros_msgs.srv import CommandLong
from rclpy.node import Node

from .mission_config import mission_config_path
from .payload import (
    PAYLOADS,
    DropSequence,
    actuator_params,
    degrees_to_us,
    parse_payload_block,
    unreachable_reason,
)

USAGE = __doc__.split('\n\n')[1]
DROP_TICK_SEC = 0.02


def parse_args(argv):
    """('servo', name, degrees) | ('rest',) | ('drop', payload); SystemExit on error."""
    if argv == ['rest']:
        return ('rest',)
    if len(argv) == 2 and argv[0] == 'drop' and argv[1] in PAYLOADS:
        return ('drop', argv[1])
    if len(argv) == 2 and argv[0] in ('slider', 'brake'):
        try:
            return ('servo', argv[0], float(argv[1]))
        except ValueError:
            pass
    raise SystemExit(f"usage:\n{USAGE}")


def drop_schedule(config, payload, tick_s=DROP_TICK_SEC):
    """(t, slider_deg, brake_deg) at every position change in a drop."""
    sequence = DropSequence(config, payload)
    schedule = []
    last = None
    step = 0
    while True:
        t = step * tick_s
        slider, brake, done = sequence.positions_at(t)
        if (slider, brake) != last:
            schedule.append((t, slider, brake))
            last = (slider, brake)
        if done:
            return schedule
        step += 1


class ServoBench(Node):
    def __init__(self, config):
        super().__init__('servo_bench')
        self.config = config
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/command...')

    def send(self, slider_deg=None, brake_deg=None, wait_for_ack=True):
        for servo, degrees in ((self.config.slider, slider_deg),
                               (self.config.brake, brake_deg)):
            if degrees is None:
                continue
            pulse = degrees_to_us(degrees, self.config.arduino_pulse_range_us)
            reason = unreachable_reason(self.config, servo, degrees)
            if reason:
                self.get_logger().warn(
                    f"{servo.name}: {reason} - PX4 will clamp it")
            self.get_logger().info(
                f"{servo.name} (actuator set {servo.actuator_set}) -> "
                f"{degrees:g} deg = {pulse:.0f} us")

        command, params = actuator_params(self.config, slider_deg, brake_deg)
        request = CommandLong.Request()
        # Same as the mission during a drop: broadcast skips MAVROS's ACK wait.
        request.broadcast = not wait_for_ack
        request.command = command
        (request.param1, request.param2, request.param3, request.param4,
         request.param5, request.param6, request.param7) = params
        future = self.client.call_async(request)
        if wait_for_ack:
            # Past MAVROS's own 5 s ACK timeout, so its verdict arrives.
            rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info('PX4 accepted')
            elif response is not None and response.result != 0:
                self.get_logger().error(
                    f'PX4 rejected (result={response.result})')
            else:
                # MAVROS reports a missing ACK as success=False, result=0.
                # The command may still have been applied: look at the servo.
                self.get_logger().warn(
                    'No ACK from PX4 - the command may still have been '
                    'applied; check the servo')

    def drop(self, payload):
        schedule = drop_schedule(self.config, payload)
        self.get_logger().info(
            f"Dropping {payload}: {len(schedule)} commands over "
            f"{self.config.total_duration_s:.2f}s")
        start = time.monotonic()
        for t, slider, brake in schedule:
            delay = start + t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self.send(slider, brake, wait_for_ack=False)
            rclpy.spin_once(self, timeout_sec=0.0)
        remaining = start + self.config.total_duration_s - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        self.get_logger().info('Drop complete')


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    action = parse_args(argv)

    with open(mission_config_path(), 'r') as stream:
        mission = yaml.safe_load(stream)
    config = parse_payload_block(mission.get('payload'))

    node = ServoBench(config)
    try:
        if action[0] == 'rest':
            node.send(config.slider_hold_deg, config.brake_rest_deg)
        elif action[0] == 'drop':
            node.drop(action[1])
        elif action[1] == 'slider':
            node.send(slider_deg=action[2])
        else:
            node.send(brake_deg=action[2])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
