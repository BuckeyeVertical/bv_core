#!/usr/bin/env python3
"""Bench tool for the payload servos, in microseconds like the mission config.

    ros2 run bv_core test_servo show            # configured pulses, DIS values
    ros2 run bv_core test_servo plate 1685      # one servo to a pulse width
    ros2 run bv_core test_servo clamp 1900
    ros2 run bv_core test_servo rest            # plate hold + clamp open
    ros2 run bv_core test_servo drop bottle     # full drop sequence (or beacon)

Reads the payload block from the mission config selected by BV_MISSION_CONFIG
(default real_params.yaml), ignoring payload.enabled and the startup range
check, so it works while the payload is disabled.

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
    parse_payload_block,
    unreachable_reason,
)

USAGE = __doc__.split('\n\n')[1]
DROP_TICK_SEC = 0.02


def parse_args(argv):
    """('servo', name, us) | ('show',) | ('rest',) | ('drop', payload).

    Raises SystemExit with the usage text on anything else.
    """
    if argv in (['show'], ['rest']):
        return (argv[0],)
    if len(argv) == 2 and argv[0] == 'drop' and argv[1] in PAYLOADS:
        return ('drop', argv[1])
    if len(argv) == 2 and argv[0] in ('plate', 'clamp'):
        try:
            return ('servo', argv[0], float(argv[1]))
        except ValueError:
            pass
    raise SystemExit(f"usage:\n{USAGE}")


def describe(config):
    """Every position with its range status, then the DIS values.

    Shows whether everything fits the PWM range and which disarmed pulses to
    set on the flight controller.
    """
    lines = []
    for servo, name, pulse in config.positions():
        reason = unreachable_reason(servo, pulse)
        status = f"OUT OF RANGE: {reason}" if reason else "ok"
        lines.append(f"{servo.name} {name}: {pulse:g} us [{status}]")
    for servo, pulse in ((config.plate, config.plate_hold_us),
                         (config.clamp, config.unclamped_us)):
        lines.append(f"PWM_MAIN_DIS ({servo.name}): {pulse:g}")
    return lines


def drop_schedule(config, payload, tick_s=DROP_TICK_SEC):
    """(t, plate_us, clamp_us) at every position change in a drop."""
    sequence = DropSequence(config, payload)
    schedule = []
    last = None
    step = 0
    while True:
        t = step * tick_s
        plate, clamp, done = sequence.positions_at(t)
        if (plate, clamp) != last:
            schedule.append((t, plate, clamp))
            last = (plate, clamp)
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

    def send(self, plate_us=None, clamp_us=None, quiet=False,
             ack_timeout_s=6.0):
        """Command the servos, addressed to PX4, and report its verdict.

        Addressed, not broadcast: PX4 on the flight controller answers a
        broadcast MAV_CMD_DO_SET_ACTUATOR with UNSUPPORTED and ignores it.
        Returns True when PX4 confirmed the command.
        """
        for servo, pulse in ((self.config.plate, plate_us),
                             (self.config.clamp, clamp_us)):
            if pulse is None:
                continue
            reason = unreachable_reason(servo, pulse)
            if reason:
                self.get_logger().warn(
                    f"{servo.name}: {reason} - PX4 will limit it to the range")
            if not quiet:
                self.get_logger().info(
                    f"{servo.name} (actuator set {servo.actuator_set}) -> "
                    f"{pulse:g} us")

        command, params = actuator_params(self.config, plate_us, clamp_us)
        request = CommandLong.Request()
        request.broadcast = False
        request.command = command
        (request.param1, request.param2, request.param3, request.param4,
         request.param5, request.param6, request.param7) = params
        future = self.client.call_async(request)
        # The default waits past MAVROS's own 5 s ACK timeout, so its
        # verdict arrives.
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=ack_timeout_s)
        response = future.result()
        if response is not None and response.success:
            if not quiet:
                self.get_logger().info('PX4 accepted')
            return True
        if response is not None and response.result != 0:
            self.get_logger().error(f'PX4 rejected (result={response.result})')
        else:
            # MAVROS reports a missing ACK as success=False, result=0.
            # The command may still have been applied: look at the servo.
            self.get_logger().warn(
                'No ACK from PX4 - the command may still have been applied; '
                'check the servo')
        return False

    def drop(self, payload):
        schedule = drop_schedule(self.config, payload)
        self.get_logger().info(
            f"Dropping {payload}: {len(schedule)} commands over "
            f"{self.config.total_duration_s:.2f}s")
        start = time.monotonic()
        failed = 0
        for t, plate, clamp in schedule:
            delay = start + t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            # Each command waits for PX4's reply (about 10 ms over serial);
            # a short timeout keeps one lost reply from stalling the drop.
            if not self.send(plate, clamp, quiet=True, ack_timeout_s=0.5):
                failed += 1
        remaining = start + self.config.total_duration_s - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        self.get_logger().info(
            f'Drop complete: {len(schedule) - failed}/{len(schedule)} '
            f'commands confirmed by PX4')


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    action = parse_args(argv)

    path = mission_config_path()
    with open(path, 'r') as stream:
        mission = yaml.safe_load(stream)
    try:
        config = parse_payload_block(mission.get('payload'))
    except ValueError as exc:
        rclpy.shutdown()
        raise SystemExit(
            f"{path}: {exc}\nThis config has no servo values; pick one that "
            f"does, e.g. BV_MISSION_CONFIG=real_params.yaml") from None

    if action[0] == 'show':
        # Pure config readout; needs no MAVROS.
        print(f"config: {mission_config_path()}")
        print('\n'.join(describe(config)))
        rclpy.shutdown()
        return

    node = ServoBench(config)
    try:
        if action[0] == 'rest':
            node.send(config.plate_hold_us, config.unclamped_us)
        elif action[0] == 'drop':
            node.drop(action[1])
        elif action[1] == 'plate':
            node.send(plate_us=action[2])
        else:
            node.send(clamp_us=action[2])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
