#!/usr/bin/env python3
"""Bench tool for the payload servos, in microseconds like the mission config.

    ros2 run bv_core test_servo show            # configured pulses, DIS values
    ros2 run bv_core test_servo plate 1685      # one servo to a pulse width
    ros2 run bv_core test_servo clamp 1900
    ros2 run bv_core test_servo rest            # plate hold + clamp open
    ros2 run bv_core test_servo drop bottle     # full drop sequence (or beacon)
    ros2 run bv_core test_servo drop bottle 150:3   # ...braking at 150 ms for 3 s

Brake phases given after the payload (toggle_ms:duration_s, one or more)
replace the configured ones for that drop only; the YAML is not touched.

Reads the payload block from the mission config selected by BV_MISSION_CONFIG
(default real_params.yaml), ignoring payload.enabled and the startup range
check, so it works while the payload is disabled.

PX4 only drives peripheral outputs while armed or pre-armed. On the bench,
remove the propellers and either arm or set COM_PREARM_MODE = 2 (Always).
See docs/HITL/payload.md.
"""

import dataclasses
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
    parse_brake_phases,
    parse_payload_block,
    unreachable_reason,
)

USAGE = __doc__.split('\n\n')[1]
# Pause before re-sending a command PX4 did not confirm (as mission_node).
RETRY_SEC = 0.2


def parse_args(argv):
    """('servo', name, us) | ('show',) | ('rest',) | ('drop', payload, phases).

    phases is None for the configured brake phases, else a tuple of
    BrakePhase from 'toggle_ms:duration_s' arguments. Raises SystemExit with
    the usage text on anything else.
    """
    if argv in (['show'], ['rest']):
        return (argv[0],)
    if len(argv) >= 2 and argv[0] == 'drop' and argv[1] in PAYLOADS:
        if len(argv) == 2:
            return ('drop', argv[1], None)
        try:
            return ('drop', argv[1], parse_brake_phases(argv[2:]))
        except ValueError as exc:
            raise SystemExit(f"{exc}\nusage:\n{USAGE}") from None
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


class ServoBench(Node):
    def __init__(self, config):
        super().__init__('servo_bench')
        self.config = config
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/command...')

    def _call(self, plate_us, clamp_us):
        """Send one command addressed to PX4; return the MAVROS future.

        Addressed, not broadcast: PX4 on the flight controller answers a
        broadcast MAV_CMD_DO_SET_ACTUATOR with UNSUPPORTED and ignores it.
        """
        command, params = actuator_params(self.config, plate_us, clamp_us)
        request = CommandLong.Request()
        request.broadcast = False
        request.command = command
        (request.param1, request.param2, request.param3, request.param4,
         request.param5, request.param6, request.param7) = params
        return self.client.call_async(request)

    def send(self, plate_us=None, clamp_us=None):
        """Command the servos and report PX4's verdict. True if confirmed."""
        for servo, pulse in ((self.config.plate, plate_us),
                             (self.config.clamp, clamp_us)):
            if pulse is None:
                continue
            reason = unreachable_reason(servo, pulse)
            if reason:
                self.get_logger().warn(
                    f"{servo.name}: {reason} - PX4 will limit it to the range")
            self.get_logger().info(
                f"{servo.name} (actuator set {servo.actuator_set}) -> "
                f"{pulse:g} us")

        future = self._call(plate_us, clamp_us)
        # Waits past MAVROS's own 5 s ACK timeout, so its verdict arrives.
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
        response = future.result()
        if response is not None and response.success:
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
        """Run the drop sequence the way mission_node does.

        First the rest positions, confirmed: the first service call from a
        freshly started program can take ~1 s, which would otherwise eat
        the pre-brake. Then one command in flight at a time, always the
        positions for *now*, re-sent after RETRY_SEC if PX4 did not confirm.
        The sequence runs on the clock and never waits on a reply.
        """
        config = self.config
        sequence = DropSequence(config, payload)
        self.get_logger().info('Starting from rest (plate hold, clamp open)')
        self.send(config.plate_hold_us, config.unclamped_us)

        self.get_logger().info(
            f"Dropping {payload}: clamp braking for {config.pre_drop_s:g}s, "
            f"then the plate moves; {config.total_duration_s:.2f}s total")
        start = time.monotonic()
        pending = None            # (future, positions) awaiting PX4's reply
        confirmed = None
        retry_at = 0.0
        sent = accepted = 0
        while True:
            now = time.monotonic()
            plate, clamp, done = sequence.positions_at(now - start)
            desired = (plate, clamp)

            if pending is not None and pending[0].done():
                future, positions = pending
                pending = None
                response = future.result()
                if response is not None and response.success:
                    accepted += 1
                    confirmed = positions
                else:
                    retry_at = now + RETRY_SEC

            if pending is None and desired != confirmed and now >= retry_at:
                pending = (self._call(plate, clamp), desired)
                sent += 1

            if done and pending is None and desired == confirmed:
                break
            if now - start > config.total_duration_s + 10.0:
                self.get_logger().error('Gave up waiting for PX4 to confirm')
                break
            rclpy.spin_once(self, timeout_sec=0.005)

        self.get_logger().info(
            f'Drop complete: {accepted}/{sent} commands confirmed by PX4')


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
            if action[2] is not None:
                node.config = dataclasses.replace(
                    config, brake_phases=action[2])
                node.get_logger().info(
                    'Brake phases from the command line: ' + ', '.join(
                        f'{p.toggle_ms:g} ms for {p.duration_s:g} s'
                        for p in action[2]))
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
