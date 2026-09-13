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
    parse_brake_phases,
    parse_payload_block,
    unreachable_reason,
)
from .payload_writer import PayloadWriter, actuator_request

USAGE = __doc__.split('\n\n')[1]
# Loop period while driving a drop; mission_node's DEPLOY timer is 20 ms.
DROP_TICK_SEC = 0.005


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

    def send(self, plate_us=None, clamp_us=None):
        """Command the servos once and report PX4's verdict. True if confirmed."""
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

        future = self.client.call_async(
            actuator_request(self.config, plate_us, clamp_us))
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
        """Run a drop through mission_node's own code.

        The same DropSequence decides the pulses and the same PayloadWriter
        sends them; only the loop driving it differs (mission_node uses a
        20 ms ROS timer). First the rest positions, confirmed: the first
        service call from a freshly started program can take ~1 s, which
        would otherwise eat the pre-brake.
        """
        config = self.config
        sequence = DropSequence(config, payload)
        writer = PayloadWriter(config, self.client, self.get_logger())

        self.get_logger().info('Starting from rest (plate hold, clamp open)')
        writer.set(config.plate_hold_us, config.unclamped_us)
        if not self._spin_until(lambda: writer.settled, timeout_s=6.0):
            self.get_logger().warn(
                'PX4 did not confirm the rest positions; dropping anyway')

        self.get_logger().info(
            f"Dropping {payload}: clamp braking for {config.pre_drop_s:g}s, "
            f"then the plate moves; {config.total_duration_s:.2f}s total")
        start = time.monotonic()
        sent_before, accepted_before = writer.sent, writer.accepted
        while True:
            plate, clamp, done = sequence.positions_at(time.monotonic() - start)
            writer.set(plate, clamp)
            writer.flush()
            if done and writer.settled:
                break
            if time.monotonic() - start > config.total_duration_s + 10.0:
                self.get_logger().error('Gave up waiting for PX4 to confirm')
                break
            rclpy.spin_once(self, timeout_sec=DROP_TICK_SEC)

        self.get_logger().info(
            f'Drop complete: {writer.accepted - accepted_before}/'
            f'{writer.sent - sent_before} commands confirmed by PX4')

    def _spin_until(self, condition, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        return True


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
