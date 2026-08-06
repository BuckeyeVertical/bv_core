#!/usr/bin/env python3
"""Watch a live mission and print a PASS/FAIL verdict for the approval gate.

Run alongside the sim so an operator does not have to read raw logs:

    ros2 run bv_core sim_approval_check

Then fly the mission and act on the detection. Ctrl-C prints the verdict.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from bv_msgs.msg import ObjectLocations, PendingDetection
from std_msgs.msg import String


class SimApprovalCheck(Node):

    def __init__(self):
        super().__init__('sim_approval_check')

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.history = HistoryPolicy.KEEP_LAST
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            PendingDetection, '/pending_obj_dets', self._on_pending, latched)
        # Latched too: mission_node publishes /mission_state TRANSIENT_LOCAL, and
        # two checks below read state history. A tool started after takeoff would
        # otherwise miss every earlier state and report a spurious FAIL.
        self.create_subscription(
            String, '/mission_state', self._on_state, latched)
        self.create_subscription(
            ObjectLocations, '/rejected_object_locations',
            self._on_rejected, reliable)

        self.states = []
        self.pendings = []
        self.rejections = []
        self.resolutions = 0
        self.state_at_pending = None

        self.get_logger().info(
            'sim_approval_check watching. Fly the mission, then Ctrl-C.')

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)

    def _on_pending(self, msg):
        if not msg.detection_id:
            # An empty detection_id is the gate's "nothing pending" convention,
            # published on every resolution: approve, reject, timeout or cancel.
            if self.pendings:
                self.resolutions += 1
            return
        self.state_at_pending = self.states[-1] if self.states else None
        self.pendings.append({
            'id': msg.detection_id,
            'class_id': int(msg.class_id),
            'crop_bytes': len(msg.annotated_crop.data),
            'timeout': float(msg.timeout_sec),
            'state': self.state_at_pending,
        })
        self.get_logger().info(
            f"pending #{len(self.pendings)} crop="
            f"{len(msg.annotated_crop.data)}B state={self.state_at_pending}")

    def _on_rejected(self, msg):
        self.rejections.append((msg.latitude, msg.longitude, int(msg.class_id)))

    def _outcomes(self):
        """Split the observed pendings into approvals and rejections.

        A pending that cleared without a matching rejection was approved — or
        timed out, which the fail-open design deliberately treats the same way.
        A pending still open (or whose clear was missed) with no rejection to
        explain it is held to the approve-path expectation rather than skipped,
        so a stalled approval cannot quietly drop its own checks.
        """
        rejected = len(self.rejections)
        approved = max(self.resolutions - rejected, 0)
        if self.pendings and rejected == 0:
            approved = max(approved, 1)
        return approved, rejected

    def verdict(self):
        checks = []
        approved, rejected = self._outcomes()

        checks.append((
            bool(self.pendings),
            f"{len(self.pendings)} pending(s) published"))

        if self.pendings:
            first = self.pendings[0]
            checks.append((
                first['state'] == 'localize',
                f"pending published while state=localize "
                f"(saw {first['state']})"))
            checks.append((
                first['crop_bytes'] > 0,
                f"pending carried a crop ({first['crop_bytes']} bytes)"))
            checks.append((
                first['timeout'] > 0,
                f"timeout advertised to the GCS ({first['timeout']:.0f}s)"))

        # Both sets apply in a mixed run: an approved detection must still reach
        # DELIVER and DEPLOY even if some other detection was rejected.
        if approved:
            checks.append((
                'deliver' in self.states,
                f"{approved} approved -> reached DELIVER"))
            checks.append((
                'deploy' in self.states,
                f"{approved} approved -> reached DEPLOY"))

        if rejected:
            checks.append((
                rejected <= len(self.pendings),
                f"{rejected} rejection(s) published for filtering suppression"))
            checks.append((
                self.states.count('scan') >= 2,
                f"{rejected} rejected -> returned to scan"))

        print(f'\n[sim-check] {approved} approved, {rejected} rejected')
        print('[sim-check] state sequence: ' + ' -> '.join(self.states))
        for ok, text in checks:
            print(f"  {'PASS' if ok else 'FAIL'}  {text}")
        result = all(ok for ok, _ in checks)
        print(f"  RESULT: {'PASS' if result else 'FAIL'}\n")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimApprovalCheck()
    passed = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        passed = node.verdict()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
