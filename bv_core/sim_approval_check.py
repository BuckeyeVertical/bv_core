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
        self.create_subscription(
            String, '/mission_state', self._on_state, reliable)
        self.create_subscription(
            ObjectLocations, '/rejected_object_locations',
            self._on_rejected, reliable)

        self.states = []
        self.pendings = []
        self.rejections = []
        self.state_at_pending = None

        self.get_logger().info(
            'sim_approval_check watching. Fly the mission, then Ctrl-C.')

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)

    def _on_pending(self, msg):
        if not msg.detection_id:
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

    def verdict(self):
        checks = []

        if not self.pendings:
            checks.append((False, 'a pending detection was published'))
        else:
            first = self.pendings[0]
            checks.append((True, f"{len(self.pendings)} pending(s) published"))
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

        reached_deliver = 'deliver' in self.states
        reached_deploy = 'deploy' in self.states

        if self.rejections:
            checks.append((True,
                           f"{len(self.rejections)} rejection(s) published "
                           f"for filtering suppression"))
            checks.append((
                self.states.count('scan') >= 2,
                'returned to scan after rejection'))
        else:
            checks.append((reached_deliver, 'reached DELIVER'))
            checks.append((reached_deploy, 'reached DEPLOY'))

        print('\n[sim-check] state sequence: ' + ' -> '.join(self.states))
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
