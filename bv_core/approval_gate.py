#!/usr/bin/env python3
"""Operator approval gate for a single localized detection.

mission_node hands a localized detection here instead of flying to it directly.
The gate publishes it for the ground station, waits for a verdict on
/detection_decision, and calls back into the mission.

This node is the *authority*: it mints the detection id and owns the timeout.
bv_gcs/approval_node is only a relay, so if the radio link or the ground station
process dies the timer here still fires and the mission continues.

The timeout fails OPEN — expiry runs the approve path. The degraded mode is
therefore exactly the autonomous behavior the aircraft already flies.
"""

import uuid

from bv_msgs.msg import PendingDetection
from bv_msgs.srv import DetectionDecision

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ApprovalGate:
    """Holds at most one pending detection awaiting an operator verdict."""

    def __init__(self, node, timeout_sec, log=None):
        """Create the gate's publisher and decision service.

        Args:
            node: the rclpy Node that owns the publisher, service and timer.
            timeout_sec: seconds before auto-approving. 0 waits forever.
            log: optional MissionLogger for structured mission events.
        """
        self._node = node
        self._timeout_sec = float(timeout_sec)
        self._log = log

        self._pending = None      # dict describing the live detection
        self._timer = None
        self._on_approve = None
        self._on_reject = None

        # TRANSIENT_LOCAL so a restarted approval_node, or a link that drops and
        # recovers mid-decision, immediately gets the live pending instead of
        # showing an empty dashboard while the aircraft counts down.
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.history = HistoryPolicy.KEEP_LAST

        self._pub = node.create_publisher(
            PendingDetection, '/pending_obj_dets', latched)
        self._srv = node.create_service(
            DetectionDecision, '/detection_decision', self._on_decision)

    # -- public API -------------------------------------------------------

    def is_pending(self):
        """Return True while a detection is awaiting a verdict."""
        return self._pending is not None

    def request(self, *, class_id, lat, lon, alt, confidence,
                drone_lat, drone_lon, annotated_crop,
                on_approve, on_reject):
        """Publish a detection for review and start the timeout.

        Args:
            class_id: semantic class id of the detection.
            lat, lon, alt: localized object position.
            confidence: detector confidence, 0.0 if unknown.
            drone_lat, drone_lon: aircraft position at localization time.
            annotated_crop: JPEG bytes, or b'' when the crop could not be built.
            on_approve: called with no arguments on approve or timeout.
            on_reject: called as on_reject(lat, lon, class_id, reason).

        Returns:
            The generated detection_id.
        """
        if self._pending is not None:
            # Replace rather than ignore: a stale detection the aircraft has
            # already moved on from is worse than a surprising swap. Clearing
            # also kills the old timer, which would otherwise fire on the FIRST
            # detection's deadline and auto-approve THIS one early.
            self._node.get_logger().warn(
                f"superseding pending approval id={self._pending['detection_id']} "
                f"with a new detection; the operator's decision window restarts")
            self._clear()

        detection_id = str(uuid.uuid4())

        self._pending = {
            'detection_id': detection_id,
            'class_id': int(class_id),
            'lat': float(lat),
            'lon': float(lon),
        }
        self._on_approve = on_approve
        self._on_reject = on_reject

        msg = PendingDetection()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.detection_id = detection_id
        msg.class_id = int(class_id)
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        msg.confidence = float(confidence)
        msg.drone_latitude = float(drone_lat)
        msg.drone_longitude = float(drone_lon)
        msg.timeout_sec = float(self._timeout_sec)
        msg.annotated_crop.header = msg.header
        msg.annotated_crop.format = 'jpeg'
        msg.annotated_crop.data = bytes(annotated_crop)
        self._pub.publish(msg)

        if self._pub.get_subscription_count() == 0:
            # Not fatal — the timeout will still deploy — but the operator
            # should know nobody is listening before they wait three minutes.
            self._node.get_logger().warn(
                'pending published but no approval_node is subscribed; '
                'the decision will time out and deploy')

        if self._timeout_sec > 0:
            self._timer = self._node.create_timer(
                self._timeout_sec, self._on_timeout)

        self._node.get_logger().info(
            f"AWAITING APPROVAL id={detection_id} class_id={class_id} "
            f"lat={lat:.6f} lon={lon:.6f} timeout={self._timeout_sec:.0f}s "
            f"crop={len(annotated_crop)}B")
        if self._log:
            self._log.event(
                'APPROVAL_REQUESTED',
                f"id={detection_id}, class={class_id}, "
                f"lat={lat:.6f}, lon={lon:.6f}, timeout={self._timeout_sec:.0f}s")

        return detection_id

    def cancel(self, reason):
        """Abandon the pending without running either callback."""
        if self._pending is None:
            return
        detection_id = self._pending['detection_id']
        self._clear()
        self._node.get_logger().info(
            f"approval cancelled id={detection_id} reason={reason}")
        if self._log:
            self._log.event('APPROVAL_CANCELLED',
                            f"id={detection_id}, reason={reason}")

    # -- callbacks --------------------------------------------------------

    def _on_decision(self, request, response):
        if self._pending is None:
            response.accepted = False
            response.message = 'no active pending detection'
            return response

        if request.detection_id != self._pending['detection_id']:
            response.accepted = False
            response.message = (
                f"detection_id mismatch "
                f"(active={self._pending['detection_id']})")
            return response

        pending = self._pending
        on_approve = self._on_approve
        on_reject = self._on_reject
        # Clear before the callback: the callback drives an FSM transition and
        # must not observe a pending that has already been decided.
        self._clear()

        if request.approved:
            response.accepted = True
            response.message = 'approved - flying to object'
            self._node.get_logger().info(
                f"APPROVED id={pending['detection_id']}")
            if self._log:
                self._log.event('APPROVAL_GRANTED',
                                f"id={pending['detection_id']}, source=operator")
            # accepted stays True: the verdict *was* accepted. The action it
            # triggered failed, and that failure is reported separately -- the
            # ground station must still get a response instead of hanging.
            self._safely(on_approve, pending, 'on_approve')
        else:
            response.accepted = True
            response.message = 'rejected - resuming scan'
            self._node.get_logger().info(
                f"REJECTED id={pending['detection_id']} "
                f"reason={request.reason!r}")
            if self._log:
                self._log.event(
                    'APPROVAL_REJECTED',
                    f"id={pending['detection_id']}, class={pending['class_id']}, "
                    f"lat={pending['lat']:.6f}, lon={pending['lon']:.6f}, "
                    f"reason={request.reason!r}")
            # As above: the rejection was accepted even if acting on it failed.
            self._safely(on_reject, pending, 'on_reject',
                         pending['lat'], pending['lon'],
                         pending['class_id'], request.reason)

        return response

    def _on_timeout(self):
        if self._pending is None:
            return
        pending = self._pending
        on_approve = self._on_approve
        self._clear()

        self._node.get_logger().warn(
            f"APPROVAL TIMEOUT id={pending['detection_id']} after "
            f"{self._timeout_sec:.0f}s - deploying anyway")
        if self._log:
            self._log.event(
                'APPROVAL_TIMEOUT',
                f"id={pending['detection_id']}, class={pending['class_id']}, "
                f"timeout={self._timeout_sec:.0f}s, action=deploy")
        self._safely(on_approve, pending, 'on_approve')

    # -- internals --------------------------------------------------------

    def _safely(self, callback, pending, name, *args):
        """Run a mission callback without letting it escape into rclpy.

        These callbacks drive MAVROS waypoint pushes. rclpy does not catch
        exceptions raised in service or timer callbacks, so an unguarded raise
        would propagate out of rclpy.spin() and take mission_node down in
        flight. Log it and keep the node alive instead.
        """
        try:
            callback(*args)
        except Exception as exc:      # noqa: BLE001 - deliberately broad
            self._node.get_logger().error(
                f"{name} failed for id={pending['detection_id']} "
                f"class={pending['class_id']} "
                f"lat={pending['lat']:.6f} lon={pending['lon']:.6f}: {exc!r}")
            if self._log:
                self._log.event(
                    'APPROVAL_CALLBACK_FAILED',
                    f"id={pending['detection_id']}, callback={name}, error={exc!r}")

    def _clear(self):
        self._pending = None
        self._on_approve = None
        self._on_reject = None
        if self._timer is not None:
            self._timer.cancel()
            # Cancelling alone leaves the timer in node._timers, where the
            # executor re-adds it to its wait set every spin iteration.
            self._node.destroy_timer(self._timer)
            self._timer = None
        # Empty detection_id is the "nothing pending" convention the GCS uses.
        self._pub.publish(PendingDetection())
