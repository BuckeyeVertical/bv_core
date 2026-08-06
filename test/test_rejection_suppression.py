#!/usr/bin/env python3
"""Tests for rejection suppression logic in filtering_node.

Exercises the pure predicate directly — no ROS spin, no simulator.
Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.filtering_node import is_within_radius  # noqa: E402


class TestIsWithinRadius:
    def test_same_point_is_within(self):
        assert is_within_radius(38.3877, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_just_inside_radius(self):
        # 0.00005 deg north of the reference, radius 0.0001 deg.
        assert is_within_radius(38.38775, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_just_outside_radius(self):
        # 0.0002 deg north (~22 m) with an 11 m radius.
        assert not is_within_radius(38.3879, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_diagonal_distance_uses_euclidean_not_per_axis(self):
        # 0.00008 in each axis: inside per-axis, outside as a circle (0.000113).
        assert not is_within_radius(
            38.38778, -76.41892, 38.3877, -76.4190, 0.0001)

    def test_zero_radius_matches_only_exact_point(self):
        assert is_within_radius(38.3877, -76.4190, 38.3877, -76.4190, 0.0)
        assert not is_within_radius(38.38771, -76.4190, 38.3877, -76.4190, 0.0)


import rclpy  # noqa: E402
from bv_msgs.msg import ObjectDetections, ObjectLocations  # noqa: E402
from geometry_msgs.msg import PoseStamped, Vector3  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

import pytest  # noqa: E402

from bv_core.filtering_node import FilteringNode  # noqa: E402

REJECT_LAT = 38.387711
REJECT_LON = -76.419098


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = FilteringNode()
    # Sensor preconditions: handle_detections returns early without these.
    gps = NavSatFix()
    gps.latitude, gps.longitude = 38.3876, -76.4190
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    node.gps_buffer.append(gps)
    node.pose_buffer.append(pose)
    node.last_rel_alt = Float64(data=15.2)
    node.state = 'scan'
    yield node
    node.destroy_node()


def _drive(node, class_id, frames=3):
    """Feed `frames` identical detections and report whether one confirmed."""
    published = []
    node.confirmed_pub.publish = lambda msg: published.append(int(msg.data))

    # Stub the projection: this test is about suppression, not geometry.
    node.localizer.get_lat_lon = (
        lambda *a, **k: [(REJECT_LAT, REJECT_LON, class_id)])

    node.frame_history = []
    node.targets[class_id]['state'] = 'undetected'

    for _ in range(frames):
        msg = ObjectDetections()
        msg.dets = [Vector3(x=640.0, y=360.0, z=float(class_id))]
        node.handle_detections(msg)

    return published


class TestRejectionLoop:
    def test_detection_confirms_before_any_rejection(self, node):
        assert _drive(node, class_id=0) == [0]

    def test_same_class_at_rejected_spot_stops_confirming(self, node):
        assert _drive(node, class_id=0) == [0]      # baseline

        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        # This is the loop the design exists to prevent.
        assert _drive(node, class_id=0) == []

    def test_different_class_at_rejected_spot_still_confirms(self, node):
        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        # Operator rejected "person here", not the ground itself.
        assert _drive(node, class_id=1) == [1]

    def test_same_class_far_away_still_confirms(self, node):
        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT + 0.01      # ~1.1 km away
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        assert _drive(node, class_id=0) == [0]


class TestSuppressionReferenceFrame:
    """Suppression must key off filtering's OWN confirmed position.

    mission_node publishes the loiter-time localize estimate, produced from a
    different altitude and angle than the scan leg. Both estimate the same
    object but carry independent projection error (~4-4.6 m each in sim), so
    the two can sit 5-9 m apart against a radius that is only ~8.7 m
    east-west at 38.4 N. Consulting the stored confirmed position instead
    cancels the cross-vantage error.
    """

    def test_suppresses_when_message_coords_diverge_from_confirmed(self, node):
        # Filtering confirms the object at REJECT_LAT/REJECT_LON.
        assert _drive(node, class_id=0) == [0]

        # The operator rejects it, but mission_node's loiter-time estimate
        # landed ~33 m away - far outside the suppression radius.
        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT + 0.0003
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        # Suppression must still hold: the stored confirmed position wins.
        assert _drive(node, class_id=0) == []

    def test_falls_back_to_message_coords_when_never_confirmed(self, node):
        # No stored confirmed position for this class.
        assert node.targets[1].get('confirmed_lat') is None

        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT
        rejection.longitude = REJECT_LON
        rejection.class_id = 1
        node.rejected_location_callback(rejection)

        # The message coordinate is the fallback, and it still suppresses.
        assert _drive(node, class_id=1) == []
