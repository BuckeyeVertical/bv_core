#!/usr/bin/env python3
"""Where mission_node goes after a payload lands.

Drives the real `MissionRunner.on_deploy_complete` against a stand-in carrying
only the attributes it touches — `MissionRunner.__init__` reads the installed
config and builds MAVROS clients, so it is never called here.

The behaviour under test: the scan runs to the end of its plan even after the
last payload is delivered. Stitching is triggered by the `return` state and is
built from frames captured during the scan, so returning early would leave the
panorama missing whatever the aircraft had not yet flown over.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.mission import MissionRunner  # noqa: E402


class _StubLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        pass


class _StubLog:
    def event(self, event_type, details):
        pass

    def deploy_complete(self, **kwargs):
        pass


class _StubPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _DeployStandIn:
    """Only what on_deploy_complete reads."""

    def __init__(self, *, delivered, to_find, scan_wp_count,
                 resume_index, last_reached):
        self.objects_delivered_count = delivered
        self.num_objects_to_find = to_find
        self.scan_waypoints = [[0.0, 0.0, 60.0]] * scan_wp_count
        self.scan_waypoint_index_on_detection = resume_index
        self.last_reached_scan_waypoint = last_reached

        self.current_target_coords = (38.3877, -76.4190, 60.0)
        self.current_target_class_id = 0
        self.current_lat, self.current_lon = 38.3877, -76.4190

        self.deployed_object_pub = _StubPublisher()
        self.log = _StubLog()
        self._logger = _StubLogger()
        self.transitions = []

    def get_logger(self):
        return self._logger

    def enter_scan_state(self):
        self.transitions.append('scan')

    def enter_rtl_state(self):
        self.transitions.append('rtl')


def deploy(**kwargs):
    node = _DeployStandIn(**kwargs)
    MissionRunner.on_deploy_complete(node)
    return node


def test_last_payload_resumes_the_scan_instead_of_returning():
    # Six scan waypoints, the final object delivered after reaching index 1.
    node = deploy(delivered=1, to_find=2, scan_wp_count=6,
                  resume_index=0, last_reached=1)

    assert node.objects_delivered_count == 2
    assert node.transitions == ['scan']          # not ['rtl']
    assert node.scan_waypoint_index_on_detection == 2


def test_scan_plan_is_not_swept_again_once_everything_is_delivered():
    """Index is left past the end so STATE_SCAN completes into RTL."""
    node = deploy(delivered=1, to_find=2, scan_wp_count=6,
                  resume_index=4, last_reached=1)

    assert node.transitions == ['scan']
    assert node.scan_waypoint_index_on_detection >= len(node.scan_waypoints)


def test_outstanding_objects_still_wrap_for_another_sweep():
    """Unchanged: keep looking when the region is covered but objects remain."""
    node = deploy(delivered=0, to_find=2, scan_wp_count=6,
                  resume_index=4, last_reached=1)

    assert node.objects_delivered_count == 1
    assert node.transitions == ['scan']
    assert node.scan_waypoint_index_on_detection == 0


def test_mid_region_delivery_advances_past_the_serviced_waypoint():
    node = deploy(delivered=0, to_find=2, scan_wp_count=6,
                  resume_index=2, last_reached=0)

    assert node.transitions == ['scan']
    assert node.scan_waypoint_index_on_detection == 3


def test_deployed_location_is_still_published():
    """The suppression contract with filtering_node must survive the change."""
    node = deploy(delivered=1, to_find=2, scan_wp_count=6,
                  resume_index=0, last_reached=1)

    assert len(node.deployed_object_pub.published) == 1
    msg = node.deployed_object_pub.published[0]
    assert msg.latitude == pytest.approx(38.3877)
    assert msg.class_id == 0
