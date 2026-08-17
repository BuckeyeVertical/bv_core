#!/usr/bin/env python3
"""Stitch capture across a delivery deviation, at the vision_node level.

test_stitch_capture.py covers the scheduler, which never sees a pause. This
covers the orchestration around it: what happens when a detection pulls the
aircraft off the row mid-scan, and what happens when it comes back.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import math
import os
import queue
import sys
import threading
from collections import deque

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from mavros_msgs.msg import WaypointReached  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from bv_core.stitch_capture import StitchCaptureScheduler  # noqa: E402
from bv_core.vision_node import VisionNode  # noqa: E402

R = 6378137.0
LAT0, LON0 = 38.3877, -76.4190
ALT = 60.96
ROW_LEN_M = 200.0
ROW_GAP_M = 40.0
SPACING_M = 38.5
CROSS_LIMIT_M = 39.5


def at(along_m, cross_m=0.0):
    """Point `along_m` east and `cross_m` north of the first row's start."""
    return (LAT0 + math.degrees(cross_m / R),
            LON0 + math.degrees(along_m / (R * math.cos(math.radians(LAT0)))))


# A two-row snake: even index starts a row, odd index ends it.
SCAN_POINTS = [
    [at(0.0)[0], at(0.0)[1], ALT],
    [at(ROW_LEN_M)[0], at(ROW_LEN_M)[1], ALT],
    [at(ROW_LEN_M, ROW_GAP_M)[0], at(ROW_LEN_M, ROW_GAP_M)[1], ALT],
    [at(0.0, ROW_GAP_M)[0], at(0.0, ROW_GAP_M)[1], ALT],
]


class _StubDetector:
    """Stands in for MLDetector, which would load a 919 MB checkpoint."""

    def __init__(self):
        self.stopped = 0

    def stop(self):
        self.stopped += 1


class StitchHarness(VisionNode):
    """VisionNode with only the stitch-capture collaborators wired up.

    VisionNode.__init__ loads the detector checkpoint and starts three worker
    threads, none of which the stitch orchestration touches. This builds the
    object without running it and supplies exactly the attributes the methods
    under test read, so the real code paths run against real geometry.
    """

    def __init__(self):
        Node.__init__(self, 'stitch_harness')
        self.scan_points = SCAN_POINTS
        self.scan_tolerance = 5.0
        self.stitch_capture = StitchCaptureScheduler(
            SPACING_M, max_cross_track_m=CROSS_LIMIT_M)
        self.stitch_lock = threading.Lock()
        self.stitch_write_queue = queue.Queue(maxsize=64)
        self.latest_stitch_frame = None
        self.stitch_frame_id = 0
        self.stitch_paused = False
        self.curr_wp = -1
        self.prev_state = ''
        self.state = ''
        self.last_wp = None
        self.latest_wp = None
        self.frame_number = 1
        self.gps_buffer = deque(maxlen=200)
        self.raw_frames_cleared = True      # keep the filesystem out of it
        self.pipeline_calls = []
        self.detector = _StubDetector()

    # The camera pipeline is not what is under test.
    def _start_scanning(self):
        self.pipeline_calls.append('start')

    def _stop_scanning(self):
        self.pipeline_calls.append('stop')

    # -- helpers -------------------------------------------------------

    def set_state(self, state):
        self._on_mission_state_changed(String(data=state))

    def goto(self, along_m, cross_m=0.0):
        """Move the aircraft, without offering a frame."""
        latitude, longitude = at(along_m, cross_m)
        fix = NavSatFix()
        fix.latitude, fix.longitude = latitude, longitude
        self.gps_buffer.append(fix)

    def fly(self, along_m, cross_m=0.0):
        """Move the aircraft and offer one frame to stitch capture."""
        self.goto(along_m, cross_m)
        self._capture_stitch_frame(np.zeros((4, 4, 3), dtype=np.uint8))

    def reach(self, wp_seq):
        self._on_waypoint_reached(WaypointReached(wp_seq=wp_seq))

    def written(self):
        """Paths written to the stitch writer, in order."""
        out = []
        while True:
            try:
                path, _frame, capture = self.stitch_write_queue.get_nowait()
            except queue.Empty:
                return out
            out.append((path, capture))


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    harness = StitchHarness()
    harness.set_state('scan')
    harness.goto(0.0)
    harness.reach(0)                 # enter row 1, anchoring at its start
    yield harness
    # Node.destroy_node, not VisionNode's: that one tears down a pipeline,
    # detector and three worker threads this harness never built.
    Node.destroy_node(harness)


def fly_row(node, start_m, end_m):
    """Fly the row from `start_m` to `end_m`, offering a frame every 5 m."""
    for step in range(int(start_m), int(end_m) + 1, 5):
        node.fly(float(step))


def test_transit_to_the_region_captures_nothing(ros):
    """mission_node publishes `scan` while still flying to the first waypoint.

    In the 2026-08-17 flight that transit lasted 57 s and ran detection well
    south of the search boundary. Stitch capture must not collect any of it —
    the panorama covers the region, not the route to it.
    """
    harness = StitchHarness()
    try:
        harness.set_state('scan')

        # En route, short of the first row and outside the region.
        for along in range(-400, -100, 20):
            harness.fly(float(along))

        assert harness.written() == []
        assert harness.stitch_capture.active is False

        # The row only opens once the first scan waypoint is actually reached.
        harness.goto(0.0)
        harness.reach(0)
        harness.fly(0.0)

        assert [c.column for _p, c in harness.written()] == [1]
    finally:
        Node.destroy_node(harness)


def test_pipeline_is_warm_before_scanning_begins(ros):
    """Starting the stream at scan entry cost 7.1 s of blindness in flight."""
    harness = StitchHarness()
    try:
        for state in ('takeoff', 'lap', 'scan'):
            harness.set_state(state)

        assert harness.pipeline_calls[0] == 'start'
        assert 'stop' not in harness.pipeline_calls
    finally:
        Node.destroy_node(harness)


def test_pipeline_stays_connected_through_the_delivery_excursion(node):
    """Restarting the stream on deliver cost 200 s of blindness in flight."""
    node.pipeline_calls.clear()

    for state in ('localize', 'deliver', 'deploy', 'scan'):
        node.set_state(state)

    assert 'stop' not in node.pipeline_calls


def test_pipeline_is_released_once_the_mission_ends(node):
    node.pipeline_calls.clear()

    node.set_state('return')

    assert node.pipeline_calls == ['stop']


def test_detector_is_released_at_rtl(node):
    """The model is dead weight from RTL on, and stitching needs the memory."""
    node.set_state('return')

    assert node.detector.stopped == 1


def test_detector_is_kept_while_the_mission_could_still_need_it(node):
    for state in ('localize', 'deliver', 'deploy', 'scan'):
        node.set_state(state)

    assert node.detector.stopped == 0


def test_detector_is_released_after_the_pipeline_is_parked(node):
    """Order matters: a frame reaching process_frame would reload the model."""
    order = []
    node._stop_scanning = lambda: order.append('stop_scanning')
    node.detector.stop = lambda: order.append('release_detector')

    node.set_state('return')

    assert order == ['stop_scanning', 'release_detector']


def test_row_one_is_active_after_reaching_its_start(node):
    assert node.stitch_capture.active is True
    assert node.curr_wp == 0


def test_detection_mid_row_pauses_capture_and_drops_the_stale_frame(node):
    fly_row(node, 0, 75)
    node.written()

    node.set_state('localize')

    assert node.stitch_paused is True
    # Cleared so a missed unpause cannot write it as the row's endpoint.
    assert node.latest_stitch_frame is None


def test_return_leg_writes_nothing(node):
    fly_row(node, 0, 75)
    node.written()
    node.set_state('localize')
    node.set_state('deliver')

    # Back from the delivery site to the loiter point, well off the row.
    for cross in range(120, -1, -10):
        node.fly(110.0, -float(cross))

    assert node.written() == []


def test_reaching_the_loiter_point_resumes_the_same_row(node):
    fly_row(node, 0, 75)
    before = node.written()
    node.set_state('localize')
    node.set_state('deliver')
    node.set_state('scan')

    # The prepended loiter waypoint: mid-row, so it matches no scan point.
    node.goto(75.0)
    node.reach(0)

    assert node.stitch_paused is False
    assert node.curr_wp == 0                     # row not restarted
    assert node.stitch_capture.active is True

    fly_row(node, 75, 130)
    after = node.written()

    # Column numbering continues rather than restarting at 1, and the targets
    # are absolute distances from the original anchor — the pause did not
    # shift the row's frame of reference.
    assert [c.column for _p, c in before] == [1, 2]
    assert [c.column for _p, c in after] == [3, 4]
    assert [c.target_m for _p, c in after] == pytest.approx(
        [SPACING_M * 2, SPACING_M * 3])


def test_missed_unpause_does_not_write_a_stale_endpoint(node):
    """The failure the cleared frame prevents.

    If the loiter waypoint message is lost, the aircraft flies the rest of the
    row still paused. Without clearing, _finish_stitch_row would write the
    pre-deviation frame as this row's endpoint.
    """
    fly_row(node, 0, 75)
    node.written()
    node.set_state('localize')
    node.set_state('deliver')
    node.set_state('scan')

    # No reach(0) here: the loiter waypoint never arrives.
    node.goto(ROW_LEN_M)
    node.reach(1)                                # row 1 end

    assert node.written() == []
    assert node.stitch_capture.active is False   # cancelled, not mis-captured


def test_deviation_while_between_rows_is_harmless(node):
    """Leaving scan on the cross-row transit, where no row is active."""
    fly_row(node, 0, 200)
    node.goto(ROW_LEN_M)
    node.reach(1)
    node.written()
    assert node.stitch_capture.active is False

    node.set_state('localize')
    assert node.stitch_paused is False           # nothing to pause

    for cross in range(0, 121, 20):
        node.fly(110.0, -float(cross))

    assert node.written() == []
