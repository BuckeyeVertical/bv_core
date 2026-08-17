"""Tests for inference-independent stitch capture selection."""

import pytest

from bv_core.stitch_capture import StitchCaptureScheduler


ANCHOR = (40.0, -83.0)
ENDPOINT = (40.0, -82.998)


def east_of_anchor(distance_m):
    longitude = ANCHOR[1] + distance_m / (111319.49 * 0.76604444)
    return ANCHOR[0], longitude


def offset_from_anchor(along_m, cross_m=0.0):
    """Point `along_m` east and `cross_m` north of the anchor."""
    latitude = ANCHOR[0] + cross_m / 111319.49
    longitude = ANCHOR[1] + along_m / (111319.49 * 0.76604444)
    return latitude, longitude


# Half a cross-track ground footprint, matching what vision_node passes.
CROSS_LIMIT_M = 40.0


def test_first_frame_is_captured_at_row_start():
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(2, ANCHOR, ENDPOINT)

    capture = scheduler.consider(ANCHOR, frame_id=1)

    assert capture.row == 2
    assert capture.column == 1
    assert capture.kind == "start"
    assert capture.target_m == 0.0


def test_frames_are_selected_at_fixed_distance_targets():
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)

    assert scheduler.consider(ANCHOR, frame_id=1) is not None
    assert scheduler.consider(east_of_anchor(39.0), frame_id=2) is None
    capture = scheduler.consider(east_of_anchor(40.5), frame_id=3)

    assert capture.column == 2
    assert capture.kind == "spacing"
    assert capture.target_m == pytest.approx(40.0)
    assert capture.distance_m == pytest.approx(40.5, abs=0.1)


def test_missed_targets_do_not_duplicate_one_camera_frame():
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    capture = scheduler.consider(east_of_anchor(125.0), frame_id=2)

    assert capture.column == 2
    assert capture.skipped_targets == 2
    assert scheduler.consider(east_of_anchor(130.0), frame_id=3) is None


def test_endpoint_frame_is_captured_once():
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    capture = scheduler.consider(ENDPOINT, frame_id=2)

    assert capture.kind == "endpoint"
    assert scheduler.active is False
    assert scheduler.finish_row(ENDPOINT, frame_id=2) is None


def test_waypoint_completion_uses_latest_frame_for_endpoint():
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    capture = scheduler.finish_row(east_of_anchor(165.0), frame_id=2)

    assert capture.kind == "endpoint"
    assert capture.column == 2
    assert scheduler.active is False


# Delivery deviations.
#
# When a detection is confirmed mid-row the aircraft leaves the row, delivers,
# and flies back. vision_node suppresses capture during that travel, but the
# flag it uses is cleared by a BEST_EFFORT waypoint message. These cover the
# geometric backstop, which holds whether or not that message arrives.


def test_off_row_frame_does_not_consume_the_capture_target():
    """The correct frame at this distance must still be captured on return."""
    scheduler = StitchCaptureScheduler(40.0, max_cross_track_m=CROSS_LIMIT_M)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    # Flown 60 m off the row toward a delivery target, projecting past 40 m.
    assert scheduler.consider(offset_from_anchor(45.0, -60.0), frame_id=2) is None

    capture = scheduler.consider(offset_from_anchor(45.0), frame_id=3)

    assert capture.column == 2
    assert capture.target_m == pytest.approx(40.0)


def test_off_row_frame_past_the_endpoint_does_not_end_the_row():
    """The worst case: a target whose projection runs past the row end."""
    scheduler = StitchCaptureScheduler(40.0, max_cross_track_m=CROSS_LIMIT_M)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    assert scheduler.consider(offset_from_anchor(250.0, -80.0), frame_id=2) is None
    assert scheduler.active is True

    assert scheduler.consider(offset_from_anchor(45.0), frame_id=3).column == 2


def test_ordinary_tracking_drift_is_still_captured():
    """The guard must not reject frames for normal cross-track error."""
    scheduler = StitchCaptureScheduler(40.0, max_cross_track_m=CROSS_LIMIT_M)
    scheduler.start_row(1, ANCHOR, ENDPOINT)

    capture = scheduler.consider(offset_from_anchor(0.0, 8.0), frame_id=1)

    assert capture is not None
    assert capture.kind == "start"


def test_row_resumes_on_target_after_a_delivery_deviation():
    """Full sequence: fly the row, deviate to a target, return, finish."""
    scheduler = StitchCaptureScheduler(40.0, max_cross_track_m=CROSS_LIMIT_M)
    scheduler.start_row(3, ANCHOR, ENDPOINT)
    frame_id = 0
    captured = []

    def fly(along_m, cross_m=0.0):
        nonlocal frame_id
        frame_id += 1
        capture = scheduler.consider(
            offset_from_anchor(along_m, cross_m), frame_id)
        if capture is not None:
            # Record where the aircraft actually was. target_m and column alone
            # cannot tell an on-row frame from a deviation frame — both fill the
            # same slot — so the position is the only assertion that bites.
            captured.append((capture, cross_m))

    for along in range(0, 81, 5):               # row up to the detection
        fly(along)
    for step in range(11):                      # out to a target off the row
        fly(80.0 + 7.0 * step, -10.0 * step)
    for step in range(10, -1, -1):              # and back to the loiter point
        fly(80.0 + 7.0 * step, -10.0 * step)
    for along in range(80, 171, 5):             # resume the row
        fly(along)

    assert [c.target_m for c, _ in captured] == pytest.approx(
        [0.0, 40.0, 80.0, 120.0, 160.0])
    assert [c.column for c, _ in captured] == [1, 2, 3, 4, 5]
    assert [cross for _, cross in captured] == [0.0] * 5
    assert scheduler.active is True


def test_without_a_limit_an_off_row_frame_is_captured():
    """Regression guard: this is the behaviour the limit exists to prevent."""
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    capture = scheduler.consider(offset_from_anchor(45.0, -60.0), frame_id=2)

    assert capture is not None
    assert capture.target_m == pytest.approx(40.0)


def test_cancel_row_deactivates_without_capturing():
    """Where vision_node lands when a pause left it no frame for the endpoint."""
    scheduler = StitchCaptureScheduler(40.0)
    scheduler.start_row(1, ANCHOR, ENDPOINT)
    scheduler.consider(ANCHOR, frame_id=1)

    scheduler.cancel_row()

    assert scheduler.active is False
    assert scheduler.finish_row(ENDPOINT, frame_id=2) is None


def test_non_positive_cross_track_limit_is_rejected():
    with pytest.raises(ValueError, match="cross-track"):
        StitchCaptureScheduler(40.0, max_cross_track_m=0.0)
