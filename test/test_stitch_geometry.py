"""Unit tests for stitch_geometry helpers."""
import math
import pytest

from bv_core.stitch_geometry import along_track_m, compute_step_m


def test_compute_step_m_35_percent_overlap():
    # fx=3582.9, width=4640, altitude=15.24 → footprint ~19.74m, step ~12.83m
    step = compute_step_m(frame_width_px=4640, fx=3582.9, altitude_m=15.24, overlap=0.35)
    assert step == pytest.approx(12.83, abs=0.02)


def test_compute_step_m_zero_overlap_equals_footprint():
    step = compute_step_m(frame_width_px=4640, fx=3582.9, altitude_m=15.24, overlap=0.0)
    footprint = 4640 * 15.24 / 3582.9
    assert step == pytest.approx(footprint, abs=1e-6)


def test_along_track_point_at_anchor_is_zero():
    anchor = (40.159520, -83.197420)
    nxt = (40.159520, -83.196820)
    d = along_track_m(anchor, anchor, nxt)
    assert d == pytest.approx(0.0, abs=0.01)


def test_along_track_point_at_endpoint_equals_segment_length():
    # Segment: 40.159520,-83.197420 → 40.159520,-83.196820 (pure east, same lat)
    anchor = (40.159520, -83.197420)
    nxt = (40.159520, -83.196820)
    d = along_track_m(nxt, anchor, nxt)
    # 0.0006 degrees longitude at lat 40.159520 ~ 51.06m
    assert d == pytest.approx(51.06, abs=0.5)


def test_along_track_ignores_lateral_offset():
    # Drone drifted 5m north of the east-west segment — along-track should still track progress
    anchor = (40.159520, -83.197420)
    nxt = (40.159520, -83.196820)
    # +5m north ≈ +4.49e-5 degrees latitude
    drifted = (40.159520 + 4.49e-5, -83.197120)  # halfway along the segment, drifted north
    d = along_track_m(drifted, anchor, nxt)
    assert d == pytest.approx(25.53, abs=1.0)


def test_along_track_negative_before_anchor():
    anchor = (40.159520, -83.197420)
    nxt = (40.159520, -83.196820)
    # A bit west of anchor
    behind = (40.159520, -83.197500)
    d = along_track_m(behind, anchor, nxt)
    assert d < 0.0
