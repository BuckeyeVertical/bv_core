import pytest

from bv_core.scan_plan import build_scan_plan


BOUNDARY = [
    [36.216341, -96.010424],
    [36.216750, -96.007550],
    [36.218054, -96.007835],
    [36.217645, -96.010709],
]
VISION = {'stitch_overlap': 0.35}
BEVY_CAMERA = {
    'image_width_px': 1280,
    'image_height_px': 960,
    'c_matrix': [
        988.3916123883231, 0.0, 640.0,
        0.0, 988.3916123883231, 480.0,
        0.0, 0.0, 1.0,
    ],
}


def mission(altitude=60.96):
    return {
        'takeoff_alt': 30.0,
        'scan_altitude': altitude,
        'scan_boundary': BOUNDARY,
    }


def test_suas_boundary_generates_three_snake_rows_at_200_feet():
    plan = build_scan_plan(mission(), VISION, BEVY_CAMERA)

    assert plan.row_count == 3
    assert len(plan.waypoints) == 6
    assert plan.capture_spacing_m == pytest.approx(38.49, abs=0.02)
    assert plan.row_spacing_m == pytest.approx(34.23, abs=0.1)
    assert all(point[2] == 60.96 for point in plan.waypoints)
    assert plan.waypoints[0][1] < plan.waypoints[1][1]
    assert plan.waypoints[2][1] > plan.waypoints[3][1]
    assert plan.waypoints[4][1] < plan.waypoints[5][1]


def test_scan_altitude_regenerates_capture_and_row_spacing():
    low = build_scan_plan(mission(60.96), VISION, BEVY_CAMERA)
    high = build_scan_plan(mission(121.92), VISION, BEVY_CAMERA)

    assert high.capture_spacing_m == pytest.approx(low.capture_spacing_m * 2.0)
    assert high.row_count == 1
    assert len(high.waypoints) == 2


def test_more_overlap_generates_more_rows():
    plan = build_scan_plan(mission(), {'stitch_overlap': 0.60}, BEVY_CAMERA)

    assert plan.row_count == 4


def test_explicit_scan_points_remain_supported():
    points = [[1.0, 2.0, 30.0], [1.0, 3.0, 30.0]]
    config = {
        'takeoff_alt': 30.0,
        'scan_altitude': 30.0,
        'scan_points': points,
    }

    plan = build_scan_plan(config, VISION, BEVY_CAMERA)

    assert plan.waypoints == points
    assert plan.row_count == 1


def test_boundary_and_explicit_points_are_rejected_together():
    config = mission()
    config['scan_points'] = [[1.0, 2.0, 30.0]]

    with pytest.raises(ValueError, match='scan_boundary or scan_points'):
        build_scan_plan(config, VISION, BEVY_CAMERA)
