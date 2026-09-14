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


def test_short_sweep_flies_rows_along_short_axis():
    long_plan = build_scan_plan(mission(), VISION, BEVY_CAMERA)
    short_mission = mission()
    short_mission['scan_sweep'] = 'short'
    short_plan = build_scan_plan(short_mission, VISION, BEVY_CAMERA)

    long_leg = abs(long_plan.waypoints[1][1] - long_plan.waypoints[0][1])
    short_leg = abs(short_plan.waypoints[1][1] - short_plan.waypoints[0][1])

    assert short_leg < long_leg
    assert short_plan.row_count > long_plan.row_count


@pytest.mark.parametrize(('start', 'comparison'), [
    ('top', lambda first, last: first >= last),
    ('bottom', lambda first, last: first <= last),
])
def test_scan_start_selects_north_or_south_end_of_route(start, comparison):
    config = mission()
    config['scan_start'] = start

    plan = build_scan_plan(config, VISION, BEVY_CAMERA)

    assert comparison(plan.waypoints[0][0], plan.waypoints[-1][0])


@pytest.mark.parametrize(('key', 'value', 'message'), [
    ('scan_sweep', 'diagonal', 'scan_sweep'),
    ('scan_start', 'middle', 'scan_start'),
])
def test_invalid_scan_route_options_are_rejected(key, value, message):
    config = mission()
    config[key] = value

    with pytest.raises(ValueError, match=message):
        build_scan_plan(config, VISION, BEVY_CAMERA)


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


@pytest.mark.parametrize('sweep,edge', [('long', (0, 1)), ('short', (1, 2))])
def test_rotated_boundary_rows_follow_selected_edge(sweep, edge):
    config = mission()
    config['scan_sweep'] = sweep
    plan = build_scan_plan(config, VISION, BEVY_CAMERA)
    a, b = (BOUNDARY[index] for index in edge)
    edge_lat, edge_lon = b[0] - a[0], b[1] - a[1]
    for first, last in zip(plan.waypoints[::2], plan.waypoints[1::2]):
        row_lat, row_lon = last[0] - first[0], last[1] - first[1]
        assert row_lat * edge_lon - row_lon * edge_lat == pytest.approx(0, abs=1e-12)
