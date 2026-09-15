"""Mission behavior while moving from the final lap to the scan route."""

from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from bv_core.mission import MissionRunner


def mission(state='lap', scan_frame=3):
    node = SimpleNamespace(
        current_state=state,
        scan_frame=scan_frame,
        lap_waypoints=[(40.0, -83.0, 45.0), (40.001, -83.0, 45.0)],
        scan_waypoints=[
            (40.002, -83.002, 45.0),
            (40.003, -83.002, 45.0),
            (40.003, -83.003, 45.0),
            (40.002, -83.003, 45.0),
        ],
        lap_velocity=10.0,
        scan_transit_velocity=8.0,
        scan_velocity=3.0,
        scan_tolerance=1.0,
        scan_waypoint_index_on_detection=0,
        objects_delivered_count=0,
        num_objects_to_find=2,
        get_logger=Mock(return_value=Mock()),
        log=Mock(),
        publish_mission_state=Mock(),
        publish_path_progress=Mock(),
        push_mission_to_autopilot=Mock(),
        set_velocity=Mock(),
        enter_lap_state=Mock(),
        enter_scan_state=Mock(),
        enter_rtl_state=Mock(),
        get_clock=Mock(return_value=SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1234))),
    )
    for name in (
        'build_waypoint_list', 'enter_scan_transit_state',
        'activate_scan_state', 'handle_state_completion',
        'on_waypoint_reached',
    ):
        setattr(node, name, MethodType(getattr(MissionRunner, name), node))
    return node


def test_lap_completion_enters_transit_instead_of_scan():
    node = mission()

    node.handle_state_completion()

    assert node.current_state == 'scan_transit'
    node.enter_scan_state.assert_not_called()


def test_transit_uploads_full_scan_route_once_at_lap_speed():
    node = mission()

    node.enter_scan_transit_state()

    assert node.desired_velocity == node.scan_transit_velocity
    assert node.expected_final_waypoint_index == 0
    assert len(node.active_waypoint_list) == len(node.scan_waypoints)
    assert all(wp.param2 == node.scan_tolerance for wp in node.active_waypoint_list)
    assert all(wp.param3 == node.scan_tolerance for wp in node.active_waypoint_list)
    node.push_mission_to_autopilot.assert_called_once()
    node.publish_mission_state.assert_called_once()


def test_first_scan_waypoint_activates_scan_without_uploading_again():
    node = mission('scan_transit')
    node.enter_scan_transit_state()
    node.is_transitioning = False
    node.push_mission_to_autopilot.reset_mock()
    node.publish_mission_state.reset_mock()

    node.on_waypoint_reached(SimpleNamespace(wp_seq=0))

    assert node.current_state == 'scan'
    assert node.scan_started_ns == 1234
    assert node.desired_velocity == node.scan_velocity
    assert node.expected_final_waypoint_index == len(node.scan_waypoints) - 1
    assert node.last_reached_scan_waypoint == 0
    node.set_velocity.assert_called_once_with(node.scan_velocity)
    node.publish_mission_state.assert_called_once()
    node.push_mission_to_autopilot.assert_not_called()


def test_scan_continues_to_the_existing_final_waypoint():
    node = mission('scan_transit')
    node.enter_scan_transit_state()
    node.is_transitioning = False
    node.on_waypoint_reached(SimpleNamespace(wp_seq=0))
    node.enter_rtl_state.reset_mock()

    node.on_waypoint_reached(SimpleNamespace(wp_seq=1))
    node.on_waypoint_reached(SimpleNamespace(wp_seq=2))
    node.enter_rtl_state.assert_not_called()
    node.on_waypoint_reached(SimpleNamespace(wp_seq=3))

    node.enter_rtl_state.assert_called_once()


def test_zero_lap_takeoff_still_uses_existing_scan_entry():
    node = mission('takeoff')
    node.lap_waypoints = []

    node.handle_state_completion()

    node.enter_scan_state.assert_called_once()


def test_transit_pushes_relative_altitudes_without_a_dem():
    node = mission()

    node.enter_scan_transit_state()

    # MAV_FRAME_GLOBAL_RELATIVE_ALT: altitudes are above the takeoff point.
    assert all(wp.frame == 3 for wp in node.active_waypoint_list)


def test_transit_pushes_amsl_when_the_plan_is_terrain_referenced():
    """Terrain-referenced altitudes are absolute, so the frame must say so."""
    node = mission(scan_frame=0)

    node.enter_scan_transit_state()

    # MAV_FRAME_GLOBAL: altitudes are AMSL, matching the lap route.
    assert all(wp.frame == 0 for wp in node.active_waypoint_list)
