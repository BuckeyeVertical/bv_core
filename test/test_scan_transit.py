"""Mission behavior while moving from the final lap to the scan route."""

from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from bv_core.mission import MissionRunner


def mission(state='lap', scan_frame=3):
    node = SimpleNamespace(
        current_state=state,
        is_transitioning=False,
        payload_writer=None,
        last_waypoint_reached=None,
        last_processed_waypoint=-1,
        set_mode_client=Mock(),
        create_timer=Mock(return_value=Mock()),
        destroy_timer=Mock(),
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
        'on_waypoint_reached', 'on_end_laps', '_process_end_laps_request',
        'main_timer_callback', 'on_waypoint_push_complete',
        '_cancel_end_laps', '_finish_end_laps_hold',
        '_on_end_laps_loiter_complete', 'on_vehicle_state_changed',
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


def test_end_laps_uses_normal_scan_route_and_rejects_repeat():
    node = mission()
    normal = mission()
    normal.handle_state_completion()

    assert node.on_end_laps(None, SimpleNamespace()).success
    complete_hold(node)
    assert node.current_state == normal.current_state == 'scan_transit'
    assert node.desired_velocity == normal.desired_velocity
    assert [(w.x_lat, w.y_long, w.z_alt, w.frame) for w in node.active_waypoint_list] == [
        (w.x_lat, w.y_long, w.z_alt, w.frame) for w in normal.active_waypoint_list]
    assert not node.on_end_laps(None, SimpleNamespace()).success
    node.push_mission_to_autopilot.assert_called_once()


def test_end_laps_rejected_outside_laps():
    for state in ('takeoff', 'scan_transit', 'scan', 'localize', 'deliver', 'deploy', 'return'):
        node = mission(state)
        assert not node.on_end_laps(None, SimpleNamespace()).success
        node.push_mission_to_autopilot.assert_not_called()


def test_end_laps_waits_for_inflight_lap_upload():
    node = mission()
    node.is_transitioning = True
    assert node.on_end_laps(None, SimpleNamespace()).success
    assert not node.on_end_laps(None, SimpleNamespace()).success
    node.main_timer_callback()
    node.push_mission_to_autopilot.assert_not_called()
    node.is_transitioning = False
    node.main_timer_callback()
    complete_hold(node)
    assert node.current_state == 'scan_transit'
    node.push_mission_to_autopilot.assert_called_once()


def test_unrequested_timer_leaves_laps_unchanged():
    node = mission()
    node.main_timer_callback()
    assert node.current_state == 'lap'
    node.push_mission_to_autopilot.assert_not_called()


def test_pending_end_laps_cannot_override_rtl():
    node = mission()
    node.is_transitioning = True
    node.on_end_laps(None, SimpleNamespace())
    node.current_state = 'return'
    node.main_timer_callback()
    node.push_mission_to_autopilot.assert_not_called()
    assert not node._end_laps_requested


def test_early_exit_ignores_lap_events_until_scan_upload_ack():
    node = mission()
    node.on_end_laps(None, SimpleNamespace())
    node.on_waypoint_reached(SimpleNamespace(wp_seq=0))
    assert node.last_waypoint_reached is None
    complete_hold(node)
    node.has_armed = True
    node.set_flight_mode = Mock()
    node.on_waypoint_push_complete(Mock(result=lambda: SimpleNamespace(success=True)))
    assert not node._early_scan_upload_pending
    node.is_transitioning = False
    node.on_waypoint_reached(SimpleNamespace(wp_seq=0))
    assert node.current_state == 'scan'
    node.on_waypoint_reached(SimpleNamespace(wp_seq=3))
    node.enter_rtl_state.assert_called_once()


def complete_hold(node):
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
    node.create_timer.assert_called_once_with(10.0, node._finish_end_laps_hold)
    node.create_timer.call_args.args[1]()


def test_end_laps_waits_for_observed_loiter_and_ten_second_timer():
    node = mission()
    node.on_end_laps(None, SimpleNamespace())
    assert node.set_mode_client.call_async.call_args.args[0].custom_mode == 'AUTO.LOITER'
    node._on_end_laps_loiter_complete(Mock(result=lambda: SimpleNamespace(mode_sent=True)))
    node.create_timer.assert_not_called()
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.MISSION', armed=True))
    node.create_timer.assert_not_called()
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
    node.create_timer.assert_called_once_with(10.0, node._finish_end_laps_hold)
    node.push_mission_to_autopilot.assert_not_called()
    assert not node.on_end_laps(None, SimpleNamespace()).success
    node.on_waypoint_reached(SimpleNamespace(wp_seq=1))
    assert node.last_waypoint_reached is None
    callback = node.create_timer.call_args.args[1]
    callback()
    callback()
    node.push_mission_to_autopilot.assert_called_once()
    node.destroy_timer.assert_called_once()


def test_mode_override_cancels_hold_and_late_timer():
    for mode in ('AUTO.RTL', 'AUTO.LAND', 'POSCTL', 'AUTO.MISSION'):
        node = mission()
        node.on_end_laps(None, SimpleNamespace())
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
        timer = node.create_timer.return_value
        node.on_vehicle_state_changed(SimpleNamespace(mode=mode, armed=True))
        timer.cancel.assert_called_once()
        node._finish_end_laps_hold()
        node.push_mission_to_autopilot.assert_not_called()


def test_loiter_rejection_or_exception_never_starts_scan():
    for future in (Mock(result=lambda: SimpleNamespace(mode_sent=False)),
                   Mock(result=Mock(side_effect=RuntimeError('unavailable')))):
        node = mission()
        node.on_end_laps(None, SimpleNamespace())
        node._on_end_laps_loiter_complete(future)
        node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
        node._finish_end_laps_hold()
        node.create_timer.assert_not_called()
        node.push_mission_to_autopilot.assert_not_called()


def test_disarm_cancels_hold():
    node = mission()
    node.on_end_laps(None, SimpleNamespace())
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=True))
    node.on_vehicle_state_changed(SimpleNamespace(mode='AUTO.LOITER', armed=False))
    node._finish_end_laps_hold()
    node.push_mission_to_autopilot.assert_not_called()


def test_lap_completion_cannot_bypass_requested_hold():
    node = mission()
    node.is_transitioning = True
    node.on_end_laps(None, SimpleNamespace())
    node.is_transitioning = False
    node.handle_state_completion()
    node.push_mission_to_autopilot.assert_not_called()
    assert node._end_laps_hold_phase == 'waiting'
    complete_hold(node)
    node.push_mission_to_autopilot.assert_called_once()
