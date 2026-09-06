import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'mission_ready_check.py'
)


def load_script():
    spec = importlib.util.spec_from_file_location(
        'mission_ready_check_script', SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_real_config_coordinates_are_loaded():
    module = load_script()
    config = SCRIPT_PATH.parents[1] / 'config' / 'real_params.yaml'

    points = module.configured_points(config)

    assert len(points) >= 4
    assert all(-90.0 <= lat <= 90.0 for lat, _ in points)
    assert all(-180.0 <= lon <= 180.0 for _, lon in points)


def test_mavros_must_be_connected_and_disarmed():
    module = load_script()
    safe_state = 'connected: true\narmed: false'
    armed_state = 'connected: true\narmed: true'

    with patch.object(
        module, 'ros_topic', return_value=(0, safe_state, None)
    ):
        assert module.mavros_state_check(1).passed
    with patch.object(
        module, 'ros_topic', return_value=(0, armed_state, None)
    ):
        result = module.mavros_state_check(1)
        assert not result.passed
        assert 'disarmed' in result.detail


def test_gps_must_be_near_a_configured_point():
    module = load_script()
    gps = 'status:\n  status: 0\nlatitude: 40.0\nlongitude: -83.0'

    with patch.object(
        module, 'ros_topic', return_value=(0, gps, None)
    ):
        assert module.gps_check(1, [(40.001, -83.001)], 5000).passed
        assert not module.gps_check(1, [(41.0, -83.0)], 5000).passed


def test_gps_rejects_navsat_no_fix_status():
    module = load_script()
    gps = 'status:\n  status: -1\nlatitude: 40.0\nlongitude: -83.0'

    with patch.object(
        module, 'ros_topic', return_value=(0, gps, None)
    ):
        result = module.gps_check(1, [(40.0, -83.0)], 5000)

    assert not result.passed
    assert 'no fix' in result.detail


def test_script_contains_no_aircraft_command_operations():
    source = SCRIPT_PATH.read_text(encoding='utf-8')
    forbidden = (
        "'topic', 'pub'",
        "'service', 'call'",
        "'ros2', 'launch'",
        '/mavros/cmd/arming',
        '/mavros/set_mode',
        '/mavros/mission/push',
    )

    assert not any(operation in source for operation in forbidden)
