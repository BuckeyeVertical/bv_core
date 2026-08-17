import pytest

from bv_core.mission_config import expand_lap_route, mission_config_name


def test_default_mission_config_name_is_unchanged():
    assert mission_config_name({}) == 'mission_params.yaml'


def test_mission_config_can_be_selected_by_environment():
    environment = {'BV_MISSION_CONFIG': 'mission_suas_params.yaml'}

    assert mission_config_name(environment) == 'mission_suas_params.yaml'


@pytest.mark.parametrize('name', ['../mission.yaml', '/tmp/mission.yaml', 'mission.txt'])
def test_mission_config_name_rejects_paths_and_non_yaml_files(name):
    with pytest.raises(ValueError, match='BV_MISSION_CONFIG'):
        mission_config_name({'BV_MISSION_CONFIG': name})


def test_one_lap_preserves_existing_route_behavior():
    route = [[1], [2], [3]]

    assert expand_lap_route(route, 1) == route


def test_multiple_laps_close_and_repeat_route_without_duplicate_junctions():
    route = [[1], [2], [3]]

    assert expand_lap_route(route, 3) == [
        [1], [2], [3], [1],
        [2], [3], [1],
        [2], [3], [1],
    ]


def test_repeated_closed_route_does_not_duplicate_start_point():
    route = [[1], [2], [1]]

    assert expand_lap_route(route, 2) == [[1], [2], [1], [2], [1]]


def test_zero_laps_returns_no_lap_waypoints():
    assert expand_lap_route([[1], [2]], 0) == []


def test_lap_count_cannot_be_negative():
    with pytest.raises(ValueError, match='lap_count'):
        expand_lap_route([[1], [2]], -1)


def test_multiple_laps_require_a_real_route():
    with pytest.raises(ValueError, match='at least two'):
        expand_lap_route([[1]], 2)
