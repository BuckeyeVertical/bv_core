import os


MISSION_CONFIG_ENV = 'BV_MISSION_CONFIG'
DEFAULT_MISSION_CONFIG = 'mission_params.yaml'


def mission_config_name(environment=None):
    environment = os.environ if environment is None else environment
    name = environment.get(MISSION_CONFIG_ENV, DEFAULT_MISSION_CONFIG)
    if os.path.basename(name) != name or not name.endswith('.yaml'):
        raise ValueError(f'invalid {MISSION_CONFIG_ENV}: {name!r}')
    return name


def mission_config_path():
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory('bv_core'),
        'config',
        mission_config_name(),
    )


def expand_lap_route(points, lap_count):
    """Expand one closed route into the requested number of laps."""
    route = list(points)
    if lap_count < 0:
        raise ValueError("lap_count cannot be negative")
    if lap_count == 0:
        return []
    if lap_count == 1:
        return route
    if len(route) < 2:
        raise ValueError("multiple laps require at least two lap waypoints")

    closed_route = route if route[0] == route[-1] else route + [route[0]]
    return closed_route + closed_route[1:] * (lap_count - 1)
