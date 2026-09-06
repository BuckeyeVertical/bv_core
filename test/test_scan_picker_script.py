import importlib.util
from pathlib import Path
import sys

import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'Scan_Picker.py'
)
BOUNDARY = [
    [40.2, -83.2],
    [40.1, -83.2],
    [40.1, -83.0],
    [40.2, -83.0],
]
MISSION = """# keep this comment
takeoff_alt: &TAKEOFF 45.72
lap_count: 0

# lap comment
points:
  - [1.0, 2.0, *TAKEOFF]
  - [3.0, 4.0, *TAKEOFF]

scan_boundary:
  - [1.0, 2.0]
  - [3.0, 2.0]
  - [3.0, 4.0]
  - [1.0, 4.0]

Scan_velocity: 3.0
"""


def load_script():
    spec = importlib.util.spec_from_file_location('scan_picker_script', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_scan_region_update_preserves_other_config_and_adds_route_options():
    module = load_script()

    result = module.update_region_yaml(
        MISSION, 'scan', BOUNDARY, sweep='short', start='bottom')
    parsed = yaml.safe_load(result)

    assert parsed['scan_sweep'] == 'short'
    assert parsed['scan_start'] == 'bottom'
    assert parsed['scan_boundary'] == BOUNDARY
    assert parsed['points'] == [[1.0, 2.0, 45.72], [3.0, 4.0, 45.72]]
    assert parsed['Scan_velocity'] == 3.0
    assert '# keep this comment' in result
    assert '# lap comment' in result


def test_counterclockwise_lap_region_is_closed_and_preserves_altitude_anchor():
    module = load_script()

    result = module.update_region_yaml(
        MISSION, 'lap', BOUNDARY, lap_direction='counterclockwise')
    parsed = yaml.safe_load(result)

    assert parsed['points'] == [
        [40.2, -83.2, 45.72],
        [40.1, -83.2, 45.72],
        [40.1, -83.0, 45.72],
        [40.2, -83.0, 45.72],
        [40.2, -83.2, 45.72],
    ]
    assert result.count('*TAKEOFF') == 5


def test_clockwise_lap_reverses_the_perimeter_after_waypoint_one():
    module = load_script()

    result = module.update_region_yaml(
        MISSION, 'lap', BOUNDARY, lap_direction='clockwise')
    parsed = yaml.safe_load(result)

    assert [point[:2] for point in parsed['points']] == [
        BOUNDARY[0], BOUNDARY[3], BOUNDARY[2], BOUNDARY[1], BOUNDARY[0]
    ]


def test_save_region_replaces_file_atomically(tmp_path):
    module = load_script()
    config = tmp_path / 'real_params.yaml'
    config.write_text(MISSION, encoding='utf-8')

    module.save_region(
        config, 'scan', BOUNDARY, sweep='long', start='top')

    parsed = yaml.safe_load(config.read_text(encoding='utf-8'))
    assert parsed['scan_boundary'] == BOUNDARY
    assert list(tmp_path.glob('*.tmp')) == []
