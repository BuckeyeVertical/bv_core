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


def test_picker_preserves_rotated_boundary_for_preview_and_yaml():
    import json
    import shutil
    import subprocess

    import pytest

    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js is required for picker JavaScript regression')
    module = load_script()
    boundary = [
        [36.216341, -96.010424], [36.216750, -96.007550],
        [36.218054, -96.007835], [36.217645, -96.010709],
    ]
    functions = []
    for name in ('yamlFor', 'boundaryFor', 'showConfiguredRegion'):
        begin = module.PAGE.index(f'    function {name}(')
        end = module.PAGE.index('\n    }', begin) + len('\n    }')
        functions.append(module.PAGE[begin:end])
    script = '''
const assert = require('node:assert/strict');
const configuredRegion = {boundary: BOUNDARY, sweep: 'long', start: 'bottom'};
let mode = 'scan', activeScanBoundary = null, activeBounds = null;
let configuredLayer = null, planLayer = null, sweep, routeStart;
const yamlBox = {}, copyButton = {}, useRegionButton = {};
const bounds = {
  getNorth: () => 37, getSouth: () => 36,
  getEast: () => -96, getWest: () => -97,
  getNorthWest: () => ({lat: 37, lng: -97}),
  getSouthWest: () => ({lat: 36, lng: -97}),
  getSouthEast: () => ({lat: 36, lng: -96}),
  getNorthEast: () => ({lat: 37, lng: -96})
};
const map = {fitBounds() {}, removeLayer() {}};
const L = {
  polygon: () => ({bindTooltip() {return this;}}),
  featureGroup: () => ({addTo() {return this;}, getBounds: () => bounds}),
  latLngBounds: () => bounds
};
function setStatus() {}
function updateChoiceButtons() {}
let preview;
function refreshPreview(bounds) {preview = boundaryFor(bounds);}
FUNCTIONS
showConfiguredRegion();
assert.deepEqual(preview, configuredRegion.boundary);
assert.deepEqual(boundaryFor(activeBounds), configuredRegion.boundary);
console.log(JSON.stringify(yamlBox.value));
activeScanBoundary = null;
assert.deepEqual(boundaryFor(bounds), [[37,-97],[36,-97],[36,-96],[37,-96]]);
'''.replace('BOUNDARY', json.dumps(boundary)).replace('FUNCTIONS', '\n'.join(functions))
    result = subprocess.run(
        [node, '-e', script], check=True, capture_output=True, text=True)
    assert yaml.safe_load(json.loads(result.stdout))['scan_boundary'] == boundary


def test_location_button_recenters_and_reports_geolocation_errors():
    import shutil
    import subprocess

    import pytest

    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js is required for picker JavaScript regression')
    module = load_script()
    functions = []
    for name in ('finishLocate', 'locationErrorMessage', 'locate'):
        begin = module.PAGE.index(f'    function {name}(')
        end = module.PAGE.index('\n    }', begin) + len('\n    }')
        functions.append(module.PAGE[begin:end])
    script = r'''
const assert = require('node:assert/strict');
let firstCorner = null, rectangle = null, configuredLayer = null;
let locationLayer = null, locationRequest = 0;
const statuses = [];
const locateButton = {disabled: false, textContent: 'My location'};
const map = {
  views: [], removed: [],
  setView(here, zoom) {this.views.push([here, zoom]);},
  removeLayer(layer) {this.removed.push(layer);}
};
const L = {
  circle(here, options) {
    return {
      here, options,
      addTo() {return this;},
      bindPopup() {return this;}
    };
  }
};
function setStatus(message) {statuses.push(message);}
let success, failure, options;
const navigator = {geolocation: {
  getCurrentPosition(onSuccess, onFailure, suppliedOptions) {
    success = onSuccess; failure = onFailure; options = suppliedOptions;
  }
}};
FUNCTIONS
locate(true);
assert.equal(locateButton.disabled, true);
assert.equal(locateButton.textContent, 'Locating…');
assert.equal(options.enableHighAccuracy, false);
success({coords: {latitude: 40.1, longitude: -83.2, accuracy: 12}});
assert.deepEqual(map.views, [[[40.1, -83.2], 19]]);
assert.equal(locateButton.disabled, false);
assert.equal(locateButton.textContent, 'My location');
assert.equal(statuses.at(-1), 'Click the first corner.');
locate(true);
failure({code: 1});
assert.match(statuses.at(-1), /permission is blocked/);
console.log(JSON.stringify(statuses));
'''.replace('FUNCTIONS', '\n'.join(functions))
    subprocess.run(
        [node, '-e', script], check=True, capture_output=True, text=True)


def test_address_search_accepts_coordinates_and_geocoder_results():
    import shutil
    import subprocess

    import pytest

    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js is required for picker JavaScript regression')
    module = load_script()
    functions = []
    for name in ('coordinatesFor', 'showSearchLocation', 'searchLocation'):
        prefix = '    async function' if name == 'searchLocation' else '    function'
        begin = module.PAGE.index(f'{prefix} {name}(')
        end = module.PAGE.index('\n    }', begin) + len('\n    }')
        functions.append(module.PAGE[begin:end])
    script = r'''
import assert from 'node:assert/strict';
const statuses = [];
let searchLayer = null;
const locationQuery = {value: '', focus() {}};
const searchLocationButton = {disabled: false, textContent: 'Find'};
const map = {
  views: [], removed: [],
  setView(point, zoom) {this.views.push([point, zoom]);},
  removeLayer(layer) {this.removed.push(layer);}
};
const L = {marker(point, options) {
  return {point, options, addTo() {return this;}};
}};
function setStatus(message) {statuses.push(message);}
const URLSearchParams = globalThis.URLSearchParams;
let fetchedUrl = null;
async function fetch(url) {
  fetchedUrl = String(url);
  return {ok: true, json: async () => [{
    lat: '40.101', lon: '-83.202', display_name: 'Test Address'
  }]};
}
FUNCTIONS
assert.deepEqual(coordinatesFor('40.1, -83.2'), [40.1, -83.2]);
assert.equal(coordinatesFor('100, -83.2'), null);
locationQuery.value = '40.2 -83.3';
await searchLocation({preventDefault() {}});
assert.deepEqual(map.views.at(-1), [[40.2, -83.3], 18]);
locationQuery.value = '1 Test Street';
await searchLocation({preventDefault() {}});
assert.match(fetchedUrl, /nominatim\.openstreetmap\.org\/search/);
assert.deepEqual(map.views.at(-1), [[40.101, -83.202], 18]);
assert.equal(statuses.at(-1), 'Showing Test Address. Click the first corner.');
assert.equal(searchLocationButton.textContent, 'Find');
'''.replace('FUNCTIONS', '\n'.join(functions))
    subprocess.run(
        [node, '--input-type=module', '-e', script],
        check=True, capture_output=True, text=True)
