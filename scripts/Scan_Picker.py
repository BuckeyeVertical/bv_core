#!/usr/bin/env python3
"""Open a browser map for selecting a rectangular scan boundary.

This utility intentionally uses only the Python standard library. The map page
loads Leaflet and OpenStreetMap tiles in the browser.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


FLOAT_PATTERN = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
POINT_PATTERN = re.compile(
    rf'^\s*-\s*\[\s*({FLOAT_PATTERN})\s*,\s*({FLOAT_PATTERN})'
)


PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Scan Region Picker</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    * { box-sizing: border-box; }
    html, body { height: 100%; margin: 0; font: 15px/1.4 system-ui, sans-serif; }
    body { display: grid; grid-template-columns: minmax(0, 1fr) 390px; color: #172033; }
    #map { height: 100%; cursor: crosshair; }
    aside { padding: 22px; overflow: auto; background: #f7f8fb; box-shadow: -2px 0 12px #0002; z-index: 500; }
    h1 { margin: 0 0 8px; font-size: 23px; }
    p { margin: 8px 0 18px; }
    #status { padding: 11px 12px; border-radius: 8px; background: #e8eefc; }
    textarea { width: 100%; height: 170px; margin: 14px 0 10px; padding: 12px; resize: vertical; border: 1px solid #b8c0d0; border-radius: 7px; font: 14px/1.55 ui-monospace, monospace; }
    button { margin: 0 7px 8px 0; padding: 9px 13px; border: 0; border-radius: 6px; background: #295bd6; color: white; font-weight: 650; cursor: pointer; }
    button.secondary { background: #596174; }
    button:disabled { opacity: .45; cursor: default; }
    .hint { margin-top: 14px; color: #596174; font-size: 13px; }
    .leaflet-control-attribution { font-size: 10px; }
    @media (max-width: 760px) {
      body { grid-template: minmax(55vh, 1fr) auto / 1fr; }
      aside { box-shadow: 0 -2px 12px #0002; }
    }
  </style>
</head>
<body>
  <main id="map" aria-label="Map for selecting scan region"></main>
  <aside>
    <h1>Scan region</h1>
    <p>Click one corner of the 1–10 acre search area, then click its opposite corner.</p>
    <div id="status">Finding your location…</div>
    <textarea id="yaml" readonly aria-label="Generated YAML"># Select two corners on the map</textarea>
    <button id="copy" disabled>Copy YAML</button>
    <button id="reset" class="secondary">Reset</button>
    <button id="locate" class="secondary">My location</button>
    <button id="configured" class="secondary">Show configured region</button>
    <button id="quit" class="secondary">Stop server</button>
    <div class="hint">Satellite imagery is shown by default; use the layer control
      to switch to streets. The generated corners are ordered around the perimeter as
      northwest, southwest, southeast, northeast. Paste the whole block into a
      mission parameter YAML file. Always verify the boundary against surveyed or
      official coordinates before flight.</div>
  </aside>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const streets = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; OpenStreetMap contributors'
    });
    const satellite = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      {
        maxNativeZoom: 19,
        maxZoom: 21,
        attribution: 'Tiles &copy; Esri &mdash; Sources: Esri, Vantor, Earthstar Geographics, and the GIS User Community'
      }
    );
    const map = L.map('map', {layers: [satellite]}).setView([39.8283, -98.5795], 4);
    L.control.layers({Satellite: satellite, Streets: streets}, null, {
      collapsed: false
    }).addTo(map);
    L.control.scale({imperial: true, metric: true}).addTo(map);

    const statusBox = document.getElementById('status');
    const yamlBox = document.getElementById('yaml');
    const copyButton = document.getElementById('copy');
    const configuredButton = document.getElementById('configured');
    const configuredRegion = __CONFIGURED_REGION__;
    let firstCorner = null;
    let rectangle = null;
    let cornerMarkers = [];
    let configuredLayer = null;

    function setStatus(message) { statusBox.textContent = message; }

    function yamlFor(bounds) {
      const north = bounds.getNorth().toFixed(8);
      const south = bounds.getSouth().toFixed(8);
      const east = bounds.getEast().toFixed(8);
      const west = bounds.getWest().toFixed(8);
      return `scan_boundary:\n` +
        `  - [${north}, ${west}]\n` +
        `  - [${south}, ${west}]\n` +
        `  - [${south}, ${east}]\n` +
        `  - [${north}, ${east}]`;
    }

    function regionSize(bounds) {
      const widthM = map.distance(bounds.getNorthWest(), bounds.getNorthEast());
      const heightM = map.distance(bounds.getNorthWest(), bounds.getSouthWest());
      const acres = widthM * heightM / 4046.8564224;
      const shownAcres = acres < 10 ? acres.toFixed(2) : acres.toFixed(1);
      return `${shownAcres} acres (${Math.round(widthM)} × ${Math.round(heightM)} m)`;
    }

    function clearSelection() {
      firstCorner = null;
      if (rectangle) map.removeLayer(rectangle);
      rectangle = null;
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [];
      yamlBox.value = '# Select two corners on the map';
      copyButton.disabled = true;
      setStatus('Click the first corner.');
    }

    function updateFinishedRectangle() {
      const bounds = L.latLngBounds(
        cornerMarkers[0].getLatLng(), cornerMarkers[1].getLatLng());
      rectangle.setBounds(bounds);
      yamlBox.value = yamlFor(bounds);
      copyButton.disabled = false;
      setStatus(`Rectangle ready — ${regionSize(bounds)}. Drag either corner to adjust it.`);
    }

    function finishRectangle(secondCorner) {
      const bounds = L.latLngBounds(firstCorner, secondCorner);
      rectangle.setBounds(bounds);
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [
        L.marker(bounds.getNorthWest(), {draggable: true}).addTo(map),
        L.marker(bounds.getSouthEast(), {draggable: true}).addTo(map)
      ];
      cornerMarkers.forEach(marker => marker.on('drag', updateFinishedRectangle));
      firstCorner = null;
      updateFinishedRectangle();
    }

    function showConfiguredRegion() {
      if (configuredLayer) map.removeLayer(configuredLayer);
      const layers = [];
      if (configuredRegion.boundary.length >= 3) {
        layers.push(L.polygon(configuredRegion.boundary, {
          color: '#20a464', weight: 4, fillOpacity: 0.12
        }).bindTooltip(`${configuredRegion.label}: scan boundary`));
      }
      if (configuredRegion.points.length >= 1) {
        layers.push(L.polyline(configuredRegion.points, {
          color: '#28a9e0', weight: 4, dashArray: '8 7'
        }).bindTooltip(`${configuredRegion.label}: scan points`));
        configuredRegion.points.forEach((point, index) => {
          layers.push(L.circleMarker(point, {
            radius: 5, color: '#ffffff', weight: 2,
            fillColor: '#28a9e0', fillOpacity: 1
          }).bindTooltip(`Scan point ${index + 1}`));
        });
      }
      configuredLayer = L.featureGroup(layers).addTo(map);
      map.fitBounds(configuredLayer.getBounds(), {padding: [45, 45], maxZoom: 19});
      setStatus(`Showing the configured scan region from ${configuredRegion.label}.`);
    }

    map.on('click', event => {
      if (!firstCorner && rectangle && copyButton.disabled === false) clearSelection();
      if (!firstCorner) {
        firstCorner = event.latlng;
        cornerMarkers = [L.marker(firstCorner).addTo(map)];
        rectangle = L.rectangle([firstCorner, firstCorner], {
          color: '#e34133', weight: 3, fillOpacity: 0.16
        }).addTo(map);
        setStatus('Now click the opposite corner.');
      } else {
        finishRectangle(event.latlng);
      }
    });

    map.on('mousemove', event => {
      if (firstCorner && rectangle) rectangle.setBounds([firstCorner, event.latlng]);
    });

    function locate(forceRecenter = false) {
      setStatus('Finding your location…');
      if (!navigator.geolocation) {
        setStatus('Location is unavailable. Pan/zoom the map, then click the first corner.');
        return;
      }
      navigator.geolocation.getCurrentPosition(position => {
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        const here = [position.coords.latitude, position.coords.longitude];
        map.setView(here, 19);
        L.circle(here, {
          radius: Math.max(position.coords.accuracy, 2),
          color: '#295bd6', fillOpacity: 0.10, weight: 2
        }).addTo(map).bindPopup(`Approximate location (±${Math.round(position.coords.accuracy)} m)`);
        setStatus(rectangle ? 'Centered on your location; the rectangle is unchanged.' :
          'Click the first corner.');
      }, () => {
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        setStatus('Location permission was unavailable. Pan/zoom the map, then click the first corner.');
      }, {enableHighAccuracy: true, timeout: 10000, maximumAge: 30000});
    }

    copyButton.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(yamlBox.value);
      } catch (_) {
        yamlBox.select();
        document.execCommand('copy');
      }
      setStatus('YAML copied to the clipboard.');
    });
    document.getElementById('reset').addEventListener('click', clearSelection);
    document.getElementById('locate').addEventListener('click', () => locate(true));
    const hasConfiguredRegion = configuredRegion.boundary.length >= 3 ||
      configuredRegion.points.length >= 1;
    configuredButton.disabled = !hasConfiguredRegion;
    configuredButton.textContent = hasConfiguredRegion ?
      `Show ${configuredRegion.label}` : 'No configured region found';
    configuredButton.addEventListener('click', showConfiguredRegion);
    document.getElementById('quit').addEventListener('click', async () => {
      await fetch('/shutdown', {method: 'POST'});
      setStatus('Server stopped. You can close this tab.');
    });
    locate();
  </script>
</body>
</html>
"""


class MapHandler(BaseHTTPRequestHandler):
    """Serve the picker page and its local shutdown endpoint."""

    page = PAGE

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path not in ('/', '/index.html'):
            self.send_error(404)
            return
        body = self.page.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != '/shutdown':
            self.send_error(404)
            return
        self.send_response(204)
        self.end_headers()
        threading.Thread(target=self.server.shutdown, daemon=True).start()

    def log_message(self, format_, *args):
        """Keep the terminal focused on useful lifecycle messages."""


def parse_args():
    parser = argparse.ArgumentParser(
        description='Open a map and generate scan_boundary YAML.'
    )
    parser.add_argument(
        '--port', type=int, default=0,
        help='localhost port (default: choose an available port)',
    )
    parser.add_argument(
        '--no-browser', action='store_true',
        help='serve the page without opening a browser automatically',
    )
    parser.add_argument(
        '--config', type=Path,
        help=('mission YAML to display (default: config/$BV_MISSION_CONFIG, '
              'or config/real_params.yaml)'),
    )
    return parser.parse_args()


def configured_region(config_path):
    """Read scan coordinates from a mission YAML without requiring PyYAML."""
    coordinates = {'scan_boundary': [], 'scan_points': []}
    active_key = None
    try:
        lines = config_path.read_text(encoding='utf-8').splitlines()
    except OSError as error:
        raise SystemExit(f'Could not read mission config {config_path}: {error}') from error

    for raw_line in lines:
        line = raw_line.split('#', 1)[0].rstrip()
        if line and not line[0].isspace():
            key = line.partition(':')[0].strip()
            active_key = key if key in coordinates else None
            continue
        if active_key:
            match = POINT_PATTERN.match(line)
            if match:
                coordinates[active_key].append([
                    float(match.group(1)), float(match.group(2))
                ])

    return {
        'label': config_path.name,
        'boundary': coordinates['scan_boundary'],
        'points': coordinates['scan_points'],
    }


def default_config_path():
    config_name = os.environ.get('BV_MISSION_CONFIG', 'real_params.yaml')
    if os.path.basename(config_name) != config_name or not config_name.endswith('.yaml'):
        raise SystemExit(f'Invalid BV_MISSION_CONFIG: {config_name!r}')
    return Path(__file__).resolve().parents[1] / 'config' / config_name


def main():
    args = parse_args()
    config_path = args.config.expanduser().resolve() if args.config else default_config_path()
    region = configured_region(config_path)
    page = PAGE.replace(
        '__CONFIGURED_REGION__',
        json.dumps(region, separators=(',', ':')).replace('</', '<\\/'),
    )
    MapHandler.page = page
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), MapHandler)
    except OSError as error:
        raise SystemExit(f'Could not start local map server: {error}') from error

    url = f'http://127.0.0.1:{server.server_port}/'
    print(f'Scan region picker: {url}')
    print('Press Ctrl+C here, or click "Stop server" in the page, when done.')
    if not args.no_browser and not webbrowser.open(url):
        print('The browser did not open automatically; open the URL above.')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nStopping scan region picker.')
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
