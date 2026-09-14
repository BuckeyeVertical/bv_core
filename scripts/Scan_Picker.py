#!/usr/bin/env python3
"""Open a browser map for selecting a rectangular scan boundary.

The map page loads Leaflet and map tiles in the browser, while its localhost
endpoint calls the same Python scan planner used by the flight mission.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import secrets
import sys
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import yaml

from bv_core.scan_plan import build_scan_plan


FLOAT_PATTERN = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
POINT_PATTERN = re.compile(
    rf'^\s*-\s*\[\s*({FLOAT_PATTERN})\s*,\s*({FLOAT_PATTERN})'
)
POINT_WITH_ALT_PATTERN = re.compile(
    rf'^\s*-\s*\[\s*{FLOAT_PATTERN}\s*,\s*{FLOAT_PATTERN}\s*,\s*([^\],]+)'
)


def _validated_boundary(boundary):
    if not isinstance(boundary, list) or len(boundary) != 4:
        raise ValueError('region must contain exactly four corners')
    result = []
    for point in boundary:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError('each corner must contain latitude and longitude')
        lat, lon = float(point[0]), float(point[1])
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            raise ValueError('region contains an invalid latitude or longitude')
        result.append([lat, lon])
    if len({tuple(point) for point in result}) != 4:
        raise ValueError('region corners must be distinct')
    return result


def _replace_yaml_block(text, key, lines):
    """Replace one top-level YAML list while preserving surrounding comments."""
    source = text.splitlines(keepends=True)
    start = next((index for index, line in enumerate(source)
                  if line.startswith(f'{key}:')), None)
    if start is None:
        prefix = '' if text.endswith('\n') or not text else '\n'
        return text + prefix + '\n'.join(lines) + '\n'

    end = start + 1
    while end < len(source):
        stripped = source[end].strip()
        if stripped and not source[end][0].isspace() and not stripped.startswith('#'):
            break
        if stripped.startswith('#') and not source[end][0].isspace():
            break
        end += 1

    replacement = [f'{line}\n' for line in lines]
    return ''.join(source[:start] + replacement + source[end:])


def _upsert_yaml_scalar(text, key, value, before):
    pattern = re.compile(rf'(?m)^{re.escape(key)}:.*$')
    line = f'{key}: {value}'
    if pattern.search(text):
        return pattern.sub(line, text, count=1)
    marker = re.search(rf'(?m)^{re.escape(before)}:', text)
    if marker:
        return text[:marker.start()] + line + '\n' + text[marker.start():]
    prefix = '' if text.endswith('\n') or not text else '\n'
    return text + prefix + line + '\n'


def update_region_yaml(text, kind, boundary, *, sweep='long', start='top',
                       lap_direction='counterclockwise'):
    """Return mission YAML with only the selected route fields changed."""
    corners = _validated_boundary(boundary)
    if kind == 'scan':
        if sweep not in ('long', 'short'):
            raise ValueError("scan sweep must be 'long' or 'short'")
        if start not in ('top', 'bottom'):
            raise ValueError("scan start must be 'top' or 'bottom'")
        text = _upsert_yaml_scalar(text, 'scan_sweep', sweep, 'scan_boundary')
        text = _upsert_yaml_scalar(text, 'scan_start', start, 'scan_boundary')
        lines = ['scan_boundary:'] + [
            f'  - [{lat:.8f}, {lon:.8f}]' for lat, lon in corners
        ] + ['']
        return _replace_yaml_block(text, 'scan_boundary', lines)

    if kind != 'lap':
        raise ValueError("region kind must be 'scan' or 'lap'")
    if lap_direction not in ('clockwise', 'counterclockwise'):
        raise ValueError('invalid lap direction')

    # The browser sends NW, SW, SE, NE. That is counterclockwise; reverse the
    # perimeter (while retaining NW as waypoint 1) for clockwise flight.
    route = corners if lap_direction == 'counterclockwise' else [
        corners[0], corners[3], corners[2], corners[1]
    ]
    route.append(route[0])
    altitude = '*LAP_MSL'
    match = next((POINT_WITH_ALT_PATTERN.match(line)
                  for line in text.splitlines()
                  if POINT_WITH_ALT_PATTERN.match(line)), None)
    if match:
        altitude = match.group(1).strip()
    lines = ['points:'] + [
        f'  - [{lat:.8f}, {lon:.8f}, {altitude}]' for lat, lon in route
    ] + ['']
    return _replace_yaml_block(text, 'points', lines)


def save_region(config_path, kind, boundary, **options):
    """Atomically update a selected region in the active mission YAML."""
    original = config_path.read_text(encoding='utf-8')
    original_mode = config_path.stat().st_mode
    updated = update_region_yaml(original, kind, boundary, **options)
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=config_path.parent,
        prefix=f'.{config_path.name}.', suffix='.tmp', delete=False,
    ) as stream:
        stream.write(updated)
        temporary_path = Path(stream.name)
    try:
        os.chmod(temporary_path, original_mode)
        os.replace(temporary_path, config_path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


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
    button.choice { background: #dfe4ef; color: #273044; }
    button.choice.active { background: #295bd6; color: white; }
    button:disabled { opacity: .45; cursor: default; }
    .location-search { display: flex; gap: 7px; margin: 14px 0 8px; }
    .location-search input { min-width: 0; flex: 1; padding: 9px 10px; border: 1px solid #b8c0d0; border-radius: 6px; font: inherit; }
    .location-search button { margin: 0; white-space: nowrap; }
    .search-note { margin: -3px 0 9px; color: #596174; font-size: 11px; }
    .tabs { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 16px; }
    .tabs button { margin: 0; background: #dfe4ef; color: #273044; }
    .tabs button.active { background: #172033; color: white; }
    .hidden { display: none; }
    .controls { margin: 15px 0 8px; padding: 12px; border: 1px solid #d2d7e2; border-radius: 8px; background: white; }
    .controls label { display: block; margin-bottom: 7px; font-size: 13px; font-weight: 700; }
    #plan-summary { min-height: 24px; margin: 8px 0; font-weight: 700; color: #24533c; }
    .direction-arrow { color: #123caa; font-size: 20px; font-weight: 900; line-height: 20px; text-shadow: 0 0 3px white, 0 0 3px white; }
    .waypoint-number { width: 25px; height: 25px; border-radius: 50%; background: #7b2fc4; color: white; border: 2px solid white; text-align: center; font: 700 13px/21px system-ui, sans-serif; box-shadow: 0 1px 4px #0008; }
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
    <div class="tabs">
      <button id="scan-tab" class="active">Scan region</button>
      <button id="lap-tab">Lap route</button>
    </div>
    <h1 id="mode-title">Scan region</h1>
    <p id="instructions">Click one corner of the 1–10 acre search area, then click its opposite corner.</p>
    <div id="status">Finding your location…</div>
    <div id="scan-controls" class="controls">
      <label>Row direction</label>
      <button id="long" class="choice active">Long way</button>
      <button id="short" class="choice">Short way</button>
      <label>Route start</label>
      <button id="top" class="choice active">Top → bottom</button>
      <button id="bottom" class="choice">Bottom → top</button>
    </div>
    <div id="lap-controls" class="controls hidden">
      <label>Lap direction</label>
      <button id="counterclockwise" class="choice active">Counterclockwise</button>
      <button id="clockwise" class="choice">Clockwise</button>
    </div>
    <div id="plan-summary">Select a region to preview the rows.</div>
    <textarea id="yaml" readonly aria-label="Generated YAML"># Select two corners on the map</textarea>
    <button id="copy" disabled>Copy YAML</button>
    <button id="use-region" disabled>Use Region</button>
    <button id="reset" class="secondary">Reset</button>
    <form id="location-search" class="location-search">
      <input id="location-query" type="search" aria-label="Address or coordinates"
        placeholder="Address or lat, lon" autocomplete="off">
      <button id="search-location" type="submit" class="secondary">Find</button>
    </form>
    <div class="search-note">Submit-only address search by
      <a href="https://www.openstreetmap.org/copyright" target="_blank">OpenStreetMap</a> ·
      <a href="https://operations.osmfoundation.org/policies/nominatim/"
        target="_blank">usage policy</a>
    </div>
    <button id="locate" class="secondary">My location</button>
    <button id="configured" class="secondary">Show configured region</button>
    <button id="quit" class="secondary">Stop server</button>
    <div class="hint">Satellite imagery is shown by default; use the layer control
      to switch to streets. The generated corners are ordered around the perimeter as
      northwest, southwest, southeast, northeast. Paste the whole block into a
      mission parameter YAML file. Blue arrows are scan rows; orange arrows are
      turns between rows. Lap markers are numbered in flight order. <b>Use Region</b>
      writes the displayed route into the active mission YAML. Always verify the
      boundary against surveyed or official coordinates before flight.</div>
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
    const useRegionButton = document.getElementById('use-region');
    const planSummary = document.getElementById('plan-summary');
    const configuredButton = document.getElementById('configured');
    const locateButton = document.getElementById('locate');
    const locationQuery = document.getElementById('location-query');
    const searchLocationButton = document.getElementById('search-location');
    const configuredRegion = __CONFIGURED_REGION__;
    const writeToken = __WRITE_TOKEN__;
    let firstCorner = null;
    let rectangle = null;
    let cornerMarkers = [];
    let configuredLayer = null;
    let planLayer = null;
    let sweep = 'long';
    let routeStart = 'top';
    let lapDirection = 'counterclockwise';
    let mode = 'scan';
    let planRequest = 0;
    let activeBounds = null;
    let activeScanBoundary = null;
    let locationLayer = null;
    let locationRequest = 0;
    let searchLayer = null;

    function setStatus(message) { statusBox.textContent = message; }

    function yamlFor(bounds) {
      const north = bounds.getNorth().toFixed(8);
      const south = bounds.getSouth().toFixed(8);
      const east = bounds.getEast().toFixed(8);
      const west = bounds.getWest().toFixed(8);
      const corners = {
        nw: `[${north}, ${west}]`, sw: `[${south}, ${west}]`,
        se: `[${south}, ${east}]`, ne: `[${north}, ${east}]`
      };
      if (mode === 'lap') {
        const order = lapDirection === 'counterclockwise' ?
          ['nw', 'sw', 'se', 'ne', 'nw'] : ['nw', 'ne', 'se', 'sw', 'nw'];
        return `points:\n` + order.map(
          corner => `  - [${corners[corner].slice(1, -1)}, *LAP_MSL]`
        ).join('\\n');
      }
      return `scan_sweep: ${sweep}\n` +
        `scan_start: ${routeStart}\n` +
        `scan_boundary:\n` +
        boundaryFor(bounds).map(
          point => `  - [${point[0].toFixed(8)}, ${point[1].toFixed(8)}]`
        ).join('\\n');
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
      activeBounds = null;
      activeScanBoundary = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [];
      yamlBox.value = '# Select two corners on the map';
      copyButton.disabled = true;
      useRegionButton.disabled = true;
      planSummary.textContent = mode === 'scan' ?
        'Select a region to preview the rows.' :
        'Select a region to preview the lap route.';
      setStatus('Click the first corner.');
    }

    function boundaryFor(bounds) {
      if (mode === 'scan' && activeScanBoundary) {
        return activeScanBoundary.map(point => [...point]);
      }
      return [bounds.getNorthWest(), bounds.getSouthWest(),
        bounds.getSouthEast(), bounds.getNorthEast()].map(
          point => [point.lat, point.lng]);
    }

    function bearingDegrees(from, to) {
      const lat1 = from.lat * Math.PI / 180;
      const lat2 = to.lat * Math.PI / 180;
      const deltaLon = (to.lng - from.lng) * Math.PI / 180;
      const y = Math.sin(deltaLon) * Math.cos(lat2);
      const x = Math.cos(lat1) * Math.sin(lat2) -
        Math.sin(lat1) * Math.cos(lat2) * Math.cos(deltaLon);
      return Math.atan2(y, x) * 180 / Math.PI;
    }

    function arrowFor(from, to, color) {
      const midpoint = L.latLng(
        (from.lat + to.lat) / 2, (from.lng + to.lng) / 2);
      const angle = bearingDegrees(from, to);
      return L.marker(midpoint, {
        interactive: false,
        icon: L.divIcon({
          className: '', iconSize: [22, 22], iconAnchor: [11, 11],
          html: `<div class="direction-arrow" style="color:${color};transform:rotate(${angle}deg)">↑</div>`
        })
      });
    }

    function drawPlan(plan) {
      if (planLayer) map.removeLayer(planLayer);
      const layers = [];
      const points = plan.waypoints.map(point => L.latLng(point[0], point[1]));
      for (let index = 0; index + 1 < points.length; index++) {
        const isRow = index % 2 === 0;
        const color = isRow ? '#245bd6' : '#e47720';
        layers.push(L.polyline([points[index], points[index + 1]], {
          color, weight: isRow ? 5 : 3,
          opacity: 0.9, dashArray: isRow ? null : '7 7'
        }));
        layers.push(arrowFor(points[index], points[index + 1], color));
        if (isRow) {
          layers.push(L.circleMarker(points[index], {
            radius: 6, color: '#fff', weight: 2,
            fillColor: color, fillOpacity: 1
          }).bindTooltip(`Row ${index / 2 + 1} start`));
        }
      }
      if (points.length) {
        layers.push(L.circleMarker(points[points.length - 1], {
          radius: 6, color: '#fff', weight: 2,
          fillColor: '#172033', fillOpacity: 1
        }).bindTooltip('Scan finish'));
      }
      planLayer = L.featureGroup(layers).addTo(map);
      planSummary.textContent = `${plan.row_count} row${plan.row_count === 1 ? '' : 's'} · ` +
        `${plan.row_spacing_m.toFixed(1)} m apart · ${plan.waypoints.length} waypoints`;
    }

    function lapRouteFor(bounds) {
      const corners = [bounds.getNorthWest(), bounds.getSouthWest(),
        bounds.getSouthEast(), bounds.getNorthEast()];
      const route = lapDirection === 'counterclockwise' ? corners :
        [corners[0], corners[3], corners[2], corners[1]];
      return [...route, route[0]];
    }

    function drawLapPlan(route) {
      if (planLayer) map.removeLayer(planLayer);
      const layers = [];
      for (let index = 0; index + 1 < route.length; index++) {
        layers.push(L.polyline([route[index], route[index + 1]], {
          color: '#7b2fc4', weight: 5, opacity: 0.9
        }));
        layers.push(arrowFor(route[index], route[index + 1], '#7b2fc4'));
        layers.push(L.marker(route[index], {
          icon: L.divIcon({
            className: '', iconSize: [25, 25], iconAnchor: [12, 12],
            html: `<div class="waypoint-number">${index + 1}</div>`
          })
        }).bindTooltip(`Lap waypoint ${index + 1}`));
      }
      planLayer = L.featureGroup(layers).addTo(map);
      const laps = configuredRegion.lap_count;
      planSummary.textContent = `4 lap waypoints · ${lapDirection} · ` +
        `${laps} configured lap${laps === 1 ? '' : 's'} · closes at waypoint 1`;
    }

    function refreshPreview(bounds) {
      if (mode === 'scan') refreshPlan(bounds);
      else drawLapPlan(lapRouteFor(bounds));
    }

    async function refreshPlan(bounds) {
      if (!bounds) return;
      const request = ++planRequest;
      planSummary.textContent = 'Generating mission rows…';
      try {
        const response = await fetch('/plan', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            boundary: boundaryFor(bounds), sweep, start: routeStart
          })
        });
        const result = await response.json();
        if (request !== planRequest) return;
        if (!response.ok) throw new Error(result.error || 'planner failed');
        drawPlan(result);
      } catch (error) {
        if (request !== planRequest) return;
        planSummary.textContent = `Could not generate rows: ${error.message}`;
      }
    }

    function updateFinishedRectangle() {
      activeScanBoundary = null;
      const bounds = L.latLngBounds(
        cornerMarkers[0].getLatLng(), cornerMarkers[1].getLatLng());
      activeBounds = bounds;
      rectangle.setBounds(bounds);
      yamlBox.value = yamlFor(bounds);
      copyButton.disabled = false;
      useRegionButton.disabled = false;
      setStatus(`Rectangle ready — ${regionSize(bounds)}. Drag either corner to adjust it.`);
      refreshPreview(bounds);
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
      configuredLayer = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      const configuredPoints = mode === 'scan' ?
        configuredRegion.boundary : configuredRegion.lap_points;
      if (configuredPoints.length < 3) {
        setStatus(`No configured ${mode} region found in ${configuredRegion.label}.`);
        return;
      }
      const layers = [];
      layers.push(L.polygon(configuredPoints, {
        color: '#20a464', weight: 4, fillOpacity: 0.12
      }).bindTooltip(`${configuredRegion.label}: ${mode} region`));
      configuredLayer = L.featureGroup(layers).addTo(map);
      map.fitBounds(configuredLayer.getBounds(), {padding: [45, 45], maxZoom: 19});
      activeBounds = L.latLngBounds(configuredPoints);
      if (mode === 'scan') {
        activeScanBoundary = configuredPoints.map(point => [...point]);
        sweep = configuredRegion.sweep;
        routeStart = configuredRegion.start;
      } else {
        lapDirection = configuredRegion.lap_direction;
      }
      updateChoiceButtons();
      yamlBox.value = yamlFor(activeBounds);
      copyButton.disabled = false;
      useRegionButton.disabled = false;
      refreshPreview(activeBounds);
      setStatus(`Showing the configured ${mode} region from ${configuredRegion.label}.`);
    }

    function updateChoiceButtons() {
      document.getElementById('long').classList.toggle('active', sweep === 'long');
      document.getElementById('short').classList.toggle('active', sweep === 'short');
      document.getElementById('top').classList.toggle('active', routeStart === 'top');
      document.getElementById('bottom').classList.toggle('active', routeStart === 'bottom');
      document.getElementById('clockwise').classList.toggle(
        'active', lapDirection === 'clockwise');
      document.getElementById('counterclockwise').classList.toggle(
        'active', lapDirection === 'counterclockwise');
      if (activeBounds && !firstCorner) {
        yamlBox.value = yamlFor(activeBounds);
        refreshPreview(activeBounds);
      }
    }

    document.getElementById('long').addEventListener('click', () => {
      sweep = 'long'; updateChoiceButtons();
    });
    document.getElementById('short').addEventListener('click', () => {
      sweep = 'short'; updateChoiceButtons();
    });
    document.getElementById('top').addEventListener('click', () => {
      routeStart = 'top'; updateChoiceButtons();
    });
    document.getElementById('bottom').addEventListener('click', () => {
      routeStart = 'bottom'; updateChoiceButtons();
    });
    document.getElementById('clockwise').addEventListener('click', () => {
      lapDirection = 'clockwise'; updateChoiceButtons();
    });
    document.getElementById('counterclockwise').addEventListener('click', () => {
      lapDirection = 'counterclockwise'; updateChoiceButtons();
    });

    function switchMode(nextMode) {
      if (nextMode === mode) return;
      mode = nextMode;
      firstCorner = null;
      if (rectangle) map.removeLayer(rectangle);
      rectangle = null;
      activeBounds = null;
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [];
      if (configuredLayer) map.removeLayer(configuredLayer);
      configuredLayer = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      document.getElementById('scan-tab').classList.toggle('active', mode === 'scan');
      document.getElementById('lap-tab').classList.toggle('active', mode === 'lap');
      document.getElementById('scan-controls').classList.toggle('hidden', mode !== 'scan');
      document.getElementById('lap-controls').classList.toggle('hidden', mode !== 'lap');
      document.getElementById('mode-title').textContent =
        mode === 'scan' ? 'Scan region' : 'Lap route';
      document.getElementById('instructions').textContent = mode === 'scan' ?
        'Click one corner of the 1–10 acre search area, then click its opposite corner.' :
        'Click two opposite corners. The rectangle becomes a numbered, closed lap route.';
      configuredButton.textContent = `Show ${configuredRegion.label} ${mode}`;
      yamlBox.value = '# Select two corners on the map';
      copyButton.disabled = true;
      useRegionButton.disabled = true;
      planSummary.textContent = mode === 'scan' ?
        'Select a region to preview the rows.' :
        'Select a region to preview the lap route.';
      showConfiguredRegion();
    }

    document.getElementById('scan-tab').addEventListener('click', () => switchMode('scan'));
    document.getElementById('lap-tab').addEventListener('click', () => switchMode('lap'));

    map.on('click', event => {
      if (!firstCorner && activeBounds && copyButton.disabled === false) clearSelection();
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

    function finishLocate(request) {
      if (request !== locationRequest) return false;
      locateButton.disabled = false;
      locateButton.textContent = 'My location';
      return true;
    }

    function locationErrorMessage(error) {
      if (error && error.code === 1) {
        return 'Location permission is blocked. Allow location for this page in your browser settings, then try again.';
      }
      if (error && error.code === 2) {
        return 'Your location could not be determined. Check Location Services or pan/zoom the map manually.';
      }
      if (error && error.code === 3) {
        return 'Finding your location timed out. Try again or pan/zoom the map manually.';
      }
      return 'Location is unavailable. Pan/zoom the map, then click the first corner.';
    }

    function coordinatesFor(query) {
      const match = query.match(/^\\s*([-+]?\\d+(?:\\.\\d+)?)\\s*[, ]\\s*([-+]?\\d+(?:\\.\\d+)?)\\s*$/);
      if (!match) return null;
      const latitude = Number(match[1]);
      const longitude = Number(match[2]);
      if (latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180) {
        return null;
      }
      return [latitude, longitude];
    }

    function showSearchLocation(latitude, longitude, label) {
      const point = [latitude, longitude];
      map.setView(point, 18);
      if (searchLayer) map.removeLayer(searchLayer);
      searchLayer = L.marker(point, {title: label}).addTo(map);
      setStatus(`Showing ${label}. Click the first corner.`);
    }

    async function searchLocation(event) {
      event.preventDefault();
      const query = locationQuery.value.trim();
      if (!query) {
        setStatus('Enter an address or latitude, longitude.');
        locationQuery.focus();
        return;
      }
      searchLocationButton.disabled = true;
      searchLocationButton.textContent = 'Finding…';
      try {
        const coordinates = coordinatesFor(query);
        if (coordinates) {
          showSearchLocation(coordinates[0], coordinates[1], query);
          return;
        }
        const parameters = new URLSearchParams({
          q: query, format: 'jsonv2', limit: '1'
        });
        const response = await fetch(
          `https://nominatim.openstreetmap.org/search?${parameters}`,
          {headers: {'Accept': 'application/json'}});
        if (!response.ok) throw new Error(`search failed (${response.status})`);
        const results = await response.json();
        if (!results.length) throw new Error('address not found');
        const latitude = Number(results[0].lat);
        const longitude = Number(results[0].lon);
        if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) {
          throw new Error('search returned invalid coordinates');
        }
        showSearchLocation(latitude, longitude, results[0].display_name);
      } catch (error) {
        setStatus(`Could not find that location: ${error.message}.`);
      } finally {
        searchLocationButton.disabled = false;
        searchLocationButton.textContent = 'Find';
      }
    }

    function locate(forceRecenter = false) {
      const request = ++locationRequest;
      setStatus('Finding your location…');
      locateButton.disabled = forceRecenter;
      locateButton.textContent = 'Locating…';
      if (!navigator.geolocation) {
        finishLocate(request);
        setStatus(locationErrorMessage());
        return;
      }
      navigator.geolocation.getCurrentPosition(position => {
        if (!finishLocate(request)) return;
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        const here = [position.coords.latitude, position.coords.longitude];
        map.setView(here, 19);
        if (locationLayer) map.removeLayer(locationLayer);
        locationLayer = L.circle(here, {
          radius: Math.max(position.coords.accuracy, 2),
          color: '#295bd6', fillOpacity: 0.10, weight: 2
        }).addTo(map).bindPopup(`Approximate location (±${Math.round(position.coords.accuracy)} m)`);
        setStatus(rectangle ? 'Centered on your location; the rectangle is unchanged.' :
          'Click the first corner.');
      }, error => {
        if (!finishLocate(request)) return;
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        setStatus(locationErrorMessage(error));
      }, {enableHighAccuracy: false, timeout: 15000, maximumAge: 300000});
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
    useRegionButton.addEventListener('click', async () => {
      if (!activeBounds) return;
      useRegionButton.disabled = true;
      setStatus(`Saving ${mode} region to ${configuredRegion.label}…`);
      try {
        const response = await fetch('/use-region', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            token: writeToken,
            kind: mode,
            boundary: boundaryFor(activeBounds),
            sweep,
            start: routeStart,
            lap_direction: lapDirection
          })
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || 'save failed');
        if (mode === 'scan') {
          configuredRegion.boundary = boundaryFor(activeBounds);
          configuredRegion.sweep = sweep;
          configuredRegion.start = routeStart;
        } else {
          configuredRegion.lap_points = lapRouteFor(activeBounds).map(
            point => [point.lat, point.lng]);
          configuredRegion.lap_direction = lapDirection;
        }
        setStatus(`Saved the ${mode} region to ${configuredRegion.label}.`);
      } catch (error) {
        setStatus(`Could not save region: ${error.message}`);
      } finally {
        useRegionButton.disabled = false;
      }
    });
    document.getElementById('reset').addEventListener('click', clearSelection);
    document.getElementById('location-search').addEventListener(
      'submit', searchLocation);
    locateButton.addEventListener('click', () => locate(true));
    const hasConfiguredRegion = configuredRegion.boundary.length >= 3 ||
      configuredRegion.lap_points.length >= 3;
    configuredButton.disabled = !hasConfiguredRegion;
    configuredButton.textContent = hasConfiguredRegion ?
      `Show ${configuredRegion.label} scan` : 'No configured region found';
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
    mission = {}
    vision = {}
    camera = {}
    config_path = None
    write_token = ''
    write_lock = threading.Lock()

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
        if self.path == '/plan':
            self._serve_plan()
            return
        if self.path == '/use-region':
            self._use_region()
            return
        if self.path != '/shutdown':
            self.send_error(404)
            return
        self.send_response(204)
        self.end_headers()
        threading.Thread(target=self.server.shutdown, daemon=True).start()

    def _serve_plan(self):
        """Generate the preview with the same planner used by mission_node."""
        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            if content_length <= 0 or content_length > 16_384:
                raise ValueError('invalid request size')
            request = json.loads(self.rfile.read(content_length))
            mission = dict(self.mission)
            mission.pop('scan_points', None)
            mission['scan_boundary'] = request['boundary']
            mission['scan_sweep'] = request['sweep']
            mission['scan_start'] = request['start']
            plan = build_scan_plan(mission, self.vision, self.camera)
            result = {
                'waypoints': plan.waypoints,
                'row_count': plan.row_count,
                'row_spacing_m': plan.row_spacing_m,
                'capture_spacing_m': plan.capture_spacing_m,
            }
            self._send_json(200, result)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {'error': str(error)})

    def _use_region(self):
        """Persist a browser-selected scan or lap region to the active config."""
        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            if content_length <= 0 or content_length > 16_384:
                raise ValueError('invalid request size')
            request = json.loads(self.rfile.read(content_length))
            if not secrets.compare_digest(
                    str(request.get('token', '')), self.write_token):
                self._send_json(403, {'error': 'invalid write token'})
                return
            kind = request['kind']
            options = {}
            if kind == 'scan':
                options = {
                    'sweep': request['sweep'],
                    'start': request['start'],
                }
            elif kind == 'lap':
                options = {'lap_direction': request['lap_direction']}
            with self.write_lock:
                save_region(
                    self.config_path, kind, request['boundary'], **options)
            self._send_json(200, {
                'saved': True,
                'kind': kind,
                'path': str(self.config_path),
            })
        except (KeyError, OSError, TypeError, ValueError,
                json.JSONDecodeError) as error:
            self._send_json(400, {'error': str(error)})

    def _send_json(self, status, value):
        body = json.dumps(value, separators=(',', ':')).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

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
    coordinates = {'scan_boundary': [], 'scan_points': [], 'points': []}
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

    lap_points = coordinates['points']
    is_closed = len(lap_points) > 1 and lap_points[0] == lap_points[-1]
    polygon = lap_points[:-1] if is_closed else lap_points
    signed_area = sum(
        polygon[index][1] * polygon[(index + 1) % len(polygon)][0]
        - polygon[(index + 1) % len(polygon)][1] * polygon[index][0]
        for index in range(len(polygon))
    ) if len(polygon) >= 3 else 0.0
    return {
        'label': config_path.name,
        'boundary': coordinates['scan_boundary'],
        'scan_points': coordinates['scan_points'],
        'lap_points': lap_points,
        'lap_direction': 'counterclockwise' if signed_area >= 0 else 'clockwise',
        'lap_count': 0,
        'sweep': 'long',
        'start': 'top',
    }


def load_planner_config(config_path):
    """Load the mission and camera inputs consumed by build_scan_plan."""
    config_dir = REPO_ROOT / 'config'
    try:
        with config_path.open(encoding='utf-8') as stream:
            mission = yaml.safe_load(stream) or {}
        with (config_dir / 'vision_params.yaml').open(encoding='utf-8') as stream:
            vision = yaml.safe_load(stream) or {}
        with (config_dir / 'filtering_params.yaml').open(encoding='utf-8') as stream:
            camera = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as error:
        raise SystemExit(f'Could not load scan planner configuration: {error}') from error
    return mission, vision, camera


def default_config_path():
    config_name = os.environ.get('BV_MISSION_CONFIG', 'real_params.yaml')
    if os.path.basename(config_name) != config_name or not config_name.endswith('.yaml'):
        raise SystemExit(f'Invalid BV_MISSION_CONFIG: {config_name!r}')
    return REPO_ROOT / 'config' / config_name


def main():
    args = parse_args()
    config_path = args.config.expanduser().resolve() if args.config else default_config_path()
    region = configured_region(config_path)
    mission, vision, camera = load_planner_config(config_path)
    region['sweep'] = mission.get('scan_sweep', 'long')
    region['start'] = mission.get('scan_start', 'top')
    region['lap_count'] = int(mission.get('lap_count', 1))
    write_token = secrets.token_urlsafe(32)
    page = PAGE.replace(
        '__CONFIGURED_REGION__',
        json.dumps(region, separators=(',', ':')).replace('</', '<\\/'),
    ).replace('__WRITE_TOKEN__', json.dumps(write_token))
    MapHandler.page = page
    MapHandler.mission = mission
    MapHandler.vision = vision
    MapHandler.camera = camera
    MapHandler.config_path = config_path
    MapHandler.write_token = write_token
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
