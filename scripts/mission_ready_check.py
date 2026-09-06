#!/usr/bin/env python3
"""Run read-only readiness checks for a real bv_core mission.

This script does not arm the aircraft, change flight mode, upload waypoints,
publish ROS messages, or call MAVROS command services. MAVROS must already be
running before this script is started.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


FLOAT_PATTERN = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
POINT_PATTERN = re.compile(
    rf'^\s*-\s*\[\s*({FLOAT_PATTERN})\s*,\s*({FLOAT_PATTERN})'
)
CONFIG_KEYS = {'points', 'scan_boundary', 'scan_points'}


@dataclass
class Result:
    name: str
    passed: bool
    detail: str


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run non-arming checks required before a real mission.'
    )
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--serial', default='/dev/ttyTHS1')
    parser.add_argument(
        '--config', type=Path,
        help=('mission YAML (default: config/$BV_MISSION_CONFIG, or '
              'config/real_params.yaml)'),
    )
    parser.add_argument(
        '--timeout', type=float, default=12.0,
        help='timeout in seconds for each camera or ROS check',
    )
    parser.add_argument(
        '--max-gps-distance-m', type=float, default=5000.0,
        help='maximum GPS distance from any configured mission point',
    )
    parser.add_argument(
        '--skip-camera', action='store_true',
        help='skip the camera capture/decode check',
    )
    return parser.parse_args()


def default_config_path():
    name = os.environ.get('BV_MISSION_CONFIG', 'real_params.yaml')
    if os.path.basename(name) != name or not name.endswith('.yaml'):
        raise SystemExit(f'Invalid BV_MISSION_CONFIG: {name!r}')
    return Path(__file__).resolve().parents[1] / 'config' / name


def configured_points(config_path):
    """Extract mission latitude/longitude pairs without requiring PyYAML."""
    active_key = None
    points = []
    try:
        lines = config_path.read_text(encoding='utf-8').splitlines()
    except OSError as error:
        raise SystemExit(
            f'Could not read mission config {config_path}: {error}'
        ) from error

    for raw_line in lines:
        line = raw_line.split('#', 1)[0].rstrip()
        if line and not line[0].isspace():
            key = line.partition(':')[0].strip()
            active_key = key if key in CONFIG_KEYS else None
            continue
        if active_key:
            match = POINT_PATTERN.match(line)
            if match:
                points.append((float(match.group(1)), float(match.group(2))))
    return points


def haversine_m(point_a, point_b):
    """Return great-circle distance in meters between two lat/lon pairs."""
    lat_a, lon_a = map(math.radians, point_a)
    lat_b, lon_b = map(math.radians, point_b)
    delta_lat = lat_b - lat_a
    delta_lon = lon_b - lon_a
    value = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat_a) * math.cos(lat_b)
        * math.sin(delta_lon / 2.0) ** 2
    )
    value = min(1.0, max(0.0, value))
    return (
        6371008.8 * 2.0
        * math.atan2(math.sqrt(value), math.sqrt(1.0 - value))
    )


def run_command(command, timeout):
    """Run one read-only diagnostic command and capture combined output."""
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode(errors='replace') \
            if isinstance(error.stdout, bytes) else (error.stdout or '')
        stderr = error.stderr.decode(errors='replace') \
            if isinstance(error.stderr, bytes) else (error.stderr or '')
        output = stdout + stderr
        return None, output.strip(), f'timed out after {timeout:g} seconds'
    except OSError as error:
        return None, '', str(error)
    return completed.returncode, completed.stdout.strip(), None


def check_device(name, path):
    device = Path(path)
    if not device.exists():
        return Result(name, False, f'{path} does not exist')
    if not os.access(device, os.R_OK | os.W_OK):
        return Result(
            name, False, f'{path} exists but is not readable/writable'
        )
    return Result(name, True, f'{path} exists and is accessible')


def camera_checks(camera, timeout):
    if not shutil.which('gst-launch-1.0'):
        return [Result(
            'Camera tools', False, 'gst-launch-1.0 was not found'
        )]

    source = [
        'gst-launch-1.0', '-q', 'v4l2src', f'device={camera}',
    ]
    caps = 'image/jpeg,width=3840,height=2160,framerate=30/1'
    decode = source + [
        'num-buffers=8', '!', caps, '!', 'jpegdec', '!', 'fakesink',
    ]
    code, output, error = run_command(decode, timeout)
    if code != 0:
        detail = error or output or f'gst-launch exited with status {code}'
        return [Result('Camera decode', False, detail)]

    return [
        Result(
            'Camera decode', True,
            'decoded eight 3840x2160 JPEG frames',
        ),
    ]


def ros_package_check(package, timeout):
    code, output, error = run_command(
        ['ros2', 'pkg', 'prefix', package], timeout
    )
    if code == 0 and output:
        return Result(
            f'ROS package {package}', True, output.splitlines()[-1]
        )
    return Result(
        f'ROS package {package}', False,
        error or output or 'package is unavailable in this ROS environment',
    )


def ros_topic(topic, timeout):
    return run_command(
        ['ros2', 'topic', 'echo', topic, '--once'], timeout
    )


def field(output, name, pattern):
    match = re.search(
        rf'^\s*{re.escape(name)}:\s*({pattern})\s*$', output, re.M
    )
    return match.group(1) if match else None


def mavros_state_check(timeout):
    code, output, error = ros_topic('/mavros/state', timeout)
    if code != 0:
        return Result(
            'MAVROS state', False,
            error or output or 'no message received from /mavros/state',
        )
    connected = field(output, 'connected', r'true|false')
    armed = field(output, 'armed', r'true|false')
    if connected != 'true':
        return Result(
            'MAVROS state', False,
            f'connected is {connected or "missing"}',
        )
    if armed != 'false':
        return Result(
            'MAVROS state', False,
            f'armed is {armed or "missing"}; expected a disarmed aircraft',
        )
    return Result(
        'MAVROS state', True, 'connected: true, armed: false'
    )


def gps_check(timeout, mission_points, maximum_distance_m):
    code, output, error = ros_topic(
        '/mavros/global_position/global', timeout
    )
    if code != 0:
        return Result(
            'GPS fix', False,
            error or output or 'no message received from GPS topic',
        )
    latitude = field(output, 'latitude', FLOAT_PATTERN)
    longitude = field(output, 'longitude', FLOAT_PATTERN)
    status = field(output, 'status', r'-?\d+')
    if latitude is None or longitude is None:
        return Result(
            'GPS fix', False, 'latitude or longitude is missing'
        )

    gps = (float(latitude), float(longitude))
    valid = (
        math.isfinite(gps[0]) and math.isfinite(gps[1])
        and -90.0 <= gps[0] <= 90.0
        and -180.0 <= gps[1] <= 180.0
    )
    if not valid:
        return Result('GPS fix', False, f'invalid coordinates: {gps}')
    if status is not None and int(status) < 0:
        return Result(
            'GPS fix', False, f'NavSatFix status is {status} (no fix)'
        )
    if not mission_points:
        return Result(
            'GPS fix', False, 'mission config contains no coordinates'
        )

    nearest = min(
        haversine_m(gps, point) for point in mission_points
    )
    detail = (
        f'{gps[0]:.8f}, {gps[1]:.8f}; '
        f'{nearest:.0f} m from nearest configured point'
    )
    return Result('GPS fix', nearest <= maximum_distance_m, detail)


def print_result(result):
    marker = 'PASS' if result.passed else 'FAIL'
    print(f'[{marker}] {result.name}: {result.detail}')


def main():
    args = parse_args()
    if args.timeout <= 0 or args.max_gps_distance_m <= 0:
        raise SystemExit(
            'Timeout and maximum GPS distance must be positive.'
        )

    config_path = (
        args.config.expanduser().resolve()
        if args.config else default_config_path()
    )
    mission_points = configured_points(config_path)
    print('MISSION READY CHECK (READ ONLY — THIS SCRIPT NEVER ARMS)')
    print(f'Mission config: {config_path}')
    print(f'Configured coordinate count: {len(mission_points)}\n')

    results = [
        check_device('PX4 serial port', args.serial),
        check_device('Camera device', args.camera),
    ]
    if args.skip_camera:
        print('[SKIP] Camera capture checks disabled by --skip-camera')
    elif results[-1].passed:
        results.extend(camera_checks(args.camera, args.timeout))

    if not shutil.which('ros2'):
        results.append(Result(
            'ROS 2 CLI', False, 'ros2 was not found in PATH'
        ))
    else:
        results.extend([
            ros_package_check('mavros', args.timeout),
            ros_package_check('bv_core', args.timeout),
            mavros_state_check(args.timeout),
            gps_check(
                args.timeout, mission_points,
                args.max_gps_distance_m,
            ),
        ])

    for result in results:
        print_result(result)

    print('\nMANUAL CONFIRMATIONS REQUIRED BEFORE LAUNCH:')
    print('[ ] Aircraft and propeller area is clear.')
    print('[ ] Herelink is connected; Loiter and RTL controls work.')
    print(
        '[ ] QGroundControl shows no battery, GPS, EKF, '
        'or preflight failures.'
    )
    print(
        f'[ ] Confirm coordinates in {config_path.name} '
        'with Scan_Picker.py.'
    )

    failed = [result for result in results if not result.passed]
    if failed:
        print(f'\nNOT READY: {len(failed)} automated check(s) failed.')
        return 1
    print(
        '\nAUTOMATED CHECKS PASSED. '
        'Complete every manual confirmation above.'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
