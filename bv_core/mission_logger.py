#!/usr/bin/env python3
"""
Centralized mission logger for bv_core drone nodes.

Writes human-readable, one-line-per-event logs to a shared file.
All nodes import this and create their own MissionLogger instance.
Controlled by enable_mission_log in mission_params.yaml.
"""

import os
import time
import math
import yaml
from datetime import datetime

# Default log directory — anchored at the repository's package directory
# so logs consistently land in bv_core/logs even when running from install/.
LOG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'bv_core', 'logs')
)


def _load_mission_config() -> dict:
    """Load mission_params.yaml from the installed share directory."""
    from ament_index_python.packages import get_package_share_directory
    config_path = os.path.join(
        get_package_share_directory('bv_core'),
        'config',
        'mission_params.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def _make_log_filename() -> str:
    """Generate log filename from current time: mission_MM-DD_H-MMpm.log"""
    now = datetime.now()
    hour_12 = now.strftime('%I').lstrip('0')  # 1-12, no leading zero
    minute = now.strftime('%M')
    ampm = now.strftime('%p').lower()
    month_day = now.strftime('%m-%d')
    return f"mission_{month_day}_{hour_12}-{minute}{ampm}.log"


def _deg_to_meters(lat1, lon1, lat2, lon2):
    """Approximate distance in meters between two lat/lon points."""
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2))
    return math.sqrt(dlat ** 2 + dlon ** 2)


class MissionLogger:
    """
    Human-readable mission logger. Each node creates one instance.

    Usage:
        self.log = MissionLogger('mission')
        self.log.event('STATE_CHANGE', 'takeoff -> lap')
    """

    def __init__(self, node_name: str):
        self._node_name = node_name
        self._file = None
        self._enabled = False
        self._last_heartbeat_time = 0.0
        self._last_frame_window_repr = None

        try:
            cfg = _load_mission_config()
        except Exception:
            return

        if not cfg.get('enable_mission_log', False):
            return

        self._enabled = True
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, _make_log_filename())
        self._file = open(log_path, 'a', buffering=1)

    def _timestamp(self) -> str:
        now = datetime.now()
        return now.strftime('[%H:%M:%S.') + f'{now.microsecond // 1000:03d}]'

    def _write(self, event_type: str, details: str):
        if not self._enabled or self._file is None:
            return
        line = f"{self._timestamp()} [{self._node_name}] {event_type} | {details}\n"
        self._file.write(line)

    def event(self, event_type: str, details: str):
        self._write(event_type, details)

    def scan_heartbeat(self, drone_pos: tuple, speed: float, alt: float, no_det_seconds: float):
        if not self._enabled:
            return
        now = time.monotonic()
        if now - self._last_heartbeat_time < 5.0:
            return
        self._last_heartbeat_time = now
        lat, lon = drone_pos
        self._write(
            'SCAN_HEARTBEAT',
            f"pos=({lat:.4f},{lon:.4f}), speed={speed:.1f}m/s, alt={alt:.1f}, "
            f"no_detections_for={no_det_seconds:.1f}s"
        )

    def detection_frame(self, frame_num: int, drone_pos: tuple, speed: float,
                        detections: list, filtered_reasons: dict = None):
        if not self._enabled or not detections:
            return
        if filtered_reasons is None:
            filtered_reasons = {}
        lat, lon = drone_pos
        parts = [f"frame={frame_num}, pos=({lat:.4f},{lon:.4f}), speed={speed:.1f}m/s, dets={len(detections)}"]
        for i, det in enumerate(detections):
            entry = (
                f"{det['class_name']}({det['confidence']:.2f}) "
                f"px=({det['px'][0]:.0f},{det['px'][1]:.0f}) -> "
                f"({det['latlon'][0]:.4f},{det['latlon'][1]:.4f})"
            )
            reason = filtered_reasons.get(i, '')
            if reason:
                entry += f" {reason}"
            parts.append(entry)
        self._write('DETECTION_FRAME', ' | '.join(parts))

    def frame_window(self, window_count: int, window_max: int, class_distances: dict):
        if not self._enabled:
            return
        current_repr = f"{window_count}|{class_distances}"
        if current_repr == self._last_frame_window_repr:
            return
        self._last_frame_window_repr = current_repr

        parts = [f"window=[{window_count}/{window_max}]"]
        for cls_name, info in class_distances.items():
            dists = info.get('dists', [])
            thresh = info.get('thresh', 0.0)
            if dists:
                dist_str = ','.join(f'{d:.1f}m' for d in dists)
                parts.append(f"{cls_name}: dists=[{dist_str}] thresh={thresh:.1f}m")
            else:
                parts.append(f"{cls_name}: [frame0]")
        self._write('FRAME_WINDOW', ' | '.join(parts))

    def deploy_complete(self, class_id: int, class_name: str,
                        target_pos: tuple, actual_pos: tuple,
                        delivered_count: int, total: int):
        if not self._enabled:
            return
        error_m = _deg_to_meters(target_pos[0], target_pos[1], actual_pos[0], actual_pos[1])
        self._write(
            'DEPLOY_COMPLETE',
            f"class={class_name}({class_id}), "
            f"target=({target_pos[0]:.6f},{target_pos[1]:.6f}), "
            f"actual=({actual_pos[0]:.6f},{actual_pos[1]:.6f}), "
            f"error={error_m:.1f}m, delivered={delivered_count}/{total}"
        )

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None
