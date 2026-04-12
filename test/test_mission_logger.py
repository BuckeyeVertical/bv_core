#!/usr/bin/env python3
"""Tests for MissionLogger — no ROS dependencies required."""

import os
import sys
import re
import pytest
from unittest.mock import patch

# Add the package root to sys.path so bv_core is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import after path setup, but patch _load_mission_config before any MissionLogger
# is instantiated (it's called in __init__, not at module level)
import bv_core.mission_logger as ml_module
from bv_core.mission_logger import MissionLogger, _deg_to_meters


class TestMissionLoggerDisabled:
    """When enable_mission_log is false, all methods are no-ops."""

    def test_no_file_created_when_disabled(self, tmp_path):
        with patch.object(ml_module, '_load_mission_config', return_value={'enable_mission_log': False}):
            with patch.object(ml_module, 'LOG_DIR', str(tmp_path / 'logs')):
                log = MissionLogger('test_node')
                log.event('STARTUP', 'num_objects=2')
                log_dir = tmp_path / 'logs'
                if log_dir.exists():
                    assert len(list(log_dir.iterdir())) == 0


class TestMissionLoggerEnabled:
    """When enable_mission_log is true, events are written to file."""

    def _make_logger(self, tmp_path, node_name='mission'):
        with patch.object(ml_module, '_load_mission_config', return_value={'enable_mission_log': True}):
            with patch.object(ml_module, 'LOG_DIR', str(tmp_path / 'logs')):
                return MissionLogger(node_name)

    def test_event_writes_line(self, tmp_path):
        log = self._make_logger(tmp_path)
        log.event('STARTUP', 'num_objects=2')
        log_dir = tmp_path / 'logs'
        files = list(log_dir.iterdir())
        assert len(files) == 1
        content = files[0].read_text()
        assert '[mission] STARTUP | num_objects=2' in content

    def test_event_format_has_timestamp(self, tmp_path):
        log = self._make_logger(tmp_path, 'vision')
        log.event('PIPELINE_START', 'type=real')
        log_dir = tmp_path / 'logs'
        content = list(log_dir.iterdir())[0].read_text()
        assert re.search(r'\[\d{2}:\d{2}:\d{2}\.\d{3}\]', content)

    def test_frame_window_deduplication(self, tmp_path):
        """frame_window should not write if content unchanged."""
        log = self._make_logger(tmp_path, 'filtering')
        window_data = {'person': {'dists': [0.3], 'thresh': 5.5}}
        log.frame_window(1, 3, window_data)
        log.frame_window(1, 3, window_data)  # same content
        log_dir = tmp_path / 'logs'
        content = list(log_dir.iterdir())[0].read_text()
        assert content.count('FRAME_WINDOW') == 1

    def test_scan_heartbeat_throttle(self, tmp_path):
        """scan_heartbeat should only write every ~5s."""
        log = self._make_logger(tmp_path, 'vision')
        log.scan_heartbeat((38.387, -76.419), 2.4, 45.7, 5.0)
        log.scan_heartbeat((38.387, -76.419), 2.4, 45.7, 5.5)
        log_dir = tmp_path / 'logs'
        content = list(log_dir.iterdir())[0].read_text()
        assert content.count('SCAN_HEARTBEAT') == 1

    def test_deploy_complete_calculates_error(self, tmp_path):
        log = self._make_logger(tmp_path)
        log.deploy_complete(0, 'person', (38.387, -76.419), (38.387, -76.419), 1, 2)
        log_dir = tmp_path / 'logs'
        content = list(log_dir.iterdir())[0].read_text()
        assert 'error=0.0m' in content

    def test_multiple_nodes_same_file(self, tmp_path):
        """Two logger instances with different node names write to same file."""
        with patch.object(ml_module, '_load_mission_config', return_value={'enable_mission_log': True}):
            with patch.object(ml_module, 'LOG_DIR', str(tmp_path / 'logs')):
                log1 = MissionLogger('mission')
                log2 = MissionLogger('vision')
        log1.event('STATE_CHANGE', 'takeoff -> lap')
        log2.event('PIPELINE_START', 'type=real')
        log_dir = tmp_path / 'logs'
        files = list(log_dir.iterdir())
        assert len(files) == 1
        content = files[0].read_text()
        assert '[mission] STATE_CHANGE' in content
        assert '[vision] PIPELINE_START' in content


class TestDegToMeters:
    def test_same_point_is_zero(self):
        assert _deg_to_meters(38.0, -76.0, 38.0, -76.0) == 0.0

    def test_known_distance(self):
        # ~111m per 0.001 degree latitude
        dist = _deg_to_meters(38.0, -76.0, 38.001, -76.0)
        assert 110 < dist < 112
