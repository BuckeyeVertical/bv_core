#!/usr/bin/env python3
"""Tests for MissionLogger — no ROS dependencies required."""

import os
import tempfile
import math
import pytest
from unittest.mock import patch


class TestMissionLoggerDisabled:
    """When enable_mission_log is false, all methods are no-ops."""

    def test_no_file_created_when_disabled(self, tmp_path):
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': False}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('test_node')
                log.event('STARTUP', 'num_objects=2')
                log_dir = tmp_path / 'logs'
                if log_dir.exists():
                    assert len(list(log_dir.iterdir())) == 0


class TestMissionLoggerEnabled:
    """When enable_mission_log is true, events are written to file."""

    def test_event_writes_line(self, tmp_path):
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('mission')
                log.event('STARTUP', 'num_objects=2')
                log_dir = tmp_path / 'logs'
                files = list(log_dir.iterdir())
                assert len(files) == 1
                content = files[0].read_text()
                assert '[mission] STARTUP | num_objects=2' in content

    def test_event_format_has_timestamp(self, tmp_path):
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('vision')
                log.event('PIPELINE_START', 'type=real')
                log_dir = tmp_path / 'logs'
                content = list(log_dir.iterdir())[0].read_text()
                import re
                assert re.search(r'\[\d{2}:\d{2}:\d{2}\.\d{3}\]', content)

    def test_frame_window_deduplication(self, tmp_path):
        """frame_window should not write if content unchanged."""
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('filtering')
                window_data = {0: {'person': {'dists': [0.3], 'thresh': 5.5}}}
                log.frame_window(1, 3, window_data)
                log.frame_window(1, 3, window_data)  # same content
                log_dir = tmp_path / 'logs'
                content = list(log_dir.iterdir())[0].read_text()
                assert content.count('FRAME_WINDOW') == 1

    def test_scan_heartbeat_throttle(self, tmp_path):
        """scan_heartbeat should only write every ~5s."""
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('vision')
                log.scan_heartbeat((38.387, -76.419), 2.4, 45.7, 5.0)
                log.scan_heartbeat((38.387, -76.419), 2.4, 45.7, 5.5)
                log_dir = tmp_path / 'logs'
                content = list(log_dir.iterdir())[0].read_text()
                assert content.count('SCAN_HEARTBEAT') == 1

    def test_deploy_complete_calculates_error(self, tmp_path):
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
                log = MissionLogger('mission')
                log.deploy_complete(0, 'person', (38.387, -76.419), (38.387, -76.419), 1, 2)
                log_dir = tmp_path / 'logs'
                content = list(log_dir.iterdir())[0].read_text()
                assert 'error=0.0m' in content

    def test_multiple_nodes_same_file(self, tmp_path):
        """Two logger instances with different node names write to same file."""
        with patch('bv_core.mission_logger._load_mission_config', return_value={'enable_mission_log': True}):
            with patch('bv_core.mission_logger.LOG_DIR', str(tmp_path / 'logs')):
                from bv_core.mission_logger import MissionLogger
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
