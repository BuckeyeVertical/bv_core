#!/usr/bin/env python3
"""Tests for SimApprovalCheck.verdict() — no rclpy spin, no sim, no MAVROS.

verdict() and the message callbacks only touch plain instance attributes, so the
real class is exercised via object.__new__ rather than a stand-in object: the
code under test is exactly what ships. Only get_logger is stubbed, since that is
the one Node facility the callbacks reach for.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.sim_approval_check import SimApprovalCheck  # noqa: E402


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    def warn(self, msg):
        self.lines.append(msg)


def _make(states=None, pendings=None, rejections=None, resolutions=0):
    """A SimApprovalCheck with its observation state set directly."""
    node = object.__new__(SimApprovalCheck)
    node.states = list(states or [])
    node.pendings = list(pendings or [])
    node.rejections = list(rejections or [])
    node.resolutions = resolutions
    node.state_at_pending = None
    node._logger = _StubLogger()
    node.get_logger = lambda: node._logger
    return node


def _pending(state='localize', crop_bytes=4096, timeout=180.0, class_id=0):
    return {
        'id': 'det-1',
        'class_id': class_id,
        'crop_bytes': crop_bytes,
        'timeout': timeout,
        'state': state,
    }


# A nominal approve run: scan -> localize -> deliver -> deploy -> scan.
_APPROVE_STATES = ['takeoff', 'lap', 'scan', 'localize', 'deliver', 'deploy',
                   'scan']
# A nominal reject run: never reaches deliver/deploy, returns to scan.
_REJECT_STATES = ['takeoff', 'lap', 'scan', 'localize', 'scan']


def _lines(capsys):
    return capsys.readouterr().out


def test_approve_only_run_passes(capsys):
    node = _make(states=_APPROVE_STATES, pendings=[_pending()], resolutions=1)
    assert node.verdict() is True
    assert 'RESULT: PASS' in _lines(capsys)


def test_reject_only_run_passes(capsys):
    node = _make(
        states=_REJECT_STATES,
        pendings=[_pending()],
        rejections=[(40.0, -83.0, 0)],
        resolutions=1,
    )
    assert node.verdict() is True
    out = _lines(capsys)
    assert 'RESULT: PASS' in out
    assert 'returned to scan' in out


def test_mixed_run_where_approved_never_deployed_fails(capsys):
    """Regression for the reject-branch shadowing the approve checks.

    Operator rejects the first detection, approves the second, and the approved
    one never reaches DEPLOY. The rejection must not excuse the approve path.
    """
    node = _make(
        # Two localizes, two returns to scan, but no deliver/deploy ever.
        states=['takeoff', 'lap', 'scan', 'localize', 'scan', 'localize',
                'scan'],
        pendings=[_pending(), _pending()],
        rejections=[(40.0, -83.0, 0)],
        resolutions=2,
    )
    assert node.verdict() is False
    out = _lines(capsys)
    assert 'RESULT: FAIL' in out
    assert '1 approved, 1 rejected' in out
    # Both sets of checks are present in a mixed run.
    assert 'reached DEPLOY' in out
    assert 'returned to scan' in out


def test_mixed_run_that_did_deploy_passes(capsys):
    node = _make(
        states=['takeoff', 'lap', 'scan', 'localize', 'scan', 'localize',
                'deliver', 'deploy', 'scan'],
        pendings=[_pending(), _pending()],
        rejections=[(40.0, -83.0, 0)],
        resolutions=2,
    )
    assert node.verdict() is True
    assert 'RESULT: PASS' in _lines(capsys)


def test_no_pendings_fails(capsys):
    node = _make(states=['takeoff', 'lap', 'scan'])
    assert node.verdict() is False
    out = _lines(capsys)
    assert 'RESULT: FAIL' in out
    assert '0 pending(s) published' in out


def test_empty_crop_fails(capsys):
    node = _make(
        states=_APPROVE_STATES,
        pendings=[_pending(crop_bytes=0)],
        resolutions=1,
    )
    assert node.verdict() is False
    out = _lines(capsys)
    assert 'RESULT: FAIL' in out
    assert 'FAIL  pending carried a crop (0 bytes)' in out


def test_pending_published_outside_localize_fails(capsys):
    node = _make(
        states=['takeoff', 'lap', 'scan'],
        pendings=[_pending(state='scan')],
        resolutions=1,
    )
    assert node.verdict() is False
    assert 'saw scan' in _lines(capsys)


def test_zero_timeout_fails(capsys):
    node = _make(
        states=_APPROVE_STATES,
        pendings=[_pending(timeout=0.0)],
        resolutions=1,
    )
    assert node.verdict() is False
    assert 'timeout advertised' in _lines(capsys)


def test_no_check_is_a_tautology(capsys):
    """Every emitted line must be capable of failing.

    A run in which nothing went right must not produce a single PASS line.
    """
    node = _make(
        states=['takeoff'],
        pendings=[_pending(state='takeoff', crop_bytes=0, timeout=0.0)],
        rejections=[(40.0, -83.0, 0), (41.0, -83.0, 0)],
        resolutions=1,
    )
    assert node.verdict() is False
    out = _lines(capsys)
    check_lines = [ln for ln in out.splitlines()
                   if ln.startswith('  PASS') or ln.startswith('  FAIL')]
    # Only the "1 pending(s) published" check can legitimately pass here.
    passing = [ln for ln in check_lines if ln.startswith('  PASS')]
    assert passing == ['  PASS  1 pending(s) published'], passing


# -- callback bookkeeping ------------------------------------------------------

class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_clear_message_counts_as_a_resolution():
    node = _make()
    node._on_pending(_Msg(
        detection_id='det-1', class_id=0, timeout_sec=180.0,
        annotated_crop=_Msg(data=b'x' * 10)))
    assert node.resolutions == 0
    node._on_pending(_Msg(detection_id=''))
    assert node.resolutions == 1


def test_clear_before_any_pending_is_not_a_resolution():
    """The latched publisher replays a trailing empty message to late joiners."""
    node = _make()
    node._on_pending(_Msg(detection_id=''))
    assert node.resolutions == 0


def test_state_history_dedupes_repeats():
    node = _make()
    for data in ['scan', 'scan', 'localize', 'localize', 'scan']:
        node._on_state(_Msg(data=data))
    assert node.states == ['scan', 'localize', 'scan']


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
