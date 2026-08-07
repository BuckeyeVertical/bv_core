#!/usr/bin/env python3
"""Tests for the M-of-N detection confirmation window in filtering_node.

Exercises the pure predicate directly — no ROS spin, no simulator.

The predicate decides what the aircraft flies to and drops a payload on, so the
cases below are deliberately blunt about the behaviour change: 3-of-3 required a
consecutive run and lost a real object to a single missed frame; 3-of-5 tolerates
the gap while still requiring three sightings that agree spatially.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.filtering_node import (  # noqa: E402
    evaluate_window,
    window_presence,
)

# Well inside the 0.0001 deg proximity threshold used throughout.
NEAR = 0.00001
THRESH = 0.0001

PERSON, TENT = 0, 1


def det(cls, offset=0.0):
    """One detection of `cls`, `offset` degrees north of the reference point."""
    return (38.3877 + offset, -76.4190, cls)


def frames(*specs):
    """Build a window: each spec is a list of (cls, offset) or () for an empty frame."""
    out = []
    for spec in specs:
        out.append([det(c, o) for c, o in spec])
    return out


def confirm(history, hits=3, thresh=THRESH):
    """First confirmed (class, positions), or None — mirrors the node's use."""
    for cls, positions, _dists, ok in evaluate_window(history, hits, thresh):
        if ok:
            return cls, positions
    return None


class TestConfirmation:
    def test_three_consecutive_hits_confirm(self):
        h = frames([(PERSON, 0)], [(PERSON, NEAR)], [(PERSON, 2 * NEAR)])
        assert confirm(h) is not None

    def test_two_hits_do_not_confirm(self):
        h = frames([(PERSON, 0)], [(PERSON, NEAR)], [])
        assert confirm(h) is None

    def test_gap_still_confirms_this_is_the_change(self):
        """The whole point: a missed middle frame no longer loses the object.

        Under the old all-of-3 rule this window confirmed nothing.
        """
        h = frames([(PERSON, 0)], [], [(PERSON, NEAR)], [(PERSON, 2 * NEAR)])
        assert confirm(h) is not None

    def test_three_hits_spread_across_five_frames(self):
        h = frames([(PERSON, 0)], [], [(PERSON, NEAR)], [], [(PERSON, 2 * NEAR)])
        assert confirm(h) is not None

    def test_hits_that_disagree_spatially_are_rejected(self):
        """Three sightings, but scattered — the proximity gate still applies."""
        h = frames([(PERSON, 0)], [(PERSON, 0.01)], [(PERSON, 0.02)])
        assert confirm(h) is None

    def test_empty_window_confirms_nothing(self):
        assert confirm(frames([], [], [], [], [])) is None

    def test_wrong_class_does_not_contribute_hits(self):
        h = frames([(PERSON, 0)], [(TENT, NEAR)], [(PERSON, 2 * NEAR)])
        assert confirm(h) is None

    def test_two_classes_are_tracked_independently(self):
        h = frames(
            [(PERSON, 0), (TENT, 0)],
            [(TENT, NEAR)],
            [(PERSON, NEAR), (TENT, 2 * NEAR)],
            [(TENT, 2 * NEAR)],
        )
        got = confirm(h)
        assert got is not None and got[0] == TENT

    def test_required_hits_is_honoured(self):
        h = frames([(PERSON, 0)], [(PERSON, NEAR)], [])
        assert confirm(h, hits=2) is not None
        assert confirm(h, hits=3) is None

    def test_positions_returned_are_only_the_frames_that_hit(self):
        h = frames([(PERSON, 0)], [], [(PERSON, NEAR)], [(PERSON, 2 * NEAR)])
        cls, positions = confirm(h)
        assert cls == PERSON
        assert len(positions) == 3, 'empty frame must not contribute a position'


class TestWindowPresence:
    """Feeds the GCS status panel."""

    def test_reports_per_frame_presence_in_order(self):
        h = frames([(PERSON, 0)], [], [(PERSON, NEAR)])
        p = window_presence(h)
        assert p[PERSON] == [True, False, True]

    def test_absent_class_is_not_reported(self):
        assert TENT not in window_presence(frames([(PERSON, 0)], [], []))

    def test_empty_history_is_empty(self):
        assert window_presence([]) == {}
