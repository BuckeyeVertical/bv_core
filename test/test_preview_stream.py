#!/usr/bin/env python3
"""Tests for preview_stream frame gating — pure numpy, no ROS or GStreamer."""

import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.preview_stream import (  # noqa: E402
    FrameGate,
    decimation_factor,
    downscale,
)


class TestDecimationFactor:
    def test_real_camera_to_target(self):
        # 4640 wide -> 1024 target. 4640 // 1024 == 4.
        assert decimation_factor(4640, 1024) == 4

    def test_sim_camera_to_target(self):
        assert decimation_factor(1280, 1024) == 1

    def test_source_narrower_than_target_never_decimates(self):
        assert decimation_factor(640, 1024) == 1

    def test_equal_widths(self):
        assert decimation_factor(1024, 1024) == 1

    def test_never_returns_zero(self):
        assert decimation_factor(1, 1024) == 1
        assert decimation_factor(0, 1024) == 1


class TestDownscale:
    def test_real_camera_frame_reaches_target_width(self):
        frame = np.zeros((3480, 4640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 1024

    def test_aspect_ratio_is_preserved(self):
        # 4640x3480 is 4:3, so 1024 wide -> 768 high.
        frame = np.zeros((3480, 4640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[0] == 768

    def test_bevy_camera_frame_matches_real_aspect_ratio(self):
        frame = np.zeros((960, 1280, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 1024
        assert out.shape[0] == 768

    def test_output_dimensions_are_even(self):
        # H.264 encoders reject odd dimensions.
        frame = np.zeros((1001, 1333, 3), dtype=np.uint8)
        out = downscale(frame, 641)
        assert out.shape[0] % 2 == 0
        assert out.shape[1] % 2 == 0

    def test_source_smaller_than_target_is_returned_unscaled(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 640

    def test_does_not_mutate_the_source(self):
        frame = np.full((720, 1280, 3), 7, dtype=np.uint8)
        before = frame.copy()
        downscale(frame, 1024)
        assert np.array_equal(frame, before)

    def test_content_survives(self):
        # A frame that is entirely one colour must stay that colour.
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        out = downscale(frame, 1024)
        assert abs(int(out.mean()) - 200) < 3


class TestFrameGate:
    def test_first_offer_is_accepted(self):
        gate = FrameGate(fps=8.0)
        assert gate.offer(np.zeros((4, 4, 3), dtype=np.uint8)) is True

    def test_second_offer_within_the_interval_is_rejected(self):
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        assert gate.offer(frame) is True
        assert gate.offer(frame) is False      # 1/8s has not elapsed

    def test_offer_after_the_interval_is_accepted(self):
        gate = FrameGate(fps=50.0)             # 20ms interval
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        assert gate.offer(frame) is True
        time.sleep(0.03)
        assert gate.offer(frame) is True

    def test_rate_limiting_thins_a_fast_source(self):
        # 24 offers as fast as possible at fps=8 should accept far fewer.
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        accepted = sum(gate.offer(frame) for _ in range(24))
        assert accepted == 1                   # all within one interval

    def test_take_returns_the_latest_and_empties_the_slot(self):
        gate = FrameGate(fps=0.0)              # 0 disables rate limiting
        a = np.full((4, 4, 3), 1, dtype=np.uint8)
        b = np.full((4, 4, 3), 2, dtype=np.uint8)
        gate.offer(a)
        gate.offer(b)
        got = gate.take()
        assert got is not None and got[0, 0, 0] == 2   # latest wins
        assert gate.take() is None                     # slot now empty

    def test_take_on_empty_gate_returns_none(self):
        assert FrameGate(fps=8.0).take() is None

    def test_stats_count_accepted_and_dropped(self):
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        for _ in range(5):
            gate.offer(frame)
        s = gate.stats
        assert s['offered'] == 5
        assert s['accepted'] == 1
        assert s['dropped'] == 4

    def test_overwriting_an_untaken_frame_counts_as_a_drop(self):
        gate = FrameGate(fps=0.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        gate.offer(frame)
        gate.offer(frame)          # overwrites the first, never encoded
        assert gate.stats['dropped'] == 1

    def test_offer_never_raises_on_garbage(self):
        gate = FrameGate(fps=0.0)
        assert gate.offer(None) is False


class _Hostile:
    """An object that raises from every dunder offer() might trip over."""

    def __bool__(self):
        raise RuntimeError('__bool__')

    def __eq__(self, other):
        raise RuntimeError('__eq__')

    def __hash__(self):
        raise RuntimeError('__hash__')

    def __len__(self):
        raise RuntimeError('__len__')

    def __repr__(self):
        raise RuntimeError('__repr__')


class TestOfferIsUnraisable:
    """offer() runs on the thread that feeds detection; it must never throw."""

    def test_offer_survives_every_garbage_type(self):
        gate = FrameGate(fps=0.0)
        for junk in (None, 0, '', b'', [], {}, object(), _Hostile(),
                     np.array([]), np.zeros((0, 0, 3), dtype=np.uint8)):
            assert gate.offer(junk) in (True, False)

    def test_hostile_frame_can_still_be_taken_and_cleared(self):
        gate = FrameGate(fps=0.0)
        assert gate.offer(_Hostile()) is True
        assert isinstance(gate.take(), _Hostile)
        assert gate.take() is None

    def test_garbage_fps_does_not_break_construction_or_offer(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        for fps in (0, 0.0, None, -1.0, False):
            gate = FrameGate(fps=fps)
            assert gate.offer(frame) is True
            assert gate.offer(frame) is True    # no limiting when fps is falsy

    def test_stats_never_raises_after_hostile_input(self):
        gate = FrameGate(fps=0.0)
        gate.offer(_Hostile())
        gate.offer(None)
        s = gate.stats
        assert s['offered'] == 2
        assert s['accepted'] == 1
        assert s['dropped'] == 1

    def test_offer_is_not_reentrant_deadlocked_across_threads(self):
        # A slow taker must never make the capture thread wait measurably.
        gate = FrameGate(fps=0.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        stop = threading.Event()

        def taker():
            while not stop.is_set():
                gate.take()

        t = threading.Thread(target=taker, daemon=True)
        t.start()
        try:
            start = time.monotonic()
            for _ in range(2000):
                gate.offer(frame)
            elapsed = time.monotonic() - start
        finally:
            stop.set()
            t.join(timeout=2.0)
        assert elapsed < 1.0
