#!/usr/bin/env python3
"""Integration tests for the preview encoder.

These exercise real GStreamer. They skip cleanly where it is unavailable so the
suite still passes on a machine without it.
"""

import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.preview_stream import PreviewConfig, PreviewStream  # noqa: E402

gst = pytest.importorskip('gi', reason='PyGObject not installed')


def _frame(w=1280, h=720):
    rng = np.random.default_rng(3)
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


class TestEncoder:
    def test_start_selects_a_tier_and_reports_running(self):
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            assert s.is_running() is True
            assert s.tier in ('jetson', 'nvidia', 'software')
        finally:
            s.stop()

    def test_offered_frames_produce_encoded_chunks(self):
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            deadline = time.time() + 5
            while not chunks and time.time() < deadline:
                time.sleep(0.05)
        finally:
            s.stop()
        assert chunks, 'encoder produced no output'
        assert all(isinstance(c, (bytes, bytearray)) for c in chunks)

    def test_first_chunk_is_an_mp4_init_segment(self):
        # fMP4 starts with an ftyp box; the browser needs it to initialise MSE.
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        s.start()
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            deadline = time.time() + 5
            while not chunks and time.time() < deadline:
                time.sleep(0.05)
        finally:
            s.stop()
        assert b'ftyp' in bytes(chunks[0])

    def test_offer_before_start_is_a_noop(self):
        s = PreviewStream(PreviewConfig(), lambda c: None)
        s.offer(_frame())            # must not raise
        assert s.is_running() is False

    def test_offer_after_stop_is_a_noop(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        s.stop()
        s.offer(_frame())            # must not raise
        assert s.is_running() is False

    def test_stop_is_idempotent(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        s.stop()
        s.stop()                     # must not raise

    def test_a_raising_callback_does_not_kill_the_encoder(self):
        def boom(_chunk):
            raise RuntimeError('downstream exploded')

        s = PreviewStream(PreviewConfig(width=640, fps=0.0), boom)
        assert s.start() is True
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            time.sleep(0.5)
            assert s.is_running() is True     # survived
        finally:
            s.stop()

    def test_stats_report_offered_and_encoded(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        try:
            for _ in range(6):
                s.offer(_frame())
                time.sleep(0.02)
            time.sleep(0.5)
            st = s.stats()
            assert st['offered'] == 6
            assert 'encoded' in st
            assert 'tier' in st
        finally:
            s.stop()
