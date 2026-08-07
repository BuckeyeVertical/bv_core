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

    def test_start_alone_emits_nothing(self):
        # Probing proves a tier can run; it must not leak a chunk. A chunk here
        # would be a second, conflicting fMP4 init segment at the consumer.
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            time.sleep(0.5)
            assert chunks == []
            assert s.stats()['encoded'] == 0
        finally:
            s.stop()

    def test_offered_frames_produce_encoded_chunks(self):
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            assert chunks == [], 'output before any frame was offered'
            # Keep offering inside the wait: mp4mux will not close a fragment
            # without further buffers, so a fixed burst then a bare sleep flakes
            # on a slow machine.
            deadline = time.time() + 10
            while not chunks and time.time() < deadline:
                s.offer(_frame())
                time.sleep(0.02)
        finally:
            s.stop()
        assert chunks, 'encoder produced no output'
        assert all(isinstance(c, (bytes, bytearray)) for c in chunks)

    def test_first_chunk_is_an_mp4_init_segment(self):
        # fMP4 starts with an ftyp box; the browser needs it to initialise MSE.
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            deadline = time.time() + 10
            while not chunks and time.time() < deadline:
                s.offer(_frame())
                time.sleep(0.02)
        finally:
            s.stop()
        assert chunks, 'encoder produced no output'
        assert b'ftyp' in bytes(chunks[0])

    def test_only_one_init_segment_is_emitted(self):
        # The consumer's MSE SourceBuffer initialises once. A rebuild on the
        # first real frame would hand it a second ftyp/moov and stall playback.
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            deadline = time.time() + 10
            while len(chunks) < 3 and time.time() < deadline:
                s.offer(_frame())
                time.sleep(0.02)
        finally:
            s.stop()
        assert chunks, 'encoder produced no output'
        inits = [c for c in chunks if b'ftyp' in bytes(c)]
        assert len(inits) == 1, f'{len(inits)} init segments in {len(chunks)} chunks'

    def test_encoded_count_only_grows_once_frames_are_offered(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        assert s.start() is True
        try:
            assert s.stats()['encoded'] == 0
            deadline = time.time() + 10
            while s.stats()['encoded'] == 0 and time.time() < deadline:
                s.offer(_frame())
                time.sleep(0.02)
            assert s.stats()['encoded'] > 0
        finally:
            s.stop()

    def test_stop_during_a_build_leaves_no_pipeline_behind(self):
        # The build runs on the encoder thread; stop() must not race it into
        # stranding a PLAYING pipeline that holds the hardware encoder.
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        assert s.start() is True
        s.offer(_frame())
        s.stop()
        assert s.is_running() is False
        assert s._pipeline is None
        assert s._appsrc is None

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
