#!/usr/bin/env python3
"""The preview hook must be free when disabled and must never break capture."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.pipelines.Vision_Pipeline import VisionPipeline  # noqa: E402


class _Pipe(VisionPipeline):
    """Concrete stand-in; the base class is abstract."""

    def start(self):
        pass

    def stop(self):
        pass


class _RecordingPreview:
    def __init__(self, explode=False):
        self.seen = []
        self._explode = explode

    def offer(self, frame):
        if self._explode:
            raise RuntimeError('preview blew up')
        self.seen.append(frame)


class TestPreviewHook:
    def test_frames_still_reach_the_queue_with_no_preview(self):
        pipe = _Pipe(max_queue_size=2)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        pipe._enqueue_frame(frame)
        assert pipe.get_frame(timeout=0.1) is not None

    def test_preview_receives_offered_frames(self):
        pipe = _Pipe(max_queue_size=2)
        preview = _RecordingPreview()
        pipe.set_preview(preview)
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert len(preview.seen) == 1

    def test_detection_queue_still_fed_when_preview_attached(self):
        pipe = _Pipe(max_queue_size=2)
        pipe.set_preview(_RecordingPreview())
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert pipe.get_frame(timeout=0.1) is not None

    def test_a_raising_preview_does_not_break_capture(self):
        # This is the whole point: the debug feature must never cost a frame.
        pipe = _Pipe(max_queue_size=2)
        pipe.set_preview(_RecordingPreview(explode=True))
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert pipe.get_frame(timeout=0.1) is not None

    def test_set_preview_none_detaches(self):
        pipe = _Pipe(max_queue_size=2)
        preview = _RecordingPreview()
        pipe.set_preview(preview)
        pipe.set_preview(None)
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert preview.seen == []

    def test_default_pipeline_has_no_preview_attached(self):
        # The disabled path: nothing published on /preview_enabled means the
        # attribute stays None and _enqueue_frame short-circuits on one compare.
        pipe = _Pipe(max_queue_size=2)
        assert pipe._preview is None
