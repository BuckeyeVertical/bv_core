"""Tests for the generic frame and metadata queue contract."""

import unittest

import numpy as np

from bv_core.pipelines.Vision_Pipeline import VisionPipeline


class StubPipeline(VisionPipeline):
    def start(self):
        pass

    def stop(self):
        pass


class TestVisionPipeline(unittest.TestCase):
    def test_legacy_get_frame_returns_only_the_image(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        pipeline = StubPipeline()
        pipeline._enqueue_frame(frame, {"sequence": 4})

        self.assertIs(pipeline.get_frame(), frame)

    def test_metadata_remains_attached_to_its_frame(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        pipeline = StubPipeline()
        pipeline._enqueue_frame(frame, {"sim_time_ns": 42})

        delivered, metadata = pipeline.get_frame_with_metadata()

        self.assertIs(delivered, frame)
        self.assertEqual(metadata, {"sim_time_ns": 42})

    def test_newest_packet_replaces_older_frame_and_metadata(self):
        pipeline = StubPipeline(max_queue_size=1)
        first = np.zeros((1, 1), dtype=np.uint8)
        second = np.ones((1, 1), dtype=np.uint8)
        pipeline._enqueue_frame(first, {"sequence": 1})
        pipeline._enqueue_frame(second, {"sequence": 2})

        frame, metadata = pipeline.get_frame_with_metadata()

        self.assertIs(frame, second)
        self.assertEqual(metadata["sequence"], 2)


if __name__ == "__main__":
    unittest.main()
