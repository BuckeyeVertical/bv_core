"""Tests for the Bevy camera stream pipeline."""

import io
import json
import socket
import threading
import unittest

import cv2
import numpy as np

from bv_core.pipelines import create_pipeline
from bv_core.pipelines.bevy_pipeline import BevyPipeline


def make_payload(width=4, height=3, sequence=4, sim_time_ns=42):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :, 1] = 180
    success, jpeg = cv2.imencode('.jpg', image)
    assert success

    header = {
        "schema": "bv.camera_frame",
        "version": 1,
        "sequence": sequence,
        "sim_stream_id": "sim-test-stream",
        "sim_sequence": 12,
        "sim_time_ns": sim_time_ns,
        "camera_id": "camera/drone",
        "camera_model": "pinhole",
        "fx": 2.0,
        "fy": 2.0,
        "cx": width / 2,
        "cy": height / 2,
        "width": width,
        "height": height,
        "encoding": "jpeg",
        "data_length": len(jpeg),
    }
    return json.dumps(header).encode() + b'\n' + jpeg.tobytes()


class TestBevyPipeline(unittest.TestCase):
    def test_decodes_frame_and_preserves_simulation_metadata(self):
        frame, metadata = BevyPipeline._read_frame(io.BytesIO(make_payload()))

        self.assertEqual(frame.shape, (3, 4, 3))
        self.assertEqual(metadata["sim_stream_id"], "sim-test-stream")
        self.assertEqual(metadata["sim_sequence"], 12)
        self.assertEqual(metadata["sim_time_ns"], 42)
        self.assertEqual(metadata["camera_model"], "pinhole")
        self.assertEqual(metadata["fx"], 2.0)

    def test_rejects_mismatched_dimensions(self):
        payload = make_payload(width=4, height=3)
        header, jpeg = payload.split(b'\n', 1)
        metadata = json.loads(header)
        metadata["width"] = 5

        with self.assertRaisesRegex(ValueError, "dimensions"):
            BevyPipeline._read_frame(
                io.BytesIO(json.dumps(metadata).encode() + b'\n' + jpeg)
            )

    def test_rejects_blank_simulation_stream_identity(self):
        payload = make_payload()
        header, jpeg = payload.split(b'\n', 1)
        metadata = json.loads(header)
        metadata["sim_stream_id"] = "  "

        with self.assertRaisesRegex(ValueError, "stream ID"):
            BevyPipeline._read_frame(
                io.BytesIO(json.dumps(metadata).encode() + b'\n' + jpeg)
            )

    def test_accepts_legacy_header_without_simulation_stream_identity(self):
        payload = make_payload()
        header, jpeg = payload.split(b'\n', 1)
        metadata = json.loads(header)
        del metadata["sim_stream_id"]

        frame, decoded_metadata = BevyPipeline._read_frame(
            io.BytesIO(json.dumps(metadata).encode() + b'\n' + jpeg)
        )

        self.assertEqual(frame.shape, (3, 4, 3))
        self.assertNotIn("sim_stream_id", decoded_metadata)

    def test_accepts_legacy_header_without_intrinsics(self):
        payload = make_payload()
        header, jpeg = payload.split(b'\n', 1)
        metadata = json.loads(header)
        for field in ("camera_model", "fx", "fy", "cx", "cy"):
            del metadata[field]

        frame, decoded_metadata = BevyPipeline._read_frame(
            io.BytesIO(json.dumps(metadata).encode() + b'\n' + jpeg)
        )

        self.assertEqual(frame.shape, (3, 4, 3))
        self.assertNotIn("camera_model", decoded_metadata)

    def test_rejects_incomplete_or_invalid_intrinsics(self):
        payload = make_payload()
        header, jpeg = payload.split(b'\n', 1)
        incomplete = json.loads(header)
        del incomplete["cy"]
        invalid = json.loads(header)
        invalid["fx"] = -1.0

        with self.assertRaisesRegex(ValueError, "Incomplete"):
            BevyPipeline._read_frame(
                io.BytesIO(json.dumps(incomplete).encode() + b'\n' + jpeg)
            )
        with self.assertRaisesRegex(ValueError, "focal lengths"):
            BevyPipeline._read_frame(
                io.BytesIO(json.dumps(invalid).encode() + b'\n' + jpeg)
            )

    def test_rejects_boolean_numeric_metadata(self):
        payload = make_payload()
        header, jpeg = payload.split(b'\n', 1)
        metadata = json.loads(header)
        metadata["sequence"] = True

        with self.assertRaisesRegex(ValueError, "sequence"):
            BevyPipeline._read_frame(
                io.BytesIO(json.dumps(metadata).encode() + b'\n' + jpeg)
            )

    def test_factory_builds_bevy_pipeline(self):
        pipeline, topic = create_pipeline(
            "bevy",
            bevy_host="camera-host",
            bevy_port=7102,
            queue_size=1,
        )

        self.assertIsInstance(pipeline, BevyPipeline)
        self.assertEqual(topic, "tcp://camera-host:7102")

    def test_delivers_frame_and_matching_metadata_together(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        metadata = {"sim_time_ns": 75}
        pipeline = BevyPipeline()
        pipeline._running = True
        pipeline._enqueue_frame(frame, metadata)

        delivered, delivered_metadata = pipeline.get_frame_with_metadata()

        self.assertIs(delivered, frame)
        self.assertEqual(delivered_metadata["sim_time_ns"], 75)
        self.assertEqual(pipeline.latest_metadata["sim_time_ns"], 75)

    def test_reconnects_after_producer_restart(self):
        listener = socket.create_server(("127.0.0.1", 0))
        port = listener.getsockname()[1]
        send_second = threading.Event()

        def serve():
            with listener:
                first, _ = listener.accept()
                with first:
                    first.sendall(make_payload(sequence=1, sim_time_ns=10))
                send_second.wait(timeout=3)
                second, _ = listener.accept()
                with second:
                    second.sendall(make_payload(sequence=2, sim_time_ns=20))

        server = threading.Thread(target=serve, daemon=True)
        server.start()
        pipeline = BevyPipeline(port=port)
        pipeline.start()

        try:
            _, first_metadata = pipeline.get_frame_with_metadata(timeout=3)
            send_second.set()
            _, second_metadata = pipeline.get_frame_with_metadata(timeout=3)
        finally:
            pipeline.stop()
            server.join(timeout=3)

        self.assertEqual(first_metadata["sim_time_ns"], 10)
        self.assertEqual(second_metadata["sim_time_ns"], 20)
        self.assertFalse(server.is_alive())


if __name__ == "__main__":
    unittest.main()
