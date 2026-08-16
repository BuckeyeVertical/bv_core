"""Tests for framework-neutral simulation frame metadata."""

import unittest
from types import SimpleNamespace

from bv_core.frame_metadata import (
    SimulationFrameMetadata,
    apply_simulation_metadata,
)


class TestSimulationFrameMetadata(unittest.TestCase):
    def test_extracts_and_applies_exact_simulation_identity(self):
        metadata = SimulationFrameMetadata.from_mapping(
            {
                "sim_stream_id": "gazebo-run",
                "sim_sequence": 17,
                "sim_time_ns": 123_456_789,
            }
        )
        message = SimpleNamespace(
            sim_stream_id="",
            sim_sequence=0,
            sim_time_ns=0,
        )

        apply_simulation_metadata(message, metadata)

        self.assertEqual(message.sim_stream_id, "gazebo-run")
        self.assertEqual(message.sim_sequence, 17)
        self.assertEqual(message.sim_time_ns, 123_456_789)

    def test_absent_simulation_metadata_is_supported(self):
        self.assertIsNone(SimulationFrameMetadata.from_mapping({}))

    def test_legacy_metadata_preserves_sequence_and_time(self):
        metadata = SimulationFrameMetadata.from_mapping(
            {"sim_sequence": 17, "sim_time_ns": 123_456_789}
        )

        self.assertEqual(metadata.stream_id, "")
        self.assertEqual(metadata.sequence, 17)
        self.assertEqual(metadata.time_ns, 123_456_789)

    def test_rejects_partial_or_boolean_metadata(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            SimulationFrameMetadata.from_mapping({"sim_time_ns": 10})
        with self.assertRaisesRegex(ValueError, "sequence"):
            SimulationFrameMetadata.from_mapping(
                {
                    "sim_stream_id": "gazebo-run",
                    "sim_sequence": True,
                    "sim_time_ns": 10,
                }
            )


if __name__ == "__main__":
    unittest.main()
