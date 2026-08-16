"""Tests for simulation metadata on ROS detection messages."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time as TimeMessage

from bv_core.frame_metadata import SimulationFrameMetadata
from bv_core.vision_node import VisionNode


class TestVisionFrameMetadata(unittest.TestCase):
    def setUp(self):
        self.node = VisionNode.__new__(VisionNode)
        self.node.obj_dets_pub = Mock()
        self.node.pipeline_topic = "tcp://camera:7002"
        self.node.det_thresh = 0.5
        self.detections = SimpleNamespace(
            xyxy=np.array([[10.0, 20.0, 30.0, 40.0]]),
            confidence=np.array([0.9]),
            class_id=np.array([1]),
        )
        self.stamp = Mock()
        self.stamp.to_msg.return_value = TimeMessage(sec=8, nanosec=9)

    def test_preserves_ros_stamp_and_adds_simulation_identity(self):
        metadata = SimulationFrameMetadata("gazebo-run", 77, 123_456_789)

        self.node._publish_detections(self.detections, self.stamp, metadata)

        message = self.node.obj_dets_pub.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.sec, 8)
        self.assertEqual(message.header.stamp.nanosec, 9)
        self.assertEqual(message.sim_stream_id, "gazebo-run")
        self.assertEqual(message.sim_sequence, 77)
        self.assertEqual(message.sim_time_ns, 123_456_789)

    def test_non_simulated_pipeline_keeps_default_simulation_fields(self):
        self.node._publish_detections(self.detections, self.stamp)

        message = self.node.obj_dets_pub.publish.call_args.args[0]
        self.assertEqual(message.sim_stream_id, "")
        self.assertEqual(message.sim_sequence, 0)
        self.assertEqual(message.sim_time_ns, 0)


if __name__ == "__main__":
    unittest.main()
