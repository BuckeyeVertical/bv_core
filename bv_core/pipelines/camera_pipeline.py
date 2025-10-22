"""Camera (real or GStreamer) based vision pipeline"""

import threading
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from .VisionPipeline import VisionPipeline


class CameraPipeline(VisionPipeline):
    """
    Captures frames from a real or simulated camera using OpenCV (GStreamer-compatible)
    and optionally publishes compressed images to a ROS topic.
    """

    def __init__(self, gst_pipeline: str, *, max_queue_size: int = 2, record: bool = False, ros_context=None, fps: float = 30.0):
        """
        Args:
            gst_pipeline: GStreamer pipeline string or camera index.
            record: If True, publish JPEG frames to /image_compressed.
            ros_context: rclpy.Node or object with create_publisher() method (required if record=True).
            fps: Target frame rate for capture loop.
        """
        super().__init__(max_queue_size=max_queue_size)

        self._gst_pipeline = gst_pipeline
        self._record = record
        self._fps = fps
        self._running = False
        self._thread = None

        self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open GStreamer pipeline:\n{gst_pipeline}")

        # Optional ROS compressed image publisher to publish to /image_compressed topic
        self._bridge = CvBridge()
        self._ros_context = ros_context
        self._publisher = None
        if self._record:
            if self._ros_context is None:
                raise ValueError("record=True requires a valid ROS context (Node).")
            self._publisher = self._ros_context.create_publisher(
                CompressedImage, '/image_compressed', 10
            )

    # Public API
    def start(self):
        """Begin capturing frames from camera in a background thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop capture thread and release camera resources."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap.isOpened():
            self._cap.release()
        self._clear_queue()

    def get_frame(self, timeout=None):
        """Return newest buffered frame (see VisionPipeline base)."""
        return super().get_frame(timeout=timeout)

    # Helper Methods
    def _capture_loop(self):
        """Continuously read frames and enqueue the latest one."""
        interval = 1.0 / self._fps
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # Push raw frame to shared queue
            self._enqueue_frame(frame)

            # Optionally publish JPEG
            if self._record and self._publisher is not None:
                self._publish_compressed(frame)

            time.sleep(interval)

    def _publish_compressed(self, frame: np.ndarray):
        """Convert and publish a compressed JPEG frame to /image_compressed."""
        msg = self._bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
        msg.header.stamp = self._ros_context.get_clock().now().to_msg()
        self._publisher.publish(msg)
