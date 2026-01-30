"""Gazebo bounding box camera detector using gz_transport."""

import queue
import threading

import numpy as np
import rclpy.logging
import supervision as sv
from gz.transport13 import Node as GzNode
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V

from .base_detector import BaseDetector
import logging


# Mapping from Gazebo label IDs (set via gz-sim-label-system plugin) to COCO class IDs
# This mapping should match the <label> tags in your world SDF file
GAZEBO_LABEL_TO_COCO = {
    0: 2,    # hatchback_red (car) -> COCO "car" (class 2)
    1: 32,   # ball -> COCO "sports ball" (class 32)
    2: 4,    # rc_cessna (airplane) -> COCO "airplane" (class 4)
    3: 0,    # person -> COCO "person" (class 0)
    4: 5,    # bus -> COCO "bus" (class 5)
    5: 11,   # stop_sign -> COCO "stop sign" (class 11)
    6: 28,   # suitcase -> COCO "suitcase" (class 28)
}


class GazeboBBoxDetector(BaseDetector):
    """Detector that receives bounding boxes from Gazebo's bounding box camera sensor.

    This detector subscribes to a Gazebo topic that publishes ground-truth
    bounding box annotations, bypassing ML inference entirely.
    """

    def __init__(self, topic: str, queue_size: int = 5):
        """Initialize the Gazebo bounding box detector.

        Args:
            topic: Gazebo topic for bounding box messages
                   (e.g., '/camera/bounding_boxes').
            queue_size: Maximum number of detection messages to buffer.
        """
        self._topic = topic
        self._queue_size = queue_size
        self._node = None
        self._running = False
        self._bbox_queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()

    def start(self) -> None:
        """Subscribe to the Gazebo bounding box topic."""
        if self._running:
            return

        self._node = GzNode()
        success = self._node.subscribe(
            AnnotatedAxisAligned2DBox_V,
            self._topic,
            self._handle_bbox_msg,
        )

        if not success:
            raise RuntimeError(
                f"Failed to subscribe to Gazebo bbox topic '{self._topic}'"
            )

        self._running = True
        rclpy.logging.get_logger("gazebo_bbox_detector").info(
            f"Subscribed to Gazebo bbox topic: {self._topic}"
        )

    def stop(self) -> None:
        """Unsubscribe and release resources."""
        if not self._running:
            return

        try:
            if self._node:
                self._node.unsubscribe(self._topic)
        except RuntimeError:
            pass

        self._running = False
        self._clear_queue()

    def process_frame(self, frame: np.ndarray, **kwargs) -> sv.Detections:
        """Return the latest bounding boxes from Gazebo.

        Args:
            frame: Input image (used for dimension validation, not for detection).
            **kwargs: Unused.

        Returns:
            supervision.Detections with bounding boxes from Gazebo.
        """
        if not self._running:
            rclpy.logging.get_logger("gazebo_bbox_detector").warning(
                "Detector not started"
            )
            return sv.Detections.empty()

        bbox_msg = self._get_latest_bbox()
        if bbox_msg is None:
            return sv.Detections.empty()
            
        return self._msg_to_detections(bbox_msg, frame.shape)

    def _handle_bbox_msg(self, msg: AnnotatedAxisAligned2DBox_V) -> None:
        """Callback for incoming bounding box messages."""
        if not self._running:
            return

        with self._lock:
            if self._bbox_queue.full():
                try:
                    self._bbox_queue.get_nowait()
                except queue.Empty:
                    pass
            self._bbox_queue.put_nowait(msg)

    def _get_latest_bbox(self) -> AnnotatedAxisAligned2DBox_V | None:
        """Retrieve the most recent bounding box message, discarding older ones."""
        latest = None
        with self._lock:
            while True:
                try:
                    latest = self._bbox_queue.get_nowait()
                except queue.Empty:
                    break
        return latest

    def _clear_queue(self) -> None:
        """Remove all buffered messages."""
        with self._lock:
            while True:
                try:
                    self._bbox_queue.get_nowait()
                except queue.Empty:
                    break

    def _msg_to_detections(
        self,
        msg: AnnotatedAxisAligned2DBox_V,
        frame_shape: tuple,
    ) -> sv.Detections:
        """Convert Gazebo bbox message to supervision Detections.

        Args:
            msg: Gazebo AnnotatedAxisAligned2DBox_V message.
            frame_shape: Shape of the image (H, W, C) for coordinate validation.

        Returns:
            supervision.Detections object.
        """
        boxes = msg.annotated_box
        if not boxes:
            return sv.Detections.empty()

        height, width = frame_shape[:2]
        xyxy_list = []
        class_ids = []
        confidences = []

        for box in boxes:
            # box.label contains the numeric label from gz-sim-label-system plugin
            # Map it to COCO class ID using our mapping table
            gazebo_label = box.label
            coco_class_id = GAZEBO_LABEL_TO_COCO.get(gazebo_label, 0)  # Default to person (0) if unknown
            
            bbox = box.box

            x_min = bbox.min_corner.x
            y_min = bbox.min_corner.y
            x_max = bbox.max_corner.x
            y_max = bbox.max_corner.y

            x_min = max(0, min(x_min, width))
            x_max = max(0, min(x_max, width))
            y_min = max(0, min(y_min, height))
            y_max = max(0, min(y_max, height))

            if x_max <= x_min or y_max <= y_min:
                continue

            xyxy_list.append([x_min, y_min, x_max, y_max])
            class_ids.append(coco_class_id)
            confidences.append(1.0)

        if not xyxy_list:
            return sv.Detections.empty()

        return sv.Detections(
            xyxy=np.array(xyxy_list, dtype=np.float32),
            class_id=np.array(class_ids, dtype=np.int32),
            confidence=np.array(confidences, dtype=np.float32),
        )
