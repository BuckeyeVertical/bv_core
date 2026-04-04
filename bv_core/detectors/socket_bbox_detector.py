"""Socket-fed detector that consumes proxied Gazebo bbox messages."""

import queue
import threading

import numpy as np
import rclpy.logging
import supervision as sv

from ..socket_proxy_client import SocketProxySubscriber
from .base_detector import BaseDetector
from .gazebo_bbox_detector import GAZEBO_LABEL_TO_COCO


class SocketBBoxDetector(BaseDetector):
    """Detector that reads bbox payloads from the host proxy instead of Gazebo."""

    def __init__(self, host: str, port: int, queue_size: int = 5):
        self._host = host
        self._port = port
        self._queue_size = queue_size
        self._running = False
        self._bbox_queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._subscriber = SocketProxySubscriber(
            host=host,
            port=port,
            kind="bbox",
            on_message=self._handle_bbox_msg,
            logger_name="socket_bbox_detector",
        )

    def start(self) -> None:
        if self._running:
            return

        self._subscriber.start()
        self._running = True
        rclpy.logging.get_logger("socket_bbox_detector").info(
            f"Connected bbox detector to host proxy {self._host}:{self._port}"
        )

    def stop(self) -> None:
        if not self._running:
            return

        self._subscriber.stop()
        self._running = False
        self._clear_queue()

    def process_frame(self, frame: np.ndarray, **kwargs) -> sv.Detections:
        if not self._running:
            rclpy.logging.get_logger("socket_bbox_detector").warning(
                "Detector not started"
            )
            return sv.Detections.empty()

        bbox_msg = self._get_latest_bbox()
        if bbox_msg is None:
            return sv.Detections.empty()

        return self._msg_to_detections(bbox_msg, frame.shape)

    def _handle_bbox_msg(self, msg: dict) -> None:
        if not self._running:
            return

        with self._lock:
            if self._bbox_queue.full():
                try:
                    self._bbox_queue.get_nowait()
                except queue.Empty:
                    pass
            self._bbox_queue.put_nowait(msg)

    def _get_latest_bbox(self) -> dict | None:
        latest = None
        with self._lock:
            while True:
                try:
                    latest = self._bbox_queue.get_nowait()
                except queue.Empty:
                    break
        return latest

    def _clear_queue(self) -> None:
        with self._lock:
            while True:
                try:
                    self._bbox_queue.get_nowait()
                except queue.Empty:
                    break

    def _msg_to_detections(
        self,
        msg: dict,
        frame_shape: tuple,
    ) -> sv.Detections:
        boxes = msg.get("detections") or []
        if not boxes:
            return sv.Detections.empty()

        height, width = frame_shape[:2]
        xyxy_list = []
        class_ids = []
        confidences = []

        for box in boxes:
            gazebo_label = int(box.get("label", 0))
            coco_class_id = GAZEBO_LABEL_TO_COCO.get(gazebo_label, 0)

            x_min = float(box.get("xmin", 0.0))
            y_min = float(box.get("ymin", 0.0))
            x_max = float(box.get("xmax", 0.0))
            y_max = float(box.get("ymax", 0.0))

            x_min = max(0.0, min(x_min, width))
            x_max = max(0.0, min(x_max, width))
            y_min = max(0.0, min(y_min, height))
            y_max = max(0.0, min(y_max, height))

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
