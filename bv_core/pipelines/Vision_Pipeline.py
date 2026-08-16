"""Abstract interfaces for vision data pipelines."""

import queue
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class FramePacket:
    frame: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)


class VisionPipeline(ABC):
    """Abstract class for gstreamer and Gz_transport pipelines
    """

    def __init__(self, max_queue_size=2):

        self._frame_queue = queue.Queue(maxsize=max_queue_size)
        self._preview = None

    def set_preview(self, preview):
        """Attach a preview sink, or None to detach.

        The sink must expose offer(frame) and must not block.
        """
        self._preview = preview

    @abstractmethod
    def start(self):
        """Prepare resources and begin receiving frames
        This should be implemented by the specific pipeline"""

    @abstractmethod
    def stop(self):
        """Release resources and stop producing frames
        This should be implemented by the specific pipeline"""

    def get_frame(self, timeout=None):
        """
        Retrieve the most recent frame available in the queue

        Args:
            timeout: Seconds to wait for the first frame before giving up

        Returns:
            The newest frame buffered or None if no frame arrives in time
        """
        packet = self._get_latest_packet(timeout)
        return None if packet is None else packet.frame

    def get_frame_with_metadata(self, timeout=None):
        """Return the newest frame and a copy of its source metadata."""
        packet = self._get_latest_packet(timeout)
        if packet is None:
            return None
        return packet.frame, packet.metadata.copy()

    def _get_latest_packet(self, timeout):
        try:
            latest = self._frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None

        while True:
            try:
                latest = self._frame_queue.get_nowait()
            except queue.Empty:
                break

        return latest

    def _enqueue_frame(self, frame, metadata=None):
        """Offer a preview and queue the frame with its source metadata."""
        # Debug preview. When detached this is a single comparison; when attached,
        # offer() is non-blocking. Wrapped because a debug feature must never cost
        # the detection path a frame.
        if self._preview is not None:
            try:
                self._preview.offer(frame)
            except Exception:       # noqa: BLE001
                pass

        packet = FramePacket(frame, dict(metadata or {}))
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass

        self._frame_queue.put_nowait(packet)

    def _clear_queue(self):
        """Remove all buffered frames"""
        while True:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break
