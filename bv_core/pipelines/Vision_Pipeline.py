"""Abstract interfaces for vision data pipelines."""

import queue
from abc import ABC, abstractmethod

import numpy as np


class VisionPipeline(ABC):
    """Abstract class for gstreamer and Gz_transport pipelines
    """

    def __init__(self, max_queue_size=2):

        self._frame_queue = queue.Queue(maxsize=max_queue_size)

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

    def _enqueue_frame(self, frame):
        """Push a new frame into the bounded queue, dropping the oldest if full"""
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass

        self._frame_queue.put_nowait(frame)

    def _clear_queue(self):
        """Remove all buffered frames"""
        while True:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break

