"""Abstract interface for object detection backends."""

from abc import ABC, abstractmethod

import numpy as np
import supervision as sv


class BaseDetector(ABC):
    """Abstract base class for object detectors.

    Implementations must handle their own resource management (model loading,
    subscriptions, etc.) in start() and stop().
    """

    @abstractmethod
    def start(self) -> None:
        """Initialize resources (model loading, subscriptions, etc.)."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Release resources and clean up."""
        pass

    @abstractmethod
    def process_frame(self, frame: np.ndarray, **kwargs) -> sv.Detections:
        """Run detection on a frame.

        Args:
            frame: Input image as numpy array (H, W, C) in BGR format.
            **kwargs: Implementation-specific parameters.

        Returns:
            supervision.Detections object with detected bounding boxes.
        """
        pass
