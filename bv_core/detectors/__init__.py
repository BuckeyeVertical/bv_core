"""Pluggable object detection backends."""

from .base_detector import BaseDetector
from .gazebo_bbox_detector import GazeboBBoxDetector
from .ml_detector import MLDetector

__all__ = [
    "BaseDetector",
    "GazeboBBoxDetector",
    "MLDetector",
    "create_detector",
]


def create_detector(detector_type: str, **config) -> BaseDetector:
    """Factory function to create a detector based on type.

    Args:
        detector_type: Type of detector to create. Options:
            - "ml": ML-based detector using RF-DETR
            - "gazebo_bbox": Gazebo bounding box camera detector
        **config: Configuration parameters passed to the detector constructor.

    Returns:
        Configured detector instance.

    Raises:
        ValueError: If detector_type is unknown.
    """
    if detector_type == "ml":
        return MLDetector(
            batch_size=config.get("batch_size", 16),
            resolution=config.get("resolution", 728),
        )
    elif detector_type == "gazebo_bbox":
        return GazeboBBoxDetector(
            topic=config.get("gazebo_bbox_topic", "/camera/bounding_boxes"),
            queue_size=config.get("queue_size", 5),
        )
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Valid options: 'ml', 'gazebo_bbox'"
        )
