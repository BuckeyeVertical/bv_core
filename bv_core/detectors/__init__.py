"""Pluggable object detection backends."""

from .base_detector import BaseDetector

__all__ = [
    "BaseDetector",
    "GazeboBBoxDetector",
    "create_detector",
]


def __getattr__(name: str):
    """Lazily import detector backends with heavy runtime dependencies."""
    if name == "GazeboBBoxDetector":
        from .gazebo_bbox_detector import GazeboBBoxDetector

        return GazeboBBoxDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def create_detector(detector_type: str, **config) -> BaseDetector:
    """Factory function to create a detector based on type.

    Args:
        detector_type: Type of detector to create. Options:
            - "ml": ML-based detector using LightlyTrain LTDETR
            - "gazebo_bbox": Gazebo bounding box camera detector
        **config: Configuration parameters passed to the detector constructor.

    Returns:
        Configured detector instance.

    Raises:
        ValueError: If detector_type is unknown.
    """
    if detector_type == "ml":
        from .ml_detector import MLDetector

        return MLDetector(
            model_path=config.get(
                "ml_model_path",
                "/Users/allenthomas/Code/Personal/inference/ltdetr.pt",
            ),
        )
    elif detector_type == "gazebo_bbox":
        from .gazebo_bbox_detector import GazeboBBoxDetector

        return GazeboBBoxDetector(
            topic=config.get("gazebo_bbox_topic", "/camera/bounding_boxes"),
            queue_size=config.get("queue_size", 5),
        )
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Valid options: 'ml', 'gazebo_bbox'"
        )
