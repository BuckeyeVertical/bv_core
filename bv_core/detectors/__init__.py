"""Pluggable object detection backends."""

from typing import TYPE_CHECKING

from .base_detector import BaseDetector

if TYPE_CHECKING:
    from .gazebo_bbox_detector import GazeboBBoxDetector
    from .ml_detector import MLDetector
    from .socket_bbox_detector import SocketBBoxDetector

__all__ = [
    "BaseDetector",
    "GazeboBBoxDetector",
    "MLDetector",
    "SocketBBoxDetector",
    "create_detector",
]


def __getattr__(name: str):
    if name == "GazeboBBoxDetector":
        from .gazebo_bbox_detector import GazeboBBoxDetector

        return GazeboBBoxDetector
    if name == "MLDetector":
        from .ml_detector import MLDetector

        return MLDetector
    if name == "SocketBBoxDetector":
        from .socket_bbox_detector import SocketBBoxDetector

        return SocketBBoxDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def create_detector(detector_type: str, **config) -> BaseDetector:
    """Factory function to create a detector based on type.

    Args:
        detector_type: Type of detector to create. Options:
            - "ml": ML-based detector using RF-DETR
            - "gazebo_bbox": Gazebo bounding box camera detector
            - "socket_bbox": Host proxy detector that receives Gazebo bbox messages
        **config: Configuration parameters passed to the detector constructor.

    Returns:
        Configured detector instance.

    Raises:
        ValueError: If detector_type is unknown.
    """
    if detector_type == "ml":
        from .ml_detector import MLDetector

        return MLDetector(
            batch_size=config.get("batch_size", 16),
            resolution=config.get("resolution", 728),
        )
    elif detector_type == "gazebo_bbox":
        from .gazebo_bbox_detector import GazeboBBoxDetector

        return GazeboBBoxDetector(
            topic=config.get("gazebo_bbox_topic", "/camera/bounding_boxes"),
            queue_size=config.get("queue_size", 5),
        )
    elif detector_type == "socket_bbox":
        from .socket_bbox_detector import SocketBBoxDetector

        return SocketBBoxDetector(
            host=config.get("socket_host", "127.0.0.1"),
            port=int(config.get("socket_port", 37031)),
            queue_size=config.get("queue_size", 5),
        )
    else:
        raise ValueError(
            f"Unknown detector type: '{detector_type}'. "
            f"Valid options: 'ml', 'gazebo_bbox', 'socket_bbox'"
        )
