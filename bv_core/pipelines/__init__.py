"""Pluggable vision pipeline backends."""

from typing import Tuple

from .Vision_Pipeline import VisionPipeline

__all__ = [
    "VisionPipeline",
    "create_pipeline",
]


def create_pipeline(pipeline_type: str, **config) -> Tuple[VisionPipeline, str]:
    """Factory function to create a pipeline based on type.

    Args:
        pipeline_type: Type of pipeline to create. Options:
            - "ros": ROS2 CompressedImage topic subscription
            - "sim" / "gazebo": Gazebo transport image subscription
            - "real" / "camera": Physical camera or GStreamer pipeline
        **config: All configuration parameters. Factory picks what it needs.
            - gz_topic: Gazebo transport topic (used by sim/gazebo)
            - ros_topic: ROS image topic (used by ros)
            - node: ROS2 node instance (used by ros, camera)
            - gst_pipeline: GStreamer pipeline string (used by real/camera)
            - queue_size: Frame buffer size (default: 5)
            - record: Publish JPEG frames (default: False)
            - fps: Target frame rate (default: 30.0)
            - qos_profile: Optional QoSProfile (used by ros)

    Returns:
        Tuple of (pipeline instance, topic string for identification).

    Raises:
        ValueError: If pipeline_type is unknown or required config is missing.
    """
    queue_size = config.get("queue_size", 5)

    if pipeline_type == "ros":
        from .ros_cam_pipeline import RosCamPipeline

        if "node" not in config:
            raise ValueError("'ros' pipeline requires 'node' in config")

        topic = config.get("ros_topic", "/image_compressed")
        pipeline = RosCamPipeline(
            node=config["node"],
            topic=topic,
            queue_size=queue_size,
            qos_profile=config.get("qos_profile"),
        )
        return pipeline, topic

    elif pipeline_type in ("sim", "gazebo"):
        from .gz_transport_pipeline import GzTransportPipeline

        topic = config.get("gz_topic", "/camera/bounding_boxes_image")
        pipeline = GzTransportPipeline(
            topic=topic,
            queue_size=queue_size,
        )
        return pipeline, topic

    elif pipeline_type in ("real", "camera"):
        from .camera_pipeline import CameraPipeline

        if "gst_pipeline" not in config:
            raise ValueError("'real' pipeline requires 'gst_pipeline' in config")

        topic = "/camera/image"
        pipeline = CameraPipeline(
            gst_pipeline=config["gst_pipeline"],
            max_queue_size=queue_size,
            record=config.get("record", False),
            node=config.get("node"),
            fps=config.get("fps", 30.0),
        )
        return pipeline, topic

    else:
        raise ValueError(
            f"Unknown pipeline type: '{pipeline_type}'. "
            f"Valid options: 'ros', 'sim', 'gazebo', 'real', 'camera'"
        )