"""Gazebo Transport-based vision pipeline"""

import importlib
import numpy as np

try:
    import gz.transport as gz_transport
except ImportError:
    gz_transport = importlib.import_module("gz.transport13")

try:
    from gz.msgs import image_pb2
except ImportError:
    image_pb2 = None
    for module_name in ("gz.msgs10.image_pb2", "gz.msgs13.image_pb2"):
        try:
            image_pb2 = importlib.import_module(module_name)
            break
        except ImportError:
            continue
    if image_pb2 is None:
        raise

from .VisionPipeline import VisionPipeline


ImageMsg = image_pb2.Image


class GzTransportPipeline(VisionPipeline):
    """Subscription-based video pipeline that sources frames from Gazebo transport"""

    def __init__(
        self,
        topic: str,
        *,
        queue_size: int = 1,
    ):
        """
        Args:
            topic: Gazebo camera topic to subscribe to (e.g. ``/camera/image``)
            queue_size: Number of frames to buffer; defaults to holding only the latest
        """
        super().__init__(max_queue_size=queue_size)

        self._topic = topic
        self._node = gz_transport.Node()
        self._running = False


    def start(self):
        """Begin consuming frames from the Gazebo transport layer"""
        if self._running:
            return
        #node.subscribe(StringMsg, topic_stringmsg, stringmsg_cb)
        success = self._node.subscribe(ImageMsg, self._topic, self._handle_image)
        if not success:
            raise RuntimeError(f"Failed to subscribe to Gazebo topic '{self._topic}'")

        self._running = True

    def get_frame(self, timeout=None):
        """Return the newest buffered frame"""
        if not self._running:
            raise RuntimeError("GzTransportPipeline must be started before getting frames")

        return super().get_frame(timeout=timeout)

    def stop(self):
        """Cancel the Gazebo transport subscription and clear buffered frames"""
        if not self._running:
            return

        try:
            self._node.unsubscribe(self._topic)
        except RuntimeError:
            pass

        self._running = False
        self._clear_queue()

    def _handle_image(self, msg: ImageMsg) -> None:
        if not self._running:
            return

        try:
            frame = self._image_to_numpy(msg)
        except ValueError:
            return

        self._enqueue_frame(frame)

    def _image_to_numpy(self, msg: ImageMsg) -> np.ndarray:
        """Convert a Gazebo image message into a numpy array, keeping stride handling robust."""
        width, height = msg.width, msg.height
        if width <= 0 or height <= 0:
            raise ValueError("Invalid image dimensions reported by Gazebo message")

        row_bytes = msg.step if getattr(msg, "step", 0) else 0
        if row_bytes <= 0:
            total_bytes = len(msg.data)
            if height == 0 or total_bytes % height != 0:
                raise ValueError("Cannot infer row stride from Gcazebo message payload")
            row_bytes = total_bytes // height

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = height * row_bytes
        if buffer.size < expected_size:
            raise ValueError("Image buffer shorter than expected for reported stride/height")
        if buffer.size > expected_size:
            buffer = buffer[:expected_size]

        if width > row_bytes:
            raise ValueError("Row stride smaller than reported width")

        bytes_per_pixel = row_bytes // width if width else 0
        if bytes_per_pixel == 0:
            raise ValueError("Unable to infer bytes-per-pixel from Gazebo message")

        if bytes_per_pixel in (1, 3, 4):
            dtype = np.uint8
        elif bytes_per_pixel in (2, 6, 8):
            dtype = np.uint16
        else:
            raise ValueError(f"Unsupported bytes-per-pixel: {bytes_per_pixel}")

        dtype_size = np.dtype(dtype).itemsize
        if bytes_per_pixel % dtype_size != 0:
            raise ValueError("Stride/dtype mismatch while decoding Gazebo image")

        channel_count = bytes_per_pixel // dtype_size

        rows = buffer.reshape(height, row_bytes)
        row_pixels_bytes = width * bytes_per_pixel
        rows = rows[:, :row_pixels_bytes]

        frame = rows.view(dtype)
        if channel_count == 1:
            frame = frame.reshape(height, width)
        else:
            frame = frame.reshape(height, width, channel_count)

        pixel_format = getattr(msg, "pixel_format_type", None)

        pf_rgb = getattr(ImageMsg, "PIXEL_FORMAT_RGB_INT8", None)
        pf_rgba = getattr(ImageMsg, "PIXEL_FORMAT_RGBA_INT8", None)

        if frame.ndim == 3 and frame.shape[2] >= 3 and frame.dtype == np.uint8:
            if pixel_format == pf_rgb:
                frame = frame[..., ::-1]
            elif pixel_format == pf_rgba:
                frame = frame[..., [2, 1, 0, 3]]

        if frame.ndim == 3 and frame.shape[2] == 4 and frame.dtype == np.uint8:
            frame = frame[..., :3]

        return frame.copy()
