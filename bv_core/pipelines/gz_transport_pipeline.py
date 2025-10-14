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
        #check if the formats are correct
        expected_format = getattr(ImageMsg, "PIXEL_FORMAT_L_INT8", None)
        if expected_format is None:
            raise RuntimeError("PIXEL_FORMAT_L_INT8 not available in gz.msgs.Image")

        if msg.pixel_format_type != expected_format:
            raise ValueError("Expected L_INT8 (mono8) pixel format from Gazebo camera")

        #flat array view on top of raw bytes from gazebo
        data = np.frombuffer(msg.data, dtype=np.uint8)
        
        #how many bytes each row occupies
        row_stride = msg.step if msg.step > 0 else msg.width
        if row_stride * msg.height > data.size:
            raise ValueError("Image buffer shorter than expected for reported stride/height")
        #convert flat bugger to 2d array 
        frame = data.reshape(msg.height, row_stride)
        #remove padding from stride > width
        frame = frame[:, : msg.width]
        #return copy so we dont run into issues with references to gazebos data
        return frame.copy()
