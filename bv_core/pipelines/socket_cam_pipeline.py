"""Socket-fed vision pipeline that consumes proxied host camera frames."""

import base64

import cv2
import numpy as np

from ..socket_proxy_client import SocketProxySubscriber
from .Vision_Pipeline import VisionPipeline


class SocketCamPipeline(VisionPipeline):
    """Receive JPEG-compressed frames from the host proxy over TCP."""

    def __init__(self, host: str, port: int, *, queue_size: int = 1):
        super().__init__(max_queue_size=queue_size)
        self._host = host
        self._port = port
        self._running = False
        self._subscriber = SocketProxySubscriber(
            host=host,
            port=port,
            kind="image",
            on_message=self._handle_image,
            logger_name="socket_cam_pipeline",
        )

    def start(self):
        if self._running:
            return

        self._subscriber.start()
        self._running = True

    def stop(self):
        if not self._running:
            return

        self._subscriber.stop()
        self._running = False
        self._clear_queue()

    def get_frame(self, timeout=None):
        if not self._running:
            raise RuntimeError("SocketCamPipeline must be started before getting frames")

        return super().get_frame(timeout=timeout)

    def _handle_image(self, message: dict) -> None:
        if not self._running:
            return

        encoded = message.get("jpeg_b64")
        if not encoded:
            return

        try:
            jpg = base64.b64decode(encoded)
        except (ValueError, TypeError):
            return

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        self._enqueue_frame(frame)
