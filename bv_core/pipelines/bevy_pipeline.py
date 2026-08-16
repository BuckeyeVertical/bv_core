"""ROS-independent Bevy camera stream pipeline."""

import json
import math
import socket
import threading
from time import sleep

import cv2
import numpy as np

from .Vision_Pipeline import VisionPipeline


class BevyPipeline(VisionPipeline):
    """Receive timestamped JPEG frames from the BV camera TCP stream."""

    _MAX_HEADER_BYTES = 64 * 1024
    _MAX_FRAME_BYTES = 100 * 1024 * 1024

    def __init__(self, host="127.0.0.1", port=7002, *, queue_size=1):
        super().__init__(max_queue_size=queue_size)
        self._host = host
        self._port = port
        self._running = False
        self._stop_event = threading.Event()
        self._thread = None
        self._socket = None
        self._socket_lock = threading.Lock()
        self._metadata = None
        self._metadata_lock = threading.Lock()

    @property
    def latest_metadata(self):
        """Return metadata for the most recently delivered frame."""
        with self._metadata_lock:
            return None if self._metadata is None else self._metadata.copy()

    def start(self):
        """Start the reconnecting camera receiver thread."""
        if self._running:
            return

        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._receive_forever,
            name="bevy-camera-pipeline",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        """Stop receiving frames and release the active connection."""
        if not self._running:
            return

        self._running = False
        self._stop_event.set()
        self._close_socket()

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        self._clear_queue()

    def get_frame(self, timeout=None):
        """Return the newest decoded BGR frame."""
        packet = self.get_frame_with_metadata(timeout=timeout)
        return None if packet is None else packet[0]

    def get_frame_with_metadata(self, timeout=None):
        """Return the newest BGR frame and its simulation metadata."""
        if not self._running:
            raise RuntimeError(
                "BevyPipeline must be started before getting frames"
            )

        packet = super().get_frame_with_metadata(timeout=timeout)
        if packet is None:
            return None

        frame, metadata = packet
        with self._metadata_lock:
            self._metadata = metadata
        return frame, metadata.copy()

    def _receive_forever(self):
        while not self._stop_event.is_set():
            try:
                connection = socket.create_connection(
                    (self._host, self._port),
                    timeout=2.0,
                )
                connection.settimeout(None)
                self._set_socket(connection)

                with connection, connection.makefile("rb") as stream:
                    while not self._stop_event.is_set():
                        frame, metadata = self._read_frame(stream)
                        self._enqueue_frame(frame, metadata)
            except (
                ConnectionError,
                json.JSONDecodeError,
                OSError,
                ValueError,
            ):
                if not self._stop_event.is_set():
                    sleep(0.5)
            finally:
                self._set_socket(None)

    def _set_socket(self, connection):
        with self._socket_lock:
            self._socket = connection

    def _close_socket(self):
        with self._socket_lock:
            connection = self._socket
            self._socket = None

        if connection is None:
            return
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()

    @classmethod
    def _read_frame(cls, stream):
        header_line = stream.readline(cls._MAX_HEADER_BYTES + 1)
        if not header_line:
            raise ConnectionError("Bevy camera stream closed")
        if (
            len(header_line) > cls._MAX_HEADER_BYTES
            or not header_line.endswith(b"\n")
        ):
            raise ValueError("Bevy camera frame header exceeds size limit")

        metadata = json.loads(header_line)
        cls._validate_metadata(metadata)
        jpeg = cls._read_exact(stream, metadata["data_length"])
        encoded = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Bevy camera payload is not a valid JPEG")
        if frame.shape[:2] != (metadata["height"], metadata["width"]):
            raise ValueError(
                "Bevy camera JPEG dimensions do not match its header"
            )

        return frame, metadata

    @classmethod
    def _validate_metadata(cls, metadata):
        if (
            metadata.get("schema") != "bv.camera_frame"
            or metadata.get("version") != 1
        ):
            raise ValueError("Unsupported Bevy camera frame schema")
        if metadata.get("encoding") != "jpeg":
            raise ValueError("Unsupported Bevy camera frame encoding")

        numeric_fields = (
            "sequence",
            "sim_sequence",
            "sim_time_ns",
            "width",
            "height",
        )
        for field in numeric_fields:
            if (
                not isinstance(metadata.get(field), int)
                or isinstance(metadata[field], bool)
                or metadata[field] < 0
            ):
                raise ValueError(f"Invalid Bevy camera frame field: {field}")
        if metadata["width"] == 0 or metadata["height"] == 0:
            raise ValueError("Bevy camera frame dimensions must be positive")
        if (
            not isinstance(metadata.get("camera_id"), str)
            or not metadata["camera_id"]
        ):
            raise ValueError("Bevy camera ID must not be empty")
        sim_stream_id = metadata.get("sim_stream_id")
        if sim_stream_id is not None and (
            not isinstance(sim_stream_id, str) or not sim_stream_id.strip()
        ):
            raise ValueError("Bevy simulation stream ID must not be empty")
        cls._validate_intrinsics(metadata)

        data_length = metadata.get("data_length")
        if (
            not isinstance(data_length, int)
            or isinstance(data_length, bool)
            or not 0 < data_length <= cls._MAX_FRAME_BYTES
        ):
            raise ValueError("Invalid Bevy camera payload length")

    @staticmethod
    def _validate_intrinsics(metadata):
        fields = ("camera_model", "fx", "fy", "cx", "cy")
        present = tuple(field in metadata for field in fields)
        if not any(present):
            return
        if not all(present):
            raise ValueError("Incomplete Bevy camera intrinsics")
        if metadata["camera_model"] != "pinhole":
            raise ValueError("Unsupported Bevy camera model")

        for field in ("fx", "fy", "cx", "cy"):
            value = metadata[field]
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError(f"Invalid Bevy camera intrinsic: {field}")
        if metadata["fx"] <= 0 or metadata["fy"] <= 0:
            raise ValueError("Bevy camera focal lengths must be positive")

    @staticmethod
    def _read_exact(stream, length):
        data = bytearray(length)
        view = memoryview(data)
        offset = 0
        while offset < length:
            count = stream.readinto(view[offset:])
            if not count:
                raise ConnectionError(
                    "Bevy camera stream closed inside a frame"
                )
            offset += count
        return data
