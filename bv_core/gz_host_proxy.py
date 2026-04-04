"""Host-side Gazebo proxy that forwards image and bbox topics over TCP."""

import argparse
import base64
import json
import queue
import signal
import socket
import struct
import threading
import time
from typing import Any

import cv2
import numpy as np
from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.msgs10.image_pb2 import Image as ImageMsg
from gz.transport13 import Node as GzNode


class ProxyServer:
    """Bridge local Gazebo transport topics into a simple TCP stream."""

    def __init__(self, host: str, port: int, image_topic: str, bbox_topic: str) -> None:
        self._host = host
        self._port = port
        self._image_topic = image_topic
        self._bbox_topic = bbox_topic
        self._node = GzNode()
        self._outgoing: "queue.Queue[bytes]" = queue.Queue(maxsize=200)
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        self._server_sock: socket.socket | None = None

        self._image_count = 0
        self._bbox_count = 0
        self._sent_count = 0
        self._dropped_count = 0
        self._stats_lock = threading.Lock()

    def start(self) -> None:
        image_ok = self._node.subscribe(ImageMsg, self._image_topic, self._handle_image)
        bbox_ok = self._node.subscribe(
            AnnotatedAxisAligned2DBox_V,
            self._bbox_topic,
            self._handle_bbox,
        )
        if not image_ok:
            raise RuntimeError(f"Failed to subscribe to image topic {self._image_topic}")
        if not bbox_ok:
            raise RuntimeError(f"Failed to subscribe to bbox topic {self._bbox_topic}")

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen()
        self._server_sock.settimeout(1.0)

        threading.Thread(target=self._accept_loop, daemon=True).start()
        threading.Thread(target=self._broadcast_loop, daemon=True).start()
        threading.Thread(target=self._stats_loop, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._node.unsubscribe(self._image_topic)
        except Exception:
            pass
        try:
            self._node.unsubscribe(self._bbox_topic)
        except Exception:
            pass
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
        with self._clients_lock:
            for client in self._clients:
                try:
                    client.close()
                except OSError:
                    pass
            self._clients.clear()

    def wait(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.25)

    def _accept_loop(self) -> None:
        assert self._server_sock is not None
        while not self._stop.is_set():
            try:
                client, addr = self._server_sock.accept()
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._clients_lock:
                    self._clients.append(client)
                print(f"[proxy] client connected from {addr[0]}:{addr[1]}", flush=True)
            except socket.timeout:
                continue
            except OSError:
                break

    def _broadcast_loop(self) -> None:
        while not self._stop.is_set():
            try:
                packet = self._outgoing.get(timeout=0.5)
            except queue.Empty:
                continue

            dead_clients: list[socket.socket] = []
            with self._clients_lock:
                for client in self._clients:
                    try:
                        client.sendall(packet)
                    except OSError:
                        dead_clients.append(client)
                for client in dead_clients:
                    try:
                        client.close()
                    except OSError:
                        pass
                    self._clients.remove(client)

            with self._stats_lock:
                self._sent_count += 1

    def _stats_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(1.0)
            with self._stats_lock, self._clients_lock:
                print(
                    "[proxy] "
                    f"images={self._image_count} "
                    f"bboxes={self._bbox_count} "
                    f"sent={self._sent_count} "
                    f"dropped={self._dropped_count} "
                    f"clients={len(self._clients)}",
                    flush=True,
                )

    def _enqueue(self, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        packet = struct.pack(">I", len(data)) + data
        try:
            self._outgoing.put_nowait(packet)
        except queue.Full:
            with self._stats_lock:
                self._dropped_count += 1

    def _handle_image(self, msg: ImageMsg) -> None:
        if self._stop.is_set():
            return
        frame = self._image_to_numpy(msg)
        if frame is None:
            return

        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            return

        with self._stats_lock:
            self._image_count += 1

        self._enqueue(
            {
                "kind": "image",
                "ts_ns": time.time_ns(),
                "width": int(msg.width),
                "height": int(msg.height),
                "pixel_format": int(getattr(msg, "pixel_format_type", 0)),
                "jpeg_b64": base64.b64encode(encoded.tobytes()).decode("ascii"),
            }
        )

    def _handle_bbox(self, msg: AnnotatedAxisAligned2DBox_V) -> None:
        if self._stop.is_set():
            return

        detections: list[dict[str, Any]] = []
        for annotated_box in msg.annotated_box:
            bbox = annotated_box.box
            detections.append(
                {
                    "label": int(annotated_box.label),
                    "xmin": float(bbox.min_corner.x),
                    "ymin": float(bbox.min_corner.y),
                    "xmax": float(bbox.max_corner.x),
                    "ymax": float(bbox.max_corner.y),
                }
            )

        with self._stats_lock:
            self._bbox_count += 1

        self._enqueue(
            {
                "kind": "bbox",
                "ts_ns": time.time_ns(),
                "detections": detections,
            }
        )

    def _image_to_numpy(self, msg: ImageMsg) -> np.ndarray | None:
        width, height = int(msg.width), int(msg.height)
        if width <= 0 or height <= 0:
            return None

        row_bytes = int(msg.step) if getattr(msg, "step", 0) else 0
        if row_bytes <= 0:
            total_bytes = len(msg.data)
            if total_bytes == 0 or total_bytes % height != 0:
                return None
            row_bytes = total_bytes // height

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = height * row_bytes
        if buffer.size < expected_size:
            return None
        if buffer.size > expected_size:
            buffer = buffer[:expected_size]
        if row_bytes < width:
            return None

        bytes_per_pixel = row_bytes // width if width else 0
        if bytes_per_pixel == 0:
            return None

        if bytes_per_pixel in (1, 3, 4):
            dtype = np.uint8
        elif bytes_per_pixel in (2, 6, 8):
            dtype = np.uint16
        else:
            return None

        dtype_size = np.dtype(dtype).itemsize
        if bytes_per_pixel % dtype_size != 0:
            return None

        channel_count = bytes_per_pixel // dtype_size
        rows = buffer.reshape(height, row_bytes)
        rows = rows[:, : width * bytes_per_pixel]
        frame = rows.view(dtype)

        if channel_count == 1:
            frame = frame.reshape(height, width)
        else:
            frame = frame.reshape(height, width, channel_count)

        pixel_format = getattr(msg, "pixel_format_type", None)

        pf_rgb = getattr(ImageMsg, "PIXEL_FORMAT_RGB_INT8", 3)
        pf_rgba = getattr(ImageMsg, "PIXEL_FORMAT_RGBA_INT8", 4)
        pf_bgra = getattr(ImageMsg, "PIXEL_FORMAT_BGRA_INT8", 5)
        pf_bgr = getattr(ImageMsg, "PIXEL_FORMAT_BGR_INT8", 8)

        if frame.ndim == 3 and frame.dtype == np.uint8:
            if pixel_format == pf_rgb and frame.shape[2] >= 3:
                frame = frame[..., ::-1]
            elif pixel_format == pf_rgba and frame.shape[2] >= 4:
                frame = frame[..., [2, 1, 0, 3]]
            elif pixel_format == pf_bgra:
                pass
            elif pixel_format == pf_bgr:
                pass

        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[..., :3]

        if frame.dtype != np.uint8:
            frame = cv2.convertScaleAbs(
                frame,
                alpha=(255.0 / max(float(frame.max()), 1.0)),
            )

        return frame.copy()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=37031)
    parser.add_argument("--image-topic", default="/camera/bounding_boxes_image")
    parser.add_argument("--bbox-topic", default="/camera/bounding_boxes")
    args = parser.parse_args()

    server = ProxyServer(
        host=args.host,
        port=args.port,
        image_topic=args.image_topic,
        bbox_topic=args.bbox_topic,
    )

    def handle_signal(_signum: int, _frame: Any) -> None:
        server.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    server.start()
    print(
        f"[proxy] listening on {args.host}:{args.port} "
        f"for {args.image_topic} and {args.bbox_topic}",
        flush=True,
    )
    server.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
