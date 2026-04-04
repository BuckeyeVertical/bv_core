"""Shared client for reading framed JSON messages from the host proxy."""

import json
import socket
import struct
import threading
import time
from typing import Any, Callable

import rclpy.logging


class SocketProxySubscriber:
    """Background TCP client that reconnects and forwards filtered messages."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        kind: str,
        on_message: Callable[[dict[str, Any]], None],
        logger_name: str,
    ) -> None:
        self._host = host
        self._port = port
        self._kind = kind
        self._on_message = on_message
        self._logger = rclpy.logging.get_logger(logger_name)

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._sock_lock = threading.Lock()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        last_error_log = 0.0

        while not self._stop.is_set():
            sock: socket.socket | None = None

            try:
                sock = socket.create_connection((self._host, self._port), timeout=2.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._sock_lock:
                    self._sock = sock

                self._logger.info(
                    f"Connected to host proxy {self._host}:{self._port} for '{self._kind}'"
                )

                while not self._stop.is_set():
                    message = self._recv_message(sock)
                    if message.get("kind") != self._kind:
                        continue

                    try:
                        self._on_message(message)
                    except Exception as exc:
                        self._logger.error(
                            f"Failed to process '{self._kind}' message from proxy: {exc}"
                        )

            except OSError as exc:
                now = time.monotonic()
                if now - last_error_log > 5.0:
                    self._logger.warn(
                        f"Host proxy '{self._kind}' connection issue on "
                        f"{self._host}:{self._port}: {exc}"
                    )
                    last_error_log = now
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                with self._sock_lock:
                    self._sock = None

            if not self._stop.is_set():
                time.sleep(1.0)

    def _close_socket(self) -> None:
        with self._sock_lock:
            sock = self._sock
            self._sock = None

        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _recv_message(self, sock: socket.socket) -> dict[str, Any]:
        header = self._recv_exact(sock, 4)
        (size,) = struct.unpack(">I", header)
        payload = self._recv_exact(sock, size)
        return json.loads(payload.decode("utf-8"))

    @staticmethod
    def _recv_exact(sock: socket.socket, size: int) -> bytes:
        chunks = []
        remaining = size

        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                raise ConnectionError("socket closed while receiving framed proxy data")
            chunks.append(chunk)
            remaining -= len(chunk)

        return b"".join(chunks)
