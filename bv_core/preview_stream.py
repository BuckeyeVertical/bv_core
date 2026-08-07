#!/usr/bin/env python3
"""Low-bitrate camera preview for the ground station.

This module is deliberately split so the part that runs on the capture thread —
frame gating — has no GStreamer dependency and can be tested exhaustively. The
encoder lives alongside it but is only exercised in integration.

The governing rule is that the capture thread is never delayed. `FrameGate.offer`
rate-limits, keeps at most one frame, and returns immediately. When the encoder is
slow the preview loses framerate; capture and detection are untouched.
"""

import threading
import time
from dataclasses import dataclass

import cv2


def decimation_factor(src_width, target_width):
    """Integer stride that gets `src_width` close to `target_width`.

    Strided slicing is far cheaper than resizing a full frame: it copies every
    nth pixel instead of reading and interpolating all of them. Returns 1 when
    the source is already at or below the target.
    """
    if target_width <= 0 or src_width <= target_width:
        return 1
    return max(1, int(src_width) // int(target_width))


def downscale(frame, target_width):
    """Reduce `frame` to `target_width`, preserving aspect ratio.

    Decimates first, then resizes the much smaller result. Output dimensions are
    forced even because H.264 encoders reject odd ones. Never mutates the input.
    """
    height, width = frame.shape[:2]
    if width <= target_width:
        return frame

    stride = decimation_factor(width, target_width)
    if stride > 1:
        frame = frame[::stride, ::stride]

    dec_h, dec_w = frame.shape[:2]
    out_w = int(target_width) - (int(target_width) % 2)
    out_h = int(round(dec_h * out_w / dec_w))
    out_h -= out_h % 2

    return cv2.resize(frame, (max(2, out_w), max(2, out_h)),
                      interpolation=cv2.INTER_AREA)


class FrameGate:
    """Rate-limited, latest-wins, single-slot handoff to the encoder thread.

    `offer` is called from the capture thread and must never block or raise.
    `take` is called from the encoder thread.
    """

    def __init__(self, fps):
        """Build a gate that accepts at most `fps` frames per second.

        Args:
            fps: accept at most this many frames per second. 0 disables limiting.
        """
        self._min_interval = 1.0 / fps if fps and fps > 0 else 0.0
        self._lock = threading.Lock()
        self._slot = None
        self._last_accept = 0.0
        self._counts = {'offered': 0, 'accepted': 0, 'dropped': 0}

    @property
    def stats(self):
        """Return a snapshot of the offered/accepted/dropped counters."""
        with self._lock:
            return dict(self._counts)

    def offer(self, frame):
        """Present a frame. Returns True if it was accepted.

        Rejects when it arrives sooner than the rate limit allows. If a previously
        accepted frame has not been taken yet, it is overwritten — the newest frame
        is always the useful one for a live view.
        """
        try:
            with self._lock:
                self._counts['offered'] += 1

                if frame is None:
                    self._counts['dropped'] += 1
                    return False

                now = time.monotonic()
                if self._min_interval and (now - self._last_accept) < self._min_interval:
                    self._counts['dropped'] += 1
                    return False

                if self._slot is not None:
                    # Never encoded; the encoder was still busy.
                    self._counts['dropped'] += 1

                self._last_accept = now
                self._slot = frame
                self._counts['accepted'] += 1
                return True
        except Exception:       # noqa: BLE001 - the capture thread must survive
            return False

    def take(self):
        """Remove and return the pending frame, or None."""
        with self._lock:
            frame, self._slot = self._slot, None
            return frame


# GStreamer is imported lazily inside _init_gst so this module can be imported
# (and Task 1's logic tested) on a machine without PyGObject.
Gst = None


def _init_gst():
    """Import and initialise GStreamer once. Returns the Gst module or None."""
    global Gst
    if Gst is not None:
        return Gst
    try:
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst as _Gst
        _Gst.init(None)
        Gst = _Gst
        return Gst
    except Exception:       # noqa: BLE001 - absence is a supported state
        return None


@dataclass
class PreviewConfig:
    """Tunables, mirrored in config/vision_params.yaml."""

    width: int = 640
    fps: float = 8.0
    bitrate_bps: int = 400_000


# Ordered best-first. The Jetson tier is the deployment target; the others exist
# so the feature can be developed against sim. Python has already downscaled the
# frame to its final size, so these differ only in colour conversion and encoder.
_TIERS = (
    ('jetson',
     'nvvidconv ! nvv4l2h264enc bitrate={bps} insert-sps-pps=true '
     'iframeinterval={gop} maxperf-enable=true'),
    ('nvidia',
     'videoconvert ! nvh264enc bitrate={kbps} preset=low-latency-hq '
     'gop-size={gop}'),
    ('software',
     'videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast '
     'bitrate={kbps} key-int-max={gop}'),
)


class PreviewStream:
    """Encodes offered frames to fragmented MP4 and hands chunks to a callback."""

    def __init__(self, cfg, on_chunk, logger=None):
        """Build a preview encoder.

        Args:
            cfg: PreviewConfig.
            on_chunk: called with `bytes` for each fMP4 chunk. Exceptions raised
                by it are caught and logged; they never stop the encoder.
            logger: optional object with .info/.warn/.error.
        """
        self._cfg = cfg
        self._on_chunk = on_chunk
        self._log = logger
        self._gate = FrameGate(cfg.fps)
        self._pipeline = None
        self._appsrc = None
        self._thread = None
        self._running = False
        self._tier = None
        self._encoded = 0
        self._dims = None

    @property
    def tier(self):
        """Name of the encoder tier in use, or None before a successful start."""
        return self._tier

    def is_running(self):
        """Return True while the encoder thread and pipeline are up."""
        return self._running

    def stats(self):
        """Return gate counters plus encoded-chunk count and selected tier."""
        s = self._gate.stats
        s['encoded'] = self._encoded
        s['tier'] = self._tier
        return s

    def offer(self, frame):
        """Called from the capture thread. Never blocks, never raises."""
        if not self._running:
            return
        self._gate.offer(frame)

    def start(self):
        """Bring up the first encoder tier that actually works. Returns success."""
        if self._running:
            return True

        gst = _init_gst()
        if gst is None:
            self._warn('GStreamer/PyGObject unavailable; preview disabled')
            return False

        # A probe frame fixes the caps. Real dimensions are learned on the first
        # offered frame, so start() uses the configured width and a 4:3 guess;
        # _ensure_dims rebuilds if the source turns out to be a different shape.
        for name, segment in _TIERS:
            if self._try_tier(gst, name, segment,
                              self._cfg.width, int(self._cfg.width * 3 / 4)):
                self._tier = name
                self._running = True
                self._thread = threading.Thread(target=self._encode_loop,
                                                daemon=True)
                self._thread.start()
                self._info(f"preview encoder: tier '{name}' "
                           f"at {self._cfg.width}px {self._cfg.fps}fps "
                           f"{self._cfg.bitrate_bps // 1000}kbps")
                return True
            self._info(f"preview encoder: tier '{name}' unavailable")

        self._warn('no usable H.264 encoder found; preview disabled')
        return False

    def stop(self):
        """Tear down. Idempotent."""
        if not self._running:
            self._teardown()
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._teardown()

    # -- internals ------------------------------------------------------------

    def _try_tier(self, gst, name, segment, width, height):
        """Build one tier and wait for it to actually reach PLAYING.

        Registry presence is NOT sufficient: nvh264enc is present on desktop
        machines where it fails to initialise. Only a pipeline that reaches
        PLAYING is accepted. A tier that does not is returned to NULL before the
        next is tried, so no half-open pipeline keeps hold of the encoder.
        """
        gop = max(1, int(self._cfg.fps * 2)) if self._cfg.fps else 16
        enc = segment.format(bps=self._cfg.bitrate_bps,
                             kbps=max(1, self._cfg.bitrate_bps // 1000),
                             gop=gop)
        fps_n = int(self._cfg.fps) if self._cfg.fps and self._cfg.fps >= 1 else 30
        desc = (
            f"appsrc name=src is-live=true format=time do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={width},height={height},"
            f"framerate={fps_n}/1 "
            f"! {enc} "
            f"! h264parse ! mp4mux fragment-duration=200 streamable=true "
            f"! appsink name=sink emit-signals=true sync=false "
            f"max-buffers=8 drop=true"
        )
        try:
            pipeline = gst.parse_launch(desc)
        except Exception as exc:    # noqa: BLE001 - missing elements land here
            self._info(f"tier '{name}' parse failed: {exc}")
            return False

        sink = pipeline.get_by_name('sink')
        sink.connect('new-sample', self._on_sample)

        if pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE:
            self._info(f"tier '{name}' refused PLAYING")
            self._shutdown(gst, pipeline)
            return False

        # One black probe frame. Nothing downstream can preroll until a buffer
        # has been through the encoder, so without this the pipeline sits in
        # PAUSED and every tier — including a working one — looks unavailable.
        appsrc = pipeline.get_by_name('src')
        try:
            appsrc.emit('push-buffer',
                        gst.Buffer.new_wrapped(bytes(width * height * 3)))
        except Exception as exc:    # noqa: BLE001 - a broken tier lands here
            self._info(f"tier '{name}' probe push failed: {exc}")
            self._shutdown(gst, pipeline)
            return False

        # set_state may return ASYNC; wait for the real answer.
        state_ret, cur, _ = pipeline.get_state(3 * gst.SECOND)
        if state_ret != gst.StateChangeReturn.SUCCESS:
            self._info(f"tier '{name}' never reached PLAYING "
                       f"(stuck at {cur.value_nick}); its elements are in the "
                       f"registry but do not run here")
            self._shutdown(gst, pipeline)
            return False

        self._pipeline = pipeline
        self._appsrc = appsrc
        self._dims = (width, height)
        return True

    @staticmethod
    def _shutdown(gst, pipeline):
        """Return a pipeline to NULL and wait for the transition to land."""
        try:
            pipeline.set_state(gst.State.NULL)
            pipeline.get_state(3 * gst.SECOND)
        except Exception:       # noqa: BLE001 - teardown is best effort
            pass

    def _on_sample(self, sink):
        """Handle an appsink sample: hand the encoded chunk to the consumer."""
        gst = Gst
        try:
            sample = sink.emit('pull-sample')
            if sample is None:
                return gst.FlowReturn.OK
            buf = sample.get_buffer()
            ok, info = buf.map(gst.MapFlags.READ)
            if ok:
                try:
                    self._encoded += 1
                    self._on_chunk(bytes(info.data))
                except Exception as exc:    # noqa: BLE001 - consumer must not kill us
                    self._warn(f"preview chunk consumer raised: {exc}")
                finally:
                    buf.unmap(info)
        except Exception as exc:    # noqa: BLE001 - nothing here may reach GStreamer
            self._warn(f"preview sample handling failed: {exc}")
        return gst.FlowReturn.OK

    def _encode_loop(self):
        gst = Gst
        while self._running:
            frame = self._gate.take()
            if frame is None:
                time.sleep(0.005)
                continue
            try:
                # downscale may hand back the caller's array by reference, so the
                # copy made by tobytes() is the last use of it.
                small = downscale(frame, self._cfg.width)
                if not self._ensure_dims(small):
                    continue
                buf = gst.Buffer.new_wrapped(small.tobytes())
                self._appsrc.emit('push-buffer', buf)
            except Exception as exc:    # noqa: BLE001 - keep the thread alive
                self._warn(f"preview encode failed: {exc}")
                time.sleep(0.1)

    def _ensure_dims(self, small):
        """Restart the pipeline if the real frame shape differs from the guess."""
        h, w = small.shape[:2]
        if self._dims == (w, h):
            return True
        self._info(f"preview: source is {w}x{h}, rebuilding encoder")
        tier, segment = next(t for t in _TIERS if t[0] == self._tier)
        self._teardown()
        if not self._try_tier(Gst, tier, segment, w, h):
            self._running = False
            self._warn('preview: rebuild at real dimensions failed')
            return False
        return True

    def _teardown(self):
        if self._pipeline is not None:
            self._shutdown(Gst, self._pipeline)
        self._pipeline = None
        self._appsrc = None

    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warn(msg)
