#!/usr/bin/env python3
"""Low-bitrate camera preview for the ground station.

This module is deliberately split so the part that runs on the capture thread —
frame gating — has no GStreamer dependency and can be tested exhaustively. The
encoder lives alongside it but is only exercised in integration.

The governing rule is that the capture thread is never delayed. `FrameGate.offer`
rate-limits, keeps at most one frame, and returns immediately. When the encoder is
slow the preview loses framerate; capture and detection are untouched.
"""

import ctypes
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


def pin_libgcc_unwinder():
    """Load libgcc's unwinder before anything can bring in libunwind.

    DO NOT REMOVE. Call this before ANY route that may initialise GStreamer,
    and call it as early in the process as you can. It is idempotent and costs
    one dlopen of an already-present library.

    Initialising GStreamer maps libunwind.so.8, which exports the same _Unwind_*
    symbols as libgcc_s.so.1. Whichever object lands first wins for every later
    lookup, so if libunwind arrives first, C++ exception unwinding inside
    Fast-DDS resolves _Unwind_Resume to libunwind while the frame's personality
    routine is still libgcc's __gcc_personality_v0. That mismatch calls abort().
    Fast-DDS throws and catches internally as ordinary control flow (e.g.
    UDPv4Transport::OpenInputChannel), so the process dies inside a later rclpy
    node, publisher or subscription creation with no Python traceback at all:

        from bv_core.preview_stream import PreviewConfig, PreviewStream
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start(); s.stop()
        import rclpy; rclpy.init()
        from rclpy.node import Node
        Node('probe')            # SIGABRT/SIGSEGV in __gcc_personality_v0

    Loading libgcc_s RTLD_GLOBAL first makes its unwinder authoritative and the
    repro above exits cleanly.

    Note that bare Gst.init(None) is enough to map libunwind - no encoder
    pipeline required - so this is NOT only a preview concern. OpenCV's
    cv2.VideoCapture(..., cv2.CAP_GSTREAMER) initialises GStreamer inside
    OpenCV, entirely bypassing _init_gst below, which is why callers pin
    explicitly rather than relying on this module's own use of it.
    """
    try:
        ctypes.CDLL('libgcc_s.so.1', mode=ctypes.RTLD_GLOBAL)
    except OSError:         # noqa: BLE001 - best effort; absence is not fatal
        pass


def _init_gst():
    """Import and initialise GStreamer once. Returns the Gst module or None."""
    global Gst
    if Gst is not None:
        return Gst
    try:
        pin_libgcc_unwinder()
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
        # Guards the pipeline lifecycle triple below. It is written from the
        # caller thread (start/stop) and the encoder thread (lazy build), and an
        # unguarded write can strand a PLAYING pipeline holding the hardware
        # encoder for the life of the process.
        self._lock = threading.Lock()
        self._pipeline = None
        self._appsrc = None
        self._dims = None
        self._thread = None
        self._running = False
        self._tier = None
        self._encoded = 0
        self._flow_ok = 0       # GST_FLOW_OK; real value cached once Gst loads.

    @property
    def tier(self):
        """Name of the encoder tier in use, or None before a successful start."""
        return self._tier

    def is_running(self):
        """Return True while the encoder thread is up and the pipeline is sound."""
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
        """Prove an encoder tier works, then run. Returns success.

        Probing only establishes capability: each candidate is built at a nominal
        size with the sample callback disconnected and torn down again, so no
        chunk ever escapes a probe. The pipeline that actually streams is built
        lazily on the first offered frame, at that frame's real dimensions —
        guessing the shape here would mean a teardown/rebuild on the first frame
        of every run and two conflicting fMP4 init segments at the consumer.
        """
        if self._running:
            return True

        gst = _init_gst()
        if gst is None:
            self._warn('GStreamer/PyGObject unavailable; preview disabled')
            return False
        self._flow_ok = gst.FlowReturn.OK

        width, height = self._probe_dims()
        for name, segment in _TIERS:
            if self._probe_tier(gst, name, segment, width, height):
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
        was_running = self._running
        self._running = False
        thread = self._thread
        if was_running and thread is not None:
            # Longer than the ladder's worst case (3 tiers x 3 s in get_state) so
            # a build in flight finishes before we tear down. Correctness does not
            # depend on it: a build that lands late re-checks _running under the
            # lock and shuts its own pipeline down rather than adopting it.
            thread.join(timeout=12.0)
        if thread is not None and not thread.is_alive():
            self._thread = None
        self._teardown()

    # -- internals ------------------------------------------------------------

    def _probe_dims(self):
        """Nominal, even dimensions used only to prove a tier can run."""
        width = max(2, int(self._cfg.width) - int(self._cfg.width) % 2)
        height = max(2, int(width * 3 / 4))
        return width, height - height % 2

    def _probe_tier(self, gst, name, segment, width, height):
        """Return True if this tier can actually reach PLAYING.

        Registry presence is NOT sufficient: nvh264enc is present on desktop
        machines where it fails to initialise. The probe pipeline is always torn
        down, success or failure — it proves capability and nothing more.
        """
        pipeline, _ = self._build(gst, name, segment, width, height,
                                  bytes(width * height * 3), connect=False)
        if pipeline is None:
            return False
        self._shutdown(gst, pipeline)
        return True

    def _build(self, gst, name, segment, width, height, prime, connect):
        """Build one tier at a fixed size and wait for it to reach PLAYING.

        Returns (pipeline, appsrc), or (None, None). A tier that does not reach
        PLAYING is returned to NULL before the caller tries the next, so no
        half-open pipeline keeps hold of the encoder.
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
            return None, None

        if connect:
            pipeline.get_by_name('sink').connect('new-sample', self._on_sample)

        if pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE:
            self._info(f"tier '{name}' refused PLAYING: {self._bus_error(pipeline)}")
            self._shutdown(gst, pipeline)
            return None, None

        # A priming buffer. Nothing downstream can preroll until a buffer has been
        # through the encoder, so without this the pipeline sits in PAUSED and
        # every tier -- including a working one -- looks unavailable.
        appsrc = pipeline.get_by_name('src')
        try:
            appsrc.emit('push-buffer', gst.Buffer.new_wrapped(prime))
        except Exception as exc:    # noqa: BLE001 - a broken tier lands here
            self._info(f"tier '{name}' priming push failed: {exc}")
            self._shutdown(gst, pipeline)
            return None, None

        # set_state may return ASYNC; wait for the real answer.
        state_ret, cur, _ = pipeline.get_state(3 * gst.SECOND)
        if state_ret != gst.StateChangeReturn.SUCCESS:
            self._info(f"tier '{name}' never reached PLAYING (stuck at "
                       f"{cur.value_nick}); its elements are in the registry but "
                       f"do not run here: {self._bus_error(pipeline)}")
            self._shutdown(gst, pipeline)
            return None, None

        return pipeline, appsrc

    def _build_ladder(self, width, height, prime):
        """Walk the whole tier ladder for a streaming pipeline at (width, height).

        The tier proved in start() is tried first — re-probing a tier that has
        already failed once costs seconds — but the whole ladder is available, so
        a tier that cannot handle this particular size falls through instead of
        killing the preview.

        Adopts the winner under the lock, but only if a stop() has not landed
        meanwhile; otherwise it shuts the new pipeline down itself. Returns
        success.
        """
        gst = Gst
        ladder = sorted(_TIERS, key=lambda t: t[0] != self._tier)
        for name, segment in ladder:
            pipeline, appsrc = self._build(gst, name, segment, width, height,
                                           prime, connect=True)
            if pipeline is None:
                continue
            with self._lock:
                if not self._running:
                    self._shutdown(gst, pipeline)
                    return False
                self._pipeline = pipeline
                self._appsrc = appsrc
                self._dims = (width, height)
                self._tier = name
            self._info(f"preview encoder: streaming {width}x{height} "
                       f"on tier '{name}'")
            return True

        self._warn(f'preview: no tier could encode {width}x{height}; '
                   f'preview disabled')
        self._running = False
        return False

    @staticmethod
    def _bus_error(pipeline):
        """Return the first error text on a pipeline's bus, for diagnostics."""
        try:
            msg = pipeline.get_bus().poll(Gst.MessageType.ERROR, 0)
            if msg is None:
                return 'no bus error'
            err, dbg = msg.parse_error()
            return f'{err.message} [{dbg}]'
        except Exception:       # noqa: BLE001 - diagnostics must not raise
            return 'bus unreadable'

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
        try:
            sample = sink.emit('pull-sample')
            if sample is not None:
                buf = sample.get_buffer()
                ok, info = buf.map(Gst.MapFlags.READ)
                if ok:
                    try:
                        self._encoded += 1
                        self._on_chunk(bytes(info.data))
                    except Exception as exc:    # noqa: BLE001 - consumer is not fatal
                        self._warn(f"preview chunk consumer raised: {exc}")
                    finally:
                        buf.unmap(info)
        except Exception as exc:    # noqa: BLE001 - nothing may reach GStreamer
            self._warn(f"preview sample handling failed: {exc}")
        return self._flow_ok

    def _encode_loop(self):
        while self._running:
            if not self._check_bus():
                break
            frame = self._gate.take()
            if frame is None:
                time.sleep(0.005)
                continue
            try:
                # downscale may hand back the caller's array by reference, so the
                # copy made by tobytes() is the last use of it.
                small = downscale(frame, self._cfg.width)
                height, width = small.shape[:2]
                payload = small.tobytes()
                if self._needs_build(width, height):
                    # The priming push carries this frame, so it is not sent twice.
                    self._rebuild(width, height, payload)
                    continue
                appsrc = self._appsrc
                if appsrc is not None:
                    appsrc.emit('push-buffer', Gst.Buffer.new_wrapped(payload))
            except Exception as exc:    # noqa: BLE001 - keep the thread alive
                self._warn(f"preview encode failed: {exc}")
                time.sleep(0.1)

    def _needs_build(self, width, height):
        """True when there is no pipeline, or it is the wrong shape."""
        with self._lock:
            return self._pipeline is None or self._dims != (width, height)

    def _rebuild(self, width, height, payload):
        """Build (or rebuild) the streaming pipeline at real frame dimensions."""
        with self._lock:
            known = self._dims
        if known is not None:
            self._info(f"preview: source changed to {width}x{height}, rebuilding")
        self._teardown()
        return self._build_ladder(width, height, payload)

    def _check_bus(self):
        """Watch for a pipeline that dies after start. False means it did.

        Without this a fault after PLAYING -- encoder error, device removed,
        failed renegotiation -- leaves is_running() lying while chunks silently
        stop.
        """
        pipeline = self._pipeline
        if pipeline is None:
            return True
        try:
            msg = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR
                                                  | Gst.MessageType.EOS)
        except Exception:       # noqa: BLE001 - never fatal
            return True
        if msg is None:
            return True
        if msg.type == Gst.MessageType.EOS:
            self._warn('preview pipeline ended (EOS); preview stopped')
        else:
            err, dbg = msg.parse_error()
            self._warn(f'preview pipeline error: {err.message} [{dbg}]; '
                       f'preview stopped')
        self._running = False
        self._teardown()
        return False

    def _teardown(self):
        with self._lock:
            pipeline, self._pipeline = self._pipeline, None
            self._appsrc = None
            self._dims = None
        if pipeline is not None:
            self._shutdown(Gst, pipeline)

    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warn(msg)
