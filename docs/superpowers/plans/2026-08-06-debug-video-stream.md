# Debug Video Stream Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A low-bitrate live camera preview in the `bv_gcs` web UI, with detection markers overlaid, for debugging what the vision system sees in flight.

**Architecture:** A hook in `VisionPipeline._enqueue_frame` offers each captured frame to a `PreviewStream`. That downscales in Python, encodes H.264 through GStreamer (hardware on Jetson), muxes fragmented MP4, and publishes chunks on a ROS topic. `approval_node` relays them to the browser over a **dedicated** WebSocket, kept separate from the control socket so video congestion can never delay an approval verdict. Detections travel separately as JSON and are drawn as a canvas overlay.

**Tech Stack:** ROS 2 Humble, `gi.repository.Gst` (PyGObject), GStreamer 1.20, numpy, OpenCV (resize only), aiohttp, React + TypeScript, Media Source Extensions.

**Spec:** `docs/superpowers/specs/2026-08-06-debug-video-stream-design.md`

## Global Constraints

- Python 3.10, ROS 2 Humble, `numpy<2`. No new Python packages — PyGObject and GStreamer 1.20 are already installed and verified on both the dev PC and the Jetson.
- Line length ≤ 99 chars (`test/test_flake8.py`).
- **The Jetson is the deployment target.** Tier 1 (`nvv4l2h264enc`) is the shipping path; tiers 2–3 exist only so the feature can be developed against sim and must not be optimised for.
- **`mission.py`, `approval_gate.py` and `filtering_node.py` must not be modified.** Zero lines. If a task seems to need it, stop and report.
- **Disabled cost is one branch per frame.** When the toggle is off, `_enqueue_frame` must short-circuit before any work.
- **Never block the capture thread.** `offer()` returns immediately, always. It must never raise, never allocate unboundedly, and never wait on the encoder.
- **Video sends drop, never queue.** A slow link loses frames; it does not build a backlog.
- **Python does the downscaling, the pipeline does not.** Pushing a full 4640×3480 BGR frame into `appsrc` would memcpy 48 MB per frame (384 MB/s at 8 fps). Python decimates and resizes to the exact target first; the GStreamer tiers then differ only in their colour-convert and encoder elements. (The spec's Components section describes `nvvidconv` scaling; it converts colour only. Same result, far less memory traffic.)
- Tests run from the package dir: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/ -v --ignore=test/test_flake8.py --ignore=test/test_pep257.py`. Baseline is **120 passed, 1 skipped**.
- The repo's `test_flake8.py` / `test_pep257.py` carry ~398 pre-existing failures package-wide. Not yours. Verify your own files with a direct per-file `flake8 --max-line-length=99`.
- Two repos: `bv_core` on `main`, `bv_gcs` on `main`. Commit to the branch each is already on. Do not create branches.

---

### Task 1: Frame gating and downscaling

Pure numpy/cv2. No ROS, no GStreamer, no threads. This is the logic that protects the capture thread, so it is tested hardest.

**Files:**
- Create: `bv_core/preview_stream.py`
- Test: `test/test_preview_stream.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `decimation_factor(src_width: int, target_width: int) -> int`
  - `downscale(frame: np.ndarray, target_width: int) -> np.ndarray`
  - `FrameGate(fps: float)` with `.offer(frame) -> bool`, `.take() -> np.ndarray | None`, `.stats -> dict`

- [ ] **Step 1: Write the failing tests**

Create `test/test_preview_stream.py`:

```python
#!/usr/bin/env python3
"""Tests for preview_stream frame gating — pure numpy, no ROS or GStreamer."""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.preview_stream import (  # noqa: E402
    FrameGate,
    decimation_factor,
    downscale,
)


class TestDecimationFactor:
    def test_real_camera_to_target(self):
        # 4640 wide -> 1024 target. 4640 // 1024 == 4.
        assert decimation_factor(4640, 1024) == 4

    def test_sim_camera_to_target(self):
        assert decimation_factor(1280, 1024) == 1

    def test_source_narrower_than_target_never_decimates(self):
        assert decimation_factor(640, 1024) == 1

    def test_equal_widths(self):
        assert decimation_factor(1024, 1024) == 1

    def test_never_returns_zero(self):
        assert decimation_factor(1, 1024) == 1
        assert decimation_factor(0, 1024) == 1


class TestDownscale:
    def test_real_camera_frame_reaches_target_width(self):
        frame = np.zeros((3480, 4640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 1024

    def test_aspect_ratio_is_preserved(self):
        # 4640x3480 is 4:3, so 1024 wide -> 768 high.
        frame = np.zeros((3480, 4640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[0] == 768

    def test_sixteen_by_nine_source(self):
        # Gazebo is 1280x720 -> 1024x576.
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 1024
        assert out.shape[0] == 576

    def test_output_dimensions_are_even(self):
        # H.264 encoders reject odd dimensions.
        frame = np.zeros((1001, 1333, 3), dtype=np.uint8)
        out = downscale(frame, 641)
        assert out.shape[0] % 2 == 0
        assert out.shape[1] % 2 == 0

    def test_source_smaller_than_target_is_returned_unscaled(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        out = downscale(frame, 1024)
        assert out.shape[1] == 640

    def test_does_not_mutate_the_source(self):
        frame = np.full((720, 1280, 3), 7, dtype=np.uint8)
        before = frame.copy()
        downscale(frame, 1024)
        assert np.array_equal(frame, before)

    def test_content_survives(self):
        # A frame that is entirely one colour must stay that colour.
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        out = downscale(frame, 1024)
        assert abs(int(out.mean()) - 200) < 3


class TestFrameGate:
    def test_first_offer_is_accepted(self):
        gate = FrameGate(fps=8.0)
        assert gate.offer(np.zeros((4, 4, 3), dtype=np.uint8)) is True

    def test_second_offer_within_the_interval_is_rejected(self):
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        assert gate.offer(frame) is True
        assert gate.offer(frame) is False      # 1/8s has not elapsed

    def test_offer_after_the_interval_is_accepted(self):
        gate = FrameGate(fps=50.0)             # 20ms interval
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        assert gate.offer(frame) is True
        time.sleep(0.03)
        assert gate.offer(frame) is True

    def test_rate_limiting_thins_a_fast_source(self):
        # 24 offers as fast as possible at fps=8 should accept far fewer.
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        accepted = sum(gate.offer(frame) for _ in range(24))
        assert accepted == 1                   # all within one interval

    def test_take_returns_the_latest_and_empties_the_slot(self):
        gate = FrameGate(fps=0.0)              # 0 disables rate limiting
        a = np.full((4, 4, 3), 1, dtype=np.uint8)
        b = np.full((4, 4, 3), 2, dtype=np.uint8)
        gate.offer(a)
        gate.offer(b)
        got = gate.take()
        assert got is not None and got[0, 0, 0] == 2   # latest wins
        assert gate.take() is None                     # slot now empty

    def test_take_on_empty_gate_returns_none(self):
        assert FrameGate(fps=8.0).take() is None

    def test_stats_count_accepted_and_dropped(self):
        gate = FrameGate(fps=8.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        for _ in range(5):
            gate.offer(frame)
        s = gate.stats
        assert s['offered'] == 5
        assert s['accepted'] == 1
        assert s['dropped'] == 4

    def test_overwriting_an_untaken_frame_counts_as_a_drop(self):
        gate = FrameGate(fps=0.0)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        gate.offer(frame)
        gate.offer(frame)          # overwrites the first, never encoded
        assert gate.stats['dropped'] == 1

    def test_offer_never_raises_on_garbage(self):
        gate = FrameGate(fps=0.0)
        assert gate.offer(None) is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_stream.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bv_core.preview_stream'`

- [ ] **Step 3: Implement**

Create `bv_core/preview_stream.py`:

```python
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

import cv2
import numpy as np


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
        """
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_stream.py -v`
Expected: PASS (21 tests)

- [ ] **Step 5: Lint**

Run: `cd ~/bv_ws/src/bv_core && python3 -m flake8 --max-line-length=99 bv_core/preview_stream.py test/test_preview_stream.py`
Expected: no output

- [ ] **Step 6: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/preview_stream.py test/test_preview_stream.py
git commit -m "feat: frame gating and downscaling for the debug preview

FrameGate is the piece that protects the capture thread: rate-limited,
latest-wins, single slot, never blocks and never raises. Downscaling decimates
with a stride before resizing, because reading every pixel of a 48 MB frame is
the dominant per-frame cost, not the encode."
```

---

### Task 2: GStreamer encoder with tier probing

**Files:**
- Modify: `bv_core/preview_stream.py`
- Test: `test/test_preview_encoder.py`

**Interfaces:**
- Consumes: `FrameGate`, `downscale` (Task 1).
- Produces:
  - `PreviewConfig(width: int = 640, fps: float = 8.0, bitrate_bps: int = 400_000)`
  - `PreviewStream(cfg: PreviewConfig, on_chunk: Callable[[bytes], None], logger=None)`
    with `.start() -> bool`, `.stop() -> None`, `.is_running() -> bool`,
    `.offer(frame) -> None`, `.stats() -> dict`, `.tier -> str | None`

- [ ] **Step 1: Write the failing tests**

Create `test/test_preview_encoder.py`:

```python
#!/usr/bin/env python3
"""Integration tests for the preview encoder.

These exercise real GStreamer. They skip cleanly where it is unavailable so the
suite still passes on a machine without it.
"""

import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.preview_stream import PreviewConfig, PreviewStream  # noqa: E402

gst = pytest.importorskip('gi', reason='PyGObject not installed')


def _frame(w=1280, h=720):
    rng = np.random.default_rng(3)
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


class TestEncoder:
    def test_start_selects_a_tier_and_reports_running(self):
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            assert s.is_running() is True
            assert s.tier in ('jetson', 'nvidia', 'software')
        finally:
            s.stop()

    def test_offered_frames_produce_encoded_chunks(self):
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        assert s.start() is True
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            deadline = time.time() + 5
            while not chunks and time.time() < deadline:
                time.sleep(0.05)
        finally:
            s.stop()
        assert chunks, 'encoder produced no output'
        assert all(isinstance(c, (bytes, bytearray)) for c in chunks)

    def test_first_chunk_is_an_mp4_init_segment(self):
        # fMP4 starts with an ftyp box; the browser needs it to initialise MSE.
        chunks = []
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), chunks.append)
        s.start()
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            deadline = time.time() + 5
            while not chunks and time.time() < deadline:
                time.sleep(0.05)
        finally:
            s.stop()
        assert b'ftyp' in bytes(chunks[0])

    def test_offer_before_start_is_a_noop(self):
        s = PreviewStream(PreviewConfig(), lambda c: None)
        s.offer(_frame())            # must not raise
        assert s.is_running() is False

    def test_offer_after_stop_is_a_noop(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        s.stop()
        s.offer(_frame())            # must not raise
        assert s.is_running() is False

    def test_stop_is_idempotent(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        s.stop()
        s.stop()                     # must not raise

    def test_a_raising_callback_does_not_kill_the_encoder(self):
        def boom(_chunk):
            raise RuntimeError('downstream exploded')

        s = PreviewStream(PreviewConfig(width=640, fps=0.0), boom)
        assert s.start() is True
        try:
            for _ in range(12):
                s.offer(_frame())
                time.sleep(0.02)
            time.sleep(0.5)
            assert s.is_running() is True     # survived
        finally:
            s.stop()

    def test_stats_report_offered_and_encoded(self):
        s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
        s.start()
        try:
            for _ in range(6):
                s.offer(_frame())
                time.sleep(0.02)
            time.sleep(0.5)
            st = s.stats()
            assert st['offered'] == 6
            assert 'encoded' in st
            assert 'tier' in st
        finally:
            s.stop()
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_encoder.py -v`
Expected: FAIL — `ImportError: cannot import name 'PreviewConfig'`

- [ ] **Step 3: Implement the encoder**

Append to `bv_core/preview_stream.py`:

```python
from dataclasses import dataclass

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
        """
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
        return self._tier

    def is_running(self):
        return self._running

    def stats(self):
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
        PLAYING is accepted.
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
            pipeline.set_state(gst.State.NULL)
            return False

        # set_state may return ASYNC; wait for the real answer.
        state_ret, _, _ = pipeline.get_state(3 * gst.SECOND)
        if state_ret != gst.StateChangeReturn.SUCCESS:
            pipeline.set_state(gst.State.NULL)
            return False

        self._pipeline = pipeline
        self._appsrc = pipeline.get_by_name('src')
        self._dims = (width, height)
        return True

    def _on_sample(self, sink):
        """appsink callback: hand the encoded chunk to the consumer."""
        gst = Gst
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
        return gst.FlowReturn.OK

    def _encode_loop(self):
        gst = Gst
        while self._running:
            frame = self._gate.take()
            if frame is None:
                time.sleep(0.005)
                continue
            try:
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
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:       # noqa: BLE001
                pass
        self._pipeline = None
        self._appsrc = None

    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warn(msg)
```

Add `from dataclasses import dataclass` to the imports at the top of the file rather than mid-file, and keep `import threading`, `import time`, `import cv2`, `import numpy as np` from Task 1.

- [ ] **Step 4: Run both test files**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_stream.py test/test_preview_encoder.py -v`
Expected: PASS (21 + 8 = 29). On this dev PC the tier will be `software` — `nvh264enc` is present but fails to initialise under WSL2, which is exactly the case `_try_tier`'s PLAYING check exists to catch.

- [ ] **Step 5: Verify the tier ladder actually falls through**

Run:
```bash
cd ~/bv_ws/src/bv_core && python3 -c "
import sys; sys.path.insert(0,'.')
from bv_core.preview_stream import PreviewStream, PreviewConfig
class L:
    def info(self,m): print(' ', m)
    def warn(self,m): print(' WARN', m)
s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None, L())
print('started:', s.start(), '-> tier', s.tier)
s.stop()
"
```
Expected: log lines showing `jetson` and `nvidia` unavailable, then `software` selected. This is the evidence that the fallback works; capture it in your report.

- [ ] **Step 6: Lint and commit**

```bash
cd ~/bv_ws/src/bv_core
python3 -m flake8 --max-line-length=99 bv_core/preview_stream.py test/test_preview_encoder.py
git add bv_core/preview_stream.py test/test_preview_encoder.py
git commit -m "feat: H.264 preview encoder with tier probing

Walks nvv4l2h264enc (Jetson, the shipping path) then nvh264enc then x264enc,
accepting the first that actually reaches PLAYING. Registry presence is not
enough: nvh264enc is present on desktop machines where it fails to initialise.
The selected tier is logged, so a Jetson silently falling back to software is
visible rather than showing up as an unexplained hot CPU."
```

---

### Task 3: Wire the preview into vision_node

**Files:**
- Modify: `bv_core/pipelines/Vision_Pipeline.py`
- Modify: `bv_core/vision_node.py`
- Modify: `config/vision_params.yaml`
- Test: `test/test_preview_hook.py`

**Interfaces:**
- Consumes: `PreviewStream`, `PreviewConfig` (Task 2).
- Produces: `/preview_stream` (`std_msgs/UInt8MultiArray`, BEST_EFFORT depth 2), consumed by Task 4. Subscribes `/preview_enabled` (`std_msgs/Bool`).

- [ ] **Step 1: Write the failing test for the hook**

Create `test/test_preview_hook.py`:

```python
#!/usr/bin/env python3
"""The preview hook must be free when disabled and must never break capture."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.pipelines.Vision_Pipeline import VisionPipeline  # noqa: E402


class _Pipe(VisionPipeline):
    """Concrete stand-in; the base class is abstract."""

    def start(self):
        pass

    def stop(self):
        pass


class _RecordingPreview:
    def __init__(self, explode=False):
        self.seen = []
        self._explode = explode

    def offer(self, frame):
        if self._explode:
            raise RuntimeError('preview blew up')
        self.seen.append(frame)


class TestPreviewHook:
    def test_frames_still_reach_the_queue_with_no_preview(self):
        pipe = _Pipe(max_queue_size=2)
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        pipe._enqueue_frame(frame)
        assert pipe.get_frame(timeout=0.1) is not None

    def test_preview_receives_offered_frames(self):
        pipe = _Pipe(max_queue_size=2)
        preview = _RecordingPreview()
        pipe.set_preview(preview)
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert len(preview.seen) == 1

    def test_detection_queue_still_fed_when_preview_attached(self):
        pipe = _Pipe(max_queue_size=2)
        pipe.set_preview(_RecordingPreview())
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert pipe.get_frame(timeout=0.1) is not None

    def test_a_raising_preview_does_not_break_capture(self):
        # This is the whole point: the debug feature must never cost a frame.
        pipe = _Pipe(max_queue_size=2)
        pipe.set_preview(_RecordingPreview(explode=True))
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert pipe.get_frame(timeout=0.1) is not None

    def test_set_preview_none_detaches(self):
        pipe = _Pipe(max_queue_size=2)
        preview = _RecordingPreview()
        pipe.set_preview(preview)
        pipe.set_preview(None)
        pipe._enqueue_frame(np.zeros((4, 4, 3), dtype=np.uint8))
        assert preview.seen == []
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_hook.py -v`
Expected: FAIL — `AttributeError: '_Pipe' object has no attribute 'set_preview'`

- [ ] **Step 3: Add the hook**

In `bv_core/pipelines/Vision_Pipeline.py`, add to `__init__`:

```python
        self._preview = None
```

Add the setter after `__init__`:

```python
    def set_preview(self, preview):
        """Attach a preview sink, or None to detach.

        The sink must expose offer(frame) and must not block.
        """
        self._preview = preview
```

And change `_enqueue_frame` to:

```python
    def _enqueue_frame(self, frame):
        """Push a new frame into the bounded queue, dropping the oldest if full"""
        # Debug preview. When detached this is a single comparison; when attached,
        # offer() is non-blocking. Wrapped because a debug feature must never cost
        # the detection path a frame.
        if self._preview is not None:
            try:
                self._preview.offer(frame)
            except Exception:       # noqa: BLE001
                pass

        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass

        self._frame_queue.put_nowait(frame)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_preview_hook.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Add config keys**

Append to `config/vision_params.yaml`:

```yaml
# ============================================================
# Debug preview stream (GCS live feed)
# Off until the operator toggles it on in the ground station. See
# docs/superpowers/specs/2026-08-06-debug-video-stream-design.md
# ============================================================
# Deliberately conservative: the Herelink budget is unknown and this is a debug
# feed, not a camera downlink. Raise once the link has actually been measured.
preview_width: 640           # output width; height follows source aspect
preview_fps: 8.0             # decimation target — the knob for CPU and bitrate
preview_bitrate_bps: 400000  # encoder target bitrate
```

- [ ] **Step 6: Wire vision_node**

Add to the imports in `bv_core/vision_node.py`, next to the other local imports:

```python
from .preview_stream import PreviewConfig, PreviewStream
```

Add to the config loader, after `self.capture_interval` is read:

```python
        # Debug preview stream
        self.preview_cfg = PreviewConfig(
            width=int(cfg.get('preview_width', 640)),
            fps=float(cfg.get('preview_fps', 8.0)),
            bitrate_bps=int(cfg.get('preview_bitrate_bps', 400_000)),
        )
```

`vision_node.py` already has `from std_msgs.msg import String, Int8, Float64` — **extend
that existing line** rather than adding a second import from the same module:

```python
from std_msgs.msg import String, Int8, Float64, Bool, UInt8MultiArray
```

In `_init_pipeline`, after `self.pipeline_running = False`, add:

```python
        # Debug preview: constructed dormant. Nothing runs until the operator
        # toggles it on, and the hook costs one comparison per frame until then.
        preview_qos = QoSProfile(depth=2)
        preview_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        preview_qos.history = HistoryPolicy.KEEP_LAST
        self.preview_pub = self.create_publisher(
            UInt8MultiArray, '/preview_stream', preview_qos)

        self.preview = PreviewStream(
            self.preview_cfg, self._publish_preview_chunk, self.get_logger())

        self.create_subscription(
            Bool, '/preview_enabled', self._on_preview_toggle, 10)
```

Add the two methods:

```python
    def _publish_preview_chunk(self, chunk):
        """Publish one fMP4 chunk. Called from the encoder thread."""
        msg = UInt8MultiArray()
        msg.data = chunk
        self.preview_pub.publish(msg)

    def _on_preview_toggle(self, msg):
        """Operator turned the debug preview on or off."""
        if msg.data:
            if self.preview.start():
                self.pipeline.set_preview(self.preview)
                self.get_logger().info('debug preview ON')
            else:
                self.get_logger().warn('debug preview requested but unavailable')
        else:
            self.pipeline.set_preview(None)
            self.preview.stop()
            self.get_logger().info('debug preview OFF')
```

Ensure `QoSProfile`, `ReliabilityPolicy` and `HistoryPolicy` are imported — `vision_node.py` already imports from `rclpy.qos`; add any that are missing.

- [ ] **Step 7: Build and verify nothing regressed**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
python3 -c "import bv_core.vision_node; print('vision_node imports OK')"
cd src/bv_core && python3 -m pytest test/ -q --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```
Expected: imports OK, and 120 baseline + 34 new = **154 passed, 1 skipped**.

- [ ] **Step 8: Verify the toggle end to end in sim**

Run the sim as usual, then in another terminal:
```bash
ros2 topic pub --once /preview_enabled std_msgs/msg/Bool "{data: true}"
ros2 topic hz /preview_stream
```
Expected: `vision_node` logs `preview encoder: tier '...'` and `debug preview ON`, and `/preview_stream` shows traffic while the mission is in scan or localize. Then:
```bash
ros2 topic pub --once /preview_enabled std_msgs/msg/Bool "{data: false}"
```
Expected: `debug preview OFF` and `/preview_stream` goes silent.

- [ ] **Step 9: Lint and commit**

```bash
cd ~/bv_ws/src/bv_core
python3 -m flake8 --max-line-length=99 bv_core/pipelines/Vision_Pipeline.py test/test_preview_hook.py
git add bv_core/pipelines/Vision_Pipeline.py bv_core/vision_node.py \
        config/vision_params.yaml test/test_preview_hook.py
git commit -m "feat: publish a debug preview stream from vision_node

The hook goes in VisionPipeline._enqueue_frame, which every pipeline funnels
through, so sim, real camera and rosbag replay all get the stream with no
per-pipeline code. Disabled it is one comparison per frame. The offer is wrapped
because a debug feature must never cost the detection path a frame."
```

---

### Task 4: Relay the stream in approval_node

**Files:**
- Modify: `../bv_gcs/bv_gcs/approval_node.py`

**Interfaces:**
- Consumes: `/preview_stream` (Task 3).
- Produces:
  - `GET /video` — binary WebSocket carrying fMP4 chunks
  - WS control message in: `{"type": "preview", "enabled": bool}` → publishes `/preview_enabled`
  - WS control message out: `{"type":"detections","dets":[{"x":float,"y":float,"class_id":int,"class_name":str}]}`

- [ ] **Step 1: Add the ROS side**

In `ApprovalNode.__init__`, after the existing subscriptions, add:

```python
        preview_qos = QoSProfile(depth=2)
        preview_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        preview_qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(
            UInt8MultiArray, '/preview_stream', self._on_preview_chunk,
            preview_qos, callback_group=self._cb_group)
        self.create_subscription(
            ObjectDetections, '/obj_dets', self._on_detections, best_effort,
            callback_group=self._cb_group)

        self.preview_pub = self.create_publisher(Bool, '/preview_enabled', latched)

        # Set by GcsServer, like `emit`.
        self.emit_video = None
```

Add the imports:

```python
from bv_msgs.msg import ObjectDetections, PendingDetection
from std_msgs.msg import Bool, String, UInt8MultiArray
```

Add the callbacks:

```python
    def _on_preview_chunk(self, msg):
        """Forward one fMP4 chunk to the video socket."""
        if self.emit_video is not None:
            self.emit_video(bytes(msg.data))

    def _on_detections(self, msg):
        """Forward detection centres, normalised to 0-1 so the browser never
        needs to know the sensor resolution.

        Width and height are configured separately rather than assuming an aspect
        ratio: the real camera is 4640x3480 (4:3) and Gazebo is 1280x720 (16:9),
        so deriving height from width would misplace every marker vertically in
        whichever environment did not match the assumption.
        """
        width = float(self.get_parameter('detection_source_width').value)
        height = float(self.get_parameter('detection_source_height').value)
        dets = []
        for v in msg.dets:
            dets.append({
                'x': float(v.x) / width,
                'y': float(v.y) / height,
                'class_id': int(v.z),
                'class_name': class_name(int(v.z)),
            })
        self._emit({'type': 'detections', 'dets': dets})

    def set_preview_enabled(self, enabled):
        """Publish the operator's toggle to vision_node."""
        self.preview_pub.publish(Bool(data=bool(enabled)))
        self.get_logger().info(f"debug preview {'ON' if enabled else 'OFF'}")
```

Add the source-dimension parameters in `__init__` alongside the others, since detection
coordinates arrive in full-sensor pixels:

```python
        # Detection centres arrive in source-sensor pixels. Defaults match the real
        # camera; config/approval_params.yaml overrides them for Gazebo (1280x720).
        self.declare_parameter('detection_source_width', 4640)
        self.declare_parameter('detection_source_height', 3480)
```

And add both to `config/approval_params.yaml` so the sim case is a config change
rather than a code change:

```yaml
    # Source frame dimensions that /obj_dets centres are expressed in. Real camera
    # is 4640x3480; set to 1280x720 when running against Gazebo, or markers will
    # land in the wrong place on the video overlay.
    detection_source_width: 4640
    detection_source_height: 3480
```

- [ ] **Step 2: Add the video socket to GcsServer**

In `GcsServer.__init__`, add:

```python
        self.video_clients: set[web.WebSocketResponse] = set()
```

Add the broadcast path and handler:

```python
    def emit_video_threadsafe(self, chunk: bytes):
        """Called from the ROS executor thread."""
        if self.loop is None or self.loop.is_closed() or not self.video_clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_video(chunk), self.loop)

    async def _broadcast_video(self, chunk: bytes):
        """Send to video clients, dropping rather than queueing.

        Video is expendable and a verdict is not, so a client whose transport is
        already backed up loses this chunk instead of delaying everything behind
        it. The next keyframe recovers the picture.
        """
        for ws in list(self.video_clients):
            try:
                if ws.closed:
                    self.video_clients.discard(ws)
                    continue
                # transport buffer already full -> skip rather than await
                transport = getattr(ws, '_writer', None)
                if transport is not None and transport.transport is not None:
                    if transport.transport.get_write_buffer_size() > 512_000:
                        continue
                await ws.send_bytes(chunk)
            except (ConnectionResetError, RuntimeError):
                self.video_clients.discard(ws)

    async def video_handler(self, request: web.Request) -> web.WebSocketResponse:
        """Dedicated video socket, separate from /ws.

        Sharing the control socket would let a backlog of H.264 delay an approval
        verdict. The aircraft would still be safe — mission_node owns the timeout —
        but the operator would be clicking Approve into a stalled pipe.
        """
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=0)
        await ws.prepare(request)
        self.video_clients.add(ws)
        self.node.get_logger().info(
            f"video client connected ({len(self.video_clients)} total)")
        try:
            async for _ in ws:
                pass          # clients never send on this socket
        finally:
            self.video_clients.discard(ws)
            self.node.get_logger().info(
                f"video client disconnected ({len(self.video_clients)} left)")
        return ws
```

Register the route in `build_app`, before the static handler:

```python
        app.router.add_get('/video', self.video_handler)
```

Wire the emitter in `run()`, next to `self.node.emit = self.emit_threadsafe`:

```python
        self.node.emit_video = self.emit_video_threadsafe
```

- [ ] **Step 3: Handle the toggle message**

In `ws_handler`'s message loop, alongside the existing `decision` branch:

```python
                if data.get('type') == 'decision':
                    await self._handle_decision(ws, data)
                elif data.get('type') == 'preview':
                    self.node.set_preview_enabled(bool(data.get('enabled')))
                    await ws.send_str(json.dumps({
                        'type': 'preview_state',
                        'enabled': bool(data.get('enabled')),
                    }))
```

- [ ] **Step 4: Build and smoke-test**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_gcs && source install/setup.bash
ros2 run bv_gcs approval_node &
sleep 3
curl -s -o /dev/null -w "GET /healthz -> %{http_code}\n" http://127.0.0.1:8765/healthz
python3 -c "
import asyncio, aiohttp
async def main():
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect('http://127.0.0.1:8765/video') as ws:
            print('video socket connected OK')
        async with s.ws_connect('http://127.0.0.1:8765/ws') as ws:
            await ws.send_str('{\"type\":\"preview\",\"enabled\":true}')
            msg = await asyncio.wait_for(ws.receive(), 5)
            print('control socket reply:', msg.data)
asyncio.run(main())
"
ros2 topic echo /preview_enabled --once
```
Expected: `/healthz -> 200`, both sockets connect, the control socket replies with
`preview_state`, and `/preview_enabled` shows `data: true`. Kill the node afterwards.

- [ ] **Step 5: Lint and commit**

```bash
cd ~/bv_ws/src/bv_gcs
python3 -m flake8 --max-line-length=99 bv_gcs/approval_node.py
git add bv_gcs/approval_node.py
git commit -m "feat: relay the debug video stream on a dedicated socket

Video gets its own /video endpoint, separate from control on /ws. Sharing one
socket would let a backlog of H.264 delay an approval verdict — the aircraft
would still be safe since mission_node owns the timeout, but the operator would
be clicking Approve into a stalled pipe. Video sends drop rather than queue."
```

---

### Task 5: Video panel and layout in the GCS

**Files:**
- Create: `../bv_gcs/web/src/net/videoClient.ts`
- Create: `../bv_gcs/web/src/components/VideoPanel.tsx`
- Create: `../bv_gcs/web/src/components/StreamToggle.tsx`
- Modify: `../bv_gcs/web/src/net/types.ts`
- Modify: `../bv_gcs/web/src/net/client.ts`
- Modify: `../bv_gcs/web/src/store/useGcsStore.ts`
- Modify: `../bv_gcs/web/src/App.tsx`

**Interfaces:**
- Consumes: `GET /video`, the `preview` / `preview_state` / `detections` messages (Task 4).
- Produces: nothing consumed downstream.

- [ ] **Step 1: Add types**

Append to `web/src/net/types.ts`:

```ts
export interface DetectionMarker {
  /** 0-1, normalised against the source frame. */
  x: number;
  y: number;
  class_id: number;
  class_name: string;
}

export type StreamState = 'off' | 'connecting' | 'live';
```

And extend `ServerMessage` with:

```ts
  | { type: 'detections'; dets: DetectionMarker[] }
  | { type: 'preview_state'; enabled: boolean }
```

- [ ] **Step 2: Add store fields**

In `web/src/store/useGcsStore.ts`, add to the interface and the initial state:

```ts
  previewEnabled: boolean;
  streamState: StreamState;
  detections: DetectionMarker[];

  setPreviewEnabled: (v: boolean) => void;
  setStreamState: (s: StreamState) => void;
  setDetections: (d: DetectionMarker[]) => void;
```

```ts
  previewEnabled: false,
  streamState: 'off',
  detections: [],

  setPreviewEnabled: (v) => set({ previewEnabled: v }),
  setStreamState: (s) => set({ streamState: s }),
  setDetections: (d) => set({ detections: d }),
```

Import `DetectionMarker` and `StreamState` from `../net/types`.

- [ ] **Step 3: Handle the new control messages**

In `web/src/net/client.ts`, add to the `switch (msg.type)` block:

```ts
    case 'detections':
      store.setDetections(msg.dets);
      break;
    case 'preview_state':
      store.setPreviewEnabled(msg.enabled);
      break;
```

And export the toggle sender:

```ts
export function setPreview(enabled: boolean) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ type: 'preview', enabled }));
  }
}
```

- [ ] **Step 4: Write the video client**

Create `web/src/net/videoClient.ts`:

```ts
import { useGcsStore } from '../store/useGcsStore';

/**
 * Video rides its own WebSocket, deliberately separate from the control socket
 * in net/client.ts. A backlog of H.264 must never delay an approval verdict.
 */
const VIDEO_PATH = '/video';
const MIME = 'video/mp4; codecs="avc1.42E01E"';

let socket: WebSocket | null = null;
let mediaSource: MediaSource | null = null;
let sourceBuffer: SourceBuffer | null = null;
const pending: ArrayBuffer[] = [];

function videoUrl(): string {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${window.location.host}${VIDEO_PATH}`;
}

function drain() {
  if (!sourceBuffer || sourceBuffer.updating || pending.length === 0) return;
  try {
    sourceBuffer.appendBuffer(pending.shift()!);
  } catch {
    // QuotaExceeded or a bad segment — drop what we have and resync.
    pending.length = 0;
  }
}

/** Trim played-back data and jump forward if we have fallen behind live. */
function keepLive(video: HTMLVideoElement) {
  if (!sourceBuffer || sourceBuffer.updating) return;
  const buffered = sourceBuffer.buffered;
  if (buffered.length === 0) return;
  const end = buffered.end(buffered.length - 1);
  if (end - video.currentTime > 2) video.currentTime = end - 0.1;
  const start = buffered.start(0);
  if (video.currentTime - start > 8) {
    try {
      sourceBuffer.remove(start, video.currentTime - 4);
    } catch {
      /* removal is best-effort */
    }
  }
}

export function startVideo(video: HTMLVideoElement) {
  stopVideo();
  useGcsStore.getState().setStreamState('connecting');

  mediaSource = new MediaSource();
  video.src = URL.createObjectURL(mediaSource);

  mediaSource.addEventListener('sourceopen', () => {
    if (!mediaSource) return;
    sourceBuffer = mediaSource.addSourceBuffer(MIME);
    sourceBuffer.mode = 'sequence';
    sourceBuffer.addEventListener('updateend', () => {
      drain();
      keepLive(video);
    });
    drain();
  });

  socket = new WebSocket(videoUrl());
  socket.binaryType = 'arraybuffer';
  socket.onmessage = (ev) => {
    if (!(ev.data instanceof ArrayBuffer)) return;
    if (useGcsStore.getState().streamState !== 'live') {
      useGcsStore.getState().setStreamState('live');
    }
    pending.push(ev.data);
    if (pending.length > 60) pending.splice(0, pending.length - 30);
    drain();
  };
  socket.onclose = () => useGcsStore.getState().setStreamState('off');
  socket.onerror = () => socket?.close();
}

export function stopVideo() {
  socket?.close();
  socket = null;
  sourceBuffer = null;
  mediaSource = null;
  pending.length = 0;
  useGcsStore.getState().setStreamState('off');
}
```

- [ ] **Step 5: Write the toggle and the panel**

Create `web/src/components/StreamToggle.tsx`:

```tsx
import { setPreview } from '../net/client';
import { startVideo, stopVideo } from '../net/videoClient';
import { useGcsStore } from '../store/useGcsStore';
import { Button } from './ui/Button';

export function StreamToggle({ videoRef }: {
  videoRef: React.RefObject<HTMLVideoElement>;
}) {
  const enabled = useGcsStore((s) => s.previewEnabled);
  const state = useGcsStore((s) => s.streamState);

  const toggle = () => {
    const next = !enabled;
    setPreview(next);
    if (next && videoRef.current) startVideo(videoRef.current);
    else stopVideo();
  };

  return (
    <section className="border border-bg-border bg-bg-panel p-4 space-y-2">
      <div className="font-mono text-[10px] uppercase tracking-[0.2em] text-ink-dim">
        Debug stream
      </div>
      <Button variant={enabled ? 'reject' : 'ghost'} className="w-full"
              onClick={toggle}>
        {enabled ? 'Stop stream' : 'Start stream'}
      </Button>
      <div className="font-mono text-[10px] text-ink-dim">{state}</div>
    </section>
  );
}
```

Create `web/src/components/VideoPanel.tsx`:

```tsx
import { useEffect, useRef } from 'react';
import { useGcsStore } from '../store/useGcsStore';

/**
 * Live feed with detection markers drawn on a canvas overlay.
 *
 * Markers are drawn browser-side rather than baked into the video: detection
 * runs at 0.67 Hz while video runs at 8 fps, so compositing them server-side
 * would make the video stutter down to the detector's rate. They persist between
 * updates instead.
 */
export function VideoPanel({ videoRef }: {
  videoRef: React.RefObject<HTMLVideoElement>;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const dets = useGcsStore((s) => s.detections);
  const state = useGcsStore((s) => s.streamState);

  useEffect(() => {
    const canvas = canvasRef.current;
    const video = videoRef.current;
    if (!canvas || !video) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    canvas.width = video.clientWidth;
    canvas.height = video.clientHeight;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    for (const d of dets) {
      const x = d.x * canvas.width;
      const y = d.y * canvas.height;
      ctx.strokeStyle = '#54dc40';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x, y, 14, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(x - 22, y);
      ctx.lineTo(x - 6, y);
      ctx.moveTo(x + 6, y);
      ctx.lineTo(x + 22, y);
      ctx.moveTo(x, y - 22);
      ctx.lineTo(x, y - 6);
      ctx.moveTo(x, y + 6);
      ctx.lineTo(x, y + 22);
      ctx.stroke();
      ctx.font = '12px ui-monospace, monospace';
      ctx.fillStyle = '#54dc40';
      ctx.fillText(d.class_name, x + 20, y - 20);
    }
  }, [dets, videoRef]);

  return (
    <div className="relative flex h-full w-full items-center justify-center
                    border border-bg-border bg-black/40">
      {/* tabIndex -1 so the video can never take focus and swallow [A]/[R]. */}
      <video ref={videoRef} tabIndex={-1} autoPlay muted playsInline
             className="max-h-full max-w-full" />
      <canvas ref={canvasRef}
              className="pointer-events-none absolute inset-0 h-full w-full" />
      {state !== 'live' && (
        <div className="absolute font-mono text-[10px] uppercase
                        tracking-[0.2em] text-ink-dim">
          {state === 'connecting' ? 'Connecting…' : 'Stream off'}
        </div>
      )}
    </div>
  );
}
```

- [ ] **Step 6: Rework the layout**

Replace the body of `web/src/App.tsx` with the three-column layout. The right column
collapses when there is no pending detection, so the feed takes the full width and the
card appearing is itself a signal:

```tsx
import { useEffect, useRef } from 'react';
import { ConnectionStatus } from './components/ConnectionStatus';
import { MissionStatePanel } from './components/MissionStatePanel';
import { PendingDetectionPanel } from './components/PendingDetectionPanel';
import { DetectionImage } from './components/DetectionImage';
import { VideoPanel } from './components/VideoPanel';
import { StreamToggle } from './components/StreamToggle';
import { useGcsStore } from './store/useGcsStore';
import { connect } from './net/client';

export default function App() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const pending = useGcsStore((s) => s.activePending);

  useEffect(() => {
    connect();
  }, []);

  return (
    <div
      className={
        'grid h-full grid-rows-[44px_1fr] bg-bg-base ' +
        (pending ? 'grid-cols-[260px_1fr_360px]' : 'grid-cols-[260px_1fr]')
      }
    >
      <header className={
        'row-start-1 flex items-center justify-between border-b ' +
        'border-bg-border bg-bg-panel px-4 ' +
        (pending ? 'col-span-3' : 'col-span-2')
      }>
        <div className="flex items-center gap-3">
          <span className="font-mono text-sm font-bold tracking-[0.3em] text-ink-primary">
            BV·GCS
          </span>
          <span className="font-mono text-[10px] uppercase text-ink-dim">
            Human-in-the-loop
          </span>
        </div>
        <ConnectionStatus />
      </header>

      <aside className="col-start-1 row-start-2 space-y-3 overflow-y-auto
                        border-r border-bg-border p-3">
        <MissionStatePanel />
        <StreamToggle videoRef={videoRef} />
      </aside>

      <main className="col-start-2 row-start-2 min-h-0 p-3">
        <VideoPanel videoRef={videoRef} />
      </main>

      {pending && (
        <aside className="col-start-3 row-start-2 flex min-h-0 flex-col gap-3
                          overflow-y-auto border-l border-bg-border p-3">
          <div className="min-h-0 flex-1">
            <DetectionImage />
          </div>
          <PendingDetectionPanel />
        </aside>
      )}
    </div>
  );
}
```

- [ ] **Step 7: Build the frontend**

Run:
```bash
cd ~/bv_ws/src/bv_gcs/web && npm run build
```
Expected: `tsc -b` clean, then a successful vite build. Fix any strict-mode errors.

- [ ] **Step 8: Verify end to end in sim**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core bv_gcs && source install/setup.bash
```
Set `Approval_required: true`, rebuild, launch the sim, open `http://localhost:8765`,
press **Start stream**.

Expected: video appears within a couple of seconds once the mission reaches scan;
green crosshair markers land on the objects the detector fires on; the approval card
appears in a third column beside the video rather than replacing it; `[A]` and `[R]`
still work while the video is playing.

- [ ] **Step 9: Commit**

```bash
cd ~/bv_ws/src/bv_gcs
git add web/src
git commit -m "feat: live video panel with detection overlay

Video rides its own WebSocket and is displayed through MSE, with detection
markers drawn on a canvas overlay rather than composited into the frames —
detection runs at 0.67 Hz while video runs at 8 fps, so baking them in would drag
the video down to the detector's rate.

Layout is three columns with the feed and the approval crop side by side; the
right column collapses when nothing is pending. The video element is tabIndex=-1
so it can never take focus and swallow the [A]/[R] shortcuts."
```

---

## Final verification

- [ ] **Full suite**

```bash
cd ~/bv_ws && colcon build && source install/setup.bash
cd src/bv_core && python3 -m pytest test/ -v \
  --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```
Expected: **154 passed, 1 skipped** (120 baseline + 34 new).

- [ ] **Disabled path is untouched**

With `preview_enabled` never published, run a normal sim mission and confirm the
mission log is structurally identical to a pre-change run: same state sequence, same
detection cadence, no `/preview_stream` traffic (`ros2 topic hz /preview_stream`
reports nothing).

- [ ] **Tier logging on the Jetson**

On the real hardware, toggle the stream on and confirm the log says
`preview encoder: tier 'jetson'`. Tier `nvidia` or `software` there means the Tegra
multimedia plugins are missing from the flight image and the hardware path has
silently regressed.

- [ ] **Load check on the real camera**

With the real 4640×3480 source and `record_video: True`, compare capture framerate and
detection cadence with the stream on versus off. The design's whole claim is that it
costs nothing the mission notices; this is the measurement that tests it.

- [ ] **Verdict latency under video load**

With the stream running, trigger a detection and time the gap between clicking Approve
and `mission_node` logging `APPROVED`. It should be indistinguishable from the
stream-off case — that is what the separate `/video` socket buys.

## Deliberate simplification against the spec

The spec's error-handling table says the UI should distinguish "encoder failed" from
"not in scan yet" using `is_running()`. That would need a status topic from
`vision_node` back to `approval_node`. This plan does not build one: the operator sees
`connecting`, and `vision_node` logs `debug preview requested but unavailable` when the
encoder could not start. For a debug tool that is enough signal, and a status channel
is machinery that would then need its own lifecycle and tests. Noted so it is a
recorded choice rather than an oversight.

## Out of scope

- A preview status topic (see above).
- Bounding boxes in the overlay. `/obj_dets` carries centres only; boxes would require
  extending `ObjectDetections`, which `filtering_node` also consumes.
- Changing `/image_compressed` or `record_video`.
- WebRTC. The encode path is transport-agnostic so this stays cheap to add later.
- Recording the preview to disk.
- Any change to `mission.py`, `approval_gate.py` or `filtering_node.py`.
