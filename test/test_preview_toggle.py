#!/usr/bin/env python3
"""The /preview_enabled callback must never block vision_node's executor.

`vision_node` runs under `rclpy.spin`, a SINGLE-THREADED executor, and that same
executor serves `localize_object` — the service `mission_node` calls on the
mission's critical path. `PreviewStream.start()` and `.stop()` can each take
12-18 s while a GStreamer tier ladder is built or torn down. Doing either inline
in the subscription callback would stall the localize service for that long;
`mission_node` retries five times and then ABANDONS a real target and resumes the
scan. A debug toggle would cost the mission a delivery.

So these tests pin the property that makes that impossible: the callback records
a desired state and returns, and a worker thread applies it. They drive the real,
unbound `VisionNode` methods lifted onto a lightweight stand-in — the same
technique test_mission_approval_wiring and test_localize_crop_index use, because
`VisionNode.__init__` builds a camera pipeline, loads a detector and creates
MAVROS subscriptions.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core import vision_node  # noqa: E402
from bv_core.pipelines.Vision_Pipeline import VisionPipeline  # noqa: E402
from bv_core.vision_node import VisionNode  # noqa: E402


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))

    def text(self):
        return ' | '.join(m for _, m in self.lines)


class _Pipe(VisionPipeline):
    """A real VisionPipeline, so set_preview/_enqueue_frame are the real ones."""

    def start(self):
        pass

    def stop(self):
        pass


class _SlowPreview:
    """A PreviewStream stand-in whose lifecycle calls block, as the real one can."""

    def __init__(self, delay=1.0, succeed=True):
        self.delay = delay
        self.succeed = succeed
        self.calls = []
        self._running = False
        self._busy = threading.Lock()
        self.overlapped = False
        self.started = threading.Event()

    def _enter(self, name):
        if not self._busy.acquire(blocking=False):
            # start() and stop() ran concurrently against the same stream.
            self.overlapped = True
            self._busy.acquire()
        self.calls.append(name)

    def start(self):
        self._enter('start')
        try:
            time.sleep(self.delay)
            self._running = bool(self.succeed)
            self.started.set()
            return self.succeed
        finally:
            self._busy.release()

    def stop(self):
        self._enter('stop')
        try:
            time.sleep(self.delay)
            self._running = False
        finally:
            self._busy.release()

    def is_running(self):
        return self._running


class _Msg:
    def __init__(self, data):
        self.data = data


class _PublishRecorder:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Node:
    """Carries exactly the attributes the preview methods touch."""

    # The real, unbound implementations under test.
    _init_preview_state = VisionNode._init_preview_state
    _on_preview_toggle = VisionNode._on_preview_toggle
    _ensure_preview_worker = VisionNode._ensure_preview_worker
    _preview_worker_loop = VisionNode._preview_worker_loop
    _apply_preview_state = VisionNode._apply_preview_state
    _publish_preview_chunk = VisionNode._publish_preview_chunk
    _join_preview_worker = VisionNode._join_preview_worker

    def __init__(self, preview):
        self.preview = preview
        self.pipeline = _Pipe(max_queue_size=2)
        self.preview_pub = _PublishRecorder()
        self.logger = _StubLogger()
        # The REAL initialiser, not a hand-copied set of attributes. A local
        # copy silently drifts from _init_pipeline; this fails loudly instead.
        self._init_preview_state()

    def get_logger(self):
        return self.logger

    def shutdown(self):
        self._preview_shutdown.set()
        self._preview_wake.set()
        if self._preview_worker is not None:
            self._preview_worker.join(timeout=10.0)


def _wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TestToggleNeverBlocksTheExecutor:
    def test_callback_returns_promptly_while_start_blocks(self):
        node = _Node(_SlowPreview(delay=2.0))
        try:
            t0 = time.monotonic()
            node._on_preview_toggle(_Msg(True))
            elapsed = time.monotonic() - t0
            # start() sleeps 2 s in the worker; the callback must not wait on it.
            assert elapsed < 0.2, f'callback blocked for {elapsed:.3f}s'
            assert node.preview.calls == [] or node.preview.calls == ['start']
        finally:
            node.shutdown()

    def test_capture_path_keeps_running_while_start_blocks(self):
        # Stands in for "the executor is still free": work that the callback
        # would otherwise have stalled completes while start() is in flight.
        node = _Node(_SlowPreview(delay=2.0))
        try:
            node._on_preview_toggle(_Msg(True))
            frame = np.zeros((4, 4, 3), dtype=np.uint8)
            for _ in range(5):
                node._on_preview_toggle(_Msg(True))
                node.pipeline._enqueue_frame(frame)
                assert node.pipeline.get_frame(timeout=0.1) is not None
            # Still mid-start: the preview is not attached yet.
            assert node.pipeline._preview is None
        finally:
            node.shutdown()

    def test_worker_eventually_attaches_the_preview(self):
        preview = _SlowPreview(delay=0.05)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)
            assert 'debug preview ON' in node.logger.text()
        finally:
            node.shutdown()

    def test_worker_detaches_before_stopping(self):
        preview = _SlowPreview(delay=0.05)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)
            node._on_preview_toggle(_Msg(False))
            assert _wait_for(lambda: 'debug preview OFF' in node.logger.text())
            assert preview.calls == ['start', 'stop']
            assert node.pipeline._preview is None
        finally:
            node.shutdown()

    def test_rapid_toggles_converge_on_the_latest_request(self):
        preview = _SlowPreview(delay=0.3)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            node._on_preview_toggle(_Msg(False))
            node._on_preview_toggle(_Msg(True))
            node._on_preview_toggle(_Msg(False))
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)
            # Converged on ON, and no start/stop pair ever overlapped.
            assert preview.is_running()
            assert not preview.overlapped
            # Latest-state-wins, not one lifecycle call per message.
            assert len(preview.calls) < 5
        finally:
            node.shutdown()

    def test_repeated_on_does_not_restart(self):
        preview = _SlowPreview(delay=0.05)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)
            for _ in range(3):
                node._on_preview_toggle(_Msg(True))
            time.sleep(0.3)
            assert preview.calls == ['start']
        finally:
            node.shutdown()

    def test_failed_start_leaves_preview_detached_and_warns(self):
        preview = _SlowPreview(delay=0.05, succeed=False)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: preview.started.is_set())
            time.sleep(0.2)
            assert node.pipeline._preview is None
            assert 'unavailable' in node.logger.text()
        finally:
            node.shutdown()

    def test_real_init_spawns_no_worker_and_engages_nothing(self):
        # Exercises VisionNode._init_preview_state itself (via _Node.__init__),
        # so it fails if initialisation ever starts a thread eagerly. The
        # disabled path is the property the whole feature is gated on.
        before = threading.active_count()
        node = _Node(_SlowPreview())
        assert node._preview_worker is None
        assert node._preview_want is False
        assert node._preview_engaged is False
        assert node._preview_shutdown.is_set() is False
        assert node._preview_wake.is_set() is False
        assert threading.active_count() == before
        assert node.pipeline._preview is None
        assert node.preview.calls == []


class TestStateAfterAStreamFault:
    """PreviewStream self-disables on a bus error; the toggle must still work.

    If the node read is_running() as intent, an OFF click after such a fault
    would early-return: the pipeline would stay attached to a dead stream and
    nothing would log, so the operator's click would appear to do nothing.
    """

    def test_off_still_detaches_and_logs_after_the_stream_died(self):
        preview = _SlowPreview(delay=0.05)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)

            preview._running = False        # bus error: stream self-disabled
            node.logger.lines.clear()

            node._on_preview_toggle(_Msg(False))
            assert _wait_for(lambda: 'debug preview OFF' in node.logger.text())
            assert node.pipeline._preview is None
            assert 'stop' in preview.calls
        finally:
            node.shutdown()

    def test_on_after_a_fault_rebuilds_rather_than_doing_nothing(self):
        preview = _SlowPreview(delay=0.05)
        node = _Node(preview)
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is preview)

            preview._running = False        # bus error
            node.logger.lines.clear()

            # The operator clicking ON again means "retry", not "no-op".
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: 'debug preview ON' in node.logger.text())
            assert preview.is_running()
            assert node.pipeline._preview is preview
            assert not preview.overlapped
        finally:
            node.shutdown()


class TestShutdownJoin:
    """destroy_node stops the stream a second time iff the worker outlasted the join.

    That second stop is what stops a start() landing after we gave up from
    leaving an encoder alive past the node's life, so the True/False return is
    load-bearing, not informational.
    """

    def test_no_worker_means_nothing_outlasted(self):
        node = _Node(_SlowPreview(delay=0.05))
        assert node._join_preview_worker() is False

    def test_reports_true_while_the_worker_is_still_inside_start(self, monkeypatch):
        monkeypatch.setattr(vision_node, 'PREVIEW_WORKER_JOIN_SEC', 0.1)
        # A start() far longer than the join: destroy_node must be told.
        node = _Node(_SlowPreview(delay=5.0))
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.preview.calls == ['start'])
            assert node._join_preview_worker() is True
        finally:
            node.shutdown()

    def test_reports_false_once_the_worker_has_finished(self, monkeypatch):
        monkeypatch.setattr(vision_node, 'PREVIEW_WORKER_JOIN_SEC', 5.0)
        node = _Node(_SlowPreview(delay=0.05))
        try:
            node._on_preview_toggle(_Msg(True))
            assert _wait_for(lambda: node.pipeline._preview is not None)
            node._preview_shutdown.set()
            node._preview_wake.set()
            assert node._join_preview_worker() is False
        finally:
            node.shutdown()

    def test_the_join_budget_exceeds_a_worst_case_start(self):
        # start() can take 12-18 s (three tiers x a 3 s get_state plus shutdown
        # waits). A join below that would routinely abandon a live build.
        assert vision_node.PREVIEW_WORKER_JOIN_SEC > 18.0


class TestChunkPublisher:
    def test_publishes_the_chunk(self):
        node = _Node(_SlowPreview())
        node._publish_preview_chunk(b'\x00\x01\x02')
        assert len(node.preview_pub.published) == 1
        assert bytes(node.preview_pub.published[0].data) == b'\x00\x01\x02'

    def test_never_stops_the_stream_from_the_encoder_thread(self):
        # _publish_preview_chunk runs ON the GStreamer streaming thread. Calling
        # stop() there deadlocks: the streaming thread would block on the
        # encoder's lock while shutdown waits for that same thread to exit.
        node = _Node(_SlowPreview())
        node._publish_preview_chunk(b'\x00')
        assert node.preview.calls == []
