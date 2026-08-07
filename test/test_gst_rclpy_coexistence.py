#!/usr/bin/env python3
"""GStreamer and rclpy must survive in one process, in either order.

Regression gate for a crash that killed the whole test suite and was a latent
in-flight abort: loading GStreamer plugins pulls in libunwind.so.8, which
exports the same _Unwind_* symbols as libgcc_s.so.1. If libunwind lands first,
C++ exception unwinding inside Fast-DDS resolves _Unwind_Resume to libunwind
while the frame's personality routine is still libgcc's __gcc_personality_v0,
and the mismatch calls abort(). Fast-DDS throws and catches internally as
ordinary control flow, so the process died at the next rclpy node creation with
no Python traceback at all — just SIGABRT.

`_pin_libgcc_unwinder()` in bv_core/preview_stream.py fixes it by loading
libgcc_s RTLD_GLOBAL before gi is imported. These tests pin that.

Each case runs in a SUBPROCESS on purpose. The failure mode is a native abort,
not an exception, so an in-process test would take the whole pytest run down
with it instead of reporting a failure — which is exactly what used to happen.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import subprocess
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

pytest.importorskip('gi', reason='PyGObject not installed')


def _run(body, timeout=120):
    """Run `body` in a fresh interpreter; return (returncode, stdout+stderr)."""
    script = textwrap.dedent("""
        import sys
        sys.path.insert(0, {repo!r})
    """).format(repo=REPO) + textwrap.dedent(body)
    proc = subprocess.run([sys.executable, '-c', script],
                          capture_output=True, text=True, timeout=timeout)
    return proc.returncode, proc.stdout + proc.stderr


_START_STOP = """
    from bv_core.preview_stream import PreviewConfig, PreviewStream
    s = PreviewStream(PreviewConfig(width=640, fps=0.0), lambda c: None)
    assert s.start() is True, 'no encoder tier available'
    s.stop()
    print('GST_OK')
"""

_MAKE_NODE = """
    import rclpy
    rclpy.init()
    from rclpy.node import Node
    n = Node('coexistence_probe')
    print('RCLPY_OK')
"""


class TestGstThenRclpy:
    def test_node_creation_survives_a_preview_start_stop(self):
        # The original crash. Without _pin_libgcc_unwinder this aborts (-6/-11).
        rc, out = _run(_START_STOP + _MAKE_NODE)
        assert 'GST_OK' in out, out
        assert rc == 0, f'exited {rc} (negative = fatal signal):\n{out}'
        assert 'RCLPY_OK' in out, out

    def test_publisher_creation_survives_too(self):
        # Publishers open transport channels, the site that throws internally.
        rc, out = _run(_START_STOP + _MAKE_NODE + """
    from std_msgs.msg import UInt8MultiArray
    for i in range(5):
        n.create_publisher(UInt8MultiArray, f'/probe_{i}', 10)
    print('PUB_OK')
""")
        assert rc == 0, f'exited {rc} (negative = fatal signal):\n{out}'
        assert 'PUB_OK' in out, out


class TestRclpyThenGst:
    def test_preview_start_stop_after_a_node_exists(self):
        # The production ordering: the node always predates the first toggle.
        rc, out = _run(_MAKE_NODE + _START_STOP + """
    m = Node('second_probe')
    print('SECOND_NODE_OK')
""")
        assert rc == 0, f'exited {rc} (negative = fatal signal):\n{out}'
        assert 'SECOND_NODE_OK' in out, out


def _opencv_has_gstreamer():
    """True if this OpenCV can initialise GStreamer via CAP_GSTREAMER."""
    import cv2
    for line in cv2.getBuildInformation().splitlines():
        if 'GStreamer' in line:
            return 'YES' in line.upper()
    return False


class TestOpenCvRoute:
    """The route that actually runs on the drone.

    CameraPipeline does cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), which
    initialises GStreamer INSIDE OpenCV without ever touching
    preview_stream._init_gst. On the Jetson OpenCV is built with GStreamer (it
    must be, or pipeline_type 'real' could not work), so on an ordinary flight
    with the preview never enabled, libunwind can still land first.
    """

    @pytest.mark.skipif(not _opencv_has_gstreamer(),
                        reason='OpenCV built without GStreamer (this box reports '
                               'GStreamer: NO), so CAP_GSTREAMER cannot init it')
    def test_node_creation_survives_an_opencv_gstreamer_capture(self):
        rc, out = _run("""
    import bv_core.vision_node          # must pin at import, before any capture
    import cv2
    cap = cv2.VideoCapture('videotestsrc num-buffers=1 ! videoconvert ! appsink',
                           cv2.CAP_GSTREAMER)
    cap.release()
    print('CV_GST_OK')
""" + _MAKE_NODE)
        assert 'CV_GST_OK' in out, out
        assert rc == 0, f'exited {rc} (negative = fatal signal):\n{out}'
        assert 'RCLPY_OK' in out, out

    def test_importing_vision_node_pins_the_unwinder(self):
        # Hardware-independent gate for the pin that covers the OpenCV route.
        #
        # It spies on pin_libgcc_unwinder rather than checking whether libgcc_s
        # is mapped: cv2, numpy and rclpy all map libgcc_s on their own, so a
        # /proc/self/maps assertion would pass with the pin deleted. Patching
        # the attribute BEFORE vision_node is imported works because
        # vision_node does `from .preview_stream import ... pin_libgcc_unwinder`
        # at import time, so it binds whatever the module holds then.
        rc, out = _run("""
    import bv_core.preview_stream as ps
    calls = []
    ps.pin_libgcc_unwinder = lambda: calls.append(1)
    import bv_core.vision_node
    print('PIN_CALLS', len(calls))
""")
        assert rc == 0, out
        assert 'PIN_CALLS 0' not in out, (
            'vision_node no longer pins the unwinder at import; the OpenCV '
            'CAP_GSTREAMER route is unprotected again:\n' + out)
        assert 'PIN_CALLS 1' in out, out

    def test_camera_pipeline_pins_before_it_opens_a_capture(self):
        # The ordering is the whole point: pinning after cv2 has already
        # initialised GStreamer is too late. Stubs VideoCapture so this runs
        # without GStreamer, and records whether the pin came first.
        rc, out = _run("""
    import cv2
    import bv_core.pipelines.camera_pipeline as cp

    order = []
    cp.pin_libgcc_unwinder = lambda: order.append('pin')

    class _Cap:
        def isOpened(self):
            return False
    def _fake_capture(*a, **k):
        order.append('capture')
        return _Cap()
    cv2.VideoCapture = _fake_capture

    try:
        cp.CameraPipeline('videotestsrc ! appsink')
    except RuntimeError:
        pass                     # expected: our stub reports not-opened
    print('ORDER', order)
""")
        assert rc == 0, out
        assert "ORDER ['pin', 'capture']" in out, (
            'CameraPipeline must pin the unwinder BEFORE cv2.VideoCapture '
            'initialises GStreamer:\n' + out)


class TestTheFixIsActuallyLoadBearing:
    def test_libunwind_arrives_with_gstreamer(self):
        # If this ever stops being true the guard may be obsolete - but check
        # by running the repro, not by deleting the guard on this evidence.
        rc, out = _run(_START_STOP + """
    libs = set()
    for line in open('/proc/self/maps'):
        if 'libunwind' in line:
            libs.add('libunwind')
        if 'libgcc_s' in line:
            libs.add('libgcc_s')
    print('LIBS', sorted(libs))
""")
        assert rc == 0, out
        assert 'libunwind' in out, out
        assert 'libgcc_s' in out, out
