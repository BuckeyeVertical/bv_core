"""Pin libgcc's C++ unwinder ahead of libunwind. IMPORT THIS FIRST.

This module must import nothing but ctypes, and `import bv_core._unwinder` must
be the FIRST import line in any module that pulls in cv2, GStreamer or gi, even
transitively. Its whole job is to win a load-order race, so anything that runs
before it defeats it.

The race
--------
GStreamer pulls in libunwind.so.8, which exports the same _Unwind_* symbols as
libgcc_s.so.1. Whichever object lands first wins for every later lookup. If
libunwind wins, C++ exception unwinding inside Fast-DDS resolves _Unwind_Resume
to libunwind while the frame's personality routine is still libgcc's
__gcc_personality_v0, and that mismatch calls abort(). Fast-DDS throws and
catches internally as ordinary control flow (e.g. UDPv4Transport
::OpenInputChannel), so the process dies inside a later rclpy node, publisher
or subscription creation with no Python traceback at all - just SIGABRT/SIGSEGV.

Why import time, and why first
------------------------------
It is not enough to pin before calling into GStreamer. Where OpenCV is built
with GStreamer - as it is on the Jetson, because pipeline_type 'real' could not
work otherwise - libopencv_videoio lists libgstreamer-1.0 in DT_NEEDED, which
pulls libunwind transitively. So `import cv2` alone maps libunwind, with no
cv2.VideoCapture call anywhere:

    import ctypes
    ctypes.CDLL('libopencv_videoio.so.4.5.4d', mode=ctypes.RTLD_GLOBAL)
    any('libunwind' in l for l in open('/proc/self/maps'))   # -> True

A pin placed below `import cv2` therefore runs after the arrival it is meant to
precede. Bare Gst.init(None) is likewise enough on its own - no encoder
pipeline needed.

DO NOT reorder this import below others, and do not let an import sorter move
it. The `# noqa: F401` on the call sites marks it as a deliberate
side-effect-only import, not dead code.
"""

import ctypes


def pin_libgcc_unwinder():
    """Load libgcc_s RTLD_GLOBAL so its unwinder wins later symbol lookups.

    Idempotent and cheap: after the first call this is a dlopen of an
    already-mapped library. Safe to call again from any explicit site that
    wants to be self-documenting about why it is there.
    """
    try:
        ctypes.CDLL('libgcc_s.so.1', mode=ctypes.RTLD_GLOBAL)
    except OSError:         # noqa: BLE001 - best effort; absence is not fatal
        pass


# Deliberate import-time side effect. This is the line that actually closes the
# race; the callable above exists for explicit call sites and for tests.
pin_libgcc_unwinder()
