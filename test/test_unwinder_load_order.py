#!/usr/bin/env python3
"""The libgcc unwinder pin must be imported before cv2, in every node process.

NO GStreamer, no PyGObject, no encoder required — deliberately. These live apart
from test_gst_rclpy_coexistence.py, which skips wholesale without `gi`, because
a lint/CI container is both the most likely place to lack `gi` and the most
likely place for an import reformat to silently break the ordering. These are
the checks that have to run everywhere.

Background: on a GStreamer-built OpenCV (the Jetson), libopencv_videoio lists
libgstreamer-1.0 in DT_NEEDED, which pulls libunwind transitively. libunwind
exports the same _Unwind_* symbols as libgcc_s, and whichever lands first wins
every later lookup — so if libunwind wins, Fast-DDS's internal C++ exceptions
abort the process with no Python traceback. `import cv2` alone is therefore
enough to lose the race, and any pin below it is too late. See
bv_core/_unwinder.py.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import subprocess
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Every console_script in setup.py, by module. Keep in step with setup.py: a
# new node that reaches cv2 without the pin is exactly the regression here.
CONSOLE_SCRIPT_MODULES = [
    'bv_core.mission',
    'bv_core.vision_node',
    'bv_core.filtering_node',
    'bv_core.stitching',
    'bv_core.bv_viz_node',
    'bv_core.test_servo',
    'bv_core.camera_pipeline_test_node',
    'bv_core.test_obj_loc',
    'bv_core.sim_approval_check',
]

# Modules that do `import cv2` themselves. Each must pin first.
CV2_IMPORTING_MODULES = [
    'bv_core.vision_node',
    'bv_core.preview_stream',
    'bv_core.pipelines.camera_pipeline',
    'bv_core.localizer',
    'bv_core.detection_crop',
    'bv_core.stitching',
    'bv_core.camera_pipeline_test_node',
]


def _import_order(module, timeout=180):
    """Import `module` in a fresh interpreter; return (unwinder_idx, cv2_idx).

    Either index is None when that module was never imported. CPython's
    sys.modules preserves insertion order, so an index is the import order.
    """
    script = textwrap.dedent("""
        import sys
        sys.path.insert(0, {repo!r})
        import {mod}
        order = list(sys.modules)
        u = order.index('bv_core._unwinder') if 'bv_core._unwinder' in order else -1
        c = order.index('cv2') if 'cv2' in order else -1
        print('IDX', u, c)
    """).format(repo=REPO, mod=module)
    # -u so nothing is lost to block buffering if the child dies abruptly.
    proc = subprocess.run([sys.executable, '-u', '-c', script],
                          capture_output=True, text=True, timeout=timeout)
    out = proc.stdout + proc.stderr
    assert proc.returncode == 0, (
        f'importing {module} exited {proc.returncode} '
        f'(negative = fatal signal):\n{out}')
    line = [ln for ln in out.splitlines() if ln.startswith('IDX ')]
    assert line, f'no index line from {module}:\n{out}'
    _, u, c = line[-1].split()
    return (None if u == '-1' else int(u)), (None if c == '-1' else int(c))


class TestEveryConsoleScript:
    """No node may reach cv2 without the pin already loaded.

    This is the enumeration made executable rather than written down once: a
    new entry point, or a new cv2 import inside an existing one, fails here.
    """

    @pytest.mark.parametrize('module', CONSOLE_SCRIPT_MODULES)
    def test_pin_precedes_cv2_if_cv2_is_reached(self, module):
        unwinder, cv2_idx = _import_order(module)
        if cv2_idx is None:
            # Node never touches OpenCV; nothing can map libunwind this way.
            return
        assert unwinder is not None, (
            f'{module} imports cv2 but never imports bv_core._unwinder. On a '
            f'GStreamer-built OpenCV this process maps libunwind unpinned, and '
            f'Fast-DDS exceptions can abort it with no traceback.')
        assert unwinder < cv2_idx, (
            f'{module} imports cv2 (index {cv2_idx}) before bv_core._unwinder '
            f'(index {unwinder}); the pin runs after the arrival it exists to '
            f'precede.')


class TestEveryCv2Importer:
    @pytest.mark.parametrize('module', CV2_IMPORTING_MODULES)
    def test_module_pins_before_importing_cv2(self, module):
        unwinder, cv2_idx = _import_order(module)
        assert cv2_idx is not None, (
            f'{module} is listed as a cv2 importer but did not import cv2; '
            f'update CV2_IMPORTING_MODULES')
        assert unwinder is not None and unwinder < cv2_idx, (
            f'{module}: unwinder={unwinder}, cv2={cv2_idx}')


class TestTheUnwinderModuleItself:
    def test_pulls_in_nothing_heavy(self):
        # Its only job is to win a race, so it must not import its competitors.
        script = textwrap.dedent("""
            import sys
            sys.path.insert(0, {repo!r})
            import bv_core._unwinder
            heavy = [m for m in ('cv2', 'gi', 'numpy', 'rclpy')
                     if m in sys.modules]
            print('HEAVY', heavy)
        """).format(repo=REPO)
        proc = subprocess.run([sys.executable, '-u', '-c', script],
                              capture_output=True, text=True, timeout=60)
        out = proc.stdout + proc.stderr
        assert proc.returncode == 0, out
        assert 'HEAVY []' in out, out

    def test_import_is_idempotent_and_callable_again(self):
        script = textwrap.dedent("""
            import sys
            sys.path.insert(0, {repo!r})
            from bv_core._unwinder import pin_libgcc_unwinder
            for _ in range(3):
                pin_libgcc_unwinder()
            print('IDEMPOTENT_OK')
        """).format(repo=REPO)
        proc = subprocess.run([sys.executable, '-u', '-c', script],
                              capture_output=True, text=True, timeout=60)
        out = proc.stdout + proc.stderr
        assert proc.returncode == 0, out
        assert 'IDEMPOTENT_OK' in out, out
