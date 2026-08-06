# Human-in-the-Loop Approval Gate — Mission Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an operator approve or reject each localized detection — judging from an annotated image crop — before the drone flies to it and drops a payload.

**Architecture:** A branch at a single point in `mission.py` (`on_vision_localization_complete`). When enabled, instead of calling `enter_deliver_state()` the mission hands the detection to an `ApprovalGate` helper that publishes it and waits for a verdict over a service. Approve and timeout converge on the same `enter_deliver_state()` call; reject publishes the location so `filtering_node` suppresses that class near that spot. Two new focused modules keep the delta to `mission.py` (already 1,151 lines) at ~20 lines.

**Tech Stack:** ROS 2 Humble, rclpy, OpenCV (`cv2`), numpy, `supervision` (`sv.Detections`), pytest.

**Spec:** `docs/superpowers/specs/2026-08-05-hitl-mission-wiring-design.md`

## Global Constraints

- Python 3.10, ROS 2 Humble, `numpy<2`. No new third-party dependencies.
- `Approval_required` defaults to **`false`**. With it false, the flight path must be byte-for-byte today's autonomous behavior.
- The timeout **fails open**: expiry deploys, it does not skip.
- Line length ≤ 99 chars (`flake8` config used by `test/test_flake8.py`).
- Class names come from the module-level `CLASS_NAMES = ("person", "tent")` already duplicated in `mission.py:47`, `vision_node.py:52`, `filtering_node.py:25`. Do **not** add a fourth copy in new modules — accept `class_names` as a parameter.
- Tests requiring `bv_msgs` need the workspace sourced first:
  `cd ~/bv_ws && colcon build && source install/setup.bash`
- Run tests from the package directory: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/ -v`
- Commit after every task. Branch is `dev`.

---

### Task 1: Adaptive detection crop

Pure numpy/cv2, no ROS. Built first because everything else can be stubbed against it and it needs no workspace build.

**Files:**
- Create: `bv_core/detection_crop.py`
- Test: `test/test_detection_crop.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `CropConfig(margin_factor: float = 2.5, min_px: int = 320, max_px: int = 1024, jpeg_quality: int = 85)` — dataclass
  - `crop_window(bbox, frame_w, frame_h, cfg) -> tuple[int, int, int, int]` — returns `(x1, y1, x2, y2)`
  - `build_annotated_crop(frame, bbox, label, cfg) -> bytes` — returns JPEG bytes

- [ ] **Step 1: Write the failing tests for the window geometry**

Create `test/test_detection_crop.py`:

```python
#!/usr/bin/env python3
"""Tests for detection_crop — pure numpy/cv2, no ROS required."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.detection_crop import CropConfig, crop_window


CFG = CropConfig(margin_factor=2.5, min_px=320, max_px=1024, jpeg_quality=85)


class TestCropWindow:
    """Geometry of the crop window before any pixels are touched."""

    def test_centred_box_is_expanded_by_margin(self):
        # 200x200 box in the middle of a 4640x3480 frame.
        x1, y1, x2, y2 = crop_window((2220, 1640, 2420, 1840), 4640, 3480, CFG)
        assert (x2 - x1) == 500      # 200 * 2.5
        assert (y2 - y1) == 500
        # Still centred on the box centre (2320, 1740).
        assert (x1 + x2) // 2 == 2320
        assert (y1 + y2) // 2 == 1740

    def test_small_box_widens_to_min_px(self):
        # 20x49 box — a person in sim. 2.5x is only 50x122, below the 320 floor.
        x1, y1, x2, y2 = crop_window((630, 330, 650, 379), 1280, 720, CFG)
        assert (x2 - x1) == 320
        assert (y2 - y1) == 320

    def test_box_at_left_edge_shifts_rather_than_shrinks(self):
        # Box hard against x=0. Window must stay full size and stay in bounds.
        x1, y1, x2, y2 = crop_window((0, 1640, 200, 1840), 4640, 3480, CFG)
        assert x1 == 0
        assert (x2 - x1) == 500
        assert x2 <= 4640

    def test_box_at_bottom_right_corner_shifts_rather_than_shrinks(self):
        x1, y1, x2, y2 = crop_window((4440, 3280, 4640, 3480), 4640, 3480, CFG)
        assert (x2 - x1) == 500
        assert (y2 - y1) == 500
        assert x2 == 4640
        assert y2 == 3480
        assert x1 >= 0 and y1 >= 0

    def test_window_never_exceeds_frame(self):
        # Requested window (min_px=320) is larger than this tiny frame.
        x1, y1, x2, y2 = crop_window((10, 10, 20, 20), 100, 80, CFG)
        assert x1 == 0 and y1 == 0
        assert x2 == 100 and y2 == 80

    def test_degenerate_zero_area_box_does_not_crash(self):
        x1, y1, x2, y2 = crop_window((500, 500, 500, 500), 4640, 3480, CFG)
        assert (x2 - x1) == 320      # falls back to the min window
        assert (y2 - y1) == 320
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_detection_crop.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bv_core.detection_crop'`

- [ ] **Step 3: Implement `CropConfig` and `crop_window`**

Create `bv_core/detection_crop.py`:

```python
#!/usr/bin/env python3
"""Build an annotated, native-resolution crop around a single detection.

The operator judges a detection from this image, so it has to stay legible. A
whole-frame downscale will not do: at 15 m AGL the real camera (fx=3582.9,
4640 px wide) puts a 1.8 m person at roughly 430 px tall, while the Gazebo sim
(fx=410.9, 1280 px) puts the same person at roughly 49 px. One fixed crop size
cannot serve both, so the window adapts to the bounding box.

Pure numpy/cv2 — no ROS — so the geometry is testable without a camera.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class CropConfig:
    """Tunables for the crop, mirrored in config/vision_params.yaml."""

    margin_factor: float = 2.5   # how much context around the box
    min_px: int = 320            # floor, so a tiny detection still gets context
    max_px: int = 1024           # ceiling on the longest output side
    jpeg_quality: int = 85


def crop_window(bbox, frame_w, frame_h, cfg):
    """Compute the crop window for one bounding box.

    Args:
        bbox: (x1, y1, x2, y2) in frame pixel coordinates.
        frame_w, frame_h: frame dimensions.
        cfg: CropConfig.

    Returns:
        (x1, y1, x2, y2) integer window, guaranteed inside the frame.

    Near a frame edge the window is *shifted* rather than shrunk, so an edge
    detection still gets its full share of context instead of a sliver.
    """
    x1, y1, x2, y2 = (float(v) for v in bbox)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    # max(..., 1.0) keeps a degenerate zero-area box from collapsing the window.
    width = max(x2 - x1, 1.0) * cfg.margin_factor
    height = max(y2 - y1, 1.0) * cfg.margin_factor

    width = max(width, float(cfg.min_px))
    height = max(height, float(cfg.min_px))

    # Can never be bigger than the frame it came from.
    width = min(width, float(frame_w))
    height = min(height, float(frame_h))

    left = min(max(cx - width / 2.0, 0.0), frame_w - width)
    top = min(max(cy - height / 2.0, 0.0), frame_h - height)

    return (
        int(round(left)),
        int(round(top)),
        int(round(left + width)),
        int(round(top + height)),
    )
```

- [ ] **Step 4: Run the window tests to verify they pass**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_detection_crop.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Write the failing tests for the encoded crop**

First extend the imports at the top of `test/test_detection_crop.py` — `cv2` is
needed to decode the result, and `build_annotated_crop` exists only from Step 7:

```python
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.detection_crop import CropConfig, build_annotated_crop, crop_window
```

Then append:

```python
def _frame(w=4640, h=3480):
    """A frame with non-uniform content so JPEG size is realistic."""
    rng = np.random.default_rng(seed=1)
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


class TestBuildAnnotatedCrop:
    """The encoded JPEG handed to the operator."""

    def test_returns_decodable_jpeg(self):
        data = build_annotated_crop(_frame(), (2220, 1640, 2420, 1840),
                                    'person 0.94', CFG)
        assert data[:2] == b'\xff\xd8'          # JPEG SOI marker
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None

    def test_output_respects_max_px(self):
        # 2000x2000 box -> 5000x5000 window, must be capped at 1024.
        data = build_annotated_crop(_frame(), (1320, 740, 3320, 2740),
                                    'tent 0.80', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert max(decoded.shape[:2]) == 1024

    def test_small_detection_keeps_native_resolution(self):
        # 20x49 box widens to the 320 floor, which is under max_px -> no rescale.
        data = build_annotated_crop(_frame(1280, 720), (630, 330, 650, 379),
                                    'person 0.91', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape[0] == 320
        assert decoded.shape[1] == 320

    def test_edge_detection_does_not_crash(self):
        data = build_annotated_crop(_frame(), (0, 0, 120, 120), 'person 0.70', CFG)
        assert len(data) > 0

    def test_grayscale_frame_is_accepted(self):
        rng = np.random.default_rng(seed=2)
        gray = rng.integers(0, 255, (720, 1280), dtype=np.uint8)
        data = build_annotated_crop(gray, (630, 330, 650, 379), 'person 0.60', CFG)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None

    def test_payload_stays_link_friendly(self):
        # Random noise is close to the worst case for JPEG; real imagery is smaller.
        data = build_annotated_crop(_frame(), (2220, 1640, 2420, 1840),
                                    'person 0.94', CFG)
        assert len(data) < 400_000
```

- [ ] **Step 6: Run to verify the new tests fail**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_detection_crop.py -v`
Expected: FAIL — `ImportError: cannot import name 'build_annotated_crop'`

This failure is the whole file erroring at import, so the six window tests will
also show as errors. That is expected and resolves in Step 7.

- [ ] **Step 7: Implement `build_annotated_crop`**

Append to `bv_core/detection_crop.py`:

```python
_BOX_BGR = (84, 220, 64)      # same green supervision draws in vision_node
_TEXT_BGR = (12, 24, 12)


def build_annotated_crop(frame, bbox, label, cfg):
    """Crop around `bbox`, draw the box and `label`, and JPEG-encode.

    Args:
        frame: HxWx3 BGR or HxW grayscale numpy array.
        bbox: (x1, y1, x2, y2) detection box in frame coordinates.
        label: text drawn above the box, e.g. "person 0.94".
        cfg: CropConfig.

    Returns:
        JPEG bytes.

    Raises:
        RuntimeError: if cv2 fails to encode.

    The box is drawn *after* the downscale on purpose. Annotating at native
    resolution and then resizing 4640 -> 1024 thins the lines to near
    invisibility; this way line weight is specified in the pixels the operator
    actually sees.
    """
    if frame.ndim == 2:
        frame = np.repeat(frame[:, :, None], 3, axis=2)

    frame_h, frame_w = frame.shape[:2]
    wx1, wy1, wx2, wy2 = crop_window(bbox, frame_w, frame_h, cfg)
    crop = frame[wy1:wy2, wx1:wx2].copy()

    # Detection box relative to the crop origin.
    bx1, by1, bx2, by2 = (float(v) for v in bbox)
    box = [bx1 - wx1, by1 - wy1, bx2 - wx1, by2 - wy1]

    crop_h, crop_w = crop.shape[:2]
    scale = min(1.0, float(cfg.max_px) / float(max(crop_h, crop_w)))
    if scale < 1.0:
        crop = cv2.resize(
            crop,
            (max(1, int(round(crop_w * scale))), max(1, int(round(crop_h * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        box = [v * scale for v in box]

    _annotate(crop, box, label)

    ok, buf = cv2.imencode(
        '.jpg', crop, [int(cv2.IMWRITE_JPEG_QUALITY), int(cfg.jpeg_quality)])
    if not ok:
        raise RuntimeError('cv2.imencode failed for annotated crop')
    return buf.tobytes()


def _annotate(image, box, label):
    """Draw the detection box and its label, in `image` pixel coordinates."""
    h, w = image.shape[:2]
    x1 = int(round(max(0, min(box[0], w - 1))))
    y1 = int(round(max(0, min(box[1], h - 1))))
    x2 = int(round(max(0, min(box[2], w - 1))))
    y2 = int(round(max(0, min(box[3], h - 1))))

    cv2.rectangle(image, (x1, y1), (x2, y2), _BOX_BGR, 2)

    (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    # Put the label inside the image when the box is flush against the top.
    label_bottom = y1 if y1 - text_h - 8 >= 0 else min(h, y2 + text_h + 8)
    label_top = max(0, label_bottom - text_h - 8)

    cv2.rectangle(image, (x1, label_top), (x1 + text_w + 8, label_bottom),
                  _BOX_BGR, -1)
    cv2.putText(image, label, (x1 + 4, label_bottom - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, _TEXT_BGR, 2, cv2.LINE_AA)
```

- [ ] **Step 8: Run all crop tests**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_detection_crop.py -v`
Expected: PASS (12 tests)

- [ ] **Step 9: Lint**

Run: `cd ~/bv_ws/src/bv_core && python3 -m flake8 --max-line-length=99 bv_core/detection_crop.py test/test_detection_crop.py`
Expected: no output

- [ ] **Step 10: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/detection_crop.py test/test_detection_crop.py
git commit -m "feat: adaptive annotated crop for operator review

Crops around the detection box with configurable margin, a minimum window so
small sim detections still get context, and a maximum output size. The box is
drawn after downscaling so line weight lands in output pixels rather than being
thinned away by a 4640->1024 resize."
```

---

### Task 2: ApprovalGate helper

**Files:**
- Create: `bv_core/approval_gate.py`
- Test: `test/test_approval_gate.py`

**Interfaces:**
- Consumes: `bv_msgs.msg.PendingDetection`, `bv_msgs.srv.DetectionDecision` (already built in phases 1–2).
- Produces:
  - `ApprovalGate(node, timeout_sec, log=None)`
  - `.request(*, class_id, lat, lon, alt, confidence, drone_lat, drone_lon, annotated_crop, on_approve, on_reject) -> str` (detection_id)
  - `.is_pending() -> bool`
  - `.cancel(reason: str) -> None`

**Requires the workspace built and sourced** (`bv_msgs` import).

- [ ] **Step 1: Write the failing tests**

Create `test/test_approval_gate.py`:

```python
#!/usr/bin/env python3
"""Tests for ApprovalGate — uses a stub node, no rclpy spin required.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.srv import DetectionDecision

from bv_core.approval_gate import ApprovalGate


class _StubTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _StubPublisher:
    def __init__(self):
        self.published = []
        self.subscribers = 1

    def publish(self, msg):
        self.published.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class StubNode:
    """Minimal stand-in for rclpy.node.Node."""

    def __init__(self):
        self.publisher = _StubPublisher()
        self.timers = []
        self.services = {}
        self._logger = _StubLogger()

    def create_publisher(self, msg_type, topic, qos):
        return self.publisher

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback
        return object()

    def create_timer(self, period, callback):
        timer = _StubTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger


@pytest.fixture
def gate_env():
    node = StubNode()
    gate = ApprovalGate(node, timeout_sec=180.0)
    calls = {'approve': 0, 'reject': []}

    def on_approve():
        calls['approve'] += 1

    def on_reject(lat, lon, class_id, reason):
        calls['reject'].append((lat, lon, class_id, reason))

    def make_request():
        return gate.request(
            class_id=0, lat=38.3877, lon=-76.4190, alt=15.2, confidence=0.94,
            drone_lat=38.3876, drone_lon=-76.4191, annotated_crop=b'\xff\xd8fake',
            on_approve=on_approve, on_reject=on_reject)

    return node, gate, calls, make_request


def _decide(node, detection_id, approved, reason=''):
    request = DetectionDecision.Request()
    request.detection_id = detection_id
    request.approved = approved
    request.reason = reason
    response = DetectionDecision.Response()
    return node.services['/detection_decision'](request, response)


class TestRequest:
    def test_publishes_pending_with_generated_id(self, gate_env):
        node, gate, _, make_request = gate_env
        detection_id = make_request()
        assert detection_id
        assert gate.is_pending()
        msg = node.publisher.published[-1]
        assert msg.detection_id == detection_id
        assert msg.class_id == 0
        assert msg.timeout_sec == pytest.approx(180.0)
        assert bytes(msg.annotated_crop.data) == b'\xff\xd8fake'
        assert msg.annotated_crop.format == 'jpeg'

    def test_arms_timeout_timer(self, gate_env):
        node, _, _, make_request = gate_env
        make_request()
        assert len(node.timers) == 1
        assert node.timers[0].period == pytest.approx(180.0)

    def test_zero_timeout_arms_no_timer(self):
        node = StubNode()
        gate = ApprovalGate(node, timeout_sec=0.0)
        gate.request(class_id=0, lat=0.0, lon=0.0, alt=0.0, confidence=0.0,
                     drone_lat=0.0, drone_lon=0.0, annotated_crop=b'',
                     on_approve=lambda: None, on_reject=lambda *a: None)
        assert node.timers == []


class TestDecision:
    def test_approve_invokes_callback_and_clears(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        response = _decide(node, detection_id, True)
        assert response.accepted is True
        assert calls['approve'] == 1
        assert not gate.is_pending()

    def test_reject_passes_location_and_reason(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        response = _decide(node, detection_id, False, 'shadow')
        assert response.accepted is True
        assert calls['reject'] == [(38.3877, -76.4190, 0, 'shadow')]
        assert not gate.is_pending()

    def test_stale_id_is_refused_and_pending_survives(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        response = _decide(node, 'not-the-active-id', True)
        assert response.accepted is False
        assert 'mismatch' in response.message
        assert calls['approve'] == 0
        assert gate.is_pending()      # a stale click must not clear a live pending

    def test_decision_with_nothing_pending_is_refused(self, gate_env):
        node, _, calls, _ = gate_env
        response = _decide(node, 'anything', True)
        assert response.accepted is False
        assert calls['approve'] == 0

    def test_clearing_publishes_empty_pending(self, gate_env):
        node, _, _, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, True)
        assert node.publisher.published[-1].detection_id == ''

    def test_decision_cancels_the_timer(self, gate_env):
        node, _, _, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, True)
        assert node.timers[0].cancelled is True


class TestTimeout:
    def test_timeout_approves(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        node.timers[0].callback()          # fire the timer
        assert calls['approve'] == 1       # fails OPEN: deploys, does not skip
        assert calls['reject'] == []
        assert not gate.is_pending()

    def test_timeout_with_nothing_pending_is_a_noop(self, gate_env):
        node, gate, calls, make_request = gate_env
        detection_id = make_request()
        _decide(node, detection_id, False)
        node.timers[0].callback()          # late fire after the FSM moved on
        assert calls['approve'] == 0


class TestCancel:
    def test_cancel_clears_without_invoking_callbacks(self, gate_env):
        node, gate, calls, make_request = gate_env
        make_request()
        gate.cancel('rtl')
        assert not gate.is_pending()
        assert calls['approve'] == 0
        assert calls['reject'] == []
        assert node.publisher.published[-1].detection_id == ''
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `source ~/bv_ws/install/setup.bash && cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_approval_gate.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bv_core.approval_gate'`

- [ ] **Step 3: Implement `ApprovalGate`**

Create `bv_core/approval_gate.py`:

```python
#!/usr/bin/env python3
"""Operator approval gate for a single localized detection.

mission_node hands a localized detection here instead of flying to it directly.
The gate publishes it for the ground station, waits for a verdict on
/detection_decision, and calls back into the mission.

This node is the *authority*: it mints the detection id and owns the timeout.
bv_gcs/approval_node is only a relay, so if the radio link or the ground station
process dies the timer here still fires and the mission continues.

The timeout fails OPEN — expiry runs the approve path. The degraded mode is
therefore exactly the autonomous behavior the aircraft already flies.
"""

import uuid

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from bv_msgs.msg import PendingDetection
from bv_msgs.srv import DetectionDecision


class ApprovalGate:
    """Holds at most one pending detection awaiting an operator verdict."""

    def __init__(self, node, timeout_sec, log=None):
        """
        Args:
            node: the rclpy Node that owns the publisher, service and timer.
            timeout_sec: seconds before auto-approving. 0 waits forever.
            log: optional MissionLogger for structured mission events.
        """
        self._node = node
        self._timeout_sec = float(timeout_sec)
        self._log = log

        self._pending = None      # dict describing the live detection
        self._timer = None
        self._on_approve = None
        self._on_reject = None

        # TRANSIENT_LOCAL so a restarted approval_node, or a link that drops and
        # recovers mid-decision, immediately gets the live pending instead of
        # showing an empty dashboard while the aircraft counts down.
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.history = HistoryPolicy.KEEP_LAST

        self._pub = node.create_publisher(
            PendingDetection, '/pending_obj_dets', latched)
        self._srv = node.create_service(
            DetectionDecision, '/detection_decision', self._on_decision)

    # -- public API -------------------------------------------------------

    def is_pending(self):
        """True while a detection is awaiting a verdict."""
        return self._pending is not None

    def request(self, *, class_id, lat, lon, alt, confidence,
                drone_lat, drone_lon, annotated_crop,
                on_approve, on_reject):
        """Publish a detection for review and start the timeout.

        Args:
            class_id: semantic class id of the detection.
            lat, lon, alt: localized object position.
            confidence: detector confidence, 0.0 if unknown.
            drone_lat, drone_lon: aircraft position at localization time.
            annotated_crop: JPEG bytes, or b'' when the crop could not be built.
            on_approve: called with no arguments on approve or timeout.
            on_reject: called as on_reject(lat, lon, class_id, reason).

        Returns:
            The generated detection_id.
        """
        detection_id = str(uuid.uuid4())

        self._pending = {
            'detection_id': detection_id,
            'class_id': int(class_id),
            'lat': float(lat),
            'lon': float(lon),
        }
        self._on_approve = on_approve
        self._on_reject = on_reject

        msg = PendingDetection()
        msg.header.stamp = self._node.get_clock().now().to_msg() \
            if hasattr(self._node, 'get_clock') else msg.header.stamp
        msg.header.frame_id = 'map'
        msg.detection_id = detection_id
        msg.class_id = int(class_id)
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        msg.confidence = float(confidence)
        msg.drone_latitude = float(drone_lat)
        msg.drone_longitude = float(drone_lon)
        msg.timeout_sec = float(self._timeout_sec)
        msg.annotated_crop.header = msg.header
        msg.annotated_crop.format = 'jpeg'
        msg.annotated_crop.data = bytes(annotated_crop)
        self._pub.publish(msg)

        if self._pub.get_subscription_count() == 0:
            # Not fatal — the timeout will still deploy — but the operator
            # should know nobody is listening before they wait three minutes.
            self._node.get_logger().warn(
                'pending published but no approval_node is subscribed; '
                'the decision will time out and deploy')

        if self._timeout_sec > 0:
            self._timer = self._node.create_timer(
                self._timeout_sec, self._on_timeout)

        self._node.get_logger().info(
            f"AWAITING APPROVAL id={detection_id} class_id={class_id} "
            f"lat={lat:.6f} lon={lon:.6f} timeout={self._timeout_sec:.0f}s "
            f"crop={len(annotated_crop)}B")
        if self._log:
            self._log.event(
                'APPROVAL_REQUESTED',
                f"id={detection_id}, class={class_id}, "
                f"lat={lat:.6f}, lon={lon:.6f}, timeout={self._timeout_sec:.0f}s")

        return detection_id

    def cancel(self, reason):
        """Abandon the pending without running either callback."""
        if self._pending is None:
            return
        detection_id = self._pending['detection_id']
        self._clear()
        self._node.get_logger().info(
            f"approval cancelled id={detection_id} reason={reason}")
        if self._log:
            self._log.event('APPROVAL_CANCELLED',
                            f"id={detection_id}, reason={reason}")

    # -- callbacks --------------------------------------------------------

    def _on_decision(self, request, response):
        if self._pending is None:
            response.accepted = False
            response.message = 'no active pending detection'
            return response

        if request.detection_id != self._pending['detection_id']:
            response.accepted = False
            response.message = (
                f"detection_id mismatch "
                f"(active={self._pending['detection_id']})")
            return response

        pending = self._pending
        on_approve = self._on_approve
        on_reject = self._on_reject
        # Clear before the callback: the callback drives an FSM transition and
        # must not observe a pending that has already been decided.
        self._clear()

        if request.approved:
            response.accepted = True
            response.message = 'approved - flying to object'
            self._node.get_logger().info(
                f"APPROVED id={pending['detection_id']}")
            if self._log:
                self._log.event('APPROVAL_GRANTED',
                                f"id={pending['detection_id']}, source=operator")
            on_approve()
        else:
            response.accepted = True
            response.message = 'rejected - resuming scan'
            self._node.get_logger().info(
                f"REJECTED id={pending['detection_id']} "
                f"reason={request.reason!r}")
            if self._log:
                self._log.event(
                    'APPROVAL_REJECTED',
                    f"id={pending['detection_id']}, class={pending['class_id']}, "
                    f"lat={pending['lat']:.6f}, lon={pending['lon']:.6f}, "
                    f"reason={request.reason!r}")
            on_reject(pending['lat'], pending['lon'],
                      pending['class_id'], request.reason)

        return response

    def _on_timeout(self):
        if self._pending is None:
            return
        pending = self._pending
        on_approve = self._on_approve
        self._clear()

        self._node.get_logger().warn(
            f"APPROVAL TIMEOUT id={pending['detection_id']} after "
            f"{self._timeout_sec:.0f}s - deploying anyway")
        if self._log:
            self._log.event(
                'APPROVAL_TIMEOUT',
                f"id={pending['detection_id']}, class={pending['class_id']}, "
                f"timeout={self._timeout_sec:.0f}s, action=deploy")
        on_approve()

    # -- internals --------------------------------------------------------

    def _clear(self):
        self._pending = None
        self._on_approve = None
        self._on_reject = None
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        # Empty detection_id is the "nothing pending" convention the GCS uses.
        self._pub.publish(PendingDetection())
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `source ~/bv_ws/install/setup.bash && cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_approval_gate.py -v`
Expected: PASS (13 tests)

If `StubNode` fails on `get_clock`, that is expected — the implementation guards it with `hasattr`. Do not add a clock to the stub.

- [ ] **Step 5: Lint**

Run: `cd ~/bv_ws/src/bv_core && python3 -m flake8 --max-line-length=99 bv_core/approval_gate.py test/test_approval_gate.py`
Expected: no output

- [ ] **Step 6: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/approval_gate.py test/test_approval_gate.py
git commit -m "feat: ApprovalGate holds one pending detection and relays verdicts

mission_node is the authority: it mints the detection id and owns the timeout,
so the aircraft recovers even if the ground station or radio link dies. The
timeout fails open, running the same approve path, so the degraded mode is the
autonomous behavior already flown."
```

---

### Task 3: vision_node fills the annotated crop

**Files:**
- Modify: `../bv_msgs/srv/LocalizeObject.srv`
- Modify: `bv_core/vision_node.py` (config load near line 103; `_handle_localize_request` lines 475–612)
- Modify: `config/vision_params.yaml`

**Interfaces:**
- Consumes: `bv_core.detection_crop.CropConfig`, `build_annotated_crop` (Task 1).
- Produces:
  - `LocalizeObject.Response.annotated_crop` — JPEG bytes with `format='jpeg'`, or left empty on failure.
  - `LocalizeObject.Response.confidence` — `float32`, the detector confidence for the returned detection, `0.0` when unavailable.

- [ ] **Step 0: Add `confidence` to the localize service response**

`mission_node` needs the detector confidence to show the operator, but only
`vision_node` has it. Without this the dashboard would display `0%` beside a
crop whose own burned-in label reads `person 0.94` — two contradictory numbers
on one screen.

In `~/bv_ws/src/bv_msgs/srv/LocalizeObject.srv`, append to the **response**
block (after `annotated_crop`):

```
# Detector confidence for the returned detection, 0.0 when unavailable.
float32 confidence
```

This is another trailing field, so existing readers are unaffected.

- [ ] **Step 1: Add the crop tunables to `config/vision_params.yaml`**

Append:

```yaml
# ============================================================
# Operator review crop (human-in-the-loop approval)
# Window around the detection sent to the ground station. Adaptive because the
# real camera (4640px) and the sim (1280px) put a person at wildly different
# pixel heights; see docs/superpowers/specs/2026-08-05-hitl-mission-wiring-design.md
# ============================================================
crop_margin_factor: 2.5    # context multiplier on the bounding box
crop_min_px: 320           # floor, so a small sim detection still gets context
crop_max_px: 1024          # ceiling on the longest output side
crop_jpeg_quality: 85
```

- [ ] **Step 2: Load the tunables in `vision_node.py`**

Add the import near the other local imports (after `from .pipelines import create_pipeline`):

```python
from .detection_crop import CropConfig, build_annotated_crop
```

In the config loader, immediately after the line `self.frame_width_px = int(cfg.get('frame_width_px', 4640))`, add:

```python
        # Operator review crop (human-in-the-loop approval gate)
        self.crop_cfg = CropConfig(
            margin_factor=float(cfg.get('crop_margin_factor', 2.5)),
            min_px=int(cfg.get('crop_min_px', 320)),
            max_px=int(cfg.get('crop_max_px', 1024)),
            jpeg_quality=int(cfg.get('crop_jpeg_quality', 85)),
        )
```

- [ ] **Step 3: Carry the detection index through the class filter**

In `_handle_localize_request`, replace this block (currently at lines 577–599):

```python
        # Filter for the requested class if specified
        if target_cls >= 0:
            matched = [c for c in coords if int(c[2]) == target_cls]
            if matched:
                coords = matched
            else:
```

with:

```python
        # Pair each coordinate with the detection index that produced it.
        # localizer.get_lat_lon appends one result per input in order, so
        # index i of coords corresponds to detections.xyxy[i]. The index is
        # needed to crop the correct box for operator review.
        indexed_coords = list(enumerate(coords))

        # Filter for the requested class if specified
        if target_cls >= 0:
            matched = [pair for pair in indexed_coords
                       if int(pair[1][2]) == target_cls]
            if matched:
                indexed_coords = matched
            else:
```

Leave the `else:` body (the warning, the `LOCALIZE_RESULT` log, and `return response`) exactly as it is.

- [ ] **Step 4: Populate the response from the indexed pair**

Replace the response-building block (currently lines 601–612, beginning `# At this point we have at least one coordinate to return`) with:

```python
        # At this point we have at least one coordinate to return
        best_index, best_coord = indexed_coords[0]
        response.success = True
        response.latitude = best_coord[0]
        response.longitude = best_coord[1]
        response.altitude = drone_pose[2]
        response.class_id = int(best_coord[2])

        # Set outside the try below: confidence cannot fail to compute, and a
        # crop failure must not silently zero it — the GCS displays this number.
        response.confidence = (
            float(detections.confidence[best_index])
            if detections.confidence is not None
            else 0.0
        )

        # Annotated crop for the human-in-the-loop gate. A failure here must
        # never fail the localization: an encoding bug would silently turn into
        # the aircraft abandoning real targets. The ground station renders an
        # explicit "no image received" state instead.
        try:
            cls_name = (
                CLASS_NAMES[response.class_id]
                if 0 <= response.class_id < len(CLASS_NAMES)
                else f"class_{response.class_id}"
            )
            response.annotated_crop.header.stamp = \
                self.get_clock().now().to_msg()
            response.annotated_crop.format = 'jpeg'
            response.annotated_crop.data = build_annotated_crop(
                frame,
                detections.xyxy[best_index],
                f"{cls_name} {response.confidence:.2f}",
                self.crop_cfg,
            )
        except Exception as exc:   # noqa: BLE001 - degrade, never fail localize
            self.get_logger().warn(f"Annotated crop failed: {exc}")
            self.log.event('CROP_FAILED', f"reason={exc}")

        self.log.event('LOCALIZE_RESULT',
```

Leave the existing `LOCALIZE_RESULT` log arguments and the final `return response` unchanged.

- [ ] **Step 5: Build and verify the interface is populated**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_detection_crop.py -q
```
Expected: build succeeds, 12 tests pass.

- [ ] **Step 6: Verify the wiring compiles and imports cleanly**

Run: `cd ~/bv_ws && source install/setup.bash && python3 -c "import bv_core.vision_node; print('vision_node imports OK')"`
Expected: `vision_node imports OK`

- [ ] **Step 7: Lint**

Run: `cd ~/bv_ws/src/bv_core && python3 -m flake8 --max-line-length=99 bv_core/vision_node.py | head`
Expected: no *new* findings. This file has pre-existing findings; compare against `git stash` if unsure.

- [ ] **Step 8: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/vision_node.py config/vision_params.yaml
git commit -m "feat: return an annotated crop from the localize service

_handle_localize_request dropped the detection index when filtering by class,
so the bounding box that produced the fix was unrecoverable. Carry (index,
coord) pairs through the filter and crop that box. Crop failures degrade to an
empty image rather than failing localization, which would turn an encoding bug
into abandoned targets."
```

---

### Task 4: filtering_node spatial rejection suppression

**Files:**
- Modify: `bv_core/filtering_node.py` (subscriptions ~line 75; config load ~line 128; `filtered_detections` ~lines 193–198)
- Modify: `config/filtering_params.yaml`
- Test: `test/test_rejection_suppression.py`

**Interfaces:**
- Consumes: `/rejected_object_locations` (`bv_msgs/ObjectLocations`) published by Task 5.
- Produces: `FilteringNode._is_rejected(lat, lon, class_id) -> bool`.

- [ ] **Step 1: Update `config/filtering_params.yaml`**

Replace the existing `deployed_ignore_radius_deg` block with:

```yaml
# NOTE: deployed_ignore_radius_deg is NOT read by filtering_node. Suppression of
# already-delivered targets is class-wide (targets[cls]["state"] == "deployed"),
# not spatial. Kept only because mission.py logs the value. Do not wire it up
# without reading the phase 5 design doc: making deployed suppression spatial
# would let the drone re-deliver to a class it has already serviced.
deployed_ignore_radius_deg: 0.0001

# Ignore detections of the SAME CLASS within this radius of a location the
# operator rejected. Degrees lat/lon; 0.0001 deg is about 11 m, which covers the
# ~4 m localization error seen in sim while leaving a genuine second target
# 20 m away still findable. A different class at the same spot is NOT
# suppressed - the operator rejected a specific claim, not the ground.
rejected_ignore_radius_deg: 0.0001
```

- [ ] **Step 2: Write the failing test**

Create `test/test_rejection_suppression.py`:

```python
#!/usr/bin/env python3
"""Tests for rejection suppression logic in filtering_node.

Exercises the pure predicate directly — no ROS spin, no simulator.
Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_core.filtering_node import is_within_radius


class TestIsWithinRadius:
    def test_same_point_is_within(self):
        assert is_within_radius(38.3877, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_just_inside_radius(self):
        # 0.00005 deg north of the reference, radius 0.0001 deg.
        assert is_within_radius(38.38775, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_just_outside_radius(self):
        # 0.0002 deg north (~22 m) with an 11 m radius.
        assert not is_within_radius(38.3879, -76.4190, 38.3877, -76.4190, 0.0001)

    def test_diagonal_distance_uses_euclidean_not_per_axis(self):
        # 0.00008 in each axis: inside per-axis, outside as a circle (0.000113).
        assert not is_within_radius(
            38.38778, -76.41892, 38.3877, -76.4190, 0.0001)

    def test_zero_radius_matches_only_exact_point(self):
        assert is_within_radius(38.3877, -76.4190, 38.3877, -76.4190, 0.0)
        assert not is_within_radius(38.38771, -76.4190, 38.3877, -76.4190, 0.0)
```

- [ ] **Step 3: Run to verify it fails**

Run: `source ~/bv_ws/install/setup.bash && cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_rejection_suppression.py -v`
Expected: FAIL — `ImportError: cannot import name 'is_within_radius'`

- [ ] **Step 4: Add the module-level helper to `filtering_node.py`**

Immediately after the `CLASS_NAMES = ("person", "tent")` line, add:

```python
def is_within_radius(lat, lon, ref_lat, ref_lon, radius_deg):
    """True when (lat, lon) is inside radius_deg of (ref_lat, ref_lon).

    Euclidean in degrees — the mission area is small enough that treating
    degrees as a flat plane is well within the localization error, and it
    matches how proximity_threshold_deg is already used for 3-frame clustering.
    """
    d_lat = lat - ref_lat
    d_lon = lon - ref_lon
    return (d_lat * d_lat + d_lon * d_lon) <= (radius_deg * radius_deg)
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `source ~/bv_ws/install/setup.bash && cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_rejection_suppression.py -v`
Expected: PASS (5 tests)

- [ ] **Step 6: Wire the subscription and the predicate**

In `FilteringNode.__init__`, immediately after the `self.deployed_location_sub` block, add:

```python
        # Locations the operator rejected. Detections of the SAME class near
        # one of these are dropped before 3-frame confirmation, so a rejected
        # false positive cannot immediately re-confirm and re-interrupt.
        self.rejected_location_sub = self.create_subscription(
            ObjectLocations,
            '/rejected_object_locations',
            self.rejected_location_callback,
            reliable_qos
        )
        self.rejected_locations = []   # list of (lat, lon, class_id)
```

In the config-loading block, right after `self.proximity_threshold_deg = 0.0001`, add:

```python
        self.rejected_ignore_radius_deg = float(
            cfg.get('rejected_ignore_radius_deg', 0.0001))
```

Add the callback and predicate next to `deployed_location_callback`:

```python
    def rejected_location_callback(self, msg: ObjectLocations):
        """Record a location the operator rejected for a specific class."""
        entry = (msg.latitude, msg.longitude, int(msg.class_id))
        self.rejected_locations.append(entry)
        cls_name = (CLASS_NAMES[entry[2]]
                    if 0 <= entry[2] < len(CLASS_NAMES) else 'unknown')
        self.get_logger().info(
            f"Suppressing {cls_name} within "
            f"{self.rejected_ignore_radius_deg * 111320.0:.1f}m of "
            f"lat={entry[0]:.6f}, lon={entry[1]:.6f}")
        self.log.event('REJECTED_LOCATION',
                       f"lat={entry[0]:.6f}, lon={entry[1]:.6f}, "
                       f"class={cls_name}({entry[2]}), "
                       f"radius={self.rejected_ignore_radius_deg}deg")

    def _is_rejected(self, lat, lon, class_id):
        """True when this class was rejected by the operator near this point."""
        return any(
            int(class_id) == rej_cls
            and is_within_radius(lat, lon, rej_lat, rej_lon,
                                 self.rejected_ignore_radius_deg)
            for rej_lat, rej_lon, rej_cls in self.rejected_locations
        )
```

Finally, extend the `filtered_detections` comprehension (currently lines 193–198):

```python
        # filter to target classes that haven't been confirmed or deployed yet,
        # and drop anything the operator already rejected at this spot
        filtered_detections = [
            (lat, lon, cls)
            for lat, lon, cls in detections_global
            if cls in self.targets
            and self.targets[cls]["state"] == "undetected"
            and not self._is_rejected(lat, lon, cls)
        ]
```

Do **not** modify the reset at lines 345–349. Resetting non-deployed targets to
`undetected` on scan re-entry is now exactly the desired behavior — the class
stays huntable, and suppression is purely spatial.

- [ ] **Step 7: Write the failing integration test for the re-confirmation loop**

This is the test that proves the feature does what it exists to do: after a
rejection, the same false positive must stop reaching 3-frame confirmation.

It drives the real `FilteringNode` in-process with `rclpy.init()` but no spin,
and stubs `localizer.get_lat_lon` so the test exercises the suppression path
without depending on camera intrinsics, pose geometry, or the `Vector3`
detection encoding — none of which this task changes.

Append to `test/test_rejection_suppression.py`:

```python
import rclpy
from bv_msgs.msg import ObjectDetections, ObjectLocations
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

import pytest

from bv_core.filtering_node import FilteringNode

REJECT_LAT = 38.387711
REJECT_LON = -76.419098


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = FilteringNode()
    # Sensor preconditions: handle_detections returns early without these.
    gps = NavSatFix()
    gps.latitude, gps.longitude = 38.3876, -76.4190
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    node.gps_buffer.append(gps)
    node.pose_buffer.append(pose)
    node.last_rel_alt = Float64(data=15.2)
    node.state = 'scan'
    yield node
    node.destroy_node()


def _drive(node, class_id, frames=3):
    """Feed `frames` identical detections and report whether one confirmed."""
    published = []
    node.confirmed_pub.publish = lambda msg: published.append(int(msg.data))

    # Stub the projection: this test is about suppression, not geometry.
    node.localizer.get_lat_lon = (
        lambda *a, **k: [(REJECT_LAT, REJECT_LON, class_id)])

    node.frame_history = []
    node.targets[class_id]['state'] = 'undetected'

    for _ in range(frames):
        msg = ObjectDetections()
        msg.dets = [Vector3(x=640.0, y=360.0, z=float(class_id))]
        node.handle_detections(msg)

    return published


class TestRejectionLoop:
    def test_detection_confirms_before_any_rejection(self, node):
        assert _drive(node, class_id=0) == [0]

    def test_same_class_at_rejected_spot_stops_confirming(self, node):
        assert _drive(node, class_id=0) == [0]      # baseline

        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        # This is the loop the design exists to prevent.
        assert _drive(node, class_id=0) == []

    def test_different_class_at_rejected_spot_still_confirms(self, node):
        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        # Operator rejected "person here", not the ground itself.
        assert _drive(node, class_id=1) == [1]

    def test_same_class_far_away_still_confirms(self, node):
        rejection = ObjectLocations()
        rejection.latitude = REJECT_LAT + 0.01      # ~1.1 km away
        rejection.longitude = REJECT_LON
        rejection.class_id = 0
        node.rejected_location_callback(rejection)

        assert _drive(node, class_id=0) == [0]
```

- [ ] **Step 8: Run the integration test**

Run: `source ~/bv_ws/install/setup.bash && cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_rejection_suppression.py -v`
Expected: PASS (9 tests total — 5 radius + 4 loop)

If `test_same_class_at_rejected_spot_stops_confirming` fails, the suppression
clause in `filtered_detections` is not wired correctly — that assertion is the
entire point of this task, so do not proceed past it.

- [ ] **Step 9: Build and run the full test suite**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
cd ~/bv_ws/src/bv_core && python3 -m pytest test/test_rejection_suppression.py test/test_detection_crop.py test/test_approval_gate.py -v
```
Expected: PASS (34 tests)

- [ ] **Step 10: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/filtering_node.py config/filtering_params.yaml test/test_rejection_suppression.py
git commit -m "feat: suppress rejected detections by location and class

Without this, rejecting a false positive resumes the scan, filtering resets the
class to undetected, and the same blob re-confirms three frames later - an
infinite interrupt loop on exactly the detection the operator dismissed.
Suppression is spatial and class-specific so a different class at the same spot,
or the same class elsewhere, still triggers.

Also documents that deployed_ignore_radius_deg is dead config."
```

---

### Task 5: mission_node wiring

**Files:**
- Modify: `bv_core/mission.py` (imports ~line 41; `load_config_from_yaml` ~line 130; `init_state_variables` ~line 190; `setup_ros_interfaces` ~line 262; `on_vision_localization_complete` ~line 963; `enter_rtl_state` ~line 569)
- Modify: `config/mission_params.yaml`

**Interfaces:**
- Consumes: `bv_core.approval_gate.ApprovalGate` (Task 2), `LocalizeObject.Response.annotated_crop` (Task 3).
- Produces: `/rejected_object_locations` (`bv_msgs/ObjectLocations`) consumed by Task 4.

- [ ] **Step 1: Add the config keys to `config/mission_params.yaml`**

Append:

```yaml
# ============================================================
# Human-in-the-loop approval gate
# ============================================================
# When true, the drone holds in AUTO.LOITER after localizing and waits for an
# operator verdict before flying to the object. Requires bv_gcs to be built.
Approval_required: false

# Seconds before auto-approving. FAILS OPEN: on expiry the drone deploys and
# continues, so a dead link or an absent operator can never strand it. 0 waits
# forever. The timer lives here, in mission_node, so it survives a ground
# station crash.
Approval_timeout_sec: 180.0
```

- [ ] **Step 2: Import the gate**

After `from .mission_logger import MissionLogger` add:

```python
from .approval_gate import ApprovalGate
```

- [ ] **Step 3: Load the config**

In `load_config_from_yaml`, after the existing tolerance/velocity reads, add:

```python
        # Human-in-the-loop approval gate
        self.approval_required = bool(config.get('Approval_required', False))
        self.approval_timeout_sec = float(
            config.get('Approval_timeout_sec', 180.0))
```

- [ ] **Step 4: Initialize the gate handle**

In `init_state_variables`, after `self.localization_retry_count = 0`, add:

```python
        # Operator approval gate; None when flying fully autonomously
        self.approval_gate = None
```

- [ ] **Step 5: Create the publisher and the gate**

In `setup_ros_interfaces`, immediately after the `self.object_detected_sub` block, add:

```python
        # Locations the operator rejected, so filtering_node stops
        # re-confirming the same false positive.
        self.rejected_object_pub = self.create_publisher(
            ObjectLocations,
            '/rejected_object_locations',
            qos_profile=reliable_qos
        )
```

At the very end of `setup_ros_interfaces` (after `self.localize_object_client`), add:

```python
        if self.approval_required:
            self.approval_gate = ApprovalGate(
                self, self.approval_timeout_sec, self.log)
            self.get_logger().info(
                f"Human-in-the-loop approval ENABLED "
                f"(timeout={self.approval_timeout_sec:.0f}s, fails open)")
        else:
            self.get_logger().info(
                "Human-in-the-loop approval disabled - flying autonomously")
```

- [ ] **Step 6: Branch in `on_vision_localization_complete`**

Replace the final two lines of `on_vision_localization_complete`:

```python
        # Proceed to delivery
        self.enter_deliver_state()
```

with:

```python
        # Proceed to delivery, or hold for an operator verdict first.
        if self.approval_gate is None:
            self.enter_deliver_state()
            return

        crop = bytes(response.annotated_crop.data)
        if not crop:
            self.get_logger().warn(
                "Localization returned no annotated crop - the operator will "
                "have to judge without an image")

        self.approval_gate.request(
            class_id=self.current_target_class_id,
            lat=response.latitude,
            lon=response.longitude,
            alt=response.altitude,
            confidence=response.confidence,
            drone_lat=self.current_lat if self.current_lat is not None else 0.0,
            drone_lon=self.current_lon if self.current_lon is not None else 0.0,
            annotated_crop=crop,
            on_approve=self._on_approval_granted,
            on_reject=self._on_approval_rejected,
        )
```

- [ ] **Step 7: Add the two callbacks**

Add immediately after `on_vision_localization_complete`:

```python
    def _on_approval_granted(self):
        """Operator approved, or the timeout expired. Deliver as normal."""
        if self.current_state != STATE_LOCALIZE:
            # The FSM moved on (RTL, abandon). Ignore a late verdict.
            self.get_logger().warn(
                f"Approval arrived in state '{self.current_state}' - ignoring")
            return
        self.enter_deliver_state()

    def _on_approval_rejected(self, lat, lon, class_id, reason):
        """Operator rejected. Suppress this spot for this class, resume scan."""
        if self.current_state != STATE_LOCALIZE:
            self.get_logger().warn(
                f"Rejection arrived in state '{self.current_state}' - ignoring")
            return

        rejected_msg = ObjectLocations()
        rejected_msg.latitude = float(lat)
        rejected_msg.longitude = float(lon)
        rejected_msg.class_id = int(class_id)
        self.rejected_object_pub.publish(rejected_msg)

        cls_name = (CLASS_NAMES[int(class_id)]
                    if 0 <= int(class_id) < len(CLASS_NAMES) else 'unknown')
        self.get_logger().info(
            f"Operator rejected {cls_name} at lat={lat:.6f}, lon={lon:.6f}"
            + (f" ({reason})" if reason else "") + " - resuming scan")

        # Same cleanup as the abandon-after-5-failures path.
        self.current_target_coords = None
        self.current_target_class_id = None
        self.confirmed_detection_class_id = -1
        self.enter_scan_state()
```

- [ ] **Step 8: Cancel a pending on RTL**

At the top of `enter_rtl_state`, before `self.current_state = STATE_RTL`, add:

```python
        if self.approval_gate is not None and self.approval_gate.is_pending():
            self.approval_gate.cancel('rtl')
```

- [ ] **Step 9: Build and verify the autonomous default is untouched**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
python3 -c "import bv_core.mission; print('mission imports OK')"
grep -n "Approval_required" ~/bv_ws/install/bv_core/share/bv_core/config/mission_params.yaml
```
Expected: imports OK, and `Approval_required: false` present in the installed config.

- [ ] **Step 10: Run the full suite**

Run: `cd ~/bv_ws/src/bv_core && python3 -m pytest test/ -v --ignore=test/test_flake8.py --ignore=test/test_pep257.py`
Expected: PASS. (`test_flake8` and `test_pep257` have 397 pre-existing failures unrelated to this work; they are excluded here and checked per-file instead.)

- [ ] **Step 11: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add bv_core/mission.py config/mission_params.yaml
git commit -m "feat: gate delivery on operator approval when enabled

Branches at on_vision_localization_complete. Approve and timeout converge on
the same enter_deliver_state() call so the fail-open path cannot drift from the
approve path. Reject publishes the location for spatial suppression and reuses
the existing abandon cleanup. Defaults to false: an unflagged launch flies
exactly as before."
```

---

### Task 6: Launch wiring, GCS QoS fix, and the sim check tool

**Files:**
- Modify: `launch/mission.launch.py`
- Modify: `../bv_gcs/bv_gcs/approval_node.py` (QoS durability)
- Create: `bv_core/sim_approval_check.py`
- Modify: `setup.py` (register the sim check as a console script)

**Interfaces:**
- Consumes: `Approval_required` from `mission_params.yaml` (Task 5), `/pending_obj_dets` (Task 2).
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Match the subscription durability in `approval_node`**

In `~/bv_ws/src/bv_gcs/bv_gcs/approval_node.py`, in `ApprovalNode.__init__`, change the `reliable` profile used for `/pending_obj_dets` so a restarted node receives the latched pending. Replace:

```python
        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        reliable.history = HistoryPolicy.KEEP_LAST
```

with:

```python
        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        reliable.history = HistoryPolicy.KEEP_LAST

        # TRANSIENT_LOCAL matches mission_node's latched publisher, so a
        # restarted approval_node or a recovered link immediately receives the
        # live pending instead of showing an empty dashboard.
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.history = HistoryPolicy.KEEP_LAST
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
```

Add `DurabilityPolicy` to the existing rclpy.qos import, and change the
`/pending_obj_dets` subscription to use `latched` instead of `reliable`.
Leave `/mission_state` on `reliable`.

- [ ] **Step 2: Conditionally launch the GCS from `mission.launch.py`**

Replace the whole file with:

```python
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _approval_required():
    """Read Approval_required from mission_params.yaml.

    Read at launch-generation time rather than passed as a launch argument
    because mission_node reads all its configuration from this same YAML; a
    launch argument would be a second source of truth for one setting.
    """
    config_path = os.path.join(
        get_package_share_directory('bv_core'),
        'config',
        'mission_params.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f) or {}
    return bool(config.get('Approval_required', False))


def generate_launch_description():
    mission_node = Node(
        package='bv_core',
        name='mission_node',
        executable='mission_node',
        output='both'
    )

    vision_node = Node(
        package='bv_core',
        executable='vision_node',
        name='vision_node',
        output='both',
    )

    stitching_node = Node(
        package='bv_core',
        executable='stitching_node',
        name='stitching_node',
        output='both',
    )

    filter_node = Node(
        package='bv_core',
        executable='filtering_node',
        name='filtering_node',
        output='both',
    )

    bv_viz_node = Node(
        package='bv_core',
        executable='bv_viz_node',
        name='bv_viz_node',
        output='both',
    )

    actions = [
        mission_node,
        vision_node,
        filter_node,
        stitching_node,
        bv_viz_node,
    ]

    # bv_gcs is only referenced when the gate is enabled, so an unbuilt bv_gcs
    # can never break an autonomous launch.
    if _approval_required():
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('bv_gcs'),
                'launch', 'gcs.launch.py'))
        ))

    return LaunchDescription(actions)
```

- [ ] **Step 3: Verify the launch file generates in both modes**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core bv_gcs && source install/setup.bash
ros2 launch bv_core mission.launch.py --show-args
python3 -c "
import sys; sys.argv=['x']
from launch import LaunchDescription
import importlib.util, os
from ament_index_python.packages import get_package_share_directory
p = os.path.join(get_package_share_directory('bv_core'),'launch','mission.launch.py')
spec = importlib.util.spec_from_file_location('ml', p); m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
ld = m.generate_launch_description()
print('actions with gate disabled:', len(ld.entities))
"
```
Expected: 5 actions with `Approval_required: false`.

Then set `Approval_required: true` in `config/mission_params.yaml`, rebuild, re-run the snippet — expect **6** actions. Set it back to `false` and rebuild before committing.

- [ ] **Step 4: Write the sim check tool**

Create `bv_core/sim_approval_check.py`. It lives in the package rather than
`test/` because `setup.py:8` uses `find_packages(exclude=['test'])`, so a
console script under `test/` would never be installed.

```python
#!/usr/bin/env python3
"""Watch a live mission and print a PASS/FAIL verdict for the approval gate.

Run alongside the sim so an operator does not have to read raw logs:

    ros2 run bv_core sim_approval_check

Then fly the mission and act on the detection. Ctrl-C prints the verdict.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from bv_msgs.msg import ObjectLocations, PendingDetection
from std_msgs.msg import String


class SimApprovalCheck(Node):

    def __init__(self):
        super().__init__('sim_approval_check')

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.history = HistoryPolicy.KEEP_LAST
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            PendingDetection, '/pending_obj_dets', self._on_pending, latched)
        self.create_subscription(
            String, '/mission_state', self._on_state, reliable)
        self.create_subscription(
            ObjectLocations, '/rejected_object_locations',
            self._on_rejected, reliable)

        self.states = []
        self.pendings = []
        self.rejections = []
        self.state_at_pending = None

        self.get_logger().info(
            'sim_approval_check watching. Fly the mission, then Ctrl-C.')

    def _on_state(self, msg):
        if not self.states or self.states[-1] != msg.data:
            self.states.append(msg.data)

    def _on_pending(self, msg):
        if not msg.detection_id:
            return
        self.state_at_pending = self.states[-1] if self.states else None
        self.pendings.append({
            'id': msg.detection_id,
            'class_id': int(msg.class_id),
            'crop_bytes': len(msg.annotated_crop.data),
            'timeout': float(msg.timeout_sec),
            'state': self.state_at_pending,
        })
        self.get_logger().info(
            f"pending #{len(self.pendings)} crop="
            f"{len(msg.annotated_crop.data)}B state={self.state_at_pending}")

    def _on_rejected(self, msg):
        self.rejections.append((msg.latitude, msg.longitude, int(msg.class_id)))

    def verdict(self):
        checks = []

        if not self.pendings:
            checks.append((False, 'a pending detection was published'))
        else:
            first = self.pendings[0]
            checks.append((True, f"{len(self.pendings)} pending(s) published"))
            checks.append((
                first['state'] == 'localize',
                f"pending published while state=localize "
                f"(saw {first['state']})"))
            checks.append((
                first['crop_bytes'] > 0,
                f"pending carried a crop ({first['crop_bytes']} bytes)"))
            checks.append((
                first['timeout'] > 0,
                f"timeout advertised to the GCS ({first['timeout']:.0f}s)"))

        reached_deliver = 'deliver' in self.states
        reached_deploy = 'deploy' in self.states

        if self.rejections:
            checks.append((True,
                           f"{len(self.rejections)} rejection(s) published "
                           f"for filtering suppression"))
            checks.append((
                self.states.count('scan') >= 2,
                'returned to scan after rejection'))
        else:
            checks.append((reached_deliver, 'reached DELIVER'))
            checks.append((reached_deploy, 'reached DEPLOY'))

        print('\n[sim-check] state sequence: ' + ' -> '.join(self.states))
        for ok, text in checks:
            print(f"  {'PASS' if ok else 'FAIL'}  {text}")
        result = all(ok for ok, _ in checks)
        print(f"  RESULT: {'PASS' if result else 'FAIL'}\n")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimApprovalCheck()
    passed = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        passed = node.verdict()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
```

- [ ] **Step 5: Register the console script**

In `setup.py`, add to `entry_points['console_scripts']`:

```python
            'sim_approval_check = bv_core.sim_approval_check:main',
```

- [ ] **Step 6: Build and smoke-test the tool without a sim**

Run:
```bash
cd ~/bv_ws && colcon build --packages-select bv_core && source install/setup.bash
timeout 3 ros2 run bv_core sim_approval_check || true
```
Expected: the node starts and logs `sim_approval_check watching.`

- [ ] **Step 7: Lint the new files**

Run:
```bash
cd ~/bv_ws/src/bv_core && python3 -m flake8 --max-line-length=99 \
  launch/mission.launch.py bv_core/sim_approval_check.py
cd ~/bv_ws/src/bv_gcs && python3 -m flake8 --max-line-length=99 bv_gcs/
```
Expected: no output from either.

- [ ] **Step 8: Commit**

```bash
cd ~/bv_ws/src/bv_core
git add launch/mission.launch.py setup.py bv_core/sim_approval_check.py
git commit -m "feat: launch the GCS when approval is enabled, add sim check tool

mission.launch.py reads Approval_required from the same YAML mission_node uses,
so there is one source of truth. bv_gcs is only referenced when the gate is on.
sim_approval_check turns an operator sim run into a single PASS/FAIL line."

cd ~/bv_ws/src/bv_gcs
git add bv_gcs/approval_node.py
git commit -m "fix: subscribe to /pending_obj_dets with TRANSIENT_LOCAL

Matches mission_node's latched publisher so a restarted approval_node, or a
Herelink link that drops and recovers mid-decision, immediately receives the
live pending instead of an empty dashboard."
```

---

## Final verification (after all tasks)

- [ ] **Automated — run everything**

```bash
cd ~/bv_ws && colcon build && source install/setup.bash
cd ~/bv_ws/src/bv_core && python3 -m pytest test/ -v \
  --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```
Expected: all pass. `test_flake8` / `test_pep257` carry 397 pre-existing
failures unrelated to this work — verify per-file with flake8 instead.

- [ ] **Automated — GCS path end to end, no aircraft**

```bash
ros2 launch bv_gcs gcs.launch.py
ros2 run bv_gcs fake_pending --ros-args -p timeout_sec:=45.0
```
Open `http://localhost:8765`, press **A** and **R**, confirm both land in the
`fake_pending` log and the panel clears each time.

- [ ] **Operator — sim run 1: regression (most important)**

`Approval_required: false`. Fly the normal mission. The state sequence must
match the baseline autonomous run: `takeoff -> lap -> scan -> localize ->
deliver -> deploy -> scan -> ... -> return`. No `/pending_obj_dets` traffic, no
`approval_node` process.

- [ ] **Operator — sim run 2: approve**

`Approval_required: true`. Run `ros2 run bv_core sim_approval_check` alongside.
Approve in the browser. Expect `RESULT: PASS` with a non-empty crop.

- [ ] **Operator — sim run 3: timeout in flight**

`Approval_timeout_sec: 15.0`. Do not respond. The drone must deploy anyway,
with `APPROVAL_TIMEOUT` in the mission log.

## Open items to confirm with the operator before sim runs

1. The exact sim launch invocation (PX4 + Gazebo + MAVROS).
2. Whether the uncommitted changes in `mission_params.yaml`,
   `vision_params.yaml`, and `filtering_params.yaml` are the values actually
   flown — tests should not be written against stale waypoints.
