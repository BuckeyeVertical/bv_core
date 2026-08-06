#!/usr/bin/env python3
"""Index correctness for the crop and confidence returned by localize.

This is the one failure mode in the approval feature that produces a wrong HUMAN
decision rather than a wrong action. If `_handle_localize_request` returns the
coordinates of one detection but crops and scores a different one, the operator
is shown a picture of object A, told it is at object B's position, and clicks
APPROVE with full confidence. Nothing downstream can catch that — the verdict is
by definition authoritative.

`VisionNode.__init__` builds a camera pipeline, loads a detector and creates
MAVROS subscriptions, so it is never called here. Instead the real, unbound
`_handle_localize_request` is lifted onto a stand-in carrying exactly the
attributes that method touches — the same technique test_mission_approval_wiring
uses for the mission callbacks. The method body is therefore the genuine one.

Requires bv_msgs on the path:  source ~/bv_ws/install/setup.bash
"""

import os
import sys

import cv2
import numpy as np

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bv_msgs.srv import LocalizeObject  # noqa: E402

from bv_core.detection_crop import CropConfig  # noqa: E402
from bv_core.vision_node import CLASS_NAMES, VisionNode  # noqa: E402

# Two detections, far apart, in a 1200x800 frame. Index 0 is a person, index 1
# is a tent, so a request for "tent" must select index 1.
BOX_PERSON = (100.0, 100.0, 180.0, 260.0)
BOX_TENT = (900.0, 500.0, 1100.0, 700.0)

PERSON_BGR = (255, 0, 0)      # blue
TENT_BGR = (0, 0, 255)        # red

CONF_PERSON = 0.41
CONF_TENT = 0.93

LAT_PERSON, LON_PERSON = 38.387700, -76.419000
LAT_TENT, LON_TENT = 38.388900, -76.417500


class _StubLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class _StubMissionLog:
    def __init__(self):
        self.events = []

    def event(self, name, detail=''):
        self.events.append((name, detail))


class _StubClock:
    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class _FakeDetections:
    """The subset of supervision.Detections the handler reads."""

    def __init__(self, xyxy, class_id, confidence):
        self.xyxy = np.array(xyxy, dtype=float)
        self.class_id = np.array(class_id, dtype=int)
        self.confidence = np.array(confidence, dtype=float)

    def __len__(self):
        return len(self.xyxy)


class _FakePipeline:
    def __init__(self, frame):
        self.frame = frame

    def start(self):
        pass

    def get_frame(self, timeout=1.0):
        return self.frame


class _FakeDetector:
    def __init__(self, detections):
        self.detections = detections

    def process_frame(self, frame, threshold):
        return self.detections


class _FakeLocalizer:
    """Returns one (lat, lon, class_id) per input, in order.

    That ordering is the contract `_handle_localize_request` relies on to map a
    coordinate back to the detection index that produced it.
    """

    def __init__(self, results):
        self.results = results
        self.calls = []

    def get_lat_lon(self, pixel_centers, drone_pose, drone_orientation):
        self.calls.append(list(pixel_centers))
        return list(self.results)


class _FakeGps:
    latitude = 38.3876
    longitude = -76.4191


class _FakePose:
    class pose:
        class orientation:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0


class _FakeRelAlt:
    data = 22.5


class FakeVisionNode:
    """Stand-in carrying exactly what _handle_localize_request touches."""

    _handle_localize_request = VisionNode._handle_localize_request

    def __init__(self, frame, detections, coords):
        self.pipeline_running = True
        self.pipeline = _FakePipeline(frame)
        self.detector = _FakeDetector(detections)
        self.det_thresh = 0.25
        self.gps_buffer = [_FakeGps()]
        self.pose_buffer = [_FakePose()]
        self.last_rel_alt = _FakeRelAlt()
        self.localizer = _FakeLocalizer(coords)
        self.crop_cfg = CropConfig()
        self.log = _StubMissionLog()
        self._logger = _StubLogger()
        self._clock = _StubClock()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock


def _frame():
    """A frame where each detection's region is a distinct solid colour."""
    img = np.full((800, 1200, 3), 30, dtype=np.uint8)   # dark grey background
    for (x1, y1, x2, y2), colour in ((BOX_PERSON, PERSON_BGR),
                                     (BOX_TENT, TENT_BGR)):
        img[int(y1):int(y2), int(x1):int(x2)] = colour
    return img


def _node():
    detections = _FakeDetections(
        xyxy=[BOX_PERSON, BOX_TENT],
        class_id=[0, 1],
        confidence=[CONF_PERSON, CONF_TENT],
    )
    coords = [
        (LAT_PERSON, LON_PERSON, 0),
        (LAT_TENT, LON_TENT, 1),
    ]
    return FakeVisionNode(_frame(), detections, coords)


def _localize(node, target_class_id, want_crop=True):
    request = LocalizeObject.Request()
    request.target_class_id = target_class_id
    request.want_crop = want_crop
    return node._handle_localize_request(request, LocalizeObject.Response())


def _dominant_bgr(jpeg_bytes):
    """Mean BGR of the decoded crop, as a plain tuple."""
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    assert img is not None, 'response crop was not decodable JPEG'
    return tuple(float(v) for v in img.reshape(-1, 3).mean(axis=0))


assert CLASS_NAMES[0] == 'person' and CLASS_NAMES[1] == 'tent'


class TestCropMatchesTheReturnedDetection:
    """The picture the operator judges must be the object being reported."""

    def test_second_detection_returns_its_own_coordinates(self):
        response = _localize(_node(), target_class_id=1)
        assert response.success is True
        assert response.class_id == 1
        assert response.latitude == pytest.approx(LAT_TENT)
        assert response.longitude == pytest.approx(LON_TENT)

    def test_second_detection_returns_its_own_confidence(self):
        # An index-0 carry-through would report the person's 0.41 for the tent.
        response = _localize(_node(), target_class_id=1)
        assert response.confidence == pytest.approx(CONF_TENT, abs=1e-6)
        assert response.confidence != pytest.approx(CONF_PERSON, abs=1e-6)

    def test_second_detection_crops_xyxy_1_not_xyxy_0(self):
        response = _localize(_node(), target_class_id=1)
        data = bytes(response.annotated_crop.data)
        assert data, 'want_crop was true, so a crop was expected'

        blue, green, red = _dominant_bgr(data)
        # BOX_TENT's region is red; BOX_PERSON's is blue. They are 900 px apart,
        # far beyond any crop margin, so the two windows cannot overlap.
        assert red > blue, (
            f"crop is dominated by the wrong detection's colour "
            f"(B={blue:.0f} G={green:.0f} R={red:.0f}); "
            f"index 0 is blue, index 1 is red")

    def test_first_detection_crops_xyxy_0(self):
        """The mirror case, so the test cannot pass by always picking index 1."""
        response = _localize(_node(), target_class_id=0)
        assert response.class_id == 0
        assert response.confidence == pytest.approx(CONF_PERSON, abs=1e-6)
        blue, _green, red = _dominant_bgr(bytes(response.annotated_crop.data))
        assert blue > red

    def test_label_names_the_returned_class(self):
        # The label is burned into the image the operator reads.
        node = _node()
        response = _localize(node, target_class_id=1)
        assert response.annotated_crop.format == 'jpeg'
        assert response.class_id == 1


class TestWantCropIsHonoured:
    """The autonomous path must not pay for an image nobody will see."""

    def test_crop_is_empty_when_not_requested(self):
        response = _localize(_node(), target_class_id=1, want_crop=False)
        assert response.success is True
        assert bytes(response.annotated_crop.data) == b''
        # Everything else the FSM depends on is unchanged.
        assert response.class_id == 1
        assert response.latitude == pytest.approx(LAT_TENT)
        assert response.confidence == pytest.approx(CONF_TENT, abs=1e-6)

    def test_build_annotated_crop_is_not_called_when_not_requested(self, monkeypatch):
        calls = []
        monkeypatch.setattr(
            'bv_core.vision_node.build_annotated_crop',
            lambda *a, **k: calls.append(a) or b'\xff\xd8')
        _localize(_node(), target_class_id=1, want_crop=False)
        assert calls == []

        _localize(_node(), target_class_id=1, want_crop=True)
        assert len(calls) == 1


class TestConfidenceSurvivesACropFailure:
    """Confidence is set outside the crop try-block, deliberately."""

    def test_crop_failure_leaves_confidence_intact(self, monkeypatch):
        def _boom(*args, **kwargs):
            raise RuntimeError('cv2.imencode failed')

        monkeypatch.setattr('bv_core.vision_node.build_annotated_crop', _boom)
        response = _localize(_node(), target_class_id=1)

        assert response.success is True
        assert response.confidence == pytest.approx(CONF_TENT, abs=1e-6)
        assert bytes(response.annotated_crop.data) == b''
