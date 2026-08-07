#!/usr/bin/env python3
"""
Vision Node for SUAS Drone System

Handles camera frame capture and object detection during scan missions.
Uses a producer-consumer pattern with separate threads for frame fetching
and detection processing.
"""

# Imports

# MUST STAY FIRST, above cv2 and everything else. Importing it pins libgcc's
# C++ unwinder before OpenCV/GStreamer can map libunwind and break Fast-DDS
# exception handling for the rest of the process. On a GStreamer-built OpenCV
# (the Jetson) `import cv2` alone maps libunwind, so a pin placed any lower is
# already too late. See bv_core/_unwinder.py.
import bv_core._unwinder  # noqa: F401  # isort: skip  (side-effect; keep first)

import os
import queue
import threading
import time
import traceback
import numpy as np
import yaml
import cv2

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

# ROS2 messages
from std_msgs.msg import String, Int8, Float64, Bool, UInt8MultiArray
from sensor_msgs.msg import CompressedImage, NavSatFix
from bv_msgs.msg import ObjectDetections
from bv_msgs.srv import LocalizeObject
from geometry_msgs.msg import Vector3, PoseStamped
from mavros_msgs.msg import WaypointReached
from collections import deque

# Local - using factory functions for lazy imports
from .detectors import create_detector
from .pipelines import create_pipeline
from .detection_crop import CropConfig, build_annotated_crop
from .localizer import Localizer
from .mission_logger import MissionLogger
from .preview_stream import PreviewConfig, PreviewStream, pin_libgcc_unwinder
from .stitch_geometry import along_track_m, compute_step_m, distance_m

# ROS2 utilities
from ament_index_python.packages import get_package_share_directory

import supervision as sv

# Before any GStreamer can load, and before rclpy builds anything. GStreamer
# maps libunwind, which hijacks the _Unwind_* symbols Fast-DDS needs from
# libgcc and turns its internal exceptions into an abort() with no traceback.
# This cannot live only in preview_stream._init_gst: on the Jetson, OpenCV is
# built with GStreamer, so cv2.VideoCapture(..., CAP_GSTREAMER) in
# CameraPipeline initialises GStreamer during _init_pipeline without ever
# reaching _init_gst — on an ordinary flight with the preview never enabled.
# See pin_libgcc_unwinder's docstring for the full mechanism and repro.
pin_libgcc_unwinder()

CLASS_NAMES = ("person", "tent")

# How long destroy_node waits for the preview toggle worker. Above
# PreviewStream.start()'s 12-18 s worst case (three tiers x a 3 s get_state,
# plus shutdown waits) so a toggle in flight completes rather than being
# abandoned half-built. Correctness does not rest on this number: destroy_node
# re-checks whether the worker outlasted it and stops the stream again if so.
PREVIEW_WORKER_JOIN_SEC = 25.0


# Fail on the ground, not in the air.
#
# rosidl Python messages use __slots__ with property setters, so assigning a
# field that does not exist raises AttributeError. Against a stale bv_msgs that
# would happen inside the localize service callback, on EVERY localization, even
# with the approval gate disabled: the FSM would see a failed localize, retry
# five times and abandon a real target — silently, in flight. mission.py already
# fails at startup in this scenario (its ApprovalGate import pulls in the new
# bv_msgs types), and this makes vision_node do the same.
def _assert_bv_msgs_is_current():
    _request = LocalizeObject.Request()
    _response = LocalizeObject.Response()
    _missing = [name for obj, name in ((_response, 'Response.confidence'),
                                       (_response, 'Response.annotated_crop'),
                                       (_request, 'Request.want_crop'))
                if not hasattr(obj, name.split('.', 1)[1])]
    if _missing:
        raise RuntimeError(
            f"bv_msgs is out of date: LocalizeObject is missing "
            f"{', '.join(_missing)}. Rebuild bv_msgs (branch "
            f"feat/hitl-interfaces) before flying:  "
            f"colcon build --packages-select bv_msgs bv_core")


_assert_bv_msgs_is_current()


# Vision Node

class VisionNode(Node):
    """
    ROS2 node for capturing and processing camera frames during scan missions.
    
    Architecture:
        - Main thread: ROS2 callbacks and publishing
        - Frame fetcher thread: Captures frames from camera pipeline
        - Worker thread: Runs object detection on queued frames
    """

    
    # Initialization
    

    def __init__(self):
        super().__init__('vision_node')

        self._load_config()
        self._init_state()
        self.log = MissionLogger('vision')
        self._frame_counter = 0
        self._last_detection_time = time.monotonic()
        self._init_pipeline()
        self._init_threading()
        self._init_subscribers()
        self._init_publishers()
        self._init_detector()
        self._init_localizer()
        self._init_services()
        self._start_worker_threads()
        # Index retained for legacy image naming conventions.
        self.frame_number = 1
        os.makedirs("raw_frames", exist_ok=True)

    def _load_config(self):
        """Load configuration from vision_params.yaml."""
        vision_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'vision_params.yaml'
        )

        with open(vision_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        # Detection settings
        self.det_thresh = cfg.get('detection_threshold', 0.5)
        self.num_scan_wp = cfg.get('num_scan_wp', 3)
        self.capture_interval = float(cfg.get('capture_interval', 1.5e9))

        # Debug preview stream
        self.preview_cfg = PreviewConfig(
            width=int(cfg.get('preview_width', 640)),
            fps=float(cfg.get('preview_fps', 8.0)),
            bitrate_bps=int(cfg.get('preview_bitrate_bps', 400_000)),
        )

        # Detector configuration
        self.detector_type = cfg.get('detector_type', 'ml')
        self.ml_model_path = cfg.get(
            'ml_model_path',
            '/Users/allenthomas/Code/Personal/inference/ltdetr.pt'
        )
        self.gazebo_bbox_topic = cfg.get('gazebo_bbox_topic', '/camera/bounding_boxes')

        # Pipeline configuration
        self.pipeline_type = cfg.get('pipeline_type', 'sim')
        self.gz_topic = cfg.get(
            'gz_topic',
            '/world/baylands/model/x500_gimbal_0/link/camera_link/sensor/camera/image'
        )
        self.gst_pipeline_str = cfg.get(
            'gst_pipeline',
            'v4l2src device=/dev/video0 io-mode=2 do-timestamp=true ! video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! queue max-size-buffers=1 leaky=downstream ! appsink drop=true sync=false'
        )
        self.camera_fps = cfg.get('camera_fps', 30.0)
        self.record_video = cfg.get('record_video', False)
        self.ros_image_topic = cfg.get('ros_image_topic', '/image_compressed')

        # Stitching capture
        self.stitch_overlap = float(cfg.get('stitch_overlap', 0.35))
        self.frame_width_px = int(cfg.get('frame_width_px', 4640))

        # Operator review crop (human-in-the-loop approval gate)
        self.crop_cfg = CropConfig(
            margin_factor=float(cfg.get('crop_margin_factor', 2.5)),
            min_px=int(cfg.get('crop_min_px', 320)),
            max_px=int(cfg.get('crop_max_px', 1024)),
            jpeg_quality=int(cfg.get('crop_jpeg_quality', 85)),
        )

        # Scan waypoints + takeoff altitude come from mission_params.yaml
        mission_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'mission_params.yaml'
        )
        with open(mission_yaml, 'r') as f:
            mcfg = yaml.safe_load(f)
        self.scan_points = mcfg.get('scan_points', [])
        self.takeoff_alt = float(mcfg.get('takeoff_alt', 15.24))
        self.scan_tolerance = float(mcfg.get('Scan_tolerance', 2.0))

    def _init_state(self):
        """Initialize state tracking variables."""
        self.prev_state = ""
        self.state = ""
        self.latest_wp = None
        self.last_wp = None
        self.obj_dets = []
        
        # GPS/pose buffers for localization
        self.gps_buffer = deque(maxlen=200)
        self.pose_buffer = deque(maxlen=200)
        self.last_rel_alt = None

        # Stitch capture state
        fx = self._read_fx_from_filtering_yaml()
        self.step_m = compute_step_m(
            frame_width_px=self.frame_width_px,
            fx=fx,
            altitude_m=self.takeoff_alt,
            overlap=self.stitch_overlap,
        )
        self.row_anchor_ll = None   # (lat, lon) of active row anchor, or None
        self.row_next_ll = None     # (lat, lon) of active row endpoint
        self.next_capture_m = 0.0
        self.col_idx = 1
        self.raw_frames_cleared = False
        self.curr_wp = -1
        self.stitch_paused = False
        self.get_logger().info(
            f"Stitch capture: step_m={self.step_m:.2f} "
            f"(overlap={self.stitch_overlap}, alt={self.takeoff_alt}, fx={fx:.1f})"
        )

    def _read_fx_from_filtering_yaml(self) -> float:
        """Read fx (c_matrix[0]) from filtering_params.yaml."""
        filtering_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'filtering_params.yaml'
        )
        with open(filtering_yaml, 'r') as f:
            cfg = yaml.safe_load(f)
        c_mat = cfg.get('c_matrix', [1.0] * 9)
        return float(c_mat[0])

    def _clear_raw_frames_dir(self):
        """Delete all files in raw_frames/ at first SCAN entry per mission."""
        raw_dir = "raw_frames"
        if not os.path.isdir(raw_dir):
            os.makedirs(raw_dir, exist_ok=True)
            return
        for name in os.listdir(raw_dir):
            path = os.path.join(raw_dir, name)
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except OSError as e:
                self.get_logger().warn(f"Could not remove {path}: {e}")
        self.get_logger().info("Cleared raw_frames/ for new mission")

    def _init_pipeline(self):
        """Initialize the camera pipeline based on configuration."""
        self.pipeline, self.pipeline_topic = create_pipeline(
            self.pipeline_type,
            gz_topic=self.gz_topic,
            ros_topic=self.ros_image_topic,
            node=self,
            gst_pipeline=self.gst_pipeline_str,
            record=self.record_video,
            fps=self.camera_fps,
            queue_size=1,
        )

        self.pipeline_running = False

        # Debug preview: constructed dormant. Nothing runs until the operator
        # toggles it on, and the hook costs one comparison per frame until then.
        #
        # RELIABLE, not BEST_EFFORT. This topic never crosses the radio: both
        # vision_node and approval_node run on the drone, and the constrained
        # hop is approval_node -> browser over TCP. So reliability is nearly
        # free here, and it is required rather than merely nice: fMP4 chunks are
        # not independently droppable the way video frames are, and a lost
        # moof/mdat can stall the browser's MSE SourceBuffer instead of just
        # skipping a frame. Depth 10 gives headroom for encoder burstiness.
        preview_qos = QoSProfile(depth=10)
        preview_qos.reliability = ReliabilityPolicy.RELIABLE
        preview_qos.history = HistoryPolicy.KEEP_LAST
        self.preview_pub = self.create_publisher(
            UInt8MultiArray, '/preview_stream', preview_qos)

        self.preview = PreviewStream(
            self.preview_cfg, self._publish_preview_chunk, self.get_logger())

        self._init_preview_state()

        self.create_subscription(
            Bool, '/preview_enabled', self._on_preview_toggle, 10)

    def _init_threading(self):
        """Initialize threading primitives."""
        self.scan_active = threading.Event()
        self.shutdown_flag = threading.Event()
        self.queue = queue.Queue(maxsize=1)

    def _init_subscribers(self):
        """Set up ROS2 subscriptions."""
        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT

        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self._on_waypoint_reached,
            qos_best_effort
        )

        self.mission_state_sub = self.create_subscription(
            String,
            '/mission_state',
            self._on_mission_state_changed,
            qos_best_effort
        )
        
        # GPS/pose subscriptions for localization service
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._on_gps,
            qos_best_effort
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._on_pose,
            qos_best_effort
        )
        
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self._on_rel_alt,
            qos_best_effort
        )

    def _init_publishers(self):
        """Set up ROS2 publishers and timers."""
        # Object detections (reliable for mission-critical data)
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE

        self.obj_dets_pub = self.create_publisher(
            ObjectDetections,
            '/obj_dets',
            qos_reliable
        )

        # Queue state (transient local so late subscribers get last value)
        qos_queue_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.queue_state_pub = self.create_publisher(
            Int8,
            '/queue_state',
            qos_queue_state
        )

        # Timer for queue state publishing
        self.timer = self.create_timer(0.1, self._on_timer)

    def _init_detector(self):
        """Initialize the object detector based on config."""
        self.detector = create_detector(
            detector_type=self.detector_type,
            ml_model_path=self.ml_model_path,
            gazebo_bbox_topic=self.gazebo_bbox_topic,
        )
        self.detector.start()

    def _init_localizer(self):
        """Initialize the localizer for GPS coordinate conversion."""
        # Load camera intrinsics from filtering_params.yaml
        filtering_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'filtering_params.yaml'
        )

        with open(filtering_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        c_mat_list = cfg.get('c_matrix', [1.0]*9)
        dist_coeff_list = cfg.get('dist_coefficients', [0.0]*5)

        camera_matrix = np.array(c_mat_list, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(dist_coeff_list, dtype=np.float64)

        self.localizer = Localizer(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs
        )

    def _init_services(self):
        """Initialize ROS2 services."""
        self.localize_srv = self.create_service(
            LocalizeObject,
            'localize_object',
            self._handle_localize_request
        )
        pass  # service ready

    def _start_worker_threads(self):
        """Start background worker threads."""
        self.frame_fetcher = threading.Thread(
            target=self._frame_fetch_loop,
            daemon=True
        )
        self.frame_fetcher.start()

        self.worker = threading.Thread(
            target=self._detection_worker_loop,
            daemon=True
        )
        self.worker.start()

    
    # ROS2 Callbacks
    

    def _on_mission_state_changed(self, msg: String):
        """
        Handle mission state transitions.
        
        Starts/stops the camera pipeline and frame capture based on
        whether we're in 'scan' or 'localize' state.
        """
        new_state = msg.data
        leaving_scan_mid_row = (
            self.prev_state == 'scan'
            and new_state != 'scan'
            and self.row_anchor_ll is not None
            and self.row_next_ll is not None
            and self.curr_wp % 2 == 0
        )

        if new_state == self.prev_state:
            return

        self.state = new_state
        if self.prev_state == 'scan' and new_state != 'scan':
            self.stitch_paused = leaving_scan_mid_row

        # Keep pipeline running in both scan and localize states
        if new_state in ('scan', 'localize'):
            if new_state == 'scan' and not self.raw_frames_cleared:
                self._clear_raw_frames_dir()
                self.raw_frames_cleared = True
            if (new_state == 'scan'
                    and self.row_anchor_ll is None
                    and self.curr_wp + 1 >= len(self.scan_points)):
                self.curr_wp = -1
                self.stitch_paused = False
            self._start_scanning()
        else:
            self._stop_scanning()

        self.prev_state = new_state

    def _on_waypoint_reached(self, msg: WaypointReached):
        """Handle waypoint reached notifications.

        Advances scan progress only when the next planned scan waypoint is reached,
        latches the long-side row anchor on even waypoints, and clears the anchor
        on odd waypoints (end of row).
        """
        self.frame_number = 1  # retained for any legacy consumers
        if self.last_wp is not None and msg.wp_seq != self.last_wp:
            self.latest_wp = msg.wp_seq
        self.last_wp = msg.wp_seq

        # Stitch row tracking only advances on planned scan waypoints.
        if self.state != 'scan' or len(self.gps_buffer) == 0:
            return

        next_scan_idx = self.curr_wp + 1
        if next_scan_idx >= len(self.scan_points):
            return

        g = self.gps_buffer[-1]
        current_ll = (g.latitude, g.longitude)
        expected_scan_ll = (
            float(self.scan_points[next_scan_idx][0]),
            float(self.scan_points[next_scan_idx][1]),
        )

        # Ignore non-scan waypoints, such as the prepended loiter return point on resume.
        if distance_m(current_ll, expected_scan_ll) > self.scan_tolerance:
            if self.stitch_paused:
                self.stitch_paused = False
                self.get_logger().info(
                    "Stitch capture resumed after return waypoint; continuing active row"
                )
            return

        self.curr_wp = next_scan_idx
        self.stitch_paused = False

        if self.curr_wp % 2 == 0 and self.curr_wp + 1 < len(self.scan_points):
            # Anchor at current GPS (drone is at scan_points[curr_wp]); next is scan_points[curr_wp+1]
            self.row_anchor_ll = current_ll
            sp_next = self.scan_points[self.curr_wp + 1]
            self.row_next_ll = (float(sp_next[0]), float(sp_next[1]))
            self.col_idx = 1
            self.next_capture_m = self.step_m
            self.get_logger().info(
                f"Stitch row entry: curr_wp={self.curr_wp}, "
                f"anchor={self.row_anchor_ll}, next={self.row_next_ll}"
            )
        else:
            # Odd curr_wp or past the scan plan: row complete.
            self.row_anchor_ll = None
            self.row_next_ll = None
            self.stitch_paused = False

    def _on_timer(self):
        """
        Publish queue state at 10Hz.
        
        Publishes 1 when queue is empty (all frames processed),
        0 when frames are still pending.
        """
        empty_flag = 1 if self.queue.unfinished_tasks == 0 else 0

        msg = Int8()
        msg.data = empty_flag
        self.queue_state_pub.publish(msg)

    def _on_gps(self, msg: NavSatFix):
        """Buffer GPS messages for localization."""
        self.gps_buffer.append(msg)

    def _on_pose(self, msg: PoseStamped):
        """Buffer pose messages for localization."""
        self.pose_buffer.append(msg)

    def _on_rel_alt(self, msg: Float64):
        """Store latest relative altitude for localization."""
        self.last_rel_alt = msg

    def _handle_localize_request(self, request, response):
        """
        Handle localize_object service request.
        
        Captures a fresh frame, runs detection, and localizes to GPS coordinates.
        """
        response.success = False
        response.latitude = 0.0
        response.longitude = 0.0
        response.altitude = 0.0
        response.class_id = -1
        target_name = (
            CLASS_NAMES[request.target_class_id]
            if 0 <= request.target_class_id < len(CLASS_NAMES)
            else 'any'
        )
        self.log.event('LOCALIZE_REQUEST', f"target_class={target_name}({request.target_class_id})")

        # Ensure pipeline is running
        if not self.pipeline_running:
            try:
                self.pipeline.start()
                self.pipeline_running = True
            except RuntimeError as exc:
                self.get_logger().error(f"Failed to start pipeline for localization: {exc}")
                self.log.event('LOCALIZE_RESULT', f"success=false, reason=pipeline_start_failed")
                return response
        
        # Capture frame
        try:
            frame = self.pipeline.get_frame(timeout=1.0)
        except RuntimeError:
            self.get_logger().error("Failed to capture frame for localization")
            self.log.event('LOCALIZE_RESULT', f"success=false, reason=no_frame")
            return response
        
        if frame is None:
            self.get_logger().error("No frame captured for localization")
            self.log.event('LOCALIZE_RESULT', f"success=false, reason=no_frame")
            return response
        
        # Convert grayscale to BGR if needed
        if frame.ndim == 2:
            frame = np.repeat(frame[:, :, None], 3, axis=2)
        
        # Run detection
        detections = self.detector.process_frame(
            frame=frame,
            threshold=self.det_thresh,
        )
        
        if len(detections) == 0:
            self.get_logger().warn("No detections found during localization")
            self.log.event('LOCALIZE_RESULT', f"success=false, reason=no_detections")
            return response
        
        # Get pixel centers: (center_x, center_y, class_id)
        pixel_centers = [
            ((x1 + x2) / 2, (y1 + y2) / 2, int(cls))
            for (x1, y1, x2, y2), cls in zip(detections.xyxy, detections.class_id)
        ]
        
        # Check for sensor data
        if len(self.gps_buffer) == 0 or len(self.pose_buffer) == 0 or self.last_rel_alt is None:
            self.get_logger().error("Missing sensor data for localization")
            self.log.event('LOCALIZE_RESULT', f"success=false, reason=missing_sensor_data")
            return response
        
        # Get latest drone pose
        best_gps = self.gps_buffer[-1]
        best_pose = self.pose_buffer[-1]
        
        drone_pose = (best_gps.latitude, best_gps.longitude, self.last_rel_alt.data)
        drone_orientation = (
            best_pose.pose.orientation.x,
            best_pose.pose.orientation.y,
            best_pose.pose.orientation.z,
            best_pose.pose.orientation.w
        )
        
        # Localize to global coordinates
        coords = self.localizer.get_lat_lon(pixel_centers, drone_pose, drone_orientation)
        
        if not coords:
            self.get_logger().warn("Localization conversion failed")
            self.log.event('LOCALIZE_RESULT',
                f"success=false, reason=conversion_failed | "
                f"drone_pos=({drone_pose[0]:.6f},{drone_pose[1]:.6f},{drone_pose[2]:.1f}), "
                f"orientation=({drone_orientation[0]:.3f},{drone_orientation[1]:.3f},{drone_orientation[2]:.3f},{drone_orientation[3]:.3f})")
            return response

        target_cls = request.target_class_id
        if target_cls >= 0:
            target_name = (
                CLASS_NAMES[target_cls]
                if target_cls < len(CLASS_NAMES)
                else "unknown"
            )
            self.get_logger().info(f"Localizing object: {target_name}")
        else:
            self.get_logger().info("Localizing: any detected object")

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
                available_class_names = [
                    CLASS_NAMES[int(c[2])] if 0 <= int(c[2]) < len(CLASS_NAMES) else f"class_{int(c[2])}"
                    for c in coords
                ]
                self.get_logger().warn(
                    f"Requested object '{target_name}' not found in frame; "
                    f"detected: {available_class_names}"
                )
                # Treat this as a localization failure so the mission
                # can retry or abandon the object instead of flying to
                # the wrong target.
                self.log.event('LOCALIZE_RESULT',
                    f"success=false, reason=class_not_in_frame, target={target_name}({request.target_class_id}), "
                    f"available={available_class_names} | "
                    f"drone_pos=({drone_pose[0]:.6f},{drone_pose[1]:.6f},{drone_pose[2]:.1f}), "
                    f"orientation=({drone_orientation[0]:.3f},{drone_orientation[1]:.3f},{drone_orientation[2]:.3f},{drone_orientation[3]:.3f})")
                return response

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

        # Annotated crop for the human-in-the-loop gate. Skipped unless the
        # caller asked for it: on the real camera this is a 4640x3480 crop plus
        # an INTER_AREA resize plus a JPEG encode, all on the FSM's critical
        # path, and an autonomous mission has no one to show it to.
        #
        # A failure here must never fail the localization: an encoding bug would
        # silently turn into the aircraft abandoning real targets. The ground
        # station renders an explicit "no image received" state instead.
        if request.want_crop:
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
            f"success=true, lat={response.latitude:.6f}, lon={response.longitude:.6f}, "
            f"alt={response.altitude:.1f}, class={CLASS_NAMES[response.class_id] if 0 <= response.class_id < len(CLASS_NAMES) else 'unknown'}({response.class_id}) | "
            f"drone_pos=({drone_pose[0]:.6f},{drone_pose[1]:.6f},{drone_pose[2]:.1f}), "
            f"orientation=({drone_orientation[0]:.3f},{drone_orientation[1]:.3f},{drone_orientation[2]:.3f},{drone_orientation[3]:.3f})")

        return response

    
    # Scanning Control
    

    def _start_scanning(self):
        """Start the camera pipeline and enable frame capture."""
        if not self.pipeline_running:
            try:
                self.pipeline.start()
                self.pipeline_running = True
                self.log.event('PIPELINE_START', f"type={self.pipeline_type}, trigger={self.state}")
            except RuntimeError as exc:
                self.get_logger().error(f"Failed to start vision pipeline: {exc}")
                self.state = self.prev_state
                return

        self.scan_active.set()

    def _stop_scanning(self):
        """Stop the camera pipeline and disable frame capture."""
        self.scan_active.clear()

        if self.pipeline_running:
            self.pipeline.stop()
            self.pipeline_running = False
            self.log.event('PIPELINE_STOP', f"trigger={self.state}")

        # Flush any blocking queue.get() calls
        self.queue.put(None)

    
    # Frame Fetcher Thread
    

    def _frame_fetch_loop(self):
        """
        Background thread that captures frames from the camera pipeline.
        
        Only queues frames when:
        - scan_active is set (we're in scan state)
        - A new waypoint was just reached (latest_wp is set)
        """
        while not self.shutdown_flag.is_set():
            # Wait for scan state to be active
            if not self.scan_active.wait(timeout=0.1):
                continue

            if self.shutdown_flag.is_set():
                break

            if not self.pipeline_running:
                time.sleep(0.05)
                continue

            # Try to get a frame from the camera
            frame = self._try_get_frame()
            if frame is None:
                continue

            # Only queue if we're in scan state (no waypoint gate)
            if self.state != 'scan':
                time.sleep(0.05)
                continue

            stamp = self.get_clock().now()
            # Discard stale frame if present ("latest wins")
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                pass
            self.queue.put_nowait((frame, stamp))

    def _try_get_frame(self):
        """Attempt to get a frame from the pipeline with error handling."""
        try:
            return self.pipeline.get_frame(timeout=0.1)
        except RuntimeError:
            time.sleep(0.05)
            return None

    
    # Detection Worker Thread
    

    def _detection_worker_loop(self):
        """
        Background thread that processes queued frames through object detection.
        
        For each frame:
        1. Runs detection model
        2. Annotates and saves the frame
        3. Publishes compressed image and detection results
        """
        while not self.shutdown_flag.is_set():
            item = self._get_queue_item()
            if item is None:
                continue

            frame, stamp = item

            try:
                self._process_frame(frame, stamp)
            except Exception as e:
                tb = traceback.format_exc()
                self.get_logger().error(f"Processing failed: {e}\n{tb}")
            finally:
                self.queue.task_done()

    def _get_queue_item(self):
        """
        Get next item from queue, handling empty queue and sentinel values.
        
        Returns None if item should be skipped.
        """
        try:
            item = self.queue.get(timeout=0.1)
        except queue.Empty:
            return None

        # Sentinel value for shutdown/state change
        if item is None:
            self.queue.task_done()
            return None

        # Skip if we're no longer in scan state
        if self.state != 'scan':
            self.queue.task_done()
            return None

        return item

    def _process_frame(self, frame, stamp):
        """Run detection on a frame and publish results."""

        # Convert grayscale to BGR if needed
        if frame.ndim == 2:
            frame = np.repeat(frame[:, :, None], 3, axis=2)

        # Run detection
        detections = self.detector.process_frame(
            frame=frame,
            threshold=self.det_thresh,
        )
        
        # Distance-based stitching capture (long-side rows only).
        if (self.state == 'scan'
                and self.row_anchor_ll is not None
                and self.row_next_ll is not None
                and self.curr_wp % 2 == 0
                and not self.stitch_paused
                and len(self.gps_buffer) > 0):
            g = self.gps_buffer[-1]
            d = along_track_m(
                (g.latitude, g.longitude),
                self.row_anchor_ll,
                self.row_next_ll,
            )
            if d >= self.next_capture_m:
                row_n = (self.curr_wp // 2) + 1
                path = f"raw_frames/row{row_n}_{self.col_idx}.jpg"
                cv2.imwrite(path, frame)
                self.get_logger().info(
                    f"Stitch capture: {path} (d={d:.2f}m, target={self.next_capture_m:.2f}m)"
                )
                self.col_idx += 1
                self.next_capture_m += self.step_m

        # Log to mission logger
        if len(detections) > 0:
            self._frame_counter += 1
            self._last_detection_time = time.monotonic()
            drone_pos = (0.0, 0.0)
            if len(self.gps_buffer) > 0:
                g = self.gps_buffer[-1]
                drone_pos = (g.latitude, g.longitude)
            det_list = []
            for (x1, y1, x2, y2), score, cls in zip(
                detections.xyxy, detections.confidence, detections.class_id
            ):
                det_list.append({
                    'class_name': CLASS_NAMES[int(cls)] if 0 <= int(cls) < len(CLASS_NAMES) else 'unknown',
                    'class_id': int(cls),
                    'confidence': float(score),
                    'px': ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                    'latlon': (0.0, 0.0),  # geolocation done by filtering_node
                })
            filtered = {}
            for i, det in enumerate(det_list):
                if det['confidence'] <= self.det_thresh:
                    filtered[i] = f"[BELOW_THRESH conf={det['confidence']:.2f} thresh={self.det_thresh:.2f}]"
            self.log.detection_frame(self._frame_counter, drone_pos, 0.0, det_list, filtered)
        else:
            # Heartbeat when no detections during scan
            if self.state == 'scan':
                alt = self.last_rel_alt.data if self.last_rel_alt is not None else 0.0
                drone_pos = (0.0, 0.0)
                if len(self.gps_buffer) > 0:
                    g = self.gps_buffer[-1]
                    drone_pos = (g.latitude, g.longitude)
                elapsed = time.monotonic() - self._last_detection_time
                self.log.scan_heartbeat(drone_pos, 0.0, alt, elapsed)

        # Annotate frame using supervision
        labels = [
            f"{CLASS_NAMES[int(class_id)] if 0 <= int(class_id) < len(CLASS_NAMES) else 'unknown'} "
            f"{confidence:.2f}"
            for class_id, confidence
            in zip(detections.class_id, detections.confidence)
        ]
        annotated_frame = frame.copy()
        annotated_frame = sv.BoxAnnotator().annotate(annotated_frame, detections)
        annotated_frame = sv.LabelAnnotator().annotate(annotated_frame, detections, labels)

        # Publish detection results
        self._publish_detections(detections, stamp)

    def _publish_detections(self, detections, stamp):
        """Publish object detections message."""
        detections_msg = ObjectDetections()
        detections_msg.dets = []
        detections_msg.header.stamp = stamp.to_msg()
        detections_msg.header.frame_id = self.pipeline_topic

        for (x1, y1, x2, y2), score, cls in zip(
            detections.xyxy,
            detections.confidence,
            detections.class_id,
        ):
            if score > self.det_thresh:
                vec = Vector3()
                vec.x = float(x1 + x2) / 2.0  # Center X
                vec.y = float(y1 + y2) / 2.0  # Center Y
                vec.z = float(cls)             # Class ID
                detections_msg.dets.append(vec)

        self.obj_dets_pub.publish(detections_msg)

    # Debug preview stream
    def _init_preview_state(self):
        """Toggle plumbing for the debug preview.

        Deliberately inert: no thread, no GStreamer, no pipeline. The worker is
        spawned lazily on the first toggle, so a run that never enables the
        preview never creates one.
        """
        self._preview_want = False        # latest operator request
        self._preview_engaged = False     # state the worker last applied
        self._preview_wake = threading.Event()
        self._preview_shutdown = threading.Event()
        self._preview_worker = None
        self._preview_worker_lock = threading.Lock()

    def _publish_preview_chunk(self, chunk):
        """Publish one fMP4 chunk. Called from the GStreamer streaming thread.

        Publish and return. This must never call self.preview.stop(): that would
        have the streaming thread block on the encoder's internal lock while the
        shutdown waits for this very thread to exit.
        """
        msg = UInt8MultiArray()
        msg.data = chunk
        self.preview_pub.publish(msg)

    def _on_preview_toggle(self, msg):
        """Operator turned the debug preview on or off. MUST return immediately.

        This runs on the single-threaded executor that also serves
        localize_object on the mission's critical path, and PreviewStream's
        start()/stop() can each take 12-18 s building or tearing down the
        GStreamer tier ladder. Blocking here would stall localize long enough
        for mission_node to exhaust its retries and abandon a real target, so a
        debug toggle would cost a delivery. Record the wish, wake the worker,
        return; the worker does the slow part and owns the logging.
        """
        self._preview_want = bool(msg.data)
        self._ensure_preview_worker()
        self._preview_wake.set()

    def _ensure_preview_worker(self):
        """Start the toggle worker on first use. Never runs if never toggled."""
        with self._preview_worker_lock:
            if self._preview_worker is None or not self._preview_worker.is_alive():
                self._preview_worker = threading.Thread(
                    target=self._preview_worker_loop,
                    name='preview_toggle',
                    daemon=True,
                )
                self._preview_worker.start()

    def _preview_worker_loop(self):
        """Converge the preview on the latest requested state, one call at a time.

        A single thread, so start() and stop() can never run concurrently against
        the same PreviewStream. It reads the desired state fresh on every pass
        rather than draining a queue of toggles, so a rapid on/off/on settles on
        "on" with at most one lifecycle call instead of three.
        """
        while not self._preview_shutdown.is_set():
            self._preview_wake.wait()
            self._preview_wake.clear()
            if self._preview_shutdown.is_set():
                break
            try:
                self._apply_preview_state(self._preview_want)
            except Exception as exc:        # noqa: BLE001
                # A debug feature must never take the node down with it.
                self.get_logger().error(f'debug preview toggle failed: {exc}')

    def _apply_preview_state(self, want):
        """Bring the preview to `want`. Slow; only ever called from the worker.

        `_preview_engaged` is the state we last applied, tracked separately from
        PreviewStream.is_running() because the stream self-disables on a bus
        error. Reading liveness as intent would make an OFF click after such a
        fault early-return: the pipeline would stay attached to a dead stream,
        nothing would log, and the operator's click would appear to do nothing.
        A debug tool that silently ignores its own toggle is worse than useless.
        """
        if not want:
            if not self._preview_engaged:
                return
            self._preview_engaged = False
            # Detach first so no frame is offered to a stream being torn down.
            self.pipeline.set_preview(None)
            self.preview.stop()
            self.get_logger().info('debug preview OFF')
            return

        if self._preview_engaged and self.preview.is_running():
            return

        if self._preview_engaged:
            # Engaged but dead: the stream faulted off the bus. Clear it down
            # before rebuilding, so ON acts as the retry the operator meant.
            self._preview_engaged = False
            self.pipeline.set_preview(None)
            self.preview.stop()

        if self.preview.start():
            self._preview_engaged = True
            self.pipeline.set_preview(self.preview)
            self.get_logger().info('debug preview ON')
        else:
            self.get_logger().warn('debug preview requested but unavailable')

    def _join_preview_worker(self):
        """Wait for the toggle worker. Returns True if it outlasted the join."""
        worker = self._preview_worker
        if worker is None or not worker.is_alive():
            return False
        worker.join(timeout=PREVIEW_WORKER_JOIN_SEC)
        return worker.is_alive()

    # Lifecycle
    def destroy_node(self):
        """Clean up resources before shutdown."""
        self.shutdown_flag.set()
        self.scan_active.set()  # Unblock any waiting threads

        # Retire the preview before the pipeline goes away. Blocking is fine
        # here: we are already shutting down, nothing is left to starve.
        self._preview_shutdown.set()
        self._preview_want = False
        self._preview_wake.set()
        outlasted = self._join_preview_worker()
        self.pipeline.set_preview(None)
        # Unconditional, NOT `if self.preview.is_running()`. If shutdown lands
        # while the worker is inside start(), is_running() is False right now
        # but an encoder is about to exist; skipping stop() would leave it
        # running past the node's life. stop() is idempotent and clears the
        # running flag, so a build that lands late tears itself down.
        self.preview.stop()
        if outlasted:
            # The worker outlasted the first join, so it may have finished a
            # start() after that stop(). Wait it out and kill what it built.
            self._join_preview_worker()
            self.preview.stop()

        if self.pipeline_running:
            self.pipeline.stop()
            self.pipeline_running = False

        self.detector.stop()
        self.queue.put(None)  # Unblock worker thread

        self.log.close()
        return super().destroy_node()


# Entry Point

def main(args=None):
    """
    Main entry point for the vision node.
    
    Note: Before running, ensure PX4's yaw mode is set:
        ros2 param set /px4 MPC_YAW_MODE 0
    """
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
