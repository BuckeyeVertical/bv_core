#!/usr/bin/env python3
"""
Vision Node for SUAS Drone System

Handles camera frame capture and object detection during scan missions.
Uses a producer-consumer pattern with separate threads for frame fetching
and detection processing.
"""

# Imports

import os
import queue
import threading
import time
import traceback
import numpy as np
import yaml

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
from std_msgs.msg import String, Int8, Float64
from sensor_msgs.msg import CompressedImage, NavSatFix
from bv_msgs.msg import ObjectDetections
from bv_msgs.srv import LocalizeObject
from geometry_msgs.msg import Vector3, PoseStamped
from mavros_msgs.msg import WaypointReached
from collections import deque

# Local - using factory functions for lazy imports
from .detectors import create_detector
from .pipelines import create_pipeline
from .localizer import Localizer

# ROS2 utilities
from ament_index_python.packages import get_package_share_directory

# Detection utilities
from rfdetr.util.coco_classes import COCO_CLASSES
import supervision as sv

# Create 0-indexed list from COCO_CLASSES (detector outputs 0-79, not original COCO IDs)
COCO_CLASS_NAMES = [COCO_CLASSES[k] for k in sorted(COCO_CLASSES.keys())]


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
        self._init_pipeline()
        self._init_threading()
        self._init_subscribers()
        self._init_publishers()
        self._init_detector()
        self._init_localizer()
        self._init_services()
        self._start_worker_threads()

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
        self.batch_size = cfg.get('batch_size', 4)
        self.det_thresh = cfg.get('detection_threshold', 0.5)
        self.resolution = cfg.get('resolution', 560)
        self.num_scan_wp = cfg.get('num_scan_wp', 3)
        self.overlap = cfg.get('overlap', 100)
        self.capture_interval = float(cfg.get('capture_interval', 1.5e9))

        # Detector configuration
        self.detector_type = cfg.get('detector_type', 'ml')
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

        self.get_logger().info(f"Initialized {self.pipeline_type} pipeline on topic: {self.pipeline_topic}")
        self.pipeline_running = False

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
            batch_size=self.batch_size,
            resolution=self.resolution,
            gazebo_bbox_topic=self.gazebo_bbox_topic,
        )
        self.detector.start()
        self.get_logger().info(f"Initialized detector: {self.detector_type}")

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
        self.get_logger().info("Initialized localizer for GPS conversion")

    def _init_services(self):
        """Initialize ROS2 services."""
        self.localize_srv = self.create_service(
            LocalizeObject,
            'localize_object',
            self._handle_localize_request
        )
        self.get_logger().info("Localize object service ready")

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

        if new_state == self.prev_state:
            return

        self.state = new_state
        self.get_logger().info(f"Vision node acknowledging state change: {self.state}")

        # Keep pipeline running in both scan and localize states
        if new_state in ('scan', 'localize'):
            self._start_scanning()
        else:
            self._stop_scanning()

        self.prev_state = new_state

    def _on_waypoint_reached(self, msg: WaypointReached):
        """
        Handle waypoint reached notifications.
        
        Sets latest_wp to trigger frame capture in the fetch loop.
        """
        if self.last_wp is not None and msg.wp_seq != self.last_wp:
            self.get_logger().info("Made it to new waypoint")
            self.latest_wp = msg.wp_seq

        self.last_wp = msg.wp_seq

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
        
        self.get_logger().info("Localize request received - processing...")
        
        # Ensure pipeline is running
        if not self.pipeline_running:
            try:
                self.pipeline.start()
                self.pipeline_running = True
            except RuntimeError as exc:
                self.get_logger().error(f"Failed to start pipeline for localization: {exc}")
                return response
        
        # Capture frame
        try:
            frame = self.pipeline.get_frame(timeout=1.0)
        except RuntimeError:
            self.get_logger().error("Failed to capture frame for localization")
            return response
        
        if frame is None:
            self.get_logger().error("No frame captured for localization")
            return response
        
        # Convert grayscale to BGR if needed
        if frame.ndim == 2:
            frame = np.repeat(frame[:, :, None], 3, axis=2)
        
        # Run detection
        detections = self.detector.process_frame(
            frame=frame,
            threshold=self.det_thresh,
            overlap=self.overlap
        )
        
        if len(detections) == 0:
            self.get_logger().warn("No detections found during localization")
            return response
        
        self.get_logger().info(f"Localization detected {len(detections)} objects")
        
        # Get pixel centers: (center_x, center_y, class_id)
        pixel_centers = [
            ((x1 + x2) / 2, (y1 + y2) / 2, int(cls))
            for (x1, y1, x2, y2), cls in zip(detections.xyxy, detections.class_id)
        ]
        
        # Check for sensor data
        if len(self.gps_buffer) == 0 or len(self.pose_buffer) == 0 or self.last_rel_alt is None:
            self.get_logger().error("Missing sensor data for localization")
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
        
        # Localize
        coords = self.localizer.get_lat_lon(pixel_centers, drone_pose, drone_orientation)
        
        if coords:
            response.success = True
            response.latitude = coords[0][0]
            response.longitude = coords[0][1]
            response.altitude = drone_pose[2]  # Use current altitude for delivery
            response.class_id = int(coords[0][2])
            self.get_logger().info(
                f"Localization successful: lat={response.latitude:.6f}, "
                f"lon={response.longitude:.6f}, class={COCO_CLASS_NAMES[response.class_id]}"
            )
        else:
            self.get_logger().warn("Localization conversion failed")
        
        return response

    
    # Scanning Control
    

    def _start_scanning(self):
        """Start the camera pipeline and enable frame capture."""
        if not self.pipeline_running:
            try:
                self.pipeline.start()
                self.pipeline_running = True
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
            self.get_logger().info("Adding to queue")
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
        self.get_logger().info("Processing frame from queue")

        # Convert grayscale to BGR if needed
        if frame.ndim == 2:
            frame = np.repeat(frame[:, :, None], 3, axis=2)

        # Run detection
        detections = self.detector.process_frame(
            frame=frame,
            threshold=self.det_thresh,
            overlap=self.overlap
        )

        # Annotate frame using supervision
        labels = [
            f"{COCO_CLASS_NAMES[int(class_id)]} {confidence:.2f}"
            for class_id, confidence
            in zip(detections.class_id, detections.confidence)
        ]
        annotated_frame = frame.copy()
        annotated_frame = sv.BoxAnnotator().annotate(annotated_frame, detections)
        annotated_frame = sv.LabelAnnotator().annotate(annotated_frame, detections, labels)
        self.get_logger().info(f"Detected {len(detections)} objects")

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

    
    # Lifecycle
    def destroy_node(self):
        """Clean up resources before shutdown."""
        self.shutdown_flag.set()
        self.scan_active.set()  # Unblock any waiting threads

        if self.pipeline_running:
            self.pipeline.stop()
            self.pipeline_running = False

        self.detector.stop()
        self.queue.put(None)  # Unblock worker thread

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