#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


from std_msgs.msg import String, Int8
from .vision import Detector
import queue
import threading
from bv_msgs.msg import ObjectDetections
from mavros_msgs.msg import WaypointReached
from geometry_msgs.msg import Vector3
import numpy as np
import traceback
from rfdetr.util.coco_classes import COCO_CLASSES
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import time

from .pipelines.gz_transport_pipeline import GzTransportPipeline

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pipeline_topic = '/camera/image'
        self.pipeline = GzTransportPipeline(topic=self.pipeline_topic, queue_size=5)
        self.pipeline_running = False
        self.scan_active = threading.Event()
        self.shutdown_flag = threading.Event()

        self.reached_sub = self.create_subscription(
            WaypointReached,
            topic='/mavros/mission/reached',
            callback=self.handle_reached,
            qos_profile=qos
        )

        self.mission_state_sub = self.create_subscription(
            msg_type=String,
            topic='/mission_state',
            callback=self.mission_state_callback,
            qos_profile=qos,
        )

        objs_pub_qos = QoSProfile(depth=10)
        objs_pub_qos.reliability = ReliabilityPolicy.RELIABLE

        self.obj_dets_pub = self.create_publisher(
            msg_type=ObjectDetections,
            topic='/obj_dets',
            qos_profile=objs_pub_qos
        )

        qos_queue = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.queue_state_pub = self.create_publisher(
            msg_type=Int8,
            topic="/queue_state",
            qos_profile=qos_queue,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_cb
        )

        vision_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'vision_params.yaml'
        )

        with open(vision_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.batch_size = cfg.get('batch_size', 4)
        self.det_thresh = cfg.get('detection_threshold', 0.5)
        self.resolution = cfg.get('resolution', 560)
        self.num_scan_wp = cfg.get('num_scan_wp', 3)
        self.overlap = cfg.get('overlap', 100)
        self.capture_interval = float(cfg.get('capture_interval', 1.5e9))

        self.obj_dets = []

        self.detector = Detector(batch_size=self.batch_size, resolution=self.resolution)

        self.queue = queue.Queue()

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.frame_fetcher = threading.Thread(target=self.frame_fetch_loop, daemon=True)
        self.frame_fetcher.start()

        self.prev_state = ""
        self.state = ""

        self.latest_wp = None
        self.last_wp = None

    def mission_state_callback(self, msg: String):
        if msg.data == self.prev_state:
            return

        new_state = msg.data
        self.state = new_state
        self.get_logger().info(f"Vision node acknowledging state change: {self.state}")

        if new_state == 'scan':
            if not self.pipeline_running:
                try:
                    self.pipeline.start()
                    self.pipeline_running = True
                except RuntimeError as exc:
                    self.get_logger().error(f"Failed to start Gazebo pipeline: {exc}")
                    self.state = self.prev_state
                    return
            self.scan_active.set()
        else:
            self.scan_active.clear()
            if self.pipeline_running:
                self.pipeline.stop()
                self.pipeline_running = False
            self.queue.put(None)

        self.prev_state = new_state

    def handle_reached(self, msg):
        if self.last_wp != None and msg.wp_seq != self.last_wp:
            self.get_logger().info("Made it to new waypoint")
            self.latest_wp = msg.wp_seq
        
        self.last_wp = msg.wp_seq
#
    def frame_fetch_loop(self):
        while not self.shutdown_flag.is_set():
            if not self.scan_active.wait(timeout=0.1):
                continue

            if self.shutdown_flag.is_set():
                break

            if not self.pipeline_running:
                time.sleep(0.05)
                continue

            try:
                frame = self.pipeline.get_frame(timeout=0.1)
            except RuntimeError:
                time.sleep(0.05)
                continue

            if frame is None:
                continue

            if self.state != 'scan' or self.latest_wp is None:
                time.sleep(0.05)
                continue

            stamp = self.get_clock().now()
            self.get_logger().info("Adding to queue")
            self.queue.put((frame, stamp))
            self.latest_wp = None

    def timer_cb(self):
        # queue.unfinished_tasks is incremented on put(), 
        # and decremented in task_done() (in your worker’s finally block).
        # When it hits zero, you know there are no pending or in‐flight tasks.
        empty_flag = 1 if self.queue.unfinished_tasks == 0 else 0

        msg = Int8()
        msg.data = empty_flag
        self.queue_state_pub.publish(msg)

    def worker_loop(self):
        while not self.shutdown_flag.is_set():
            try:
                item = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if item is None:
                self.queue.task_done()
                continue

            if self.state != 'scan':
                self.queue.task_done()
                continue

            frame, stamp = item
            self.get_logger().info("Processsing frame from queue")
            try:
                if frame.ndim == 2:
                    frame = np.repeat(frame[:, :, None], 3, axis=2)

                detections = self.detector.process_frame(frame=frame, threshold=self.det_thresh, overlap=self.overlap)

                labels = [
                    f"{COCO_CLASSES[class_id]} {confidence:.2f}"
                    for class_id, confidence
                    in zip(detections.class_id, detections.confidence)
                ]

                annotated_frame = self.detector.annotate_frame(frame, detections, labels)
                self.detector.save_frame(annotated_frame, "annotated_frames")

                self.get_logger().info("Saved Frame")

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
                        vec.x = float(x1 + x2) / 2.0
                        vec.y = float(y1 + y2) / 2.0
                        vec.z = float(cls)
                        detections_msg.dets.append(vec)

                self.obj_dets_pub.publish(detections_msg)

            except Exception as e:
                tb = traceback.format_exc()
                self.get_logger().error(
                    f"Processing failed: {e}\n{tb}"
                )
            finally:
                self.queue.task_done()

    def destroy_node(self):
        self.shutdown_flag.set()
        self.scan_active.set()
        if self.pipeline_running:
            self.pipeline.stop()
            self.pipeline_running = False
        self.queue.put(None)
        return super().destroy_node()

def main(args=None):
    # Before running, ensure PX4’s yaw mode is set:
    # ros2 param set /px4 MPC_YAW_MODE 0
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

# def main(args=None):
#     # Before running, ensure PX4’s yaw mode is set:
#     # ros2 param set /px4 MPC_YAW_MODE 0
#     rclpy.init(args=args)
#     node = VisionNode()

#     executor = MultiThreadedExecutor(num_threads=4)
#     executor.add_node(node)
#     executor.spin()
    
#     rclpy.shutdown()


if __name__ == '__main__':
    main()
