#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
from .vision import Detector
import queue
import threading
from rclpy.executors import MultiThreadedExecutor
from bv_msgs.msg import ObjectDetections
from mavros_msgs.msg import WaypointReached
from geometry_msgs.msg import Vector3
import numpy as np
import traceback
from rfdetr.util.coco_classes import COCO_CLASSES
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from message_filters import Subscriber, ApproximateTimeSynchronizer

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.image_sub = Subscriber(
            self,
            Image, 
            '/image_raw',
            qos_profile=qos
        )

        self.reached_sub = Subscriber(
            self,
            WaypointReached,
            '/mavros/mission/reached',
            qos_profile=qos
        )

        self.time_sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.reached_sub],
            2,
            0.5
        )

        self.time_sync.registerCallback(self.camera_callback)

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

        self.prev_state = ""
        self.state = ""
        self.last_enqueue = self.get_clock().now()

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            self.get_logger().info(f"Vision node acknowledging state change: {self.state}")

            if self.state == 'scan':
                self.worker.start()

            if self.prev_state == 'scan':
                self.worker.join()
                
        self.prev_state = msg.data

    def camera_callback(self, msg_image, msg_reached):
        now = self.get_clock().now()
        self.get_logger().info("in camera_callback")
        idx = msg_reached.wp_seq
        if self.state != 'scan' or idx == self.num_scan_wp-1:
            return
        self.get_logger().info(f"Adding to que {(now - self.last_enqueue).nanoseconds}")
        self.queue.put(msg_image)
        self.last_enqueue = now

    def timer_cb(self):
        # queue.unfinished_tasks is incremented on put(), 
        # and decremented in task_done() (in your worker’s finally block).
        # When it hits zero, you know there are no pending or in‐flight tasks.
        empty_flag = 1 if self.queue.unfinished_tasks == 0 else 0

        msg = Int8()
        msg.data = empty_flag
        self.queue_state_pub.publish(msg)

    def worker_loop(self):
        while rclpy.ok() and self.state == 'scan':
            msg = self.queue.get()
            self.get_logger().info(f"Processsing frame from queue")
            try:
                flat = np.frombuffer(msg.data, dtype=np.uint8)
                frame = flat.reshape((msg.height, msg.width, 3))
                detections = self.detector.process_frame(frame=frame, threshold=self.det_thresh, overlap=self.overlap)

                labels = [
                    f"{COCO_CLASSES[class_id]} {confidence:.2f}"
                    for class_id, confidence
                    in zip(detections.class_id, detections.confidence)
                ]

                annotated_frame = self.detector.annotate_frame(frame, detections, labels)
                self.detector.save_frame(annotated_frame, "annotated_frames")

                self.get_logger().info(f"Saved Frame")

                detections_msg = ObjectDetections()
                detections_msg.dets = []
                detections_msg.header = msg.header

                for (x1, y1, x2, y2), score, cls in zip(
                                                            detections.xyxy,
                                                            detections.confidence,
                                                            detections.class_id
                                                        ):
                    if score > self.det_thresh:
                        vec = Vector3()
                        vec.x = float(x1 + x2)/2.0
                        vec.y = float(y1 + y2)/2.0
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

def main(args=None):
    # Before running, ensure PX4’s yaw mode is set:
    # ros2 param set /px4 MPC_YAW_MODE 0
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
