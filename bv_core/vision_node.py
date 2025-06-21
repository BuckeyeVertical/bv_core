#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from .vision import Detector
import queue
import threading
from rclpy.executors import MultiThreadedExecutor
from bv_msgs.msg import ObjectDetections
from geometry_msgs.msg import Vector3
import numpy as np
import traceback
from rfdetr.util.coco_classes import COCO_CLASSES

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.image_sub = self.create_subscription(
            msg_type=Image, 
            topic='/image_raw',
            callback=self.camera_callback,
            qos_profile=qos,
        )

        self.mission_state_sub = self.create_subscription(
            msg_type=String,
            topic='/mission_state',
            callback=self.mission_state_callback,
            qos_profile=qos,
        )

        objs_pub_qos = QoSProfile(depth=10)
        objs_pub_qos.reliability = ReliabilityPolicy.RELIABLE

        # TODO: fix the message
        self.obj_dets_pub = self.create_publisher(
            msg_type=ObjectDetections,
            topic='/obj_dets',
            qos_profile=objs_pub_qos
        )

        self.declare_parameter("batch_size", 4)
        self.batch_size = self.get_parameter("batch_size").value

        self.declare_parameter("detection_threshold", 0.5)
        self.det_thresh = self.get_parameter("detection_threshold").value

        self.obj_dets = []

        self.detector = Detector(batch_size=self.batch_size)

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

    def camera_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_enqueue).nanoseconds < 0.5e9:
            return
        self.queue.put(msg)
        self.last_enqueue = now

    def worker_loop(self):
        while rclpy.ok() and self.state == 'scan':
            msg = self.queue.get()
            try:
                flat = np.frombuffer(msg.data, dtype=np.uint8)
                frame = flat.reshape((msg.height, msg.width, 3))
                detections = self.detector.process_frame(frame)

                labels = [
                    f"{COCO_CLASSES[class_id]} {confidence:.2f}"
                    for class_id, confidence
                    in zip(detections.class_id, detections.confidence)
                ]

                annotated_frame = self.detector.annotate_frame(frame, detections, labels)
                self.detector.save_frame(annotated_frame, "annotated_frames")

                detections_msg = ObjectDetections()
                detections_msg.dets = []

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
    rclpy.init(args=args)
    node = VisionNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
