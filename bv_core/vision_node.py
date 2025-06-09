#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from bv_core.bv_core.vision import Detector, Localizer
import queue
import threading
from rclpy.executors import MultiThreadedExecutor

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.image_sub = self.create_subscription(
            type=Image, 
            topic='/camera/image_raw',
            callback=self.camera_callback,
            qos_profile=qos,
        )

        self.mission_state_sub = self.create_subscription(
            type=String,
            topic='/mission_state',
            callback=self.mission_state_callback,
            qos_profile=qos,
        )

        # TODO: fix the message
        self.objs_pub = self.create_publisher(
            type=TBD,
            topic='/obj_locs',
        )

        self.declare_parameter("batch_size", 8)
        self.batch_size = self.get_parameter("batch_size").value

        # TODO: add a parameter for the k matrix and make a parser to parse the matrix
        k = None

        self.detector = Detector(batch_size=self.batch_size)

        self.localizer = Localizer(k)

        self.image_buffer = queue.Queue()

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.prev_state = ""
        self.state = ""

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            self.get_logger().log(f"Vision node acknowledging state change: {self.state}")

            if self.prev_state == 'scan':
                # TODO: Add the parameters for this function (object locations at the current time)
                # Make sure to add the atomic lock on the list of bounding boxes
                obj_locs = self.localizer.estimate_locations()

                # TODO: fix this. you can't just publish the array. Once you create a message use it to fix this.
                self.objs_pub.publish(obj_locs)

        self.prev_state = msg.data

    def camera_callback(self, msg):
        now = self.get_clock().now
        if (now - self.last_enqueue).to_sec() < 0.5:
            return
        self.queue.put(msg)
        self.last_enqueue = now

    def worker_loop(self):
        while rclpy.ok():
            msg = self.queue.get()
            try:
                self.detector.process_frame(msg.data)
            except Exception as e:
                self.get_logger().error(f"Processing failed: {e}")
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
