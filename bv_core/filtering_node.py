#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from .vision import Localizer
import queue
import threading
from rclpy.executors import MultiThreadedExecutor
from bv_msgs.msg import ObjectDetections
from geometry_msgs.msg import Vector3
import numpy as np
import traceback
from rfdetr.util.coco_classes import COCO_CLASSES

class FilteringNode(Node):
    def __init__(self):
        super().__init__('filtering_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.object_det_sub = self.create_subscription(
            msg_type=ObjectDetections,
            topic='/obj_dets',
            callback=self.handle_detections,
            qos_profile=qos,
        )

        self.mission_state_sub = self.create_subscription(
            msg_type=String,
            topic='/mission_state',
            callback=self.mission_state_callback,
            qos_profile=qos,
        )

        # Declare a c_matrix, dist_coefficients parameters in the config file

        # Define a list to keep all the global_dets list of tuples lat, lon, class
        # Define a list to keep all object locations (obj_locs) of tuples lat, lon, class

        # Set up a ros service to get the obj_locs

        # Pass in the c_matrix and dist_coefficients from the config file into the Localizer constructor
        self.localizer = Localizer()
        
    def handle_detections(self, msg):
        # Take in the detections (msg.dets) which is a list of Vectors (x, y, class) from the current processed frame
        # Use localization's get_lat_lon to convert this to lat lon list and store that into the detections_lobal list

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            self.get_logger().info(f"Vision node acknowledging state change: {self.state}")

            if self.prev_state == 'scan':
                obj_locs = self.localizer.estimate_locations(self.global_dets)
                
        self.prev_state = msg.data

def main(args=None):
    # Before running, ensure PX4’s yaw mode is set:
    # ros2 param set /px4 MPC_YAW_MODE 0
    rclpy.init(args=args)
    node = FilteringNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
