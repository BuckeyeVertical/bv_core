#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from .vision import Localizer
from rclpy.executors import MultiThreadedExecutor
from bv_msgs.msg import ObjectDetections
from geometry_msgs.msg import PoseStamped
import numpy as np
from rfdetr.util.coco_classes import COCO_CLASSES
from sensor_msgs.msg import NavSatFix

from bv_msgs.srv import GetObjectLocations     # your custom srv
from bv_msgs.msg import ObjectLocations         # your custom msg
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from message_filters import Subscriber, ApproximateTimeSynchronizer

class FilteringNode(Node):
    def __init__(self):
        super().__init__('filtering_node')

        # === subscriptions ===
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.det_sub  = Subscriber(
            self, 
            ObjectDetections,    
            '/obj_dets',                   
            qos_profile=qos
        )

        # Since rel_alt is headerless it is going to be ahead of the rest
        # We are assuming that drone doesn't change altitude much in 0.5 seconds
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.handle_rel_alt,
            qos
        )

        self.gps_sub  = Subscriber(
            self, 
            NavSatFix,           
            '/mavros/global_position/global', 
            qos_profile=qos
        )

        self.pose_sub = Subscriber(
            self, 
            PoseStamped,         
            '/mavros/local_position/pose',    
            qos_profile=qos
        )

        self.object_det_sub = Subscriber(
            ObjectDetections,
            '/obj_dets',
            self.handle_detections,
            qos
        )

        self.mission_state_sub = self.create_subscription(
            String,
            '/mission_state',
            self.mission_state_callback,
            qos
        )

        self.time_sync = ApproximateTimeSynchronizer(
            [self.gps_sub, self.pose_sub, self.object_det_sub],
            queue_size=10,
            slop=0.05
        )

        self.time_sync.registerCallbacks(self.handle_detections)

        # === internal state ===
        self.global_dets = []   # list of (lat, lon, class_id) for all detections so far
        self.obj_locs = []      # list of clustered (lat, lon, class_id)
        self.prev_state = None
        self.state = None

        # === service to expose the object locations ===
        self.get_obj_locs_srv = self.create_service(
            GetObjectLocations,
            'get_object_locations',
            self.handle_get_object_locations
        )

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

        self.get_logger().info(f"Camera Matrix: {camera_matrix}\nDist Coefficients: {dist_coeffs}")

        self.localizer = Localizer(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs
        )

        self.global_pos = None
        self.last_gps = None
        self.last_rel_alt = None
        self.drone_orientation = None

    def handle_gps(self, msg: NavSatFix):
        self.last_gps = msg
        self.maybe_global_pos()

    def handle_rel_alt(self, msg: Float64):
        self.last_rel_alt = msg

    def handle_detections(self, obj_det_msg: ObjectDetections, gps_msg: NavSatFix, pos_msg: PoseStamped):
        if self.last_rel_alt == None:
            self.get_logger().info("Relative Altitude is None")
            return
        
        self.global_pos = (
                gps_msg.latitude, 
                gps_msg.longitude, 
                self.last_rel_alt.data
                )
        
        self.drone_orientation = (pos_msg.pose.orientation.x, 
                            pos_msg.pose.orientation.y,
                            pos_msg.pose.orientation.z,
                            pos_msg.pose.orientation.w)

        # msg.dets is a sequence of Vectors: x, y, class_id
        pixel_centers = [
            (det.x, det.y, int(det.z))
            for det in obj_det_msg.dets
        ]

        # convert to global lat/lon
        detections_global = self.localizer.get_lat_lon(
            pixel_centers,
            drone_global_pos=self.global_pos,
            drone_orientation=self.drone_orientation
            # drone_orientation=(1.0, 0.0, 0.0, 0.0)
        )

        # store for later clustering once mission state changes
        self.global_dets = detections_global

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            self.get_logger().info(f"Filtering node acknowledging state change: {self.state}")

            # once we leave the 'scan' state, cluster what we've seen
            if self.prev_state == 'scan':
                self.obj_locs = self.localizer.estimate_locations(self.global_dets)

                with open('finalized_object_locations.txt', 'w') as f:
                        print("Writing lat lon")
                        for lat, lon, cls in self.obj_locs:
                            f.write(f"{lat:.6f},{lon:.6f},{COCO_CLASSES[cls]}\n")

        self.prev_state = msg.data

    def handle_get_object_locations(self, request, response):
        # populate service response with our clustered locations
        for lat, lon, cls_id in self.obj_locs:
            loc = ObjectLocations()
            loc.latitude = float(lat)
            loc.longitude = float(lon)
            loc.class_id = int(cls_id)
            response.locations.append(loc)
        return response

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
