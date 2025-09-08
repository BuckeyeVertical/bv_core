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
from collections import deque
from rclpy.time import Time

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

        self.object_det_sub = self.create_subscription(
            ObjectDetections,
            '/obj_dets',
            self.handle_detections,
            qos
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.handle_gps,
            qos
        )
        
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.handle_rel_alt,
            qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.handle_local_pose,
            qos
        )

        self.mission_state_sub = self.create_subscription(
            String,
            '/mission_state',
            self.mission_state_callback,
            qos
        )

        self.gps_buffer  = deque(maxlen=200)
        self.pose_buffer = deque(maxlen=200)

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

        self.last_gps = None
        self.last_rel_alt = None

    def handle_gps(self, msg: NavSatFix):
        self.gps_buffer.append(msg)

    def handle_rel_alt(self, msg: Float64):
        self.last_rel_alt = msg

    def handle_local_pose(self, msg: PoseStamped):
        self.pose_buffer.append(msg)

    def handle_detections(self, msg: ObjectDetections):
        if len(self.pose_buffer) == 0 or len(self.gps_buffer) == 0 or self.last_rel_alt == None:
            self.get_logger().info("Waiting for sensor data")
            return
        
        # convert detection stamp into a float or ns
        t_det = Time.from_msg(msg.header.stamp).nanoseconds

        # find the GPS msg whose stamp is closest
        best_gps = min(
            self.gps_buffer,
            key=lambda m: abs(Time.from_msg(m.header.stamp).nanoseconds - t_det),
            default=None
        )
        # same for pose
        best_pose = min(
            self.pose_buffer,
            key=lambda m: abs(Time.from_msg(m.header.stamp).nanoseconds - t_det),
            default=None
        )

        if not best_gps or not best_pose:
            self.get_logger().warn("No matching GPS or Pose yet.")
            return
        
        drone_pose = (
            best_gps.latitude,
            best_gps.longitude,
            self.last_rel_alt.data
        )

        drone_orientation = (
            best_pose.pose.orientation.x,
            best_pose.pose.orientation.y,
            best_pose.pose.orientation.z,
            best_pose.pose.orientation.w
        )
        
        # msg.dets is a sequence of Vectors: x, y, class_id
        pixel_centers = [
            (det.x, det.y, int(det.z))
            for det in msg.dets
        ]

        # convert to global lat/lon
        detections_global = self.localizer.get_lat_lon(
            pixel_centers,
            drone_global_pos=drone_pose,
            drone_orientation=drone_orientation
            # drone_orientation=(1.0, 0.0, 0.0, 0.0)
        )

        # store for later clustering once mission state changes
        self.global_dets.extend(detections_global)

    def mission_state_callback(self, msg: String):
        if msg.data != self.prev_state:
            self.state = msg.data
            self.get_logger().info(f"Filtering node acknowledging state change: {self.state}")

            # once we leave the 'scan' state, cluster what we've seen
            if self.prev_state == 'scan':
                self.obj_locs = self.localizer.estimate_locations_v2(self.global_dets)

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