#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, List, Tuple
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import Waypoint, WaypointList
from sensor_msgs.msg import NavSatFix  # NEW IMPORT


def yaw_from_quaternion(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DroneVizNode(Node):
    def __init__(self):
        super().__init__('drone_viz_node')

        # ---------- Parameters ----------
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('global_topic', '/mavros/global_position/global') # NEW PARAM
        self.declare_parameter('dets_topic', '/obj_dets')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('queue_state_topic', '/queue_state')
        self.declare_parameter('wp_reached_topic', '/mavros/mission/reached')
        self.declare_parameter('wp_waypoints_traj_topic', '/mavros/mission/waypoints')

        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('drone_scale', 0.6)
        self.declare_parameter('text_scale', 0.35)
        self.declare_parameter('line_width', 0.15)
        self.declare_parameter('accumulate_path', True)
        self.declare_parameter('max_path_pts', 2000)
        self.declare_parameter('dets_z', 0.0)  
        self.declare_parameter('hud_anchor', [0.0, 0.0, 2.0])  
        
        self.declare_parameter('mesh_uri', 'package://bv_core/meshes/Render_CAD.STL')
        self.declare_parameter('mesh_scale', 0.001) 
        self.declare_parameter('offsetX', -0.8)
        self.declare_parameter('offsetY', 0.6)
        self.declare_parameter('offsetZ', -0.2)
        
        self.offsetX = float(self.get_parameter('offsetX').value)
        self.offsetY = float(self.get_parameter('offsetY').value)
        self.offsetZ = float(self.get_parameter('offsetZ').value)
        self.mesh_scale = float(self.get_parameter('mesh_scale').value)
        self.mesh_uri = self.get_parameter('mesh_uri').value

        self.frame_id = self.get_parameter('fixed_frame').value
        
        pose_topic = self.get_parameter('pose_topic').value
        global_topic = self.get_parameter('global_topic').value
        dets_topic = self.get_parameter('dets_topic').value
        mission_state_topic = self.get_parameter('mission_state_topic').value
        queue_state_topic = self.get_parameter('queue_state_topic').value
        wp_reached_topic = self.get_parameter('wp_reached_topic').value
        wp_waypoints_traj_topic = self.get_parameter('wp_waypoints_traj_topic').value

        # ---------- Publishers ----------
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', marker_qos)

        # ---------- State ----------
        self._last_pose: Optional[PoseStamped] = None
        self._last_global: Optional[NavSatFix] = None # Tracks live GPS
        self._raw_waypoints: List[Waypoint] = []
        self._mission_anchored = False # Ensures we only calculate offset once per mission

        self._path_pts: List[Point] = []
        self._reached_markers_count = 0  
        self._last_reached_seq = -1  
        self._mission_state = None  
        self._queue_state = None    

        self._last_markers_pose: List[Marker] = []
        self._last_markers_path: List[Marker] = []
        self._last_markers_dets: List[Marker] = []
        self._last_markers_hud: List[Marker] = []
        self._accum_wp_markers: List[Marker] = []
        self._last_markers_planned_wp: List[Marker] = []

        # ---------- Subscriptions ----------
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos_profile=pose_qos)
        self.create_subscription(NavSatFix, global_topic, self.on_global, qos_profile=pose_qos) # NEW SUB
        self.create_subscription(String, mission_state_topic, self.on_mission_state, 10)
        self.create_subscription(Int8, queue_state_topic, self.on_queue_state, 10)
        self.create_subscription(WaypointReached, wp_reached_topic, self.on_wp_reached, 10)
        self.create_subscription(WaypointList, wp_waypoints_traj_topic, self.on_wp, 10)

        self.timer = self.create_timer(0.2, self.publish_all)

    # ---------- Marker builders ----------
    def _drone_markers(self, pose_msg: PoseStamped) -> List[Marker]:
        frame_id = self.frame_id
        markers: List[Marker] = []

        del_all = Marker()
        del_all.header.frame_id = frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'drone'
        del_all.id = 0
        del_all.action = Marker.DELETEALL
        markers.append(del_all)

        drone_mesh = Marker()
        drone_mesh.header.frame_id = frame_id
        drone_mesh.header.stamp = self.get_clock().now().to_msg()
        drone_mesh.ns = 'drone'
        drone_mesh.id = 3
        drone_mesh.type = Marker.MESH_RESOURCE
        drone_mesh.action = Marker.ADD
        drone_mesh.pose = pose_msg.pose

        drone_mesh.pose.position.x += self.offsetX
        drone_mesh.pose.position.y += self.offsetY
        drone_mesh.pose.position.z += self.offsetZ

        s = self.mesh_scale
        drone_mesh.scale.x = drone_mesh.scale.y = drone_mesh.scale.z = s
        drone_mesh.color.r = 1.0
        drone_mesh.color.g = 1.0
        drone_mesh.color.b = 1.0
        drone_mesh.color.a = 1.0
        drone_mesh.mesh_resource = self.mesh_uri
        drone_mesh.mesh_use_embedded_materials = False

        markers.extend([drone_mesh])
        return markers

    def _path_markers(self, frame_id: str) -> List[Marker]:
        del_all = Marker()
        del_all.header.frame_id = frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'path'
        del_all.id = 0
        del_all.action = Marker.DELETEALL

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = del_all.header.stamp
        line.ns = 'path'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.points = self._path_pts
        line.scale.x = float(self.get_parameter('line_width').value)
        line.color.r = 0.7
        line.color.g = 0.2
        line.color.b = 0.9
        line.color.a = 0.9

        markers = [del_all, line]

        if self._path_pts:
            start = Marker()
            start.header.frame_id = frame_id
            start.header.stamp = del_all.header.stamp
            start.ns = 'path'
            start.id = 2
            start.type = Marker.SPHERE
            start.action = Marker.ADD
            start.pose.position = self._path_pts[0]
            start.scale.x = start.scale.y = start.scale.z = max(0.15, line.scale.x * 3.0)
            start.color.r = 0.2
            start.color.g = 1.0
            start.color.b = 0.2
            start.color.a = 0.9

            end = Marker()
            end.header.frame_id = frame_id
            end.header.stamp = del_all.header.stamp
            end.ns = 'path'
            end.id = 3
            end.type = Marker.SPHERE
            end.action = Marker.ADD
            end.pose.position = self._path_pts[-1]
            end.scale.x = end.scale.y = end.scale.z = max(0.15, line.scale.x * 3.0)
            end.color.r = 1.0
            end.color.g = 0.3
            end.color.b = 0.3
            end.color.a = 0.9

            markers.extend([start])
        return markers

    def _detection_markers_bv(self, dets_msg) -> List[Marker]:
        frame_id = self.get_parameter('fixed_frame').value
        z0 = float(self.get_parameter('dets_z').value)

        del_all = Marker()
        del_all.header.frame_id = frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'detections'
        del_all.id = 0
        del_all.action = Marker.DELETEALL

        markers: List[Marker] = [del_all]
        m_id = 1
        for vec in getattr(dets_msg, 'dets', []):
            cls_id = int(vec.z)
            x_pos = float(vec.x) * 0.01
            y_pos = float(vec.y) * 0.01

            cube = Marker()
            cube.header.frame_id = frame_id
            cube.header.stamp = del_all.header.stamp
            cube.ns = 'detections'
            cube.id = m_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x_pos
            cube.pose.position.y = y_pos
            cube.pose.position.z = z0 + 0.25 
            cube.scale.x = cube.scale.y = cube.scale.z = 0.5
            
            cube.color.a = 0.85
            if cls_id == 0:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 0.2, 0.2 
            elif cls_id == 1:
                cube.color.r, cube.color.g, cube.color.b = 0.2, 1.0, 0.2 
            else:
                cube.color.r, cube.color.g, cube.color.b = 0.2, 0.5, 1.0 

            label = Marker()
            label.header = cube.header
            label.ns = 'detections'
            label.id = m_id + 1000 
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x_pos
            label.pose.position.y = y_pos
            label.pose.position.z = z0 + 0.8 
            label.scale.z = float(self.get_parameter('text_scale').value)
            label.text = f"Class {cls_id}"
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0

            markers.extend([cube, label])
            m_id += 1

        return markers

    def _hud_markers(self) -> List[Marker]:
        frame_id = self.get_parameter('fixed_frame').value
        ax, ay, az = self.get_parameter('hud_anchor').value
        text_scale = float(self.get_parameter('text_scale').value)

        del_all = Marker()
        del_all.header.frame_id = frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'hud'
        del_all.id = 0
        del_all.action = Marker.DELETEALL

        m_state = Marker()
        m_state.header.frame_id = frame_id
        m_state.header.stamp = del_all.header.stamp
        m_state.ns = 'hud'
        m_state.id = 1
        m_state.type = Marker.TEXT_VIEW_FACING
        m_state.action = Marker.ADD
        m_state.pose.position.x = ax
        m_state.pose.position.y = ay
        m_state.pose.position.z = az
        m_state.scale.z = text_scale
        m_state.text = f"Mission: {self._mission_state or '-'}"
        m_state.color.r = 0.9
        m_state.color.g = 0.9
        m_state.color.b = 1.0
        m_state.color.a = 0.95

        q_state = Marker()
        q_state.header.frame_id = frame_id
        q_state.header.stamp = del_all.header.stamp
        q_state.ns = 'hud'
        q_state.id = 2
        q_state.type = Marker.TEXT_VIEW_FACING
        q_state.action = Marker.ADD
        q_state.pose.position.x = ax
        q_state.pose.position.y = ay + 0.5
        q_state.pose.position.z = az
        q_state.scale.z = text_scale
        if self._queue_state is None:
            txt, col = "Queue: -", (1.0, 1.0, 1.0)
        else:
            if int(self._queue_state) == 1:
                txt, col = "Queue: empty", (0.3, 1.0, 0.3)
            else:
                txt, col = "Queue: processing", (1.0, 0.6, 0.2)
        q_state.text = txt
        q_state.color.r, q_state.color.g, q_state.color.b = col
        q_state.color.a = 0.95

        return [del_all, m_state, q_state]

    def _wp_markers_add(self, seq: int):
        if self._last_pose is None:
            return
        frame_id = self.frame_id
        pos = self._last_pose.pose.position

        self._reached_markers_count += 1
        sid = 1000 + self._reached_markers_count
        s = Marker()
        s.header.frame_id = frame_id
        s.header.stamp = self.get_clock().now().to_msg()
        s.ns = 'wp'
        s.id = sid
        s.type = Marker.SPHERE
        s.action = Marker.ADD
        s.pose.position.x = pos.x
        s.pose.position.y = pos.y
        s.pose.position.z = pos.z
        s.scale.x = s.scale.y = s.scale.z = 0.25
        s.color.r = 0.1
        s.color.g = 0.8
        s.color.b = 1.0
        s.color.a = 0.9

        self._reached_markers_count += 1
        tid = 1000 + self._reached_markers_count
        t = Marker()
        t.header.frame_id = frame_id
        t.header.stamp = s.header.stamp
        t.ns = 'wp'
        t.id = tid
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = pos.x
        t.pose.position.y = pos.y
        t.pose.position.z = pos.z + 0.4
        t.scale.z = float(self.get_parameter('text_scale').value)
        t.text = f"WP {seq}"
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 0.95

        self._accum_wp_markers.extend([s, t])

    def _marker_from_waypoint(self, x_m: float, y_m: float, z_m: float, marker_id: int) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planned_wp'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x_m
        m.pose.position.y = y_m
        m.pose.position.z = z_m
        m.scale.x = m.scale.y = m.scale.z = 0.5 
        m.color.r = 0.9
        m.color.g = 0.9
        m.color.b = 0.1
        m.color.a = 1.0
        return m

    def _build_planned_wp_markers(self):
        """Calculates exact offset between GPS and Local to anchor waypoints perfectly"""
        del_all = Marker()
        del_all.header.frame_id = self.frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'planned_wp'
        del_all.id = 0
        del_all.action = Marker.DELETEALL
        
        new_markers = [del_all]

        # Use drone's current live location as the universal translation anchor
        anchor_lat = self._last_global.latitude
        anchor_lon = self._last_global.longitude
        anchor_local_x = self._last_pose.pose.position.x
        anchor_local_y = self._last_pose.pose.position.y

        for i, wp in enumerate(self._raw_waypoints):
            # 1. Delta in degrees from the anchor
            delta_lat = wp.x_lat - anchor_lat
            delta_lon = wp.y_long - anchor_lon

            # 2. Convert to meters (ROS ENU: X is East/Lon, Y is North/Lat)
            dist_x = delta_lon * 111319.9 * math.cos(math.radians(anchor_lat))
            dist_y = delta_lat * 111319.9

            # 3. Apply offset to local coordinate system
            local_x = anchor_local_x + dist_x
            local_y = anchor_local_y + dist_y

            m = self._marker_from_waypoint(local_x, local_y, wp.z_alt, marker_id=i+1)
            new_markers.append(m)

        self._last_markers_planned_wp = new_markers
        self._mission_anchored = True

    # ---------- Callbacks ----------

    def on_global(self, msg: NavSatFix):
        self._last_global = msg
        # If we received a mission before GPS lock, draw it now
        if self._raw_waypoints and not self._mission_anchored and self._last_pose:
            self._build_planned_wp_markers()

    def on_wp(self, msg: WaypointList):
        if not msg.waypoints:
            return
        
        self._raw_waypoints = msg.waypoints
        self._mission_anchored = False # Reset flag for new mission
        
        # Draw immediately if we already have GPS lock
        if self._last_global and self._last_pose:
            self._build_planned_wp_markers()

    def on_pose(self, msg: PoseStamped):
        self._last_pose = msg
        frame_id = self.frame_id

        if bool(self.get_parameter('accumulate_path').value):
            pt = Point()
            pt.x = msg.pose.position.x
            pt.y = msg.pose.position.y
            pt.z = msg.pose.position.z
            self._path_pts.append(pt)
            max_n = int(self.get_parameter('max_path_pts').value)
            if len(self._path_pts) > max_n:
                self._path_pts = self._path_pts[-max_n:]

        self._last_markers_pose = self._drone_markers(msg)
        self._last_markers_path = self._path_markers(frame_id)

    def on_obj_dets(self, msg):
        self._last_markers_dets = self._detection_markers_bv(msg)

    def on_mission_state(self, msg: String):
        self._mission_state = msg.data
        self._last_markers_hud = self._hud_markers()

    def on_queue_state(self, msg: Int8):
        self._queue_state = msg.data
        self._last_markers_hud = self._hud_markers()

    def on_wp_reached(self, msg: WaypointReached):
        try:
            seq = int(msg.wp_seq)
        except Exception:
            seq = 0
            
        if seq != getattr(self, '_last_reached_seq', -1):
            self.get_logger().info(f"New Waypoint reached: seq={seq}")
            self._wp_markers_add(seq)
            self._last_reached_seq = seq

    # ---------- Publishing ----------
    def publish_all(self):
        ma = MarkerArray()
       
        for group in (
            self._last_markers_pose,
            self._last_markers_path,
            self._last_markers_dets,
            self._last_markers_hud,
            self._accum_wp_markers,
            self._last_markers_planned_wp
        ):
            if group: 
                ma.markers.extend(group)
        if ma.markers:
            self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = DroneVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
