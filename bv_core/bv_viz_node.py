#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_viz_node.py  (Aligned with your Vision & Mission topics)

Subscribes (from your repo):
  • /mavros/local_position/pose   (geometry_msgs/PoseStamped)        [filtering_node]
  • /obj_dets                     (bv_msgs/ObjectDetections)         [vision_node publishes]
  • /mission_state                (std_msgs/String)                  [mission publishes]
  • /queue_state                  (std_msgs/Int8)                    [vision publishes]
  • /mavros/mission/reached       (mavros_msgs/WaypointReached)      [vision subscribes, mission publishes events]

Publishes for Foxglove/RViz:
  • /viz/markers (visualization_msgs/MarkerArray)
      - Drone arrow + HUD text (yaw/alt)
      - Accumulated path line with start/end dots
      - Detections as text labels (from ObjectDetections.dets Vector3: x,y = px, z = class id)
      - Mission/Queue HUD text
      - Waypoint reached spheres + labels (accumulated)

Notes:
  • QoS for markers uses BEST_EFFORT so they persist in Foxglove/RViz.
  • All topic/frame names and sizes are configurable via parameters.
"""

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


def yaw_from_quaternion(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DroneVizNode(Node):
    def __init__(self):
        super().__init__('drone_viz_node')

        # ---------- Parameters ----------
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('dets_topic', '/obj_dets')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('queue_state_topic', '/queue_state')
        self.declare_parameter('wp_reached_topic', '/mavros/mission/reached')

        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('drone_scale', 0.6)
        self.declare_parameter('text_scale', 0.35)
        self.declare_parameter('line_width', 0.07)
        self.declare_parameter('accumulate_path', True)
        self.declare_parameter('max_path_pts', 2000)
        self.declare_parameter('dets_z', 0.0)  # z-height for 2D det labels (pixels -> world placeholder)
        self.declare_parameter('hud_anchor', [0.0, 0.0, 2.0])  # where to place mission/queue HUD text
        
        self.declare_parameter('mesh_uri', 'package://bv_core/meshes/Render_CAD.STL')
        self.declare_parameter('mesh_scale', 0.001) 
        self.declare_parameter('offsetX', 0.9)
        self.declare_parameter('offsetY', -0.1)
        self.declare_parameter('offsetZ', -0.3)
        

        self.offsetX = float(self.get_parameter('offsetX').value)
        self.offsetY = float(self.get_parameter('offsetY').value)
        self.offsetZ = float(self.get_parameter('offsetZ').value)
        self.mesh_scale = float(self.get_parameter('mesh_scale').value)
        self.mesh_uri = self.get_parameter('mesh_uri').value


        self.frame_id = self.get_parameter('fixed_frame').value
        
        pose_topic = self.get_parameter('pose_topic').value
        dets_topic = self.get_parameter('dets_topic').value
        mission_state_topic = self.get_parameter('mission_state_topic').value
        queue_state_topic = self.get_parameter('queue_state_topic').value
        wp_reached_topic = self.get_parameter('wp_reached_topic').value

        # ---------- Publishers ----------
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # RViz/Foxglove-friendly persistence
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', marker_qos)

        # ---------- State ----------
        self._last_pose: Optional[PoseStamped] = None
        self._path_pts: List[Point] = []
        self._reached_markers_count = 0  # to assign unique IDs and accumulate
        self._mission_state = None  # String
        self._queue_state = None    # Int8 (0=processing, 1=empty)

        self._last_markers_pose: List[Marker] = []
        self._last_markers_path: List[Marker] = []
        self._last_markers_dets: List[Marker] = []
        self._last_markers_hud: List[Marker] = []
        # Waypoint markers accumulate; we store them separately to append over time
        self._accum_wp_markers: List[Marker] = []

        # ---------- Subscriptions ----------
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos_profile=pose_qos)

        self.create_subscription(String, mission_state_topic, self.on_mission_state, 10)
        self.create_subscription(Int8, queue_state_topic, self.on_queue_state, 10)
        self.create_subscription(WaypointReached, wp_reached_topic, self.on_wp_reached, 10)

        # Publish at a steady rate so Foxglove receives updates even without new messages
        self.timer = self.create_timer(0.2, self.publish_all)

    # ---------- Marker builders ----------
    def _drone_markers(self, pose_msg: PoseStamped) -> List[Marker]:
        frame_id = self.frame_id
        markers: List[Marker] = []

        # Clear existing "drone" namespace
        del_all = Marker()
        del_all.header.frame_id = frame_id
        del_all.header.stamp = self.get_clock().now().to_msg()
        del_all.ns = 'drone'
        del_all.id = 0
        del_all.action = Marker.DELETEALL
        markers.append(del_all)

        # Arrow
        # arrow = Marker()
        # arrow.header.frame_id = frame_id
        # arrow.header.stamp = self.get_clock().now().to_msg()
        # arrow.ns = 'drone'
        # arrow.id = 1
        # arrow.type = Marker.ARROW
        # arrow.action = Marker.ADD
        # arrow.pose = pose_msg.pose
        # scale = float(self.get_parameter('drone_scale').value)
        # arrow.scale.x = scale * 1.5
        # arrow.scale.y = scale * 0.3
        # arrow.scale.z = scale * 0.3
        # arrow.color.r = 0.0
        # arrow.color.g = 0.8
        # arrow.color.b = 0.7
        # arrow.color.a = 1.0

        # Text (yaw/alt)
        # text = Marker()
        # text.header.frame_id = frame_id
        # text.header.stamp = del_all.header.stamp
        # text.ns = 'drone'
        # text.id = 2
        # text.type = Marker.TEXT_VIEW_FACING
        # text.action = Marker.ADD
        # text.pose.position.x = pose_msg.pose.position.x
        # text.pose.position.y = pose_msg.pose.position.y
        # text.pose.position.z = pose_msg.pose.position.z + scale * 1.0
        # text.scale.z = float(self.get_parameter('text_scale').value)
        # yaw = yaw_from_quaternion(
        #     pose_msg.pose.orientation.x,
        #     pose_msg.pose.orientation.y,
        #     pose_msg.pose.orientation.z,
        #     pose_msg.pose.orientation.w
        # )
        # text.text = f"Yaw {math.degrees(yaw):.1f}° | Alt {pose_msg.pose.position.z:.1f} m"
        # text.color.r = 1.0
        # text.color.g = 1.0
        # text.color.b = 1.0
        # text.color.a = 0.95
        #

        #Drone Mesh Marker
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
        
        # markers.append(arrow)
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
        """Render 2D detections as text labels in fixed_frame at z=dets_z.
        (No camera projection; uses scaled pixel coords.)"""
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
            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = del_all.header.stamp
            label.ns = 'detections'
            label.id = m_id
            m_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(vec.x) * 0.01
            label.pose.position.y = float(vec.y) * 0.01
            label.pose.position.z = z0
            label.scale.z = float(self.get_parameter('text_scale').value)
            cls_id = int(vec.z)
            label.text = f"cls {cls_id}"
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.color.a = 0.95
            markers.append(label)
        return markers

    def _hud_markers(self) -> List[Marker]:
        """Mission/Queue text in a fixed position (HUD)."""
        frame_id = self.get_parameter('fixed_frame').value
        ax, ay, az = self.get_parameter('hud_anchor').value
        text_scale = float(self.get_parameter('text_scale').value)

        # Clear existing "hud" namespace
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
        """Add a sphere + label at the last known drone position when a waypoint is reached."""
        if self._last_pose is None:
            return
        frame_id = self.frame_id
        pos = self._last_pose.pose.position

        # Sphere
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

        # Label
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

    # ---------- Callbacks ----------
    def on_pose(self, msg: PoseStamped):
        self._last_pose = msg
        frame_id = self.frame_id

        # update path accumulator
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
        self._wp_markers_add(seq)

    # ---------- Publishing ----------
    def publish_all(self):
        ma = MarkerArray()
        # DO NOT clear 'wp' namespace; we accumulate it over time.
        # For other namespaces we already include DELETEALL on each group build.
        for group in (
            self._last_markers_pose,
            self._last_markers_path,
            self._last_markers_dets,
            self._last_markers_hud,
            self._accum_wp_markers,
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
