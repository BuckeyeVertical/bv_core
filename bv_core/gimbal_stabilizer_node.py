#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double as DoubleMsg
from gz.msgs10.imu_pb2 import IMU as GzImuMsg


class GimbalStabilizerNode(Node):
    def __init__(self):
        super().__init__('gimbal_stabilizer_node')

        self.declare_parameter(
            'attitude_topic',
            '/world/baylands/model/x500_gimbal_0/link/camera_link/sensor/camera_imu/imu'
        )
        self.declare_parameter(
            'roll_cmd_topic',
            '/model/x500_gimbal_0/command/gimbal_roll'
        )
        self.declare_parameter(
            'pitch_cmd_topic',
            '/model/x500_gimbal_0/command/gimbal_pitch'
        )

        attitude_topic = self.get_parameter('attitude_topic').value
        self.roll_topic = self.get_parameter('roll_cmd_topic').value
        self.pitch_topic = self.get_parameter('pitch_cmd_topic').value

        self._gz_node = GzNode()
        self._gazebo_publishers = {}
        self._ensure_gz_publisher(self.roll_topic)
        self._ensure_gz_publisher(self.pitch_topic)

        self._roll_cmd = 0.0
        self._pitch_cmd = 0.0

        subscribed = self._gz_node.subscribe(GzImuMsg, attitude_topic, self._handle_gz_imu)
        if not subscribed:
            raise RuntimeError(f"Failed to subscribe to Gazebo IMU topic '{attitude_topic}'")

        self._control_timer = self.create_timer(1.0 / 30.0, self._control_loop)

        self.get_logger().info(
            f"Gimbal stabilizer listening to {attitude_topic} and publishing roll"
            f" commands to {self.roll_topic}, pitch commands to {self.pitch_topic}"
        )

    def _handle_gz_imu(self, msg: GzImuMsg):
        orientation = msg.orientation
        roll, pitch = self._quaternion_to_roll_pitch(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        if roll is None or pitch is None:
            self.get_logger().warn('Received invalid orientation quaternion; skipping command.')
            return

        self._roll_cmd = 0.0
        # Target joint angle = negative of measured pitch to compensate and point down
        self._pitch_cmd = pitch

    def _control_loop(self):
        self._publish_gz(self.roll_topic, self._roll_cmd)
        self._publish_gz(self.pitch_topic, self._pitch_cmd)

    @staticmethod
    def _quaternion_to_roll_pitch(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return None, None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        return roll, pitch

    def _ensure_gz_publisher(self, topic):
        publisher = self._gz_node.advertise(topic, DoubleMsg)
        if not publisher:
            raise RuntimeError(f"Failed to advertise Gazebo topic '{topic}'")
        self._gazebo_publishers[topic] = publisher

    def _publish_gz(self, topic, value):
        msg = DoubleMsg()
        msg.data = value
        publisher = self._gazebo_publishers.get(topic)
        publish_fn = getattr(publisher, "publish", None) or getattr(publisher, "Publish", None)
        if publish_fn:
            publish_fn(msg)
        else:
            self._gz_node.publish(topic, msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
