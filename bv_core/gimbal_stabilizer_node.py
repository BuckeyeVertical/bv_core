#!/usr/bin/env python3

import math
import time
import csv

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
        self.declare_parameter('log_file', '/tmp/gimbal_pitch_log.csv')

        attitude_topic = self.get_parameter('attitude_topic').value
        self.roll_topic = self.get_parameter('roll_cmd_topic').value
        self.pitch_topic = self.get_parameter('pitch_cmd_topic').value
        self._log_file = self.get_parameter('log_file').value

        # Data logging for PID tuning
        self._start_time = time.time()
        self._pitch_data = []  # (time, actual_pitch, target_joint_angle, pitch_cmd)

        self._gz_node = GzNode()
        self._gazebo_publishers = {}
        self._ensure_gz_publisher(self.roll_topic)
        self._ensure_gz_publisher(self.pitch_topic)

        subscribed = self._gz_node.subscribe(GzImuMsg, attitude_topic, self._handle_gz_imu)
        if not subscribed:
            raise RuntimeError(f"Failed to subscribe to Gazebo IMU topic '{attitude_topic}'")

        self.get_logger().info(
            f"Gimbal stabilizer listening to {attitude_topic} and publishing roll"
            f" commands to {self.roll_topic}, pitch commands to {self.pitch_topic}"
        )
        self.get_logger().info(f"Logging pitch data to {self._log_file}")

    def _handle_gz_imu(self, msg: GzImuMsg):
        orientation = msg.orientation
        roll, pitch = self._quaternion_to_roll_pitch(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        if roll is None or pitch is None:
            self.get_logger().warn('Received invalid orientation quaternion; skipping command.')
            return

        roll_cmd = 0.0
        # Target joint angle = negative of measured pitch to compensate and point down
        pitch_cmd = pitch  

        # Log data for PID tuning
        # target = 0 (our GOAL: camera pointing straight down in world frame)
        # error = actual_pitch - 0 = actual_pitch (how far from pointing down)
        elapsed = time.time() - self._start_time
        target_world_pitch = 0.0  # Goal: camera pointing down
        self._pitch_data.append((elapsed, pitch, target_world_pitch, pitch_cmd))

        self._publish_gz(self.roll_topic, roll_cmd)
        self._publish_gz(self.pitch_topic, pitch_cmd)

    @staticmethod
    def _quaternion_to_roll_pitch(x, y, z, w):
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0.0:
            return None, None
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

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

    def save_data(self):
        """Save logged pitch data to CSV file."""
        if not self._pitch_data:
            self.get_logger().warn("No pitch data to save")
            return
        with open(self._log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'actual_pitch_rad', 'target_pitch_rad', 'pitch_cmd_rad'])
            writer.writerows(self._pitch_data)
        self.get_logger().info(f"Saved {len(self._pitch_data)} samples to {self._log_file}")


def plot_pitch_data(csv_file):
    """Plot pitch vs target from CSV file for PID tuning analysis."""
    import matplotlib.pyplot as plt

    times, actual_pitches, target_pitches = [], [], []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time_s']))
            actual_pitches.append(float(row['actual_pitch_rad']))
            target_pitches.append(float(row['target_pitch_rad']))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1.plot(times, actual_pitches, 'b-', label='Actual Pitch', linewidth=1)
    ax1.plot(times, target_pitches, 'r--', label='Target (0 rad)', linewidth=2)
    ax1.set_ylabel('Pitch (radians)')
    ax1.set_title('Gimbal Pitch Stabilization - PID Tuning Analysis')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    errors = [a - t for a, t in zip(actual_pitches, target_pitches)]
    ax2.plot(times, errors, 'g-', label='Error', linewidth=1)
    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Error (radians)')
    ax2.set_title('Pitch Error Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(plot_file, dpi=150)
    print(f"Plot saved to {plot_file}")
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--plot':
        csv_file = sys.argv[2] if len(sys.argv) > 2 else '/tmp/gimbal_pitch_log.csv'
        plot_pitch_data(csv_file)
    else:
        main()