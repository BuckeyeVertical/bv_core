from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from .Vision_Pipeline import VisionPipeline

class RosCamPipeline(VisionPipeline):
    """
    A VisionPipeline that subscribes to a ROS2 CompressedImage topic
    and feeds decoded frames into the standardized processing queue.
    """

    def __init__(self, node: Node, topic: str, *, queue_size: int = 2, qos_profile: QoSProfile = None):
        """
        Args:
            node: The ROS2 node instance.
            topic: The ROS topic string 
            queue_size: Max frames to buffer.
            qos_profile: Optional QoS profile for subscription.
        """
        super().__init__(max_queue_size=queue_size)

        self._node = node
        self._topic = topic
        self._bridge = CvBridge()
        self._sub = None

        if qos_profile is None:
            self._qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:
            self._qos = qos_profile

    def start(self):
        #Subscribe to the compressed topic.
        if self._sub is not None:
            return
        #Sanity check to make sure right topic
        self._node.get_logger().info(f"RosCamPipeline starting subscription to {self._topic}")
        
        # Subscribe to compressed image topic (most recent rosbag uses this as opposed to ImageRaw)
        self._sub = self._node.create_subscription(
            CompressedImage,
            self._topic,
            self._img_callback,
            self._qos
        )

    def stop(self):
        #Stop subscription.
        if self._sub is None:
            return

        self._node.destroy_subscription(self._sub)
        self._sub = None
        self._clear_queue()
        self._node.get_logger().info(f"RosCamPipeline stopped subscription.")

    def get_frame(self, timeout=None):
        return super().get_frame(timeout=timeout)

    def _img_callback(self, msg: CompressedImage):
        #compressed Rosimg to numpy.ndarray
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            self._enqueue_frame(frame)
            
        except CvBridgeError as e:
            self._node.get_logger().warn(f"RosCamPipeline Bridge Error: {e}")
        except Exception as e:
            self._node.get_logger().error(f"RosCamPipeline Processing Error: {e}")