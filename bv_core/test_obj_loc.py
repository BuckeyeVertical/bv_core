import rclpy
from rclpy.node import Node
from bv_msgs.srv import GetObjectLocations     # your custom srv
from bv_msgs.msg import ObjectLocations         # your custom msg
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class TestObjLoc(Node):

    def __init__(self):
        super().__init__('test_obj_loc')  # Node name visible
        self.get_logger().info('test object locations service node has started')

        locations_yaml = os.path.join(
            get_package_share_directory('bv_core'),
            'config',
            'test_obj_loc.yaml'
        )

        with open(locations_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.locations = cfg.get("locations", [])

        self.get_obj_locs_srv = self.create_service(
            GetObjectLocations,
            'get_object_locations',
            self.handle_get_object_locations
        )

    def handle_get_object_locations(self, request, response):
        for (lon, lat, cls_id) in self.locations:
            loc = ObjectLocations()
            loc.latitude = float()
            loc.longitude = float()
            loc.class_id = int()
            response.locations.append(loc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TestObjLoc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
