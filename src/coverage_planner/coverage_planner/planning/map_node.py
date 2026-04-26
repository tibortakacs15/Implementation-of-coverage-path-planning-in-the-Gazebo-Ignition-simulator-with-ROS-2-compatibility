import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class MapNode(Node):

    def __init__(self):
        super().__init__('map_node')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.get_logger().info("Map received!")

        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data).reshape((height, width))

        print("Map shape:", data.shape)
        print("First values:", data[:5, :5])


def main(args=None): 
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()