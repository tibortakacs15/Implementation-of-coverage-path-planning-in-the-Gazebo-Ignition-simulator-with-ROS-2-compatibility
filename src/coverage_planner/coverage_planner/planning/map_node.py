import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

# map_node.py

class MapNode(Node):

    def __init__(self):
        super().__init__('map_node')

        # Subscribe to occupancy grid messages on /map
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.get_logger().info("Map received!")

        # Read the map dimensions in cells
        width = msg.info.width
        height = msg.info.height

        occupancy_grid = np.array(msg.data).reshape((height, width))

        # Print the grid shape and its top-left 5-by-5 block for debugging
        print("Map shape:", occupancy_grid.shape)
        print("First values:", occupancy_grid[:5, :5])


def main(args=None): 
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()