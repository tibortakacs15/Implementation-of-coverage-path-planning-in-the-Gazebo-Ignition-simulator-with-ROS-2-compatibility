#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

from rclpy.action import ActionClient
from coverage_interfaces.action import Coverage

from action_msgs.msg import GoalStatus


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Parameters
        self.declare_parameter("map_name", "my_room.png")
        self.declare_parameter("robot_size", 11)
        self.declare_parameter("threshold", 0.55)
        self.declare_parameter("angle_degrees", 0.0)

        self.map_name = self.get_parameter("map_name").value
        self.robot_size = self.get_parameter("robot_size").value
        self.threshold = self.get_parameter("threshold").value
        self.angle_degrees = self.get_parameter("angle_degrees").value

        # Publishers
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path_viz',
            path_qos
        )
        self.marker_pub = self.create_publisher(Marker, '/coverage_marker', 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Timer
        self.timer = self.create_timer(1.0, self.publish_path)

        # Action client
        self._action_client = ActionClient(self, Coverage, '/coverage_path')

        # Data
        self.points = []

        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.get_logger().info(f"Map: {self.map_name}, robot: {self.robot_size}, thr: {self.threshold}")

        #self.send_goal()

        self.goal_timer = self.create_timer(
            0.5,
            self.try_send_goal
        )

    # MAP 

    # Store the map dimensions, resolution, and origin position
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.get_logger().info(
            f"Map: {self.map_width}x{self.map_height}, res={self.map_resolution}"
        )

    # PUBLISH 

    # Convert pixel coordinates to meters and publish the path and marker
    def publish_path(self):

        if not self.points:
            return

        if self.map_resolution is None:
            self.get_logger().warn("Waiting for map...")
            return

        height = self.map_height

        # PATH 
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for x, y in self.points:
            pose = PoseStamped()
            pose.header.frame_id = "map"

            world_x = (x + 0.5) * self.map_resolution + self.map_origin_x
            world_y = ((height - y) - 0.5) * self.map_resolution + self.map_origin_y

            pose.pose.position.x = float(world_x)
            pose.pose.position.y = float(world_y)
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.get_logger().info(f"Start point{path_msg.poses[0].pose.position.x} es {path_msg.poses[0].pose.position.y}")
        self.path_pub.publish(path_msg)

        # MARKER 
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "coverage_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in self.points:
            p = Point()

            p.x = (x + 0.5) * self.map_resolution + self.map_origin_x
            p.y = ((height - y) - 0.5) * self.map_resolution + self.map_origin_y
            p.z = 0.0

            marker.points.append(p)

        self.marker_pub.publish(marker)

        self.get_logger().info("Path published")

    # Send one planning request when the action server is ready
    def try_send_goal(self):
        if not self._action_client.server_is_ready():
            return
        self.goal_timer.cancel()
        self.send_goal()

    # Build and send the coverage planning request
    def send_goal(self):
        goal_msg = Coverage.Goal()

        import os
        home = os.path.expanduser("~")

        map_path = os.path.join(
            home,
            "coverage_planner_ws",
            "maps",
            self.map_name
        )

        goal_msg.map_path = map_path
        goal_msg.robot_size = int(self.robot_size)
        goal_msg.threshold = float(self.threshold)
        goal_msg.angle_degrees = float(self.angle_degrees)

        self.get_logger().info(f"MAP USED: {map_path}")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Request the result if the server accepts the goal
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Validate the planning result and store the coordinates as point pairs
    def get_result_callback(self, future):
        response = future.result()

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.points = []
            self.get_logger().error("Coverage planning did not succeed.")
            return

        result = response.result

        raw = result.path

        if len(raw) % 2 != 0:
            self.points = []
            self.get_logger().error("Invalid path: an even number of coordinates is required.")
            return

        self.points = []

        for i in range(0, len(raw), 2):
            x = raw[i]
            y = raw[i + 1]
            self.points.append((x, y))

        self.get_logger().info(f"Received {len(self.points)} points")


# MAIN 
def main():
    rclpy.init()
    node = PathVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
