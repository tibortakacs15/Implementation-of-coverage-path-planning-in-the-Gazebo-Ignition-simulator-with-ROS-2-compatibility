import rclpy
import os
import subprocess

from rclpy.node import Node
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster 


class PathStartSpawner(Node):
    # Set up the spawn parameters, transform broadcaster, and path subscription
    def __init__(self):
        super().__init__("path_start_spawner")

        self.spawn_requested = False

        self.declare_parameter("robot_file", "")
        self.declare_parameter("world_name", "generated_map")

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_sub = self.create_subscription(
            Path,
            "/coverage_path_viz",
            self.path_callback,
            path_qos
        )

    # Publish the static map-to-odom transform at the spawn point
    def publish_map_to_odom(self, x, y):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    # Spawn the robot at the first waypoint of a valid path
    def path_callback(self, msg):

        if self.spawn_requested:
            return
        
        if not msg.poses:
            return

        if msg.header.frame_id != "map":
            return

        first_position = msg.poses[0].pose.position

        robot_file = self.get_parameter("robot_file").value
        world_name = self.get_parameter("world_name").value

        if not os.path.isfile(robot_file):
            self.get_logger().warning(f"Robot file is not found: {robot_file}")
            return

        spawn_command = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", world_name,
            "-name", "burger",
            "-file", robot_file,
            "-x", str(first_position.x), "-y", str(first_position.y),
            "-z", "0.01"
        ]

        # Prevent repeated spawn attempts when the path is published again
        self.spawn_requested = True

        try:
            subprocess.run(spawn_command, check=True, timeout=30)
        except (OSError, subprocess.SubprocessError) as error:
            self.get_logger().warning(f"Failed to spawn robot: {error}")
            return

        self.publish_map_to_odom(first_position.x, first_position.y)

        self.get_logger().info(f"Spawn point: x={first_position.x:.3f} m, y={first_position.y:.3f} m")

    
        


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = PathStartSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()