import rclpy
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.signals import SignalHandlerOptions


class PathFollower(Node):

    # Set up subscriptions, velocity publishing, and the control timer
    def __init__(self):
        super().__init__("path_follower")

        self.ground_truth_x = None
        self.ground_truth_y = None
        self.ground_truth_yaw = None

        self.last_ground_truth_time = None
        self.pose_timeout = 0.5
        self.pose_fault = False

        self.ground_truth_sub = self.create_subscription(
            PoseStamped,
            "/model/burger/pose",
            self.ground_truth_callback,
            10
        )


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = None
        self.y = None
        self.yaw = None

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )

        self.received_path = None

        self.waypoints = []

        self.target_index = 0

        self.path_initialized = False

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

        self.route_finished = False

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

    # Store the position and heading estimated from odometry
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation

        siny = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)

        self.yaw = math.atan2(siny, cosy)

    # Publish the requested forward and turning speeds
    def publish_velocity(self, linear_speed, angular_speed):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = linear_speed 
        msg.twist.angular.z = angular_speed

        self.cmd_pub.publish(msg)

    # Drive toward the current waypoint while the pose data is fresh
    def control_loop(self):
        if self.ground_truth_x is None  or self.ground_truth_y is None or self.ground_truth_yaw is None or self.last_ground_truth_time is None:
            self.publish_velocity(0.0, 0.0)
            return

        if self.pose_fault:
            self.get_logger().warning(
                "Pose fault is latched. Restart the follower to resume.",
                throttle_duration_sec=1.0
            )
            self.publish_velocity(0.0, 0.0)
            return

        pose_age = self.get_clock().now() - self.last_ground_truth_time
        pose_age = pose_age.nanoseconds / 1_000_000_000

        # Stop on stale pose data and require a restart to resume
        if pose_age > self.pose_timeout:
            self.pose_fault = True
            self.get_logger().warning("Ground-truth pose is stale. Stopping robot.", throttle_duration_sec=1.0)
            self.publish_velocity(0.0, 0.0)
            return

        if self.target_index >= len(self.waypoints):
            self.publish_velocity(0.0, 0.0)

            if self.path_initialized == True and self.route_finished == False:
                self.get_logger().info("Route finished")
                self.route_finished = True
                
            return

        target_x =  self.waypoints[self.target_index][0]
        target_y =  self.waypoints[self.target_index][1]

        dx = target_x - self.ground_truth_x
        dy = target_y - self.ground_truth_y

        distance = math.hypot(dx, dy)

        # Advance to the next waypoint when the robot is within 5 cm
        if distance < 0.05:
            self.get_logger().info(f"Waypoint {self.target_index + 1} reached.")
            self.target_index += 1
            self.publish_velocity(0.0, 0.0)
            return

        target_yaw = math.atan2(dy, dx)
        angle_error = target_yaw - self.ground_truth_yaw

        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        angular_speed = 2* angle_error

        if angular_speed > 0.2:
            angular_speed = 0.2
        elif angular_speed < -0.2:
            angular_speed = -0.2

        # Turn in place until the robot is nearly facing the waypoint
        if abs(angle_error) > 0.1:
            linear_speed = 0.0
        else:
            linear_speed = 0.05

        self.publish_velocity(linear_speed, angular_speed)

    # Accept the first valid path in the map frame once the robot pose is available
    def path_callback(self, msg):

        if self.path_initialized:
            return

        self.received_path = msg

        received_points = []

        for pose in msg.poses:
            received_points.append((pose.pose.position.x, pose.pose.position.y))

        if len(received_points) == 0:
            self.get_logger().warning("Received an empty path.")
            return

        if msg.header.frame_id != "map":
            self.get_logger().warning("Expected path in map frame.")
            return

        if self.ground_truth_x is None or self.ground_truth_y is None or self.ground_truth_yaw is None:
            self.get_logger().warning("Waiting for ground-truth before preparing the path.")
            return

        self.waypoints = received_points
        self.target_index = 0
        self.route_finished = False
        self.path_initialized = True

        self.get_logger().info(f"Received path: frame={msg.header.frame_id}, points={len(msg.poses)}")

    # Update the Gazebo pose and compare it with the odometry estimate in the map frame
    def ground_truth_callback(self, msg):

        self.last_ground_truth_time = self.get_clock().now()

        self.ground_truth_x = msg.pose.position.x
        self.ground_truth_y = msg.pose.position.y

        orientation = msg.pose.orientation

        siny = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        
        self.ground_truth_yaw = math.atan2(siny, cosy)

        if self.x is None or self.y is None or self.yaw is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for odometry transform: {error}",
                throttle_duration_sec=1.0
            )
            return

        translation_x = transform.transform.translation.x
        translation_y = transform.transform.translation.y

        rotation = transform.transform.rotation
        
        siny = 2 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy = 1 - 2 * (rotation.y ** 2 + rotation.z ** 2)
        transform_yaw = math.atan2(siny, cosy)

        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)

        estimated_map_x = cos_yaw * self.x - sin_yaw * self.y + translation_x
        estimated_map_y = sin_yaw * self.x + cos_yaw * self.y + translation_y

        position_error_x = estimated_map_x - self.ground_truth_x
        position_error_y = estimated_map_y - self.ground_truth_y
        position_error = math.hypot(position_error_x, position_error_y)

        self.get_logger().info(f"elteres: {position_error:.3f} m", throttle_duration_sec=1.0)

        estimated_map_yaw = self.yaw + transform_yaw

        yaw_error = estimated_map_yaw - self.ground_truth_yaw

        yaw_error = math.atan2(
            math.sin(yaw_error),
            math.cos(yaw_error)
        )

        self.get_logger().info(f"Yaw error: {math.degrees(yaw_error):.3f} deg", throttle_duration_sec=1.0)
        
        self.get_logger().info(f"Ground truth: x={self.ground_truth_x:.3f}, y = {self.ground_truth_y:.3f}, yaw= {self.ground_truth_yaw:.3f}", throttle_duration_sec=1.0)
        

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.control_timer.cancel()
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()