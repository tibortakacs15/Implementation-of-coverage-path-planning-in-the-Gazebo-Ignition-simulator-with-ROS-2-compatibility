#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from coverage_interfaces.action import Coverage
from coverage_planner.planning.planner import plan_coverage

import os
import time

class CoverageActionServer(Node):

    def __init__(self):
        super().__init__('coverage_action_server')

        self._action_server = ActionServer(
            self,
            Coverage,
            'coverage_path',
            self.execute_callback
        )

        self.get_logger().info("Coverage Action Server started")

    async def execute_callback(self, goal_handle):

        self.get_logger().info("=== EXECUTE CALLBACK STARTED ===")

        # Request parameters

        filename = goal_handle.request.map_path
        robot_size = goal_handle.request.robot_size
        threshold_value = goal_handle.request.threshold
        angle_degrees = goal_handle.request.angle_degrees

        start_time = time.perf_counter()

        if not os.path.exists(filename):
            self.get_logger().error("MAP FILE NOT FOUND!")
            goal_handle.abort()
            return Coverage.Result()

        # Coverage path planning

        try:
            visited = plan_coverage(filename, robot_size, threshold_value, angle_degrees, 2)
        except (ValueError, OSError) as error:
            self.get_logger().error(f"Coverage planning failed: {error}")
            goal_handle.abort()
            return Coverage.Result()

        
        # Result formatting

        flat = []
        for p in visited:
            flat.extend([float(p[0]), float(p[1])])

        result = Coverage.Result()
        result.path = flat

        goal_handle.succeed()

        end_time = time.perf_counter()
        self.get_logger().info(f"Done in {end_time - start_time:.2f}s")

        return result


def main(args=None):
    rclpy.init(args=args)
    node = CoverageActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


