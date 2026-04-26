#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from coverage_interfaces.action import Coverage

import numpy as np
import matplotlib.image as mpimg
import os
import time
from scipy import ndimage

# planning modulok
from coverage_planner.planning.map_processing import *
from coverage_planner.planning.calculation_of_parallel_lines import *
from coverage_planner.planning.save_map_and_obstacle_boundaries import *
from coverage_planner.planning.calculation_of_intersection_points import *
from coverage_planner.planning.calculating_auxiliary_lines import *
from coverage_planner.planning.staggering_of_main_lines import *


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

        filename = goal_handle.request.map_path
        robot_size = goal_handle.request.robot_size
        threshold_value = goal_handle.request.threshold

        start_time = time.perf_counter()

        if not os.path.exists(filename):
            self.get_logger().error("MAP FILE NOT FOUND!")
            goal_handle.abort()
            return Coverage.Result()

        img = mpimg.imread(filename)
        self.get_logger().info(f"IMAGE SHAPE: {img.shape}")

        # 1. MAP PROCESSING

        gray = rgb2gray(img)
        binary = np.where(gray > threshold_value, 1, 0)

        labeled, num_features = ndimage.label(binary)

        coordinate_points = calculate_edges_coordinates(labeled, 0)
        outside = calculate_edges_coordinates(labeled, 1)
        internal = calculate_edges_coordinates(labeled, 2)
        obstacle = calculate_edges_coordinates(labeled, num_features)

        n_coord = np.array(coordinate_points)
        n_outside = np.array(outside)
        n_obstacle = np.array(obstacle)

        minmax = min_max_coord(coordinate_points)

        angle = 177
        scale = 2

        # 2. PARALLEL LINES

        lines = draw_parallel_lines_with_angle(img, minmax, angle, robot_size, scale)
        b_lines = [bresenham(l[0][0], l[0][1], l[1][0], l[1][1]) for l in lines]

        # 3. SHAPES

        shape_coords = [connecting_the_points(n_coord)]
        insert_and_remove_one_coordinate_array(shape_coords)

        polygon = []
        is_merging = True
        is_checking = True

        while is_merging:
            merge_segments(shape_coords[0])
            shape = perfect_shapes(shape_coords[0])
            if len(shape) > 0:
                polygon.extend(shape)
            is_merging = checking_start_and_end_points1(shape_coords[0])

        if not is_merging:
            while is_checking:
                merge_segments(shape_coords[0])
                shape = perfect_shapes(shape_coords[0])
                if len(shape) > 0:
                    polygon.extend(shape)
                is_checking = checking_start_and_end_points2(shape_coords[0])

        biggest = max(polygon, key=len)
        shapes = [biggest] + [s for s in polygon if s is not biggest]

        # 4. INTERSECTION POINTS

        edges = edges_of_the_shapes(b_lines, shapes)
        inters = calculate_intersection_points(edges, b_lines, angle)

        deleting_bad_intersection_points(inters, n_coord)

        good_lines = selecting_good_lines(
            inters, b_lines, robot_size, n_obstacle, n_outside, n_coord
        )

        # 5. CUT + SECTIONS

        cut_lines = []
        for g in good_lines:
            if g:
                c = cut_start_and_end_points_bresenham_lines(g, shapes, robot_size, angle)
                if c:
                    cut_lines.append(c)

        squares = create_boundary_lines_on_the_sections(cut_lines, robot_size)
        b_s_lines = lines_convert_to_bresenham_lines(squares)

        ips = calculate_intersection_points_with_bresenham(shapes, b_s_lines, angle)

        cut_sections = []
        for cgbl, ip in zip(cut_lines, ips):
            if cgbl:
                cut_sections.append(
                    cut_bresenham_line_intersection_points(cgbl, ip, angle)
                )

        good_cut = [[] for _ in range(len(cut_sections))]
        for i, cs in enumerate(cut_sections):
            for c in cs:
                good_cut[i].append([c[0], c[-1]])

        final_sections = [sum(gcs, []) for gcs in good_cut if gcs]

        self.get_logger().info(f"Final sections: {len(final_sections)}")

        # 6. GRAPH + PATH 

        neighborhood = creating_nearborn_list(final_sections, shapes)

        first = next(iter(neighborhood))
        second = neighborhood[first][0][0]

        visited = []
        visited = connecting_sections(neighborhood, first, second, visited)

        visited_set = set(visited)

        two_path_equil = False
        actual_path = []

        while len(visited_set) != len(neighborhood) and not two_path_equil:

            missing = [k for k in neighborhood.keys() if k not in visited]

            previous_path = actual_path
            distances = {}

            for m in missing:
                dist, path = dijkstra(neighborhood, visited[-1], m)
                distances[dist] = path

            min_dist = min(distances.keys())
            actual_path = distances[min_dist]

            if actual_path == previous_path:
                self.get_logger().warn("No further connection between sections")
                visited = visited[:-len(previous_path)] if len(visited) >= len(previous_path) else []
                two_path_equil = True
                break

            else:
                for p in actual_path[1:]:
                    visited.append(p)

                point = visited[-1]

                if point in neighborhood:
                    pos = list(neighborhood.keys()).index(point)
                else:
                    self.get_logger().error("Point not in neighborhood!")
                    break

                first = list(neighborhood.keys())[pos]
                second = neighborhood[first][0][0]

                visited = connecting_sections(neighborhood, first, second, visited)
                visited_set = set(visited)

                self.get_logger().info(f"{len(visited_set)} / {len(neighborhood)} covered")

        self.get_logger().info(f"Visited points: {len(visited)}")

        # RESULT

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
    rclpy.spin(node)
    rclpy.shutdown()


