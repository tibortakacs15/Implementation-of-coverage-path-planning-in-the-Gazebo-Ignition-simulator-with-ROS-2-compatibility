#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import time
import os

from scipy import ndimage

# IMPORT YOUR MODULES
from coverage_planner.planning.map_processing import (
    rgb2gray,
    calculate_edges_coordinates,
    min_max_coord
)
from coverage_planner.planning.calculation_of_parallel_lines import (
    draw_parallel_lines_with_angle,
    bresenham
)
from coverage_planner.planning.calculation_of_intersection_points import (
    calculate_distance,
    
)
from coverage_planner.planning.save_map_and_obstacle_boundaries import (
    connecting_the_points,
    insert_and_remove_one_coordinate_array,
    merge_segments,
    perfect_shapes,
    checking_start_and_end_points1,
    result_detection
)
from coverage_planner.planning.calculation_of_intersection_points import (
    edges_of_the_shapes,
    calculate_intersection_points,
    deleting_bad_intersection_points,
    selecting_good_lines,
    find_intersection_of_segments
)
from coverage_planner.planning.calculating_auxiliary_lines import (
    creating_nearborn_list,
    connecting_sections,
    dijkstra
)
from coverage_planner.planning.staggering_of_main_lines import (
    cut_start_and_end_points_bresenham_lines,
    create_boundary_lines_on_the_sections,
    lines_convert_to_bresenham_lines,
    calculate_intersection_points_with_bresenham,
    cut_bresenham_line_intersection_points
)
#from my_first_ros.calculation_of_coverage import 

from std_msgs.msg import Float32MultiArray


class CoverageNode(Node):

    def __init__(self):
        super().__init__('coverage_node')

        self.get_logger().info("Coverage node started")

        # Publisher 
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'coverage_path',
            10
        )

        # Parameters
        self.declare_parameter('map_path', 'Living_room.png')
        self.declare_parameter('robot_size', 15)
        self.declare_parameter('threshold', 0.55)

        self.run_pipeline()


    def run_pipeline(self):

        start_time = time.perf_counter()

        filename = self.get_parameter('map_path').get_parameter_value().string_value
        robot_size = self.get_parameter('robot_size').get_parameter_value().integer_value
        threshold_value = self.get_parameter('threshold').get_parameter_value().double_value

        self.get_logger().info(f"Loading map: {filename}")

        try:
            #img = mpimg.imread(filename) ez helyett
            from ament_index_python.packages import get_package_share_directory

            package_share_dir = get_package_share_directory('my_first_ros')
            img_path = os.path.join(package_share_dir, 'Maps', filename)

            img = mpimg.imread(img_path)
        except:
            self.get_logger().error("Map file not found!")
            return

        gray_image = rgb2gray(img)

        binary_image = np.where(gray_image > threshold_value, 1, 0)
        n_binary_image = np.array(binary_image)

        labeled_edges, num_features = ndimage.label(n_binary_image)

        coordinate_points = calculate_edges_coordinates(labeled_edges, 0)
        outside_of_map = calculate_edges_coordinates(labeled_edges, 1)
        internal_coordinate_points = calculate_edges_coordinates(labeled_edges, 2)
        internal_coordinate_points_of_obstacle = calculate_edges_coordinates(labeled_edges, num_features)

        n_coordinate_points = np.array(coordinate_points)
        n_outside_of_map = np.array(outside_of_map)
        n_internal_coordinate_points = np.array(internal_coordinate_points)
        n_internal_coordinate_points_of_obstacle = np.array(internal_coordinate_points_of_obstacle)

        min_max_x_y_coord = min_max_coord(coordinate_points)

        angle_degrees = 1
        scale_factor = 2

        # PARALLEL LINES
        lines = draw_parallel_lines_with_angle(
            img, min_max_x_y_coord, angle_degrees, robot_size, scale_factor
        )

        b_lines = []
        for l in lines:
            b_lines.append(bresenham(l[0][0], l[0][1], l[1][0], l[1][1]))

        # SHAPES
        shape_coordinates = []
        shape_coordinates.append(connecting_the_points(n_coordinate_points))

        insert_and_remove_one_coordinate_array(shape_coordinates)

        polygon = []
        is_merging = True

        while is_merging:
            merge_segments(shape_coordinates[0])
            shape = perfect_shapes(shape_coordinates[0])
            if len(shape) > 0:
                polygon.extend(shape)
            is_merging = checking_start_and_end_points1(shape_coordinates[0])

        the_biggest_polygon = max(polygon, key=len)
        shapes = [the_biggest_polygon] + [s for s in polygon if s is not the_biggest_polygon]

        result_detection(shape_coordinates[0], shapes)

        # INTERSECTION POINTS
        edge_coordinates = edges_of_the_shapes(b_lines, shapes)

        intersection_points = calculate_intersection_points(
            edge_coordinates, b_lines, angle_degrees
        )
        
        deleting_bad_intersection_points(intersection_points, n_coordinate_points)

        good_b_lines = selecting_good_lines(
            intersection_points,
            b_lines,
            robot_size,
            n_internal_coordinate_points_of_obstacle,
            n_outside_of_map,
            n_coordinate_points
        )
        
        # CUT SECTIONS
        cut_good_b_lines = []
        for g_b_lines in good_b_lines:
            if len(g_b_lines) == 0:
                continue
            c_g_b_line = cut_start_and_end_points_bresenham_lines(g_b_lines, shapes, robot_size, angle_degrees)
            if c_g_b_line:
                cut_good_b_lines.append(c_g_b_line)

        self.get_logger().info("Finally_good_cut_sections:")
        
        square_lines_on_the_sections = create_boundary_lines_on_the_sections(cut_good_b_lines, robot_size)

        b_s_lines_on_the_sections = lines_convert_to_bresenham_lines(square_lines_on_the_sections)

        intersection_points_on_the_sections = calculate_intersection_points_with_bresenham(shapes, b_s_lines_on_the_sections, angle_degrees)

        cut_sections = []
        for  cgbl, ips in zip(cut_good_b_lines, intersection_points_on_the_sections):
            if len(cgbl) == 0:
                continue
            cut_section =  cut_bresenham_line_intersection_points(cgbl, ips, angle_degrees)
            cut_sections.append(cut_section)

        good_cut_sections = [[] for _ in range(len(cut_sections))]
        for count, c_sections in enumerate(cut_sections):
            for cs in c_sections:
                good_cut_sections[count].append([cs[0], cs[-1]])  

        # Merges internal lists
        finally_good_cut_sections = [sum(gcs, []) for gcs in good_cut_sections if gcs]

        # Auxiliary path
        neighborhood_list = creating_nearborn_list(finally_good_cut_sections, shapes)
        neighborhood_list = creating_nearborn_list(finally_good_cut_sections, shapes)

        first_point = next(iter(neighborhood_list))
        second_point = neighborhood_list[first_point][0][0]

        if second_point in neighborhood_list and neighborhood_list[second_point]:
            closest_point = min(neighborhood_list[second_point], key=lambda x: x[1])  # Select the smallest distance
            second_point = neighborhood_list[closest_point[0]][0][0]

        # Create a section
        first_point = next(iter(neighborhood_list))
        second_point = neighborhood_list[first_point][0][0]

        visited_points = []
        visited_points = connecting_sections(neighborhood_list, first_point, second_point, visited_points)

        visited_points_set = set(visited_points)

        two_path_equil = False
        actual_path = []
        while len(visited_points_set) != len(neighborhood_list) and not(two_path_equil):
            dict_keys = list(neighborhood_list.keys())
            missing_in_list = [key for key in dict_keys if key not in visited_points]

            previous_path = actual_path
            distances = {}
            for ml in missing_in_list:
                distance, path = dijkstra(neighborhood_list, visited_points[-1], ml)
                distances[distance] = path

            min_distance = min(distances.keys())
            actual_path = distances[min_distance]

            if actual_path == previous_path:
                visited_points = visited_points[:-len(previous_path)] if len(visited_points) >= len(previous_path) else []
                print('No further connection between sections')
                two_path_equil = True
            else:
                for md in range(1, len(distances[min_distance])):
                    visited_points.append(distances[min_distance][md])

                point = visited_points[-1]
                if point in neighborhood_list:
                    pozicio = list(neighborhood_list.keys()).index(point)
                else:
                    print("There is no such point the neighborhood list.")
            
                first_point = list(neighborhood_list.keys())[pozicio]
                second_point = neighborhood_list[first_point][0][0]

                visited_points = connecting_sections(neighborhood_list, first_point, second_point, visited_points)

                visited_points_set = set(visited_points)

                print(len(neighborhood_list), len(visited_points_set)) # Checking that we have covered all points

        self.get_logger().info("VISITED POINTS:")

        for i, p in enumerate(visited_points):
            self.get_logger().info(f"{i}: {p}")
        
        # PUBLISH PATH
        msg = Float32MultiArray()

        flat_path = []
        for p in visited_points:
            flat_path.extend([float(p[0]), float(p[1])])

        msg.data = flat_path
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published path with {len(visited_points)} points")

        # VISUALIZATION
        plt.imshow(img)
        xs = [p[0] for p in visited_points]
        ys = [p[1] for p in visited_points]
        plt.plot(xs, ys, 'r-')
        plt.title("Coverage Path")
        save_path = os.path.expanduser("~/ros2_ws/src/my_first_ros/my_first_ros/coverage_result.png")
        plt.savefig(save_path)
        self.get_logger().info(f"Saved visualization to {save_path}")
        self.get_logger().info("Saved visualization to /tmp/coverage_result.png")

        end_time = time.perf_counter()
        self.get_logger().info(f"Done in {end_time - start_time:.2f} sec")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()