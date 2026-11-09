import numpy as np
import matplotlib.image as mpimg
from scipy import ndimage
import math

from .map_processing import rgb2gray, calculate_edges_coordinates, min_max_coord
from .calculation_of_parallel_lines import draw_parallel_lines_with_angle, bresenham
from .save_map_and_obstacle_boundaries import connecting_the_points, insert_and_remove_one_coordinate_array, segments_state, merge_segments, checking_start_and_end_points1, perfect_shapes, checking_start_and_end_points2, calculate_distance, SQRT_2, polygon_area, simplify_shape
from .calculation_of_intersection_points import edges_of_the_shapes, calculate_intersection_points, deleting_bad_intersection_points, selecting_good_lines
from .staggering_of_main_lines import cut_start_and_end_points_bresenham_lines, cut_bresenham_line_intersection_points
from .calculating_auxiliary_lines import creating_nearborn_list, connecting_sections, dijkstra

def plan_coverage(map_path, robot_size, threshold=0.55, angle_degrees=0, scale_factor=2):
    if robot_size <= 0:
        raise ValueError("Robot size must be greater than zero.")

    # 1. Load the map and identify free space
    img=mpimg.imread(map_path)
    gray_image = rgb2gray(img)
    binary_image = np.where(gray_image > threshold, 1, 0)
    labeled_edges, num_features = ndimage.label(binary_image)

    room_label = None
    largest_area = 0
    outside_labels = []

    for label in range(1, num_features + 1):
        pixel_count = np.count_nonzero(labeled_edges == label)

        touches_border = (
            np.any(labeled_edges[0, :] == label)
            or np.any(labeled_edges[-1, :] == label)
            or np.any(labeled_edges[:, 0] == label)
            or np.any(labeled_edges[:, -1] == label)
        )

        if touches_border:
            outside_labels.append(label)
        elif pixel_count > largest_area:
            room_label = label
            largest_area = pixel_count

    if room_label is None:
        raise ValueError("No enclosed white region found in the image.")

    free_space = labeled_edges == room_label
    coordinate_points = calculate_edges_coordinates(labeled_edges, 0)
    n_coordinate_points = np.array(coordinate_points, dtype=int).reshape(-1, 2)

    outside_of_map = []
    for label in outside_labels:
        points = calculate_edges_coordinates(labeled_edges, label)
        outside_of_map.extend(points)
    n_outside_of_map = np.array(outside_of_map, dtype=int).reshape(-1, 2)

    internal_coordinate_points_of_obstacle = []
    for label in range(1, num_features + 1):
        if label != room_label and label not in outside_labels:
            points = calculate_edges_coordinates(labeled_edges, label)
            internal_coordinate_points_of_obstacle.extend(points)
    n_internal_coordinate_points_of_obstacle = np.array(internal_coordinate_points_of_obstacle, dtype=int).reshape(-1, 2)

    min_max_x_y_coord = min_max_coord(coordinate_points)

    # 2. Calculate parallel lines
    lines = draw_parallel_lines_with_angle(img, min_max_x_y_coord, angle_degrees, robot_size, scale_factor)

    b_lines = []
    for start, end in lines:
        points = bresenham(start[0], start[1], end[0], end[1])
        b_lines.append(points)

    # 3. Save map and obstacle boundaries
    shape_coordinates = [connecting_the_points(n_coordinate_points)]
    insert_and_remove_one_coordinate_array(shape_coordinates)

    polygon = []
    is_merging = True
    is_checking = True
    seen_merging_states = set()

    while is_merging:
        state = segments_state(shape_coordinates[0])

        if state in seen_merging_states:
            is_merging = False
        else:
            seen_merging_states.add(state)

        if is_merging:
            merge_segments(shape_coordinates[0])
            closed_shapes = perfect_shapes(shape_coordinates[0])
            polygon.extend(closed_shapes)
            is_merging = checking_start_and_end_points1(shape_coordinates[0])

    seen_checking_states = set()
    while is_checking:
        state = segments_state(shape_coordinates[0])

        if state in seen_checking_states:
            break
        else:
            seen_checking_states.add(state)

        merge_segments(shape_coordinates[0])
        closed_shapes = perfect_shapes(shape_coordinates[0])
        polygon.extend(closed_shapes)
        is_checking = checking_start_and_end_points2(shape_coordinates[0])

    remaining = shape_coordinates[0]
    for r in range(len(remaining) - 1, -1, -1):
        if len(remaining[r]) != 1:
            continue
        point = remaining[r][0]
        candidates = []
        
        for polygon_index, poly in enumerate(polygon):
            
            for edge_index in range(len(poly) - 1): 
                distance1 = calculate_distance(point, poly[edge_index])
                distance2 = calculate_distance(point, poly[edge_index + 1])
                
                if (distance1 > 0 and distance1 <= SQRT_2) and (distance2 > 0 and distance2 <= SQRT_2):
                    candidates.append((polygon_index, edge_index))
        
        if len(candidates) == 1:
            polygon_index, edge_index = candidates[0]
            polygon[polygon_index] = np.insert(polygon[polygon_index], edge_index + 1, point, axis=0)
            del remaining[r]

    if len(remaining) != 0:
        raise ValueError(f"Could not construct closed polygons from {len(remaining)} remaining segments.")

    if len(polygon) == 0:
        raise ValueError("No closed polygon could be constructed from the map.")

    the_biggest_polygon = max(polygon, key=polygon_area)

    shapes = [the_biggest_polygon]
    for shape in polygon:

        if shape is not the_biggest_polygon:
            shapes.append(shape)

    simplified_shapes = simplify_shape(shapes)

    print("pontok")
    print(simplified_shapes[1])

     # 4. Calculation of intersection points
    edge_coordinates = edges_of_the_shapes(b_lines, simplified_shapes)
    intersection_points = calculate_intersection_points(edge_coordinates, b_lines, angle_degrees)
    deleting_bad_intersection_points(intersection_points, n_coordinate_points)
    good_b_lines = selecting_good_lines(intersection_points, b_lines, robot_size, n_internal_coordinate_points_of_obstacle, n_outside_of_map, n_coordinate_points)

    # 5. Trim and split main lines using the robot footprint
    cut_good_b_lines = []
    for line_group in good_b_lines:

        if len(line_group) == 0:
            continue
        
        trimmed_group = cut_start_and_end_points_bresenham_lines(line_group, robot_size, free_space)

        if len(trimmed_group) != 0:
            cut_good_b_lines.append(trimmed_group)

    cut_sections = []
    for line_group in cut_good_b_lines:
        free_sections = cut_bresenham_line_intersection_points(line_group, robot_size, free_space)
        cut_sections.append(free_sections)

    good_cut_sections = []

    for section_group in cut_sections:
        endpoints_group = []

        for section in section_group:
            points = [section[0], section[-1]]
            endpoints_group.append(points)

        good_cut_sections.append(endpoints_group)

    finally_good_cut_sections = []
    for g_cut_sections in good_cut_sections:

        if len(g_cut_sections) == 0:
            continue
        end_points = []

        for gcs in g_cut_sections:
           end_points.extend(gcs)

        finally_good_cut_sections.append(end_points)

    # 6. Calculating auxiliary lines
    neighborhood_list = creating_nearborn_list(finally_good_cut_sections, robot_size, free_space)
    required_sections = set()

    for f_good_cut_sections in finally_good_cut_sections:

        for fgcs in range(0, len(f_good_cut_sections) - 1, 2):
            start, end = tuple(f_good_cut_sections[fgcs]), tuple(f_good_cut_sections[fgcs + 1])
            required_sections.add(frozenset((start, end)))

    # 7. Build a route through all required sections
    covered_sections = set()

    if len(neighborhood_list) == 0:
        raise ValueError("No sections are available for route construction.")

    first_point = next(iter(neighborhood_list))

    if len(neighborhood_list[first_point]) == 0:
        raise ValueError("The starting point has no neighbors.")

    second_point = neighborhood_list[first_point][0][0]
    visited_points = []
    visited_points = connecting_sections(neighborhood_list, first_point, second_point, visited_points, covered_sections)

    while not required_sections.issubset(covered_sections):
        missing_sections = required_sections - covered_sections
        missing_endpoints = set()

        for m_sections in missing_sections:
            missing_endpoints.update(m_sections)

        candidates = []
        for target in neighborhood_list.keys():
            if target not in missing_endpoints:
                continue

            distance, path = dijkstra(neighborhood_list, visited_points[-1], target)      

            if math.isfinite(distance) and len(path) != 0:
                candidates.append((distance, path))  

        if len(candidates) == 0:
            raise ValueError("Some required sections are unreachable.")

        best_distance, actual_path = min(candidates, key=lambda item:item[0])
        covered_before = len(required_sections & covered_sections)

        for start, end in zip(actual_path, actual_path[1:]):
            section = frozenset((start, end))
            if section in required_sections:
                covered_sections.add(section)

        visited_points.extend(actual_path[1:])
        first_point = visited_points[-1]
        remaining_section = None
        sections = required_sections - covered_sections
        for s in sections:

            if first_point in s:
                remaining_section = s
                break
            
        if remaining_section is not None:    
            for r_section in remaining_section:
                if r_section != first_point:
                    second_point = r_section
                    break

            visited_points = connecting_sections(neighborhood_list, first_point, second_point, visited_points, covered_sections)

        covered_after = len(required_sections & covered_sections)
        if covered_after == covered_before:
            raise ValueError("Route construction made no progress.")

    # 8. Validate the completed route
    route_sections = set()
    invalid_steps = []
    for start, end in zip(visited_points, visited_points[1:]):
        start, end = tuple(start), tuple(end)
        has_connection = False
        for neighbor, distance in neighborhood_list.get(start, []):
            if neighbor == end:
                has_connection = True
                break

        if has_connection == False:
            invalid_steps.append((start, end))

        route_sections.add(frozenset((start, end)))

    missing_from_route = required_sections - route_sections

    if len(invalid_steps) != 0 or len(missing_from_route) != 0:
        raise ValueError("Route validation failed.")

    return visited_points