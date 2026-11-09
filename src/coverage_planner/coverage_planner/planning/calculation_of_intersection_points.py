import numpy as np
import matplotlib.pyplot as plt
import math

from .calculation_of_parallel_lines import bresenham
from .save_map_and_obstacle_boundaries import calculate_distance

# calculation_of_intersection_points.py

# Collect boundary edges whose Y range overlaps each sweep line's Y range
def edges_of_the_shapes(lines, shapes):
    edge_coordinates = []

    for line in lines:
        y_min = min(line[0][1], line[-1][1])
        y_max = max(line[0][1], line[-1][1])
        edges = []

        for shape in shapes:
            for i in range(len(shape) - 1):
                start = shape[i]
                end = shape[i + 1]

                edge_y_min = min(start[1], end[1])
                edge_y_max = max(start[1], end[1])

                if edge_y_max >= y_min and edge_y_min <= y_max:
                    edges.append(np.array([start, end]))

        edge_coordinates.append(edges)

    return edge_coordinates

# Find where the sweep lines intersect the boundary edges
def calculate_intersection_points(edge_coordinates, lines, angle_degrees):
    angle_degrees = angle_degrees % 360
    intersection_points = [set() for _ in range(len(lines))]
    for count, (edge_c, line) in enumerate(zip(edge_coordinates, lines)):
        if len(edge_c) == 0 or len(line) < 2:
            continue
        for l in range(len(line) - 1):
            for ec in edge_c:
                if len(ec) < 2:
                    continue
                for e in range(len(ec) - 1):
                    ip = find_intersection_of_segments(ec[e], ec[e + 1], line[l], line[l + 1])
                    if ip is not None:
                        if len(ip) == 2:  # One piece intersection points
                            if angle_degrees >= 0 and angle_degrees < 90 or angle_degrees >= 180 and angle_degrees < 270:
                                r_ip = (math.floor(ip[0]), math.floor(ip[1]))
                            else:
                                r_ip = (math.ceil(ip[0]), math.floor(ip[1]))
                            intersection_points[count].add(r_ip)  # Add to set
                        elif len(ip) == 4:
                            for i in range(0, len(ip), 2):
                                x = ip[i]
                                y = ip[i + 1]

                                if (
                                    0 <= angle_degrees < 90
                                    or 180 <= angle_degrees < 270
                                ):
                                    rounded_point = (math.floor(x), math.floor(y))
                                else:
                                    rounded_point = (math.ceil(x), math.floor(y))

                                intersection_points[count].add(rounded_point)
    # set to list conversion
    intersection_points = [list(ips) for ips in intersection_points]
    if angle_degrees >= 0 and angle_degrees < 90:
        sorted_intersection_points = [sorted(row, key=lambda point: (point[0], -point[1])) for row in intersection_points]
    elif angle_degrees >= 90 and angle_degrees < 180:
        sorted_intersection_points = [sorted(row, key=lambda point: (-point[0], -point[1])) for row in intersection_points]
    elif angle_degrees >= 180 and angle_degrees < 270:
        sorted_intersection_points = [sorted(row, key=lambda point: (-point[0], point[1])) for row in intersection_points]
    elif angle_degrees >= 270 and angle_degrees < 360:
        sorted_intersection_points = [sorted(row, key=lambda point: (point[0], point[1])) for row in intersection_points]
    return sorted_intersection_points

# Calculate intersection points between two segments       
def find_intersection_of_segments(segment1_f, segment1_e, segment2_f, segment2_e):
    # Start and end points of the first segment
    x1, y1 = segment1_f
    x2, y2 = segment1_e
    
    # Start and end points of the second segment
    x3, y3 = segment2_f
    x4, y4 = segment2_e
    
    # Direction vectors of the first and second segments
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    # Squared segment lengths
    length1_squared = dx1 * dx1 + dy1 * dy1
    length2_squared = dx2 * dx2 + dy2 * dy2

    epsilon = 1e-6  # Adjusted tolerance limit
    if length1_squared == 0 and length2_squared == 0:
        distance = math.hypot(x3 - x1, y3 - y1)

        if distance <= epsilon:
            return (x1, y1)

        return None

    if length1_squared == 0:
        parameter = (
            (x1 - x3) * dx2 + (y1 - y3) * dy2
        ) / length2_squared

        parameter = max(0.0, min(1.0, parameter))

        closest_x = x3 + parameter * dx2
        closest_y = y3 + parameter * dy2

        distance = math.hypot(x1 - closest_x, y1 - closest_y)

        if distance <= epsilon:
            return (x1, y1)

        return None

    if length2_squared == 0:
        parameter = (
            (x3 - x1) * dx1 + (y3 - y1) * dy1
        ) / length1_squared

        parameter = max(0.0, min(1.0, parameter))

        closest_x = x1 + parameter * dx1
        closest_y = y1 + parameter * dy1

        distance = math.hypot(x3 - closest_x, y3 - closest_y)

        if distance <= epsilon:
            return (x3, y3)

        return None
    # Solving a system of linear equations according to Cramer's rule
    denominator = dx1 * dy2 - dy1 * dx2  # Determinant

    if abs(denominator) < epsilon:
        # Check whether both segments lie on the same line
        cross = (x3 - x1) * dy1 - (y3 - y1) * dx1

        if abs(cross) > epsilon * math.sqrt(length1_squared):
            return None

        # Positions of the second segment's endpoints along the first segment
        t3 = (
            (x3 - x1) * dx1 + (y3 - y1) * dy1
        ) / length1_squared

        t4 = (
            (x4 - x1) * dx1 + (y4 - y1) * dy1
        ) / length1_squared

        # Intersection with the first segment's [0, 1] interval
        overlap_start = max(0.0, min(t3, t4))
        overlap_end = min(1.0, max(t3, t4))

        if overlap_start > overlap_end:
            return None

        start_x = x1 + overlap_start * dx1
        start_y = y1 + overlap_start * dy1
        end_x = x1 + overlap_end * dx1
        end_y = y1 + overlap_end * dy1

        if math.hypot(end_x - start_x, end_y - start_y) <= epsilon:
            return (start_x, start_y)

        return (start_x, start_y, end_x, end_y)
    
    # Use Cramer's rule to find the intersection parameters
    numerator_t = (x3 - x1) * dy2 - (y3 - y1) * dx2
    numerator_u = (x3 - x1) * dy1 - (y3 - y1) * dx1
    
    t = numerator_t / denominator
    u = numerator_u / denominator

    # Check if the intersection point is within both segments (0 <= t, u <= 1) with adjusted epsilon  
    if -epsilon <= t <= 1 + epsilon and -epsilon <= u <= 1 + epsilon:
        # The coordinate of the point of intersection
        intersection_x = x1 + t * dx1
        intersection_y = y1 + t * dy1
        return (intersection_x, intersection_y)
    else:
        return None  # There is no intersection between the two segments

    

#Deleting bad intersection points when not in coordinate points
def deleting_bad_intersection_points(intersection_points, n_coordinate_points):
    del_ip = []
    for ip in range(len(intersection_points)):
        for idx in range(len(intersection_points[ip])):
            in_coordinate_points = np.where((n_coordinate_points == intersection_points[ip][idx]).all(axis=1))[0]
            if in_coordinate_points.size == 0:
                del_ip.append([ip, idx])

    for row, idx in sorted(del_ip, reverse=True):
        del intersection_points[row][idx]

#Searching good lines between intersection points
def selecting_good_lines(intersection_points, b_lines, robot_size, n_internal_coordinate_points_of_obstacle, n_outside_of_map, n_coordinate_points):
    finally_b_lines = [[] for _ in range(len(b_lines))]
    for count, ip in enumerate(intersection_points):
        if len(ip) == 0:
            continue
        for i in range(len(ip) - 1):
            dist = calculate_distance(ip[i], ip[i + 1]) 
            if dist > robot_size:
                line = bresenham(ip[i][0], ip[i][1], ip[i + 1][0], ip[i + 1][1])
                n_line = np.array(line)
                if n_line.size > 0:
                    line_is_good = True
                    for  l in range(len(n_line)):
                        in_obstacle = np.where((n_internal_coordinate_points_of_obstacle == n_line[l]).all(axis=1))[0]
                        in_outside = np.where((n_outside_of_map == n_line[l]).all(axis=1))[0]
                        if in_obstacle.size != 0 or in_outside.size != 0:
                            line_is_good = False
                            break
                    if line_is_good:
                        # Intersection endpoints may lie on black boundary pixels
                        # Interior points must not cross a black boundary
                        for point in n_line[1:-1]:
                            on_boundary = np.any(
                                np.all(n_coordinate_points == point, axis=1)
                            )

                            if on_boundary:
                                line_is_good = False
                                break

                        if line_is_good:
                            finally_b_lines[count].append(n_line)
    return finally_b_lines

# Draw lines in map
def draw_lines(points, color):
    for pt in points:
        if pt == []:
            continue
        else:
            for p in range(0,len(pt) - 1, 2):
                plt.plot((pt[p][0], pt[p + 1][0]), (pt[p][1], pt[p + 1][1]), color=color, linewidth=1)

# Return the maximum nesting depth of the list
def get_list_depth(lst, depth=0):
    if isinstance(lst, list):
        return max(get_list_depth(item, depth + 1) for item in lst) if lst else depth
    
    return depth
