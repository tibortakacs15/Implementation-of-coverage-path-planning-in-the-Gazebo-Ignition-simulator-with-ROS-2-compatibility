import numpy as np
import math
import sys
import copy

# save_map_and_obstacle_boundaries.py


SQRT_2 = math.sqrt(2)

def deep_swap(array, index1, index2):
    temp = copy.deepcopy(array[index1]) 
    array[index1] = copy.deepcopy(array[index2])  
    array[index2] = temp  

# Calculate the Euclidean distance between two points
def calculate_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

def segments_state(segments):
    return tuple(
        tuple(tuple(point) for point in segment)
        for segment in segments
    )

# Group neighboring boundary pixels into segments, preferring horizontal and vertical neighbors
def connecting_the_points(edge_coordinates):
    segment = []
    
    # We go through all the points
    for ec in edge_coordinates:  
        found_segment = False
        potential_segment_index = None  # Here we store the index of the segment with a distance of 1.41421
        
        # The current point can be added to a segment
        for s in range(len(segment)):
            dist = calculate_distance(segment[s][-1], ec)
            
            # The distance is 1 the point is added to the segment
            if dist == 1:
                segment[s] = np.vstack([segment[s], ec])
                found_segment = True
                break
            
            # The distance close to 1.41421 is stored but not yet added
            elif 1 < dist < 1.42 and potential_segment_index is None:
                potential_segment_index = s
        
        # Did not find distance 1, but there was a distance of 1.4142, a point can be added to the segment
        if not found_segment and potential_segment_index is not None:
            segment[potential_segment_index] = np.vstack([segment[potential_segment_index], ec])
            found_segment = True

        # Did not find distance 1 or distance 1.4142, create a new segment
        if not found_segment:
            segment.append(np.array([ec]))

    return segment

# Attach isolated points to nearby segment endpoints
def insert_and_remove_one_coordinate_array(shape_coordinates):
    for shape_c in shape_coordinates:
        to_remove = []
        for sc in range(len(shape_c) - 1, -1, -1):
            if len(shape_c[sc]) == 1:
                for idx in range(len(shape_c)):
                    if idx == sc or idx in to_remove:
                        continue

                    dist1 = calculate_distance(shape_c[sc][0], shape_c[idx][0])
                    dist2 = calculate_distance(shape_c[sc][0], shape_c[idx][-1])
                    
                    if dist1 < 1.42:
                        shape_c[idx] = np.vstack([shape_c[sc], shape_c[idx]])
                        to_remove.append(sc)
                        break
                    elif dist2 < 1.42:
                        shape_c[idx] = np.vstack([shape_c[idx], shape_c[sc]])
                        to_remove.append(sc)
                        break

        for idx in sorted(to_remove, reverse=True):
            del shape_c[idx]

# Merge segments whose endpoints are close enough to connect
def merge_segments(shape_coordinates):
    while True:
        merged = False

        for idx1 in range(len(shape_coordinates) - 1):
            for idx2 in range(idx1 + 1, len(shape_coordinates)):
                start1 = shape_coordinates[idx1][0]
                end1 = shape_coordinates[idx1][-1]

                start2 = shape_coordinates[idx2][0]
                end2 = shape_coordinates[idx2][-1]

                if calculate_distance(end1, start2) <= SQRT_2:
                    shape_coordinates[idx1] = np.vstack([
                        shape_coordinates[idx1],
                        shape_coordinates[idx2]
                    ])
                    del shape_coordinates[idx2]
                    merged = True
                    break
                elif calculate_distance(start1, end2) <= SQRT_2:
                    shape_coordinates[idx1] = np.vstack([
                        shape_coordinates[idx2],
                        shape_coordinates[idx1]
                    ])
                    del shape_coordinates[idx2]
                    merged = True
                    break
                elif calculate_distance(start1, start2) <= SQRT_2:
                    shape_coordinates[idx1] = np.vstack([
                        shape_coordinates[idx2][::-1],
                        shape_coordinates[idx1]
                    ])
                    del shape_coordinates[idx2]
                    merged = True
                    break
                elif calculate_distance(end1, end2) <= SQRT_2:
                    shape_coordinates[idx1] = np.vstack([
                        shape_coordinates[idx1],
                        shape_coordinates[idx2][::-1]
                    ])
                    del shape_coordinates[idx2]
                    merged = True
                    break
            if merged:
                break

        if not merged:
            break

# Calculate the area enclosed by the polygon
def polygon_area(points):
    x = points[:, 0]
    y = points[:, 1]

    return abs(
        np.dot(x, np.roll(y, -1))
        - np.dot(y, np.roll(x, -1))
    ) / 2

# Close segments with nearby endpoints and extract the resulting polygons
def perfect_shapes(shape_coordinates):
    del_shapes = []
    shapes = []
    if len(shape_coordinates) > 0:
        for sc in range(len(shape_coordinates)):
            if len(shape_coordinates[sc]) < 3:
                continue
            if polygon_area(shape_coordinates[sc]) == 0:
                continue
            first_coord = shape_coordinates[sc][0]
            last_coord = shape_coordinates[sc][-1]
            dist = calculate_distance(first_coord, last_coord)
            if dist <= SQRT_2:
                if not np.array_equal(first_coord, last_coord):
                    shape_coordinates[sc] = np.vstack([
                        shape_coordinates[sc], first_coord
                    ])
                shapes.append(shape_coordinates[sc])
                del_shapes.append(sc)
        sort_del_shape = sorted(del_shapes, reverse=True)
        if len(sort_del_shape) > 0:  
                for ds_idx in range(len(sort_del_shape)):
                    del (shape_coordinates[sort_del_shape[ds_idx]])
    
    return shapes

# Checking start and end points in arrays
def checking_start_and_end_points1(shape_coordinates):
    if len(shape_coordinates) == 0:
        return False
    is_change = False
    for sc in range(len(shape_coordinates)):
        first_coord = shape_coordinates[sc][0]
        last_coord = shape_coordinates[sc][-1]
        dist = calculate_distance(first_coord, last_coord)
        if dist <= SQRT_2:
            continue
        else:
            sublist = shape_coordinates[sc]
            if len(sublist) > 6:
                dist1 = calculate_distance(sublist[0], sublist[1])
                dist2 = calculate_distance(sublist[1], sublist[2])
                dist3 = calculate_distance(sublist[0], sublist[2])
                dist4 = calculate_distance(sublist[0], sublist[3])
                
                if (dist1 == 1 and dist2 == 1 and dist3 == SQRT_2 and dist4 == 2):
                    deep_swap(sublist, 0, 1)
                    deep_swap(sublist, 0, 2)
                    is_change = True
                elif dist1 == 1 and dist2 == SQRT_2 and dist3 == 1:
                    deep_swap(sublist, 0, 1)
                    is_change = True
            
            if len(sublist) > 6:
                dist1 = calculate_distance(sublist[-1], sublist[-2])
                dist2 = calculate_distance(sublist[-2], sublist[-3])
                dist3 = calculate_distance(sublist[-1], sublist[-3])
                dist4 = calculate_distance(sublist[-1], sublist[-4])
                
                if dist1 == 1 and dist2 == 1 and dist3 == SQRT_2 and dist4 == 2:
                    deep_swap(sublist, -1,-2)
                    deep_swap(sublist, -1, -3)
                    is_change = True
                elif dist1 == 1 and dist2 == SQRT_2 and dist3 == 1:
                    deep_swap(sublist, -1, -2)
                    is_change = True                      
    
    return is_change

# Checking start and end points in arrays
def checking_start_and_end_points2(shape_coordinates):
    is_change = False

    for sublist in shape_coordinates:
        first_coord = sublist[0]
        last_coord = sublist[-1]

        if calculate_distance(first_coord, last_coord) <= SQRT_2:
            continue

        if len(sublist) <= 6:
            continue

        # Check the start of the segment
        dist1 = calculate_distance(sublist[0], sublist[1])
        dist2 = calculate_distance(sublist[1], sublist[2])
        dist3 = calculate_distance(sublist[0], sublist[2])

        if dist1 == 1 and dist2 == 1 and dist3 == SQRT_2:
            deep_swap(sublist, 0, 1)
            is_change = True

        # Check the end of the segment
        dist1 = calculate_distance(sublist[-1], sublist[-2])
        dist2 = calculate_distance(sublist[-2], sublist[-3])
        dist3 = calculate_distance(sublist[-1], sublist[-3])

        if dist1 == 1 and dist2 == 1 and dist3 == SQRT_2:
            deep_swap(sublist, -1, -2)
            is_change = True

    return is_change

def result_detection(shape_coordinates, shapes):
    is_empty = len(shape_coordinates) == 0
    ok_shape = True
    for s in shapes:
        if not np.array_equal(s[0], s[-1]):
            ok_shape = False

    if is_empty and ok_shape:
       print('All shapes were detected!')
    else:
        print('Not all shapes were detected!')
        sys.exit("Exiting program: shape detection failed.")

# Remove redundant points along straight edges while keeping each polygon closed
def simplify_shape(shapes):
    simplified_shapes = []

    for shape in shapes:
        points = shape[:-1]
        simplified = []

        for i in range(len(points)):
            previous = points[i - 1]
            current = points[i]
            following = points[(i + 1) % len(points)]

            incoming = current - previous
            outgoing = following - current

            cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]

            if cross != 0 or dot <= 0:
                simplified.append(current)

        if len(simplified) < 3:
            raise ValueError(
                "Simplification produced fewer than three vertices."
            )

        simplified.append(simplified[0].copy())
        simplified_shapes.append(np.array(simplified))

    return simplified_shapes