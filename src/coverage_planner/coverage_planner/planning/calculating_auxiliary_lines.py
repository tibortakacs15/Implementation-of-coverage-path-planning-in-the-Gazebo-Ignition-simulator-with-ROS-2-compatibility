import math

from .calculation_of_parallel_lines import bresenham
from .staggering_of_main_lines import square_is_free
from .calculation_of_intersection_points import calculate_distance

# calculate_auxiliary_lines.py

def connection_is_free(start, end, robot_size, free_space):
    if robot_size <= 0:
        raise ValueError("Robot size must be greater than zero.")

    half_size = robot_size / 2

    points = bresenham(
        start[0], start[1],
        end[0], end[1]
    )

    for x, y in points:
        is_free = square_is_free(
            math.floor(x - half_size),
            math.floor(y - half_size),
            math.ceil(x + half_size),
            math.ceil(y + half_size),
            free_space
        )

        if not is_free:
            return False

    return True

# Build a list of collision-free connections
def creating_nearborn_list(intersection_points, robot_size, free_space):
    neighborhood_list = {}

    # Collect all endpoints without duplicates
    all_points = list(dict.fromkeys(
        tuple(point)
        for row in intersection_points
        for point in row
    ))

    for current_row in intersection_points:
        for i in range(0, len(current_row) - 1, 2):
            key1 = tuple(current_row[i])
            key2 = tuple(current_row[i + 1])

            if not connection_is_free(
                key1, key2, robot_size, free_space
            ):
                raise ValueError(
                    f"Main section is not free: {key1} -> {key2}"
                )

            distance = calculate_distance(key1, key2)

            # Put the other endpoint of the same segment first
            neighbors_key1 = [(key2, distance)]
            neighbors_key2 = [(key1, distance)]

            for target in all_points:
                if target == key1 or target == key2:
                    continue

                if connection_is_free(
                    key1, target, robot_size, free_space
                ):
                    neighbors_key1.append((
                        target,
                        calculate_distance(key1, target)
                    ))

                if connection_is_free(
                    key2, target, robot_size, free_space
                ):
                    neighbors_key2.append((
                        target,
                        calculate_distance(key2, target)
                    ))

            neighborhood_list.setdefault(key1, []).extend(
                neighbors_key1
            )
            neighborhood_list.setdefault(key2, []).extend(
                neighbors_key2
            )

    return neighborhood_list

# Connecting sections by nearest point
def connecting_sections(neighborhood_list, first_point, second_point, visited_points, covered_sections):
    if not visited_points or visited_points[-1] != first_point:
        visited_points.append(first_point)

    if visited_points[-1] != second_point:
        visited_points.append(second_point)

    covered_sections.add(frozenset((first_point, second_point)))

    while (second_point in neighborhood_list and neighborhood_list[second_point]):
        # Collect reachable endpoints of uncovered segments
        candidates = []

        for target, distance in neighborhood_list[second_point]:
            if not neighborhood_list.get(target):
                continue

            partner = neighborhood_list[target][0][0]
            section = frozenset((target, partner))

            if section not in covered_sections:
                candidates.append((target, distance))

        closest_point = min(
            candidates,
            key=lambda item: item[1],
            default=None
        )
        
        # Stop if no uncovered segment is directly reachable
        if closest_point is None:
            print("No directly reachable unvisited sections.")
            break

        first_point = closest_point[0] 
        visited_points.append(first_point)  # Add the visited_points
        
        second_point = neighborhood_list[first_point][0][0]
        visited_points.append(second_point)

        covered_sections.add(frozenset((first_point, second_point)))
       
    
    return visited_points

# Move an item up to restore the min-heap property
def heapify_up(heap, index):
    parent = (index - 1) // 2
    while index > 0 and heap[index][0] < heap[parent][0]:
        heap[index], heap[parent] = heap[parent], heap[index]
        index = parent
        parent = (index - 1) // 2

# Move an item down to restore the min-heap property
def heapify_down(heap, index):
    smallest = index
    left = 2 * index + 1
    right = 2 * index + 2

    if left < len(heap) and heap[left][0] < heap[smallest][0]:
        smallest = left
    if right < len(heap) and heap[right][0] < heap[smallest][0]:
        smallest = right
    if smallest != index:
        heap[index], heap[smallest] = heap[smallest], heap[index]
        heapify_down(heap, smallest)

# Insert item
def push_heap(heap, item):
    heap.append(item)
    heapify_up(heap, len(heap) - 1)

# Remove item
def pop_heap(heap):
    if len(heap) == 1:
        return heap.pop()
    root = heap[0]
    heap[0] = heap.pop()
    heapify_down(heap, 0)
    return root

# Reconstruct the shortest path from the predecessor links
def dijkstra(neighborhood_list, start, end):
    if start not in neighborhood_list or end not in neighborhood_list:
        return float("inf"), []
    
    pq = []  
    distances = {node: float('inf') for node in neighborhood_list}
    previous_nodes = {node: None for node in neighborhood_list}
    distances[start] = 0
    push_heap(pq, (0, start))

    while pq:
        curr_dist, curr_node = pop_heap(pq)

        if curr_node == end:
            break

        if curr_dist > distances[curr_node]:
            continue

        for neighbor, weight in neighborhood_list[curr_node]:
            distance = curr_dist + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = curr_node
                push_heap(pq, (distance, neighbor))
    if distances[end] == float("inf"):
        return float("inf"), []
    
    # Decrypt route
    path = []
    node = end
    while node is not None:
        path.append(node)
        node = previous_nodes[node]
    path.reverse()

    return distances[end], path
