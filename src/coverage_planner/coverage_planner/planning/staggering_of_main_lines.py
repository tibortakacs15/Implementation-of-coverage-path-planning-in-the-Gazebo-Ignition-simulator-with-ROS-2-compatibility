import math
import numpy as np

# staggering_of_main_lines.py

# Check that the entire footprint is within the map and all its cells are free
def square_is_free(x_min, y_min, x_max, y_max, free_space):
    height, width = free_space.shape

    if (
        x_min < 0
        or y_min < 0
        or x_max >= width
        or y_max >= height
    ):
        return False

    area = free_space[y_min:y_max + 1, x_min:x_max + 1]
    return bool(np.all(area))

# Trim both ends of each line until the robot fits at both endpoints
def cut_start_and_end_points_bresenham_lines(
    b_lines, robot_size, free_space
):
    if robot_size <= 0:
        raise ValueError("Robot size must be greater than zero.")

    half_size = robot_size / 2
    cut_lines = []

    for b_line in b_lines:
        if len(b_line) == 0:
            continue

        # Find the first point where the robot fits
        start_idx = int(half_size) + 1
        is_blocked = True

        while is_blocked and start_idx < len(b_line):
            x, y = b_line[start_idx]

            is_blocked = not square_is_free(
                math.floor(x - half_size),
                math.floor(y - half_size),
                math.ceil(x + half_size),
                math.ceil(y + half_size),
                free_space
            )

            if is_blocked:
                start_idx += 1

        # Find the last point where the robot fits
        end_idx = len(b_line) - int(half_size) - 1
        is_blocked = True

        while is_blocked and end_idx >= start_idx:
            x, y = b_line[end_idx]

            is_blocked = not square_is_free(
                math.floor(x - half_size),
                math.floor(y - half_size),
                math.ceil(x + half_size),
                math.ceil(y + half_size),
                free_space
            )

            if is_blocked:
                end_idx -= 1

        if is_blocked or end_idx <= start_idx:
            continue

        trimmed_points = b_line[start_idx:end_idx + 1]

        if len(trimmed_points) > 1:
            cut_lines.append(trimmed_points)

    return cut_lines

# Split each line into continuous sections where the robot fits at every point
def cut_bresenham_line_intersection_points(
    b_lines, robot_size, free_space
):
    if robot_size <= 0:
        raise ValueError("Robot size must be greater than zero.")

    half_size = robot_size / 2
    new_lines = []

    for b_line in b_lines:
        current_section = []

        for point in b_line:
            x, y = point

            is_free = square_is_free(
                math.floor(x - half_size),
                math.floor(y - half_size),
                math.ceil(x + half_size),
                math.ceil(y + half_size),
                free_space
            )

            if is_free:
                current_section.append(point)
            else:
                if len(current_section) > 1:
                    new_lines.append(np.array(current_section))

                current_section = []

        if len(current_section) > 1:
            new_lines.append(np.array(current_section))

    return new_lines