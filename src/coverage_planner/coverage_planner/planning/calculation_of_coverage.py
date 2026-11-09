import numpy as np
import math
from scipy.ndimage import binary_dilation
from matplotlib.path import Path
import matplotlib.pyplot as plt

from coverage_planner.planning.calculation_of_intersection_points import calculate_distance

# calculate_of_coverage.py

def calculating_route_distance(visited_points):
    route_distance = 0.0

    for i in range(len(visited_points) - 1):
        route_distance += calculate_distance(
            visited_points[i],
            visited_points[i + 1]
        )

    return route_distance


# Grid size (based on min-max boundaries, square grid)
def calculate_coverage(shapes, visited_points, robot_size, save_path=None):

    # --- MAP BOUNDS ---
    outer_polygon = shapes[0]

    xmin, ymin = outer_polygon.min(axis=0)
    xmax, ymax = outer_polygon.max(axis=0)

    grid_width = int(xmax - xmin + 1)
    grid_height = int(ymax - ymin + 1)

    # Fix the size of the grid based on the map so that it does not change according to the route
    grid_size = (grid_height, grid_width)

     # Initialisation of the grid
    original_grid = np.zeros(grid_size, dtype=np.uint8)

    # Initialising the paths of external and internal polygons
    outer_path = Path(outer_polygon)
    inner_paths = [Path(inner) for inner in shapes[1:]]

    
    for y in range(grid_height):
        for x in range(grid_width):

            real_x = x + xmin
            real_y = y + ymin
            point = (real_x, real_y)

            if not outer_path.contains_point(point, radius=-1e-9):
                continue

            is_inner = any(inner.contains_point(point, radius=-1e-9) for inner in inner_paths)

            is_on_edge = any(
                any(
                    math.isclose(px, real_x, abs_tol=0.5) and
                    math.isclose(py, real_y, abs_tol=0.5)
                    for (px, py) in shape
                )
                for shape in shapes
            )

            if not is_inner and not is_on_edge:
                original_grid[y, x] = 1

    # Tracking the robot's path
    grid = original_grid.copy()

    
    for i in range(len(visited_points) - 1):

        x1, y1 = visited_points[i]
        x2, y2 = visited_points[i + 1]

        steps = int(max(abs(x2 - x1), abs(y2 - y1)))

        for step in range(steps + 1):

            if steps == 0:
                real_x, real_y = int(x1), int(y1)
            else:
                real_x = int(round(x1 + step * (x2 - x1) / steps))
                real_y = int(round(y1 + step * (y2 - y1) / steps))

            gx = real_x - xmin
            gy = real_y - ymin

            if 0 <= gx < grid_width and 0 <= gy < grid_height:
                if grid[gy, gx] == 1:
                    grid[gy, gx] = 2

    # Robot size taking into account dilatation
    structure = np.ones((robot_size, robot_size))
    dilated = binary_dilation(grid == 2, structure=structure)

    # Calculating the area covered
    covered_area = np.sum(dilated & (original_grid == 1))
    total_area = np.sum(original_grid == 1)

    if total_area == 0:
        return 0.0

    coverage_percentage = (covered_area / total_area) * 100.0

    # Graphical display
    if save_path is not None:

        plt.figure(figsize=(6, 6))

        plt.imshow(original_grid, cmap="gray_r", origin="upper")
        plt.imshow(dilated, cmap="Blues", alpha=0.5, origin="upper")

        px = [p[0] - xmin for p in visited_points]
        py = [p[1] - ymin for p in visited_points]

        plt.plot(px, py, 'r-', linewidth=1.5)

        plt.scatter(
            outer_polygon[:, 0] - xmin,
            outer_polygon[:, 1] - ymin,
            color="black",
            s=5
        )

        for inner in shapes[1:]:
            plt.scatter(
                inner[:, 0] - xmin,
                inner[:, 1] - ymin,
                color="blue",
                s=5
            )

        plt.title(f"Coverage: {coverage_percentage:.2f}%")

        plt.savefig(save_path)
        plt.close()

    return coverage_percentage

