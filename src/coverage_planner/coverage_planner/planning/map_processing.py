import numpy as np

# map_processing.py

# Convert RGB to grayscale manually
def rgb2gray(rgb):
    if rgb.ndim == 2:
        return rgb
    else:
        return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

# Calculate edges coordinates points
def calculate_edges_coordinates(labeled_edges, label):
    coordinate_points = []
    coord = np.where(labeled_edges == label)

    for row, column in zip(coord[0], coord[1]):
        coordinate_points.append([column, row])

    return coordinate_points

# Calculating minimum and maximum points by coordinates points 
def min_max_coord(coordinate_points):
    if len(coordinate_points) == 0:
        raise ValueError("Cannot calculate bounds: the coordinate list is empty.")
    x_coord = []
    y_coord = []
    for cp in coordinate_points:
        x_coord.append(cp[0])
        y_coord.append(cp[1])
    min_x_coord = min(x_coord)
    max_x_coord = max(x_coord)
    min_y_coord = min(y_coord)
    max_y_coord = max(y_coord)
    min_max_x_y = [min_x_coord, min_y_coord, max_x_coord, max_y_coord]
    return min_max_x_y