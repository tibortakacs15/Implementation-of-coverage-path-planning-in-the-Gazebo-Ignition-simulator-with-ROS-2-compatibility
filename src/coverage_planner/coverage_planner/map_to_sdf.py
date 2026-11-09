import numpy as np
from scipy import ndimage
from pathlib import Path

from PIL import Image

# Group consecutive occupied columns into horizontal runs
def find_runs(occupied_columns):
    if not occupied_columns:
        return []
    section = [occupied_columns[0]]
    sections = []
    previous_column = occupied_columns[0]
    for index in range(1, len(occupied_columns)):
        current_column = occupied_columns[index]
        if current_column - previous_column != 1:
            section.append(previous_column)
            sections.append(section)
            section = [current_column]
        previous_column = current_column
    section.append(previous_column)
    sections.append(section)
    return sections

# Generate and save a Gazebo SDF world from a map image
def generate_world(map_path, output_path, resolution):

    gray_image = Image.open(map_path).convert("L")

    width, height = gray_image.size

    pixels = np.array(gray_image)
    bright_mask = pixels > 0.55 * 255

    labels, num_regions = ndimage.label(bright_mask)


    outside_labels = set()

    room_label = None
    largest_area = 0

    # Find the largest bright region that does not touch the image border
    for label in range(1, num_regions + 1):

        touches_border = (
            np.any(labels[0, :] == label)
            or np.any(labels[-1, :] == label)
            or np.any(labels[:, 0] == label)
            or np.any(labels[:, -1] == label)
        )

        if touches_border:
            outside_labels.add(label)
        else:
            pixel_count = np.count_nonzero(labels == label)
            if pixel_count > largest_area:
                largest_area = pixel_count
                room_label = label

    if room_label is None:
            raise ValueError("No enclosed white region found in the image.")
    outside_mask  = np.isin(labels, list(outside_labels))
    obstacle_mask = (labels != room_label) & (~outside_mask)

    # Find horizontal runs of obstacle pixels in each row
    runs_by_row = []

    for row in range(height):
        runs_by_row.append(find_runs(np.flatnonzero(obstacle_mask[row]).tolist()))
                        
    active_runs = {}
    rectangles = []

    for row, sections in enumerate(runs_by_row):
        current_runs = set()

        for section in sections:
            current_runs.add(tuple(section))

        for run in list(active_runs):
            if run not in current_runs:
                start_row = active_runs[run]
                end_row = row - 1
                start_column, end_column = run

                rectangles.append((start_row, end_row, start_column, end_column))
                del active_runs[run]

        for run in current_runs:
            if run not in active_runs:
                active_runs[run] = row

    for run, start_row in active_runs.items():
        start_column, end_column = run
        end_row = height - 1
        rectangles.append((start_row, end_row, start_column, end_column))


    sdf_models = []

    # Convert each rectangle into a static box with visual and collision geometry
    for number, rectangle in enumerate(rectangles):
        start_row, end_row, start_column, end_column = rectangle
        width_pixels = end_column - start_column + 1
        height_pixels = end_row - start_row + 1
        
        size_x = width_pixels * resolution
        size_y = height_pixels * resolution
        size_z = 1

        center_column = (start_column + end_column) / 2
        center_row = (start_row + end_row) / 2

        world_x = (center_column + 0.5) * resolution
        world_y = (height - center_row - 0.5) * resolution
        world_z = size_z / 2

        model_name = f"obstacle_{number}"
        size_text = f"{size_x} {size_y} {size_z}"
        pose_text = f"{world_x} {world_y} {world_z} 0 0 0"

        model_sdf = f"""
        <model name="{model_name}">
            <static>true</static>
            <pose>{pose_text}</pose>
            <link name="{model_name}_test">
            <visual name="{model_name}_visualization">
                <geometry>
                    <box>
                    <size>{size_text}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.5 0.5 0.5 1</ambient>
                    <diffuse>0.5 0.5 0.5 1</diffuse>
                </material>
            </visual>

            <collision name="{model_name}_collision">
            <geometry>
                <box>
                <size>{size_text}</size>
                </box>
            </geometry>
            </collision>
        </link>
        </model>
        """

        sdf_models.append(model_sdf)

    # Create a ground platform with its top surface at Z = 0
    ground_size_x = width * resolution
    ground_size_y = height * resolution
    ground_size_z = 0.2

    ground_x = (width * resolution) / 2
    ground_y = (height * resolution) / 2
    ground_z = -0.2 / 2

    ground_sdf = f"""
        <model name="ground">
            <static>true</static>
            <pose>{ground_x} {ground_y} {ground_z} 0 0 0</pose>
            <link name="ground_test">
            <visual name="ground_visualization">
                <geometry>
                    <box>
                    <size>{ground_size_x} {ground_size_y} {ground_size_z}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.5 0.5 0.5 1</ambient>
                    <diffuse>0.5 0.5 0.5 1</diffuse>
                </material>
            </visual>

            <collision name="ground_collision">
            <geometry>
                <box>
                <size>{ground_size_x} {ground_size_y} {ground_size_z}</size>
                </box>
            </geometry>
            </collision>
        </link>
        </model>
        """
    sdf_models.append(ground_sdf)
    models_text = "\n".join(sdf_models)

    print(len(sdf_models))

    world_sdf = f"""
        <sdf version="1.10">
            <world name="generated_map">
            <light name="sun" type="directional">
                <pose>0 0 10 0 0 0</pose>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <direction>-0.5 0.1 -0.9</direction>
            </light>
            {models_text}
            </world>
        </sdf>
    """

    # Create the output directory if needed and save the SDF file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(world_sdf, encoding="utf-8")

    return output_path


