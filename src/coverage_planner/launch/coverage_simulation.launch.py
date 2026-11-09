import os
import math
import yaml

from pathlib import Path
from coverage_planner.map_to_sdf import generate_world

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import  DeclareLaunchArgument, ExecuteProcess, AppendEnvironmentVariable, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# Convert the robot size and clearance from meters to map pixels
def robot_size_from_resolution(resolution):
    clearance_m = 0.05
    robot_size = (0.248 + 2 * clearance_m) / resolution
    return math.ceil(robot_size)

# Calculate the robot size automatically or validate the supplied pixel value
def resolve_robot_size(requested_size, resolution):
    if requested_size == "auto":
        return robot_size_from_resolution(resolution)
    else:
        requested_size = int(requested_size)

    if requested_size <= 0:
        raise ValueError("robot size must be a positive integer in pixels.")

    return requested_size

def read_map_resolution(map_yaml_path):
   
    with open(map_yaml_path, "r", encoding="utf-8") as fajl:
        map_data = yaml.safe_load(fajl)

    return float(map_data["resolution"])

def read_launch_config(config_path):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    
    return config

# Apply the configuration settings and resolve paths relative to the config file
def load_config_file(context):
    config_path = LaunchConfiguration("config_file").perform(context)

    if len(config_path) == 0:
        return []

    config_path = os.path.expanduser(config_path)
    
    config = read_launch_config(config_path)

    config_dir = os.path.dirname(os.path.abspath(config_path))

    for key in ("map_yaml", "world_file"):

        if key in config:
            path = os.path.expanduser(config[key])
            config[key] = os.path.join(config_dir, path)

    config_actions = []

    for key, value in config.items():
        config_actions.append(SetLaunchConfiguration(key, str(value)))

    return config_actions

# Configure the path visualizer with the selected robot size and sweep angle
def create_path_visualizer(context):
    map_yaml_path = LaunchConfiguration("map_yaml").perform(context)
    
    requested_size = LaunchConfiguration("robot_size").perform(context)
    
    resolution = read_map_resolution(map_yaml_path)

    robot_size = resolve_robot_size(requested_size, resolution)

    # Lefedési útvonaltervező
    return [Node(
        package='coverage_planner',
        executable='coverage_path_visualizer',
        name='path_visualizer',
        output='screen',
        parameters=[{
            'map_name': LaunchConfiguration('map_name'),
            'robot_size': robot_size,
            'threshold': 0.55,
            'angle_degrees': LaunchConfiguration('angle_degrees'),
        }]
    )]

# Generate the Gazebo world from the map image and set its launch path
def prepare_world(context):
    map_yaml_path = Path(LaunchConfiguration("map_yaml").perform(context))

    with open(map_yaml_path, "r", encoding="utf-8") as fajl:
        map_data = yaml.safe_load(fajl)

    map_path = map_yaml_path.parent / map_data["image"]

    resolution = float(map_data["resolution"])

    output_path = (
        Path.home()
        / "coverage_planner_ws"
        / "world_sdf"
        / f"{map_path.stem}_world.sdf"
    )

    generated_file = generate_world(map_path, output_path, resolution)

    return [SetLaunchConfiguration("world_file", str(generated_file))]

# Define the startup sequence for the coverage simulation
def generate_launch_description():

    map_yaml = os.path.expanduser(
        '~/coverage_planner_ws/maps/Map3.yaml'
    )

    rviz_config = os.path.expanduser(
        '~/coverage_planner_ws/config/coverage_planner.rviz'
    )

    world_file = os.path.expanduser(
        '~/coverage_planner_ws/generated_map.sdf'
    )

    robot_file = os.path.expanduser(
        '~/coverage_planner_ws/models/burger_with_pose.sdf'
    )

    turtlebot3_models = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name="config_file",
            default_value=os.path.expanduser(
                "~/coverage_planner_ws/initialization.txt"
            ),
            description="Path to the configuration file",
        ),

        DeclareLaunchArgument(
            "start_follower",
            default_value="false",
            description="Start the coverage path follower"
        ),

        DeclareLaunchArgument(
            name="map_name",
            default_value="Map3.png",
            description="Map image filename"
        ),

        DeclareLaunchArgument(
            name="map_yaml",
            default_value=map_yaml,
            description="Map YAML file path"
        ),

        DeclareLaunchArgument(
            name="world_file",
            default_value=world_file,
            description="Gazebo world SDF file path"
        ),

        DeclareLaunchArgument(
            name="angle_degrees",
            default_value="5.0",
            description="Coverage sweep angle in degrees"
        ),

        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=turtlebot3_models
        ),

        DeclareLaunchArgument(
            name="robot_size",
            default_value="auto",
            description="Robot size in map pixels, or auto"
        ),

        # Load the settings before generating the world and starting Gazebo.
        OpaqueFunction(function=load_config_file),
        OpaqueFunction(function=prepare_world),

        ExecuteProcess(
           cmd=["gz", "sim", "-r", LaunchConfiguration("world_file")],
           output="screen"
        ),

        Node(
            package="coverage_planner",
            executable="coverage_spawn_from_path",
            name="path_from_spawner",
            output="screen",
            parameters=[{"robot_file": robot_file, "world_name":"generated_map"}]
        ),

        Node(
            package= "ros_gz_bridge",
            executable= "parameter_bridge",
            name= "simulation_bridge",
            output= "screen",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/model/burger/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
            ] 
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml')
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        Node(
            package='coverage_planner',
            executable='coverage_action_server',
            name='coverage_action_server',
            output='screen',
        ),

        Node(
            package='coverage_planner',
            executable='coverage_path_follower',
            name='path_follower',
            output='screen',
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("start_follower"))
        ),

        OpaqueFunction(function=create_path_visualizer),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

    ])
