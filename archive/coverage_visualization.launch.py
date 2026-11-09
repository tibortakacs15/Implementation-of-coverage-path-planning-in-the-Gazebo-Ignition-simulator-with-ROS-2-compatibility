from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # A saját csomag telepített könyvtára
    package_dir = get_package_share_directory('coverage_planner')

    # Térkép YAML-fájlja
    map_yaml = os.path.expanduser(
        '~/coverage_planner_ws/maps/Map3.yaml'
    )

    # Elmentett RViz-konfiguráció
    rviz_config = os.path.expanduser(
        '~/coverage_planner_ws/coverage_planner.rviz'
    )

    return LaunchDescription([

        # Térképkiszolgáló
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml
            }]
        ),

        # A térképkiszolgáló aktiválása
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

        # 
        Node(
            package='coverage_planner',
            executable='coverage_action_server',
            name='coverage_action_server',
            output='screen',
        ),

        # Lefedési útvonaltervező
        Node(
            package='coverage_planner',
            executable='coverage_path_visualizer',
            name='path_visualizer',
            output='screen',
            parameters=[{
                'map_name': 'Map3.png',
                'robot_size': 19,
                'threshold': 0.55,
                'angle_degrees': 5.0
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

    ])
