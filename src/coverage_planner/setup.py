import os
from setuptools import setup, find_packages

package_name = 'coverage_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            [
                'launch/coverage_visualization.launch.py',
                'launch/coverage_simulation.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takacs',
    maintainer_email='takacs@email.com',
    description='Coverage planner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_path_visualizer = coverage_planner.visualization.path_visualizer:main',
            'coverage_action_server = coverage_planner.action_server.server:main',
            'coverage_path_follower = coverage_planner.path_follower:main',
            'coverage_spawn_from_path = coverage_planner.spawn_from_path:main'
        ],
    },
)