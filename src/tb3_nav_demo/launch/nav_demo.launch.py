"""One-command TurtleBot3 + Nav2 demo on this package's world, map and params.

Wraps nav2_bringup's tb3_simulation_launch.py so the stock, well-tested bringup does the work.

  ros2 launch tb3_nav_demo nav_demo.launch.py              # navigate on the saved map (AMCL)
  ros2 launch tb3_nav_demo nav_demo.launch.py slam:=True   # build a new map with slam_toolbox
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('tb3_nav_demo')
    bringup = get_package_share_directory('nav2_bringup')

    # (name, default, description), all passed straight through to tb3_simulation_launch.py
    args = [
        ('world', os.path.join(pkg, 'worlds', 'my_world.sdf.xacro'), 'Gazebo world (SDF xacro)'),
        ('map', os.path.join(pkg, 'maps', 'my_world.yaml'), 'Map for AMCL (ignored when slam:=True)'),
        ('params_file', os.path.join(pkg, 'config', 'nav2_params.yaml'), 'Nav2 parameters'),
        ('slam', 'False', 'Map with slam_toolbox instead of localizing with AMCL'),
        ('headless', 'False', 'Run Gazebo without its GUI'),
        ('use_rviz', 'True', 'Start RViz'),
        ('rviz_config_file', os.path.join(pkg, 'rviz', 'nav.rviz'),
         'RViz layout: map, costmaps, laser, robot, plans'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name, _, _ in args}.items(),
    )

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, default, desc in args]
        + [sim]
    )
