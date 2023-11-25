import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the nav2_bringup package
    nav2_bringup_share_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    # Localization and navigation launch file paths
    localization_launch_file = os.path.join(nav2_bringup_share_dir, 'launch', 'localization_launch.py')
    navigation_launch_file = os.path.join(nav2_bringup_share_dir, 'launch', 'navigation_launch.py')

    # Map file path
    map_file_path = './src/robot_simulation/maps/house_map.yaml'

    # Launch description
    ld = LaunchDescription()

    # Set the 'use_sim_time' configuration
    ld.add_action(SetLaunchConfiguration(name='use_sim_time', value='true'))

    # Include the localization launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={'map': map_file_path}.items()
    ))

    # Include the navigation launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={'map_subscribe_transient_local': 'true'}.items()
    ))

    return ld
