# humanoid_navigation.launch.py
# This file is a placeholder for a ROS 2 launch file to bring up the Nav2 stack
# with configurations adapted for a humanoid robot.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Get the share directory for the current package (nav2_humanoid_navigation)
    pkg_share_dir = get_package_share_directory('nav2_humanoid_navigation') # Replace with actual package name

    # Path to the Nav2 params file for the humanoid
    nav2_params_file = os.path.join(pkg_share_dir, 'config', 'humanoid_nav2.yaml')

    # Path to the Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = LaunchConfiguration('map', default='[path_to_initial_map.yaml]') # TODO: Provide path to a static map if used

    return LaunchDescription([
        # Set env var to use ros_ign_bridge for Gazebo -> ROS 2 if needed
        # SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ...),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically start up the Nav2 stack'),
        DeclareLaunchArgument(
            'map', default_value=map_yaml_file,
            description='Full path to map yaml file to load'),

        # TODO: Launch the ROS 2 to TF bridge (if required for humanoid model)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_footprint',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        # ),

        # Include the main Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': nav2_params_file,
                # 'robot_base_frame': 'base_link', # Ensure this matches your robot's base frame
                # 'odom_frame': 'odom',
                # 'global_frame': 'map',
            }.items()
        ),

        # TODO: Potentially launch additional nodes specific to humanoid navigation
        # e.g., a node for adapting Nav2 commands to bipedal locomotion,
        # or a custom local planner.
    ])