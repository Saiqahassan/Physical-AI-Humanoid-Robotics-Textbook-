import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    urdf_share_dir = get_package_share_directory('urdf_display')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_model_path = os.path.join(
        get_package_share_directory('urdf_display'),
        'urdf',
        'simple_humanoid.urdf')

    # Define the nodes for the launch file
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf_model_path}],
        arguments=[urdf_model_path]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(urdf_share_dir, 'rviz', 'simple_humanoid.rviz')]
    )

    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
