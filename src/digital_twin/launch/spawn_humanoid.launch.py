import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('digital_twin'),
        'models',
        'humanoid.urdf'
    )

    # Get the path to the world file
    world_file_path = os.path.join(
        get_package_share_directory('digital_twin'),
        'worlds',
        'empty.world'
    )

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path,
                 '-s', 'libgazebo_ros_init.so', # For use_sim_time
                 '-s', 'libgazebo_ros_factory.so'], # For spawning entities
            output='screen'
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', urdf_file_path],
            output='screen'
        ),

        # Start the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read(),
                         'use_sim_time': use_sim_time}]
        ),
    ])
