from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_core',
            executable='speech_to_text_node',
            name='speech_to_text_node',
            output='screen'
        ),
        Node(
            package='vla_core',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen'
        ),
        Node(
            package='vla_core',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        ),
    ])
