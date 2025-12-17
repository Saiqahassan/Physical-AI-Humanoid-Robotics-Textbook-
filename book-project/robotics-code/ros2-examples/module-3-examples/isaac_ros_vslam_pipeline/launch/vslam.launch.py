# vslam.launch.py
# This script is a placeholder for a ROS 2 launch file for the Isaac ROS VSLAM pipeline.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TODO: Launch the Isaac ROS VSLAM node
        # Node(
        #     package='isaac_ros_visual_slam',
        #     executable='visual_slam_node',
        #     name='visual_slam_node',
        #     parameters=[{
        #         'enable_image_denoising': False,
        #         'enable_imu_fusion': True,
        #         'gyro_noise_density': 0.000206,
        #         'accel_noise_density': 0.003,
        #         'calibration_l_frame': 'camera_link',
        #         'base_frame': 'base_link',
        #         'odom_frame': 'odom',
        #         'publish_tf': True,
        #         'left_camera_frame': 'camera_link',
        #         'right_camera_frame': 'right_camera_link',
        #         'enable_localization_n_mapping': True,
        #         'map_frame': 'map',
        #         'qos_history_depth': 10
        #     }],
        #     remappings=[
        #         ('left_image_rect', '/camera/image_raw'),
        #         ('left_camera_info', '/camera/camera_info'),
        #         ('imu', '/imu/data')
        #     ]
        # ),

        # TODO: Launch the sim_data_bridge node
        Node(
            package='isaac_ros_vslam_pipeline', # Replace with actual package name if different
            executable='sim_data_bridge',
            name='sim_data_bridge',
            output='screen'
        )
    ])
