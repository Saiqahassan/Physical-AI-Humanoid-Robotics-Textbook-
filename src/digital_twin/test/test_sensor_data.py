import os
import pytest
import rclpy
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from sensor_msgs.msg import LaserScan, Image, Imu
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    urdf_file_path = os.path.join(
        get_package_share_directory('digital_twin'),
        'models',
        'humanoid.urdf'
    )
    world_file_path = os.path.join(
        get_package_share_directory('digital_twin'),
        'worlds',
        'empty.world'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', urdf_file_path],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read(),
                         'use_sim_time': True}],
            emulate_tty=True
        ),
        ReadyToTest()
    ])


class TestSensorData:

    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_sensor_data_node')
        cls.lidar_received = False
        cls.depth_camera_received = False
        cls.imu_received = False

        cls.lidar_subscriber = cls.node.create_subscription(
            LaserScan,
            '/lidar',
            cls.lidar_callback,
            10
        )
        cls.depth_camera_subscriber = cls.node.create_subscription(
            Image,
            '/depth_camera',
            cls.depth_camera_callback,
            10
        )
        cls.imu_subscriber = cls.node.create_subscription(
            Imu,
            '/imu',
            cls.imu_callback,
            10
        )

    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def lidar_callback(cls, msg):
        cls.lidar_received = True

    @classmethod
    def depth_camera_callback(cls, msg):
        cls.depth_camera_received = True

    @classmethod
    def imu_callback(cls, msg):
        cls.imu_received = True

    def test_sensor_data_published(self):
        timeout_sec = 20  # Increased timeout for Gazebo startup
                start_time = time.time()
                
                while (not self.lidar_received or not self.depth_camera_received or               not self.imu_received) and \
              ((time.time() - start_time) < timeout_sec):
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    
                    assert self.lidar_received, "Did not receive LiDAR message"        assert self.depth_camera_received, \
                        "Did not receive Depth Camera message"
                    assert self.imu_received, "Did not receive IMU message"
            
