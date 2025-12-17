from setuptools import find_packages, setup

package_name = 'pubsub_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='rclpy publisher and subscriber examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = pubsub_rclpy.simple_publisher:main',
            'simple_subscriber = pubsub_rclpy.simple_subscriber:main',
        ],
    },
)