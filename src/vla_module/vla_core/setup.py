from setuptools import setup

package_name = 'vla_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='VLA core package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_text_node = vla_core.nodes.speech_to_text_node:main',
            'robot_controller_node = vla_core.nodes.robot_controller_node:main',
            'llm_planner_node = vla_core.nodes.llm_planner_node:main',
            'perception_node = vla_core.nodes.perception_node:main',
        ],
    },
)
