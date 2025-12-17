cmake_minimum_required(VERSION 3.8)
project(digital_twin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pytest REQUIRED)
find_package(launch_testing REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS src/joint_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    ament_add_pytest_test(test/test_joint_control.py)
    ament_add_pytest_test(test/test_sensor_data.py)
endif()

ament_package()
