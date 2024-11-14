cmake_minimum_required(VERSION 3.5)
project(my_package)

# Find ament_cmake for the build system and rclpy for ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
