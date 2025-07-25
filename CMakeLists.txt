cmake_minimum_required(VERSION 3.8)
project(scv_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable for velocity publisher
add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp nav_msgs std_msgs)

# Install executable
install(TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Add submodule packages as subdirectories
add_subdirectory(sensors/velodyne_simulator/velodyne_description)
add_subdirectory(sensors/velodyne_simulator/velodyne_gazebo_plugins)

install(DIRECTORY
  launch
  urdf
  meshes
  config
  robots
  sensors
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()