cmake_minimum_required(VERSION 3.8)
project(scv_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

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
  maps
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()