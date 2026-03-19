cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_cmake NONE)

find_package(ament_cmake REQUIRED)

# Install cmake modules to the ament share directory.
# The *-extras.cmake hook adds this path to CMAKE_MODULE_PATH automatically.
install(
  FILES
    cmake/ROS2MedkitCompat.cmake
    cmake/ROS2MedkitCcache.cmake
    cmake/ROS2MedkitLinting.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Install .clang-tidy config alongside the cmake modules so
# ROS2MedkitLinting.cmake can find it via CMAKE_CURRENT_LIST_DIR.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(
  CONFIG_EXTRAS cmake/ros2_medkit_cmake-extras.cmake
)
