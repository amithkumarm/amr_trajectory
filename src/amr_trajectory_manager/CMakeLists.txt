cmake_minimum_required(VERSION 3.8)
project(amr_trajectory_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS Yes)
set(CMAKE_BUILD_TYPE "Debug")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(amr_trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include/amr_trajectory_manager)

# Add executable nodes
add_executable(turtlebot3_autonomous src/turtlebot3_autonomous.cpp)
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_executable(trajectory_follower src/trajectory_follower.cpp)

ament_target_dependencies(turtlebot3_autonomous rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(trajectory_publisher_saver rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_ros amr_trajectory_msgs tf2_geometry_msgs)
ament_target_dependencies(trajectory_follower rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs amr_trajectory_msgs)

install(
  TARGETS turtlebot3_autonomous trajectory_publisher_saver trajectory_follower
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
