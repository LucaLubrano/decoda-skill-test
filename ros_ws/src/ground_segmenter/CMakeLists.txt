cmake_minimum_required(VERSION 3.8)
project(ground_segmenter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Segmentation Node
# add_executable(bag_segmentation_node
# src/bag_segmentation_node.cpp)

# LiDAR Healther Checker Node
add_executable(lidar_checker
src/lidar_checker.cpp)

# ament_target_dependencies(bag_segmentation_node
#   rclcpp
#   std_msgs
#   sensor_msgs
# )

ament_target_dependencies(lidar_checker
  rclcpp
  std_msgs
  sensor_msgs
)

# Install
# install(TARGETS bag_segmentation_node
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS lidar_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()