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
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)


# Segmentation Node
add_executable(ground_segmentation
src/ground_segmentation.cpp)

# LiDAR Healther Checker Node
add_executable(lidar_checker
src/lidar_checker.cpp)

ament_target_dependencies(ground_segmentation
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(lidar_checker
  rclcpp
  std_msgs
  sensor_msgs
)

# Install
install(TARGETS ground_segmentation
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lidar_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()