cmake_minimum_required(VERSION 3.8)
project(post_process)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(sync_via_approx_time_policy src/sync_via_approx_time_policy.cpp)
ament_target_dependencies(
  sync_via_approx_time_policy
  rclcpp
  std_msgs
  message_filters
  sensor_msgs
  pcl_ros
  pcl_conversions
  OpenCV
  cv_bridge)

install(
  TARGETS
  sync_via_approx_time_policy
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
