cmake_minimum_required(VERSION 3.28)
project(lekiwi_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(cloudini_lib REQUIRED)
find_package(cloudini_ros REQUIRED)

# Point cloud compressor composable node
add_library(colored_pointcloud_compressor_node SHARED
  src/pcl_compressor_node.cpp
)
target_include_directories(colored_pointcloud_compressor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(colored_pointcloud_compressor_node
  rclcpp
  rclcpp_components
  sensor_msgs
  point_cloud_interfaces
  cloudini_lib
  cloudini_ros
)
rclcpp_components_register_node(colored_pointcloud_compressor_node
  PLUGIN "lekiwi_bringup::PCLCompressorNode"
  EXECUTABLE pcl_compressor
)

install(TARGETS colored_pointcloud_compressor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()