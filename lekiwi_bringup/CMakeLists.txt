cmake_minimum_required(VERSION 3.28)
project(lekiwi_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(cloudini_lib REQUIRED)
find_package(cloudini_ros REQUIRED)

# Point cloud compressor composable node
add_library(pcl_compressor_node SHARED src/pcl_compressor_node.cpp)
target_link_libraries(pcl_compressor_node
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${point_cloud_interfaces_TARGETS}
  cloudini_lib::cloudini_lib
  cloudini_ros::cloudini_ros
)
rclcpp_components_register_node(pcl_compressor_node
  PLUGIN "lekiwi_bringup::PCLCompressorNode"
  EXECUTABLE pcl_compressor
)

install(TARGETS pcl_compressor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  udev
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_args test/test_launch_args.py
    TIMEOUT 120
  )
  ament_add_pytest_test(test_laser_filter test/test_laser_filter.py
    TIMEOUT 60
  )
endif()

ament_package()
