cmake_minimum_required(VERSION 3.22)
project(mujoco_ros2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)

# One library holds every plugin: add a source file here and a class to plugins.xml.
set(PLUGIN_SOURCES src/emergency_stop_plugin.cpp)

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  ${std_srvs_TARGETS}
  pluginlib::pluginlib)

# Registers the plugin with the mujoco_ros2_control plugin loader.
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_emergency_stop test/test_emergency_stop.cpp)
  target_include_directories(test_emergency_stop PRIVATE include)
  target_link_libraries(test_emergency_stop ${PROJECT_NAME} mujoco::mujoco)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_plugin_registration test/test_plugin_registration.py TIMEOUT 60)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_srvs pluginlib mujoco_vendor mujoco_ros2_control_plugins)
ament_package()
