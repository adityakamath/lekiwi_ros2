cmake_minimum_required(VERSION 3.16)
project(lekiwi_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Build C++ executables
add_executable(motor_diagnostics src/motor_diagnostics_node.cpp)
ament_target_dependencies(motor_diagnostics rclcpp diagnostic_msgs control_msgs)

# Install C++ executables
install(TARGETS
  motor_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

# Platform-specific RPATH handling (macOS workaround for Pixi environments)
if(APPLE)
  set_target_properties(motor_diagnostics PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH FALSE
  )
endif()

# Install launch files, config, URDF, and documentation
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
