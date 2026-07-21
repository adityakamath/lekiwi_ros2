cmake_minimum_required(VERSION 3.28)
project(lekiwi_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py
    TIMEOUT 60
  )
endif()

ament_package()