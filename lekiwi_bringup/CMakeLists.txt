cmake_minimum_required(VERSION 3.28)
project(lekiwi_bringup)

find_package(ament_cmake REQUIRED)

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
endif()

ament_package()
