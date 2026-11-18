cmake_minimum_required(VERSION 3.16)
project(so101_mtc_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the launch file loads with its arguments, and the plugins name frames and bodies that exist
  ament_add_pytest_test(test_bringup test/test_bringup.py TIMEOUT 60)
endif()

ament_package()
