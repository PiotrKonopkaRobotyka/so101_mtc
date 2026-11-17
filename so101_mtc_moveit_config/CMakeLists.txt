cmake_minimum_required(VERSION 3.16)
project(so101_mtc_moveit_config NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
# MoveItConfigsBuilder reads it from the installed package: where the URDF and the SRDF are
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the installed config: SRDF ↔ URDF, named states, collisions (moveit_py), controllers, KDL, limits
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py TIMEOUT 120)
endif()

ament_package()
