cmake_minimum_required(VERSION 3.16)
project(so101_mtc_task LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)

# ADR 0003's generator of grasp and place poses that the 5-joint arm reaches exactly
add_library(grasp_poses SHARED src/grasp_poses.cpp)
target_include_directories(grasp_poses PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(grasp_poses PUBLIC Eigen3 moveit_core rclcpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS grasp_poses EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(so101_mtc_description REQUIRED)
  find_package(xacro REQUIRED)
  find_package(yaml-cpp REQUIRED)

  # The robot as move_group loads it: so101_mtc_description's xacro, expanded at build time (xacro tracks the files
  # it includes, so a change of the description rebuilds it)
  set(test_urdf "${CMAKE_CURRENT_BINARY_DIR}/so101_mtc.urdf")
  xacro_add_xacro_file("${so101_mtc_description_DIR}/../urdf/so101_mtc.urdf.xacro" "${test_urdf}")
  add_custom_target(test_urdf ALL DEPENDS "${test_urdf}")

  # The generator against MoveIt's KDL, FK and collision check over the task area (about 20 s)
  ament_add_gtest(test_grasp_poses test/test_grasp_poses.cpp TIMEOUT 300 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_grasp_poses grasp_poses yaml-cpp)
  ament_target_dependencies(test_grasp_poses ament_index_cpp moveit_ros_planning)
  target_compile_definitions(test_grasp_poses PRIVATE TEST_URDF="${test_urdf}")
  add_dependencies(test_grasp_poses test_urdf)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 moveit_core rclcpp)
ament_package()
