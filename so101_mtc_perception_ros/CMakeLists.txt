cmake_minimum_required(VERSION 3.16)
project(so101_mtc_perception_ros NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The ROS-free core lives next to this package, with its own pyproject.toml for pytest and CI, and colcon ignores it.
# ADR 0002 keeps Python to apt packages without a venv, so it is installed here, where the node imports it.
ament_python_install_package(so101_mtc_perception PACKAGE_DIR ../so101_mtc_perception/so101_mtc_perception)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/block_detector DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the node's image → pose conversion on a synthetic image, without a simulator
  ament_add_pytest_test(test_block_detector test/test_block_detector.py TIMEOUT 60)
endif()

ament_package()
