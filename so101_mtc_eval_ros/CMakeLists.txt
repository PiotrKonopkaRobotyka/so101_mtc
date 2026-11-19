cmake_minimum_required(VERSION 3.16)
project(so101_mtc_eval_ros NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The ROS-free core lives next to this package, with its own pyproject.toml for pytest and CI, and colcon ignores it.
# ADR 0002 keeps Python to apt packages without a venv, so it is installed here, where the runner imports it.
ament_python_install_package(so101_mtc_eval PACKAGE_DIR ../so101_mtc_eval/so101_mtc_eval)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/run_trials DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the installed command, called as a person calls it, with the default arguments (a dry run: no simulator)
  ament_add_pytest_test(test_run_trials test/test_run_trials.py TIMEOUT 60)
endif()

ament_package()
