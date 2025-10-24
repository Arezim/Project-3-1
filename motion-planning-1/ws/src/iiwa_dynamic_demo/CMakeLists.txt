cmake_minimum_required(VERSION 3.0.2)
project(iiwa_dynamic_demo)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/plan_and_replan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
