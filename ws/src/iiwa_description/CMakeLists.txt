cmake_minimum_required(VERSION 3.0.2)
project(iiwa_description)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS xacro)

## Declare catkin package
catkin_package()

## (Optional) Install the description resources so other packages can find them
install(DIRECTORY urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
