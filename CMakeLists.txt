cmake_minimum_required(VERSION 2.8.3)
project(reflex_takktile2_ros_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  reflex
  reflex_driver
  reflex_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()



# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


