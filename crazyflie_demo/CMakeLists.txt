cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_demo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

