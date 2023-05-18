cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
  
)


catkin_package(  
  CATKIN_DEPENDS
    crazyflie_driver)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Declare a cpp executable
add_executable(crazyflie_controller src/Leader.cpp)
add_executable(follower_controller src/Follower.cpp)


target_link_libraries(crazyflie_controller ${catkin_LIBRARIES})
target_link_libraries(follower_controller ${catkin_LIBRARIES})



