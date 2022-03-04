cmake_minimum_required(VERSION 3.0.2)
project(eaglex_rover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  rosserial_arduino
  sensor_msgs
  std_msgs
)

catkin_package()

add_executable(base_module src/base_module/base_module.cpp)
target_link_libraries(base_module ${catkin_LIBRARIES})
# add_dependencies(base_module eaglex_rover_generate_messages_cpp)

add_executable(delta_coordinates src/autonomy_module/delta_coordinates/delta_coordinates.cpp)
target_link_libraries(delta_coordinates ${catkin_LIBRARIES})
# add_dependencies(delta_coordinates eaglex_rover_generate_messages_cpp)


# add_executable(talker src/drivers_module/scripts/tst/talker.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
##add_dependencies(talker eaglex_rover_generate_messages_cpp)

# add_executable(echoer src/drivers_module/scripts/tst/echoer.cpp)
# target_link_libraries(echoer ${catkin_LIBRARIES})
# ##add_dependencies(echoer eaglex_rover_generate_messages_cpp)

# add_executable(listener src/drivers_module/scripts/tst/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
##add_dependencies(listener eaglex_rover_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


