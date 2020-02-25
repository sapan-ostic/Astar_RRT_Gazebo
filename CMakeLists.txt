cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  move_base_msgs
  tf
  map_msgs
  message_generation 
  nav_msgs 
  roslib 
)

add_compile_options(-std=c++11)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES $(PROJECT_NAME)
	CATKIN_DEPENDS nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_walker src/random_walker.cpp)
add_executable(pub_map_odom_tf src/main_pub_map_odom.cpp)
add_executable(sensing src/sensing.cpp)
add_executable(talker src/talker.cpp)

target_link_libraries(random_walker ${catkin_LIBRARIES})
target_link_libraries(pub_map_odom_tf ${catkin_LIBRARIES})
target_link_libraries(sensing ${catkin_LIBRARIES})
target_link_libraries(talker ${catkin_LIBRARIES})