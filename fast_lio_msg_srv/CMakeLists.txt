cmake_minimum_required(VERSION 2.8.3)
project(fast_lio_msg_srv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  Pose6D.msg
)

add_service_files(
  FILES
  neighbor_points.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
)
