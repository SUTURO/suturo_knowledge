cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
  knowrob
  rosprolog
  )

# add_message_files(
#   FILES
#   DetectedObject.msg
# )

# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
#   std_msgs
#)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

