cmake_minimum_required(VERSION 3.0.2)

project(nlg_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)



add_message_files(

  DIRECTORY msg

  FILES  

  KeyValuePair.msg
  MeaningRepresentation.msg

)



generate_messages()



catkin_package(

  INCLUDE_DIRS include

  CATKIN_DEPENDS message_runtime)



install(DIRECTORY include/nlg_msgs/

  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

  PATTERN "*.h")
