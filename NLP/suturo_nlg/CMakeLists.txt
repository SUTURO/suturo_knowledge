cmake_minimum_required(VERSION 3.0.2)

project(suturo_nlg)

find_package(catkin REQUIRED COMPONENTS message_generation)



add_message_files(

  DIRECTORY msg

  FILES  

  KeyValuePair.msg
  MeaningRepresentation.msg

)



generate_messages()



catkin_package(
  CATKIN_DEPENDS message_runtime)



install(DIRECTORY include/nlg_msgs/

  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

  PATTERN "*.h")
