cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  std_msgs
  knowrob
  rosprolog
  )

find_package(PkgConfig)

# find the C++ SWIPL Compatability Library
pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS rosprolog
)

include_directories(
  include ${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)


# Compile the NLP color lib
add_library(nlp_color SHARED src/naturallanguage/color/nlp_color.cpp)
target_link_libraries(nlp_color ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(nlp_color
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})


