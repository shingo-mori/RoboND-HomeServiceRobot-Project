cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/add_markers.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
