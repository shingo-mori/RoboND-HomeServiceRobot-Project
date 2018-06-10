cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    move_base_msgs
    actionlib
    roscpp
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  move_base_msgs
)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/pick_objects.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
