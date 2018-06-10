cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    sensor_msgs
    roscpp
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/wall_follower.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
