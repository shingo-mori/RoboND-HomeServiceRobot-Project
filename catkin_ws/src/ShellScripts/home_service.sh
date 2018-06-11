#!/bin/bash

catkin_ws_dir=/home/workspace/RoboND-HomeServiceRobot-Project/catkin_ws

# deploy turtlebot in the project environment
xterm -e roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${catkin_ws_dir}/src/World/udacity.world &

sleep 5

# run amcl to perform global localization
xterm -e roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${catkin_ws_dir}/src/World/udacity.yaml &

sleep 3

# observe the navigation in Rviz
xterm -e roslaunch turtlebot_rviz_launchers view_navigation.launch &

sleep 3

# add markers at the pickup and drop off zone
xterm -e rosrun add_markers add_markers &

sleep 3

# launch pick_objects node to define multiple goals
xterm -hold -e rosrun pick_objects pick_objects
