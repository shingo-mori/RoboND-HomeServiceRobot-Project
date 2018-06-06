#!/bin/bash

# deploy turtlebot in the project environment
xterm -e roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/RoboND-HomeServiceRobot-Project/catkin_ws/src/World/udacity.world &

sleep 5

# run gmapping to perform SLAM
xterm -e roslaunch turtlebot_gazebo gmapping_demo.launch &

sleep 3

# observe the generated map in Rviz
xterm -e roslaunch turtlebot_rviz_launchers view_navigation.launch &

sleep 3

# teleop turtlebot using keyboard
xterm -e roslaunch turtlebot_teleop keyboard_teleop.launch
