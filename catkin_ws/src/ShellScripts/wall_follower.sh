#!/bin/bash

catkin_ws_dir=/home/workspace/RoboND-HomeServiceRobot-Project/catkin_ws
# deploy turtlebot in the project environment
xterm -e roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${catkin_ws_dir}/src/World/udacity.world &

sleep 5

# run gmapping to perform SLAM
xterm -e roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=${catkin_ws_dir}/src/wall_follower/launch/include/gmapping.launch.xml &

sleep 3

# observe the generated map in Rviz
xterm -e roslaunch turtlebot_rviz_launchers view_navigation.launch &

sleep 3

# teleop turtlebot using keyboard
xterm -hold -e rosrun wall_follower wall_follower
