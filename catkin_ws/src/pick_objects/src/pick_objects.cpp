#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a
// SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base actin server to come up
  while (!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1, goal2;

  // Send the first goal
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  goal1.target_pose.pose.position.x = 2.3;
  goal1.target_pose.pose.position.y = 8.0;
  goal1.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("sending the 1st goal");
  ac.sendGoal(goal1);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the first goal");
  else
    ROS_INFO("The base failed to reach the goal for some reason");

  // Pause for 5 seconds
  ros::Duration(5.0).sleep();

  // Send the second goal
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.pose.position.x = 3.0;
  goal2.target_pose.pose.position.y = 0.7;
  goal2.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("sending the second goal");
  ac.sendGoal(goal2);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the second goal");
  else
    ROS_INFO("The base failed to reach the goal for some reason");
  return 0;
}
