#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

static const double PICKUP_POS_X  = 2.3;
static const double PICKUP_POS_Y  = 8.0;
static const double DROPOFF_POS_X = 3.0;
static const double DROPOFF_POS_Y = 0.7;

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

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  // Send the first goal
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = PICKUP_POS_X;
  goal.target_pose.pose.position.y = PICKUP_POS_Y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("sending the 1st goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the first goal");
  else
    ROS_INFO("The base failed to reach the goal for some reason");

  // Pause for 5 seconds
  ros::Duration(5.0).sleep();

  // Send the second goal
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = DROPOFF_POS_X;
  goal.target_pose.pose.position.y = DROPOFF_POS_Y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("sending the second goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base reached the second goal");
  else
    ROS_INFO("The base failed to reach the goal for some reason");
  return 0;
}
