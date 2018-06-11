#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

static ros::Publisher marker_pub;

static bool pickingUp = false;

void onAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  ROS_INFO("Received amcl_pose topic!");

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  const double POS_THRESH = 0.1;
  const double PICKUP_POS_X = 2.3;
  const double PICKUP_POS_Y = 8.0;
  const double DROPOFF_POS_X = 3.0;
  const double DROPOFF_POS_Y = 0.7;
  
  // Set marker shape type to a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Create a marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker";
  marker.id = 0;
  marker.type = shape;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();

  if(!pickingUp &&
     abs(x - PICKUP_POS_X) < POS_THRESH &&
     abs(y - PICKUP_POS_Y) < POS_THRESH){
    ROS_INFO("Pick the object up!");

    // Hide the marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    pickingUp = true;
  }
  else if(pickingUp &&
          abs(x - DROPOFF_POS_X) < POS_THRESH &&
          abs(y - DROPOFF_POS_Y) < POS_THRESH){
    // Publish the marker at the dropoff zone
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3.0;
    marker.pose.position.y = 0.7;
    marker.pose.orientation.w = 1.0;
    marker_pub.publish(marker);

    ROS_INFO("Drop off the object, job done!");

    pickingUp = false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;

  ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, onAmclPose);
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){
     return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }

  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Create a marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker";
  marker.id = 0;
  marker.type = shape;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();

  // Publish the marker at the pickup zone
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 2.3;
  marker.pose.position.y = 8.0;
  marker.pose.orientation.w = 1.0;
  marker_pub.publish(marker);

  ros::spin();

  return 0;
}
