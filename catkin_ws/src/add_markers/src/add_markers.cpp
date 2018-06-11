#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   
  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok())
      return 0;
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }

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

  // Publish the marker at the pickup zone
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 2.3;
  marker.pose.position.y = 8.0;
  marker.pose.orientation.w = 1.0;
  marker_pub.publish(marker);

  ros::Duration(5.0).sleep(); 

  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  ros::Duration(5.0).sleep(); 

  // Publish the marker at the dropoff zone
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 3.0;
  marker.pose.position.y = 0.7;
  marker.pose.orientation.w = 1.0;
  marker_pub.publish(marker);
}
