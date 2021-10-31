#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>
nav_msgs::Path waypoint_path;
ros::Publisher waypoint_pub;
ros::Subscriber waypoint_sub;
bool first_flag;
void receive_waypoint(const geometry_msgs::PoseStampedConstPtr &msg)
{
    geometry_msgs::PoseStamped mGoal;
    mGoal.pose.position.x = msg->pose.position.x;
    mGoal.pose.position.y = msg->pose.position.y;
    mGoal.pose.orientation = msg->pose.orientation;
    mGoal.header.frame_id = msg->header.frame_id;
    waypoint_path.poses.push_back(mGoal);
    waypoint_pub.publish(waypoint_path);
    
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_GPS_input");
  ros::NodeHandle n;
  waypoint_path.header.frame_id = "map";
  geometry_msgs::PoseStamped Origin_point;
  // Origin_point.pose.position.x = 0;
  // Origin_point.pose.position.y = 0;
  // waypoint_path.poses.push_back(Origin_point);
  waypoint_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &receive_waypoint);
  waypoint_pub = n.advertise<nav_msgs::Path>("waypoint",10);
  
  ros::Rate looprate(10);
  while(ros::ok())
  {

    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}
