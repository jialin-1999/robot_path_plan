#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_GPS_input");
  ros::NodeHandle n;

  ros::Publisher goalGPSpub = n.advertise<nav_msgs::Path>("/goalGPS",10);
  // ros::Publisher robotGPSpub = n.advertise<sensor_msgs::NavSatFix>("/fix_test",10);


  // sensor_msgs::NavSatFix robotGPS;
  // robotGPS.latitude = 36.65048450;
  // robotGPS.longitude = 117.027958921;
  nav_msgs::Path goals;
  geometry_msgs::PoseStamped temp;
  temp.pose.position.x = 36.5763166;
  temp.pose.position.y = 116.8301985;
  goals.poses.push_back(temp);
  // temp.pose.position.x = 36.65059667;
  // temp.pose.position.y = 117.0291845;
  // goals.poses.push_back(temp);
  // temp.pose.position.x = 36.9999;
  // temp.pose.position.y = 115.9999;
  // goals.poses.push_back(temp);

  sleep(5);
  goalGPSpub.publish(goals);
  ROS_INFO("GPS_input-->pub goal");
  ros::Rate looprate(100);
  while(ros::ok())
  {
    // robotGPSpub.publish(robotGPS);

    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}
