#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc,char** argv)
{

  ros::init(argc,argv,"rotate_listener");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  uint32_t line = visualization_msgs::Marker::LINE_STRIP;
  visualization_msgs::Marker base_line;
  base_line.header.frame_id = "/odom";
  base_line.header.stamp    = ros::Time::now();
  base_line.ns              = "basic_shapes";
  base_line.action          = visualization_msgs::Marker::ADD;
  base_line.pose.orientation.w = 1.0;

  base_line.id = 0;
  base_line.type=line;


  base_line.scale.x = 0.1;
  base_line.color.a = 1.0;
  base_line.color.r = 1.0;
  base_line.color.g = 1.0;
  base_line.color.b = 0.0;


  tf::TransformListener listener;
  ros::Rate rate(10);
//  ros::Duration(1.0).sleep();
  while(ros::ok())
  {
    tf::StampedTransform transform1;
    try
    {
            listener.waitForTransform("odom","base_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("odom","base_link", ros::Time(0), transform1);
     }
     catch (tf::TransformException &ex)
     {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
     }

    geometry_msgs::Point po1;
    po1.x = transform1.getOrigin().x();
    po1.y = transform1.getOrigin().y();
    po1.z = transform1.getOrigin().z();
    if(base_line.points.size() >= 1000)
    {    //当这个ｖｅｃｔｅｒ的长度大于13０个，就删除前面的数
        base_line.points.erase(base_line.points.begin());
    }
    base_line.points.push_back(po1);

    
    marker_pub.publish(base_line);

    rate.sleep();

  }
  return 0;
}



