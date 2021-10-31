#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace ros;

ros::Publisher pc2Pub;
ros::Subscriber laserSub;


void scan_cb(const sensor_msgs::LaserScan::ConstPtr &input)
{
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  if(!listener_.waitForTransform(input->header.frame_id,
                                 "/hokuyo_link",
                                 input->header.stamp+ros::Duration().fromSec(input->ranges.size()*input->time_increment),
                                 ros::Duration(1)))
  {
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("/hokuyo_link",*input,cloud,listener_);
  pc2Pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"laserscan_to_pointcloud2");
  ros::NodeHandle nh;

  laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan",5,scan_cb);
  pc2Pub = nh.advertise<sensor_msgs::PointCloud2>("/outpointcloud2",5);

  ros::spin();
  return 0;
}
