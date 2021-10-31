#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <unistd.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <csc_nav2d_navigator/WGS84ToUTM.h>

class GPSTrans
{
public:
  GPSTrans(tf::TransformListener& ls):mls(&ls)
  {
    ros::NodeHandle n;
    mGPSsubscriber = n.subscribe<sensor_msgs::NavSatFix>("/fix_test",10,&GPSTrans::receiveRobotGPS,this);
    mIMUsubscriber = n.subscribe<sensor_msgs::Imu>("/imu_data",10,&GPSTrans::receiveIMU,this);
    mGoalGPSsubscriber = n.subscribe<nav_msgs::Path>
        ("/goalGPS",10, &GPSTrans::receiveGoalGPS,this);
    mFinishSubscriber = n.subscribe<std_msgs::Bool>("mission_finish",10,&GPSTrans::receiveFinish,this);

    mGoalGPSpublisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    mGoalMarkerpublisher = n.advertise<visualization_msgs::Marker>("/GoalGPSMarkers",10);

    n.param("map_frame", mMapFrame, std::string("map"));
    n.param("robot_frame", mRobotFrame, std::string("base_link"));
    mFinish = true;
    mExeCount = 0;
    mGoalPos.clear();
    mGoalInd = 0;

    mMapToWorld = Eigen::Isometry3d::Identity().matrix();
    Eigen::AngleAxisd rollAngle(0*3.14159/180, Eigen::Vector3d::UnitX());//180
    Eigen::AngleAxisd pitchAngle(0*3.14159/180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(90*3.14159/180, Eigen::Vector3d::UnitZ());//-90
    Eigen::Quaterniond Q = rollAngle * pitchAngle * yawAngle;
    m_imuTobaselink = Q.toRotationMatrix();
  }

  ~GPSTrans(){}
  //mission over
  void receiveFinish(const std_msgs::Bool::ConstPtr& msg)
  {
    mFinish = msg->data;//if mission is over,mark true;otherwise false.
  }

  void receiveRobotGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    if(mGoalPos.front().mGetGPS)//if we got goal gps, then we can receive robot gps and chcek tf
    {
      if(mls->waitForTransform(mMapFrame, mRobotFrame, ros::Time::now(), ros::Duration(0.1)))
      {
        mGoalPos.front().mGetGPS = false;
        mRobotPos.mGetGPS = true;
        double lat = msg->latitude;
        double lon = msg->longitude;
        LatLonToUTMXY(lat/180.0*M_PI, lon/180.0*M_PI, 50, mRobotPos.mUTMCoor);

        mls->lookupTransform(mMapFrame,mRobotFrame,ros::Time(0), mMaptoRobotTrans);
        PublishGoalGPS();
        ROS_INFO("GPStoTF-->get Robot GPS");
      }
      else
        ROS_WARN("GPStoTF-->cont get map to robot link");
    }
  }
  //gps goal points
  void receiveGoalGPS(const nav_msgs::Path::ConstPtr& msg)
  {
    Position temp;
    mGoalPos.clear();
    for(size_t i = 0; i < msg->poses.size(); i++)
    {
      temp.mGetGPS = true;
      double lat = msg->poses.at(i).pose.position.x;//latitude
      double lon = msg->poses.at(i).pose.position.y;//longitude
      LatLonToUTMXY(lat/180.0*M_PI, lon/180.0*M_PI, 50, temp.mUTMCoor);
      mGoalPos.push_back(temp);
    }
    mGoalInd = 0;//index of goal;
  }

  void receiveIMU(const sensor_msgs::Imu::ConstPtr& msg)
  {
    mRobotPos.mGetIMU = true;
    Eigen::Quaterniond q;
    q.w() = msg->orientation.w;
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
    //将IMU数据转换到Lidar坐标系下
    Eigen::Matrix3d temp = Eigen::Matrix3d::Identity();
    temp.block<3,3>(0,0) = q.toRotationMatrix();
    Eigen::Matrix3d t_3d = temp*m_imuTobaselink;
    Eigen::Quaterniond quat(t_3d);
    mRobotPos.mIMUdata.orientation.w = quat.w();
    mRobotPos.mIMUdata.orientation.x = quat.x();
    mRobotPos.mIMUdata.orientation.y = quat.y();
    mRobotPos.mIMUdata.orientation.z = quat.z();
  }

  void GoalGPSImpl()
  {
    //if we have mission goals but planner is idle;
    if((mFinish || mGoalInd == 0) && mGoalInd < mGoalPos.size() && mRobotPos.mGetGPS)
    {
      mFinish = false;
      mGoalPos.at(mGoalInd).mGetGPS = false;
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = mMapFrame;
      goal.pose.orientation.x = 0;
      goal.pose.orientation.y = 0;
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 1;
      //change to robot frame whitout rotation
      goal.pose.position.x = mGoalPos.at(mGoalInd).mUTMCoor.x - mRobotPos.mUTMCoor.x
                                                              + mMaptoRobotTrans.getOrigin().x();
      goal.pose.position.y = mGoalPos.at(mGoalInd).mUTMCoor.y - mRobotPos.mUTMCoor.y
                                                              + mMaptoRobotTrans.getOrigin().y();
      mGoalGPSpublisher.publish(goal);
      ROS_INFO("GPStoTF-->X:%f,Y:%f",goal.pose.position.x,goal.pose.position.y);
      mGoalInd ++;
    }

    if(mGoalInd == mGoalPos.size())
    {
      mRobotPos.mGetGPS = false;
    }
  }

  void PublishGoalGPS()
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = mMapFrame.c_str();
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.ns = "GPSgoal_point";
    SetMarkerData(&marker,
                  0, 0, 0,
                  0, 0, 0, 1,
                  0.5, 0.5, 0.5,
                  1, 0, 0, 1);
    marker.id = 0;
    marker.points.clear();

    std::vector<Position>::iterator it;
    geometry_msgs::Point temp;
    for(it = mGoalPos.begin();it!=mGoalPos.end();it++)
    {
      temp.x = it->mUTMCoor.x - mRobotPos.mUTMCoor.x + mMaptoRobotTrans.getOrigin().x();
      temp.y = it->mUTMCoor.y - mRobotPos.mUTMCoor.y + mMaptoRobotTrans.getOrigin().y();
      temp.z = 0;
      marker.points.push_back(temp);
    }
    mGoalMarkerpublisher.publish(marker);
  }

  void SetMarkerData(visualization_msgs::Marker* marker,
                     double px, double py, double pz, double ox, double oy, double oz, double ow,
                     double sx, double sy, double sz, double r, double g, double b, double a)
  {
    marker->pose.position.x = px;
    marker->pose.position.y = py;
    marker->pose.position.z = pz;
    marker->pose.orientation.x = ox;
    marker->pose.orientation.y = oy;
    marker->pose.orientation.z = oz;
    marker->pose.orientation.w = ow;

    marker->scale.x = sx;
    marker->scale.y = sy;
    marker->scale.z = sz;

    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker->color.a = a;
  }

private:
  struct Position
  {
    UTMCoor mUTMCoor;
    sensor_msgs::Imu mIMUdata;
    bool mGetGPS;
    bool mGetIMU;
  };

  ros::Subscriber mGPSsubscriber;
  ros::Subscriber mIMUsubscriber;
  ros::Subscriber mGoalGPSsubscriber;
  ros::Subscriber mFinishSubscriber;

  ros::Publisher mGoalGPSpublisher;
  ros::Publisher mGoalMarkerpublisher;

  tf::TransformListener* mls;
  tf::StampedTransform mMaptoRobotTrans;
  std::string mMapFrame;
  std::string mRobotFrame;

  std::vector<Position> mGoalPos;
  int mGoalInd;
  Position mRobotPos;
  bool mFinish;
  int mExeCount;

  Eigen::Matrix4d mMapToWorld;
  Eigen::Matrix3d m_imuTobaselink;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GPStoTF");
  ros::NodeHandle n;

  tf::TransformListener ls;
  GPSTrans gps(ls);

  ros::Rate loop(5);
  //sleep 60s before imu started
  // ros::Duration(0.25).sleep();
//  sleep(60);
  while(ros::ok())
  {
    gps.GoalGPSImpl();

    ros::spinOnce();
    loop.sleep();
  }
  ros::spin();

  return 0;
}
