#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>


#include <std_msgs/Float64MultiArray.h>

class TopGuide
{
public:
  TopGuide(tf::TransformListener& listener):mListener(&listener)
  {
    ros::NodeHandle n;
    mGoalSubscriber = n.subscribe<geometry_msgs::PoseStamped>
        ("waypoint_pub", 1, &TopGuide::receiveGoal,this);
    mTrackTargetSubscriber = n.subscribe<std_msgs::Float64MultiArray>
        ("/target",10,&TopGuide::receiveTrackTarget,this);
    mFinishSubscriber = n.subscribe<std_msgs::Bool>("mission_finish",10,&TopGuide::receiveFinish,this);
    mLocalmapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("localGridMap",1,
                                                               &TopGuide::receiveLocalmap,this);

    mTargetPublisher = n.advertise<geometry_msgs::Pose2D>("target_pose",10);
    mGuideLinePublisher = n.advertise<visualization_msgs::Marker>("guide_line",10);

    n.param("map_frame", mMapFrame, std::string("map"));
    n.param("robot_frame", mRobotFrame, std::string("base_link"));
    mFinish = true;
    mGoal.pose.position.x = 0;
    mGoal.pose.position.y = 0;
    mGoal.pose.orientation.x = 0;
    mGoal.pose.orientation.y = 0;
    mGoal.pose.orientation.z = 0;
    mGoal.pose.orientation.w = 1;
  }

  ~TopGuide(){}

  //track goal
  void receiveTrackTarget(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    mGoal.pose.position.x = msg->data[0];
    mGoal.pose.position.y = msg->data[1];
    mGoal.header.frame_id = mRobotFrame;
    mFinish = false;
  }
  //Rviz goal test or gps goal
  void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    mGoal.pose.position.x = msg->pose.position.x;
    mGoal.pose.position.y = msg->pose.position.y;
    mGoal.pose.orientation = msg->pose.orientation;
    mGoal.header.frame_id = msg->header.frame_id;
    mFinish = false;
  }

  void receiveLocalmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    mLocalmapX = (msg->info.width-3)*msg->info.resolution;
    mLocalmapY = (msg->info.height-3)*msg->info.resolution;
  }

  void receiveFinish(const std_msgs::Bool::ConstPtr& msg)
  {
    mFinish = msg->data;
    ROS_INFO("hhhhhhhhhh");
  }

  void GetLocalTarget()
  {
    if(!mFinish)
    {
      mFinish = true;
      tf::StampedTransform trans;
      try
      {
          mListener->lookupTransform(mMapFrame, mRobotFrame, ros::Time(0), trans);
      }
      catch(tf::TransformException ex)
      {
          ROS_ERROR("Could not get target position: %s", ex.what());
          return ;
      }
      //frame change to robot
      geometry_msgs::Pose2D local_goal;
      if(mGoal.header.frame_id == mMapFrame)
      {
        local_goal.x = mGoal.pose.position.x - trans.getOrigin().x();
        local_goal.y = mGoal.pose.position.y - trans.getOrigin().y();
      }
      else if(mGoal.header.frame_id == mRobotFrame)
      {
        local_goal.x = mGoal.pose.position.x;
        local_goal.y = mGoal.pose.position.y;
      }

      if(fabs(local_goal.x) >= mLocalmapX/2 || fabs(local_goal.y) >= mLocalmapY/2)
      {
        if(fabs(local_goal.x)<0.01)
        {
          if(local_goal.y > 0)
            mLocalGoal.y = mLocalmapY/2;
          else
            mLocalGoal.y = -mLocalmapY/2;
          mLocalGoal.x = 0;
        }
        else
        {
          double scale = local_goal.y/local_goal.x;
          if(fabs(scale) > mLocalmapY/mLocalmapX)
          {
            if(local_goal.y > 0)
              mLocalGoal.y = mLocalmapY/2;
            else
              mLocalGoal.y = -mLocalmapY/2;
            mLocalGoal.x = mLocalGoal.y/scale;
          }
          else
          {
            if(local_goal.x > 0)
              mLocalGoal.x = mLocalmapX/2;
            else
              mLocalGoal.x = -mLocalmapX/2;
            mLocalGoal.y = mLocalGoal.x * scale;
          }
        }
        //frame id change to map
        mLocalGoal.x = mLocalGoal.x + trans.getOrigin().x();
        mLocalGoal.y = mLocalGoal.y + trans.getOrigin().y();
        mLocalGoal.theta = 0;
      }
      else
      {
        mLocalGoal.x = mGoal.pose.position.x;
        mLocalGoal.y = mGoal.pose.position.y;
        mLocalGoal.theta = tf::getYaw(mGoal.pose.orientation);
      }
      PublishGuideLine(trans.getOrigin().x(),trans.getOrigin().y(),mGoal.header.frame_id);
//      ROS_INFO("se_target-->X:%f,Y:%f",mLocalGoal.x,mLocalGoal.y);
      mTargetPublisher.publish(mLocalGoal);
    }
  }

private:
  void PublishGuideLine(double originX,double originY,std::string goal_frame)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = mMapFrame.c_str();
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.ns = "line";
    marker.id = 0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.1;
    marker.scale.y = 0;
    marker.scale.z = 0;

    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1;

    geometry_msgs::Point p;
    p.x = originX;
    p.y = originY;
    p.z = 0;
    marker.points.push_back(p);
    if(goal_frame == mMapFrame)
    {
      p.x = mGoal.pose.position.x;
      p.y = mGoal.pose.position.y;
    }
    else if(goal_frame == mRobotFrame)
    {
      p.x = mGoal.pose.position.x + originX;
      p.y = mGoal.pose.position.y + originY;
    }

    marker.points.push_back(p);
    marker.id++;
    mGuideLinePublisher.publish(marker);
  }
private:
  ros::Subscriber mGoalSubscriber;
  ros::Subscriber mFinishSubscriber;
  ros::Subscriber mLocalmapSubscriber;
  ros::Subscriber mTrackTargetSubscriber;
  ros::Publisher mTargetPublisher;
  ros::Publisher mGuideLinePublisher;

  geometry_msgs::PoseStamped mGoal;
  geometry_msgs::Pose2D mLocalGoal;

  bool mFinish;
  double mLocalmapX,mLocalmapY;
  tf::TransformListener* mListener;

  std::string mMapFrame;
  std::string mRobotFrame;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SetTarget");
  ros::NodeHandle n;
  tf::TransformListener Listener;
  TopGuide g(Listener);

  ros::Rate looprate(15);
  while(ros::ok())
  {
    g.GetLocalTarget();

    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}
