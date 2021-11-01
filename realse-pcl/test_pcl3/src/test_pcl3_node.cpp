#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <dynamic_reconfigure/server.h>
#include "test_pcl3/My_cfgConfig.h"

/*
  以下是pointcloud2转化的必要的头文件
*/
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros//point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <unistd.h>
static const std::string pointstopic ="/camera1/depth/color/points";
using namespace Eigen;
ros::Publisher points_pub;
ros::Publisher points_pub1;
double yaw2,pitch2,roll2;
float x_p,y_p,z_p;

tf::TransformBroadcaster* g_br;
tf::StampedTransform tf_camera_op2camera_link;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZ>);

void callback(test_pcl3::My_cfgConfig &config)
{
  x_p=config.x1_param;
  y_p=config.y1_param;
  z_p=config.z1_param;
  roll2=config.roll_param;
  pitch2=config.pitch_param;
  yaw2=config.yaw_param;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> v_cloud;
  pcl::PointCloud<pcl::PointXYZ> v_cloud_b;
  pcl::fromROSMsg(*msg, v_cloud);
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(v_cloud, v_cloud,index);
  pcl::VoxelGrid<pcl::PointXYZ> sor_cur;
  sor_cur.setInputCloud(v_cloud.makeShared());
  sor_cur.setLeafSize(0.1, 0.1, 0.1);
  sor_cur.filter(v_cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for(it = v_cloud.points.begin();it != v_cloud.points.end();it++)
  {
    if(atan2(it->y,it->x) > 3.14*3/4.0||atan2(it->y,it->x) < -3.14*3/4.0)
    {
      it->x = 10;
      it->y = 10;
      it->z = 10;
    }
      
  }

  // v_cloud += *cloud_trans;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(v_cloud.makeShared());
  sor.setMeanK(50);//50个临近点
  sor.setStddevMulThresh(1.0);//距离大于1倍标准方差

  sor.filter(v_cloud);

  // v_cloud.width = v_cloud.points.size()+100;
  // v_cloud.height = 1;
  // v_cloud.points.resize(v_cloud.width*v_cloud.height);
  pcl::PointXYZ p;
  for(int i =0;i<1000;i++)
  {
    p.x = cos(3.14*0.70+i/1000.0*3.14/1.7)*20;
    p.y = sin(3.14*0.70+i/1000.0*3.14/1.7)*20;
    p.z = 0;
    v_cloud_b.points.push_back(p);
  }
  
  v_cloud += v_cloud_b;

  Eigen::Matrix4f tf_bas2link=Eigen::Matrix4f::Identity();

  Eigen::Vector3f    ea0(roll2,pitch2,yaw2);
  Eigen::Vector3f    ea1(x_p,y_p,z_p);
  Eigen::Quaternionf q=Eigen::AngleAxisf(ea0[2], ::Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(ea0[1], ::Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(ea0[0], ::Eigen::Vector3f::UnitX());
  Eigen::Matrix3f    R_rot=q.normalized().matrix();

  tf_bas2link.block<3,3>(0,0)=R_rot;
  tf_bas2link.block<3,1>(0,3)=ea1.transpose();
  Eigen::Quaternionf eigen_quat(tf_bas2link.block<3,3>(0,0));
  Eigen::Vector3f eigen_trans(tf_bas2link.block<3,1>(0,3));
  tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
  tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));

  tf::Transform transform(tf_quat, tf_trans);
  g_br->sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","velodyne"));

  pcl::transformPointCloud(v_cloud, v_cloud_b,tf_bas2link);

  sensor_msgs::PointCloud2 point_out;
  
  pcl::toROSMsg(v_cloud_b, point_out);
  point_out.header.frame_id = "base_link";
  points_pub.publish(point_out);
}

void pointsdo(const sensor_msgs::PointCloud2ConstPtr &msg)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trgb (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(*msg, *cloud);

  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud,index);
  pcl::VoxelGrid<pcl::PointXYZ> sor_cur;
  sor_cur.setInputCloud(cloud);
  sor_cur.setLeafSize(0.05, 0.05, 0.05);
  sor_cur.filter(*cloud);

  Eigen::Translation3f tl_op2link(tf_camera_op2camera_link.getOrigin().getX(), tf_camera_op2camera_link.getOrigin().getY(), tf_camera_op2camera_link.getOrigin().getZ());
  double roll, pitch, yaw;

  tf::Matrix3x3(tf_camera_op2camera_link.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf  rot_x_op2link(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf  rot_y_op2link(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf  rot_z_op2link(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f    tf_op2link = (tl_op2link * rot_z_op2link * rot_y_op2link * rot_x_op2link).matrix();
  Eigen::Matrix4f    tf_link2op = tf_op2link.inverse();

  Eigen::Matrix4f tf_bas2link=Eigen::Matrix4f::Identity();

  Eigen::Vector3f    ea0(roll2,pitch2,yaw2);
  Eigen::Vector3f    ea1(x_p,y_p,z_p);
  Eigen::Quaternionf q=Eigen::AngleAxisf(ea0[2], ::Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(ea0[1], ::Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(ea0[0], ::Eigen::Vector3f::UnitX());
  Eigen::Matrix3f    R_rot=q.normalized().matrix();

  tf_bas2link.block<3,3>(0,0)=R_rot;
  tf_bas2link.block<3,1>(0,3)=ea1.transpose();

  Eigen::Matrix4f tf_bas2op=tf_bas2link*tf_link2op;

  Eigen::Quaternionf eigen_quat(tf_bas2link.block<3,3>(0,0));
  Eigen::Vector3f eigen_trans(tf_bas2link.block<3,1>(0,3));
//  Eigen::Quaternionf eigen_quat(tf_bas2op.block<3,3>(0,0).cast());
//  Eigen::Vector3f eigen_trans(tf_bas2op.block<3,1>(0,3).cast());
  tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
  tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));

  tf::Transform transform(tf_quat, tf_trans);
  g_br->sendTransform(tf::StampedTransform(transform,ros::Time::now(),"velodyne","camera1_link"));


  pcl::transformPointCloud(*cloud, *cloud_trans,tf_bas2op);

  // Create the filtering object
    
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_trans);
    // sor.setMeanK(20);//50个临近点
    // sor.setStddevMulThresh(1.0);//距离大于1倍标准方差

    // sor.filter(*cloud_inliner);

  //   cloud_trgb->points.resize(cloud_trans->size());
  //   for(int num=0;num<cloud_trans->points.size();num++)
  //   {
  //     cloud_trgb->points[num].x=cloud_trans->points[num].x;
  //     cloud_trgb->points[num].y=cloud_trans->points[num].y;
  //     cloud_trgb->points[num].z=cloud_trans->points[num].z;
  //     if(0<=cloud_trans->points[num].z && cloud_trans->points[num].z<=0.3)
  //     {
  //       cloud_trgb->points[num].r=255;
  //       cloud_trgb->points[num].g=0;
  //       cloud_trgb->points[num].b=0;
  //     }
  //     else if(cloud_trans->points[num].z<0)
  //     {
  //       cloud_trgb->points[num].r=0;
  //       cloud_trgb->points[num].g=255;
  //       cloud_trgb->points[num].b=0;
  //     }
  //     else{
  //       cloud_trgb->points[num].r=0;
  //       cloud_trgb->points[num].g=0;
  //       cloud_trgb->points[num].b=255;
  //     }
  //   }

  // sensor_msgs::PointCloud2 point_out;
  // pcl::toROSMsg(*cloud_trans, point_out);

  // point_out.header.frame_id="velodyne";
  // points_pub1.publish(point_out);
}

int main(int argc,char** argv)
{
   ros::init(argc,argv,"test_pcl");
   ros::NodeHandle n;
   ros::Subscriber sub=n.subscribe(pointstopic,1,pointsdo);
   ros::Subscriber sub1=n.subscribe("/velodyne_points",1,cloud_cb);

   dynamic_reconfigure::Server<test_pcl3::My_cfgConfig> server;
   dynamic_reconfigure::Server<test_pcl3::My_cfgConfig>::CallbackType f;

   f = boost::bind(&callback, _1); //绑定回调函数
   server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
   sleep(5);
   ROS_INFO("Spinning node");

   tf::TransformBroadcaster br;
   g_br=&br;
   tf::TransformListener tf_listener;
   //先获得相机坐标系到矫正的camera_link坐标系的tf
   tf_listener.waitForTransform("camera1_depth_optical_frame", "camera1_link", ros::Time(0), ros::Duration(1.0));
   tf_listener.lookupTransform("camera1_depth_optical_frame", "camera1_link", ros::Time(0), tf_camera_op2camera_link);


  //  points_pub1=n.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_trans",1);
   points_pub=n.advertise<sensor_msgs::PointCloud2>("/cloud_f",1);
   ros::spin();
   return 0;
}



