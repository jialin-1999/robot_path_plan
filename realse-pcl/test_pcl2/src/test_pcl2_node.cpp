#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
#include <stdio.h>
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
#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <pcl/segmentation/sac_segmentation.h>


static const std::string pointstopic = "/camera/depth/points_trans";//设置topic名称

ros::Publisher points_pub;//定义ROS消息发布器
/*以下为滤波函数，首先需要将sensor_msg::PointCloud2格式转化成pcl::PCLPointCloud2格式，然后再使用VoxelGrid滤波 */
void pointsdo(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PCLPointCloud2 cloud,cloud_out;
    pcl_conversions::toPCL(*msg,cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud,*temp_cloud);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // 创建一个分割器
   pcl::SACSegmentation<pcl::PointXYZ> seg;
    //pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setOptimizeCoefficients (true);
    // Mandatory-设置目标几何形状
    seg.setModelType (pcl::SACMODEL_PLANE);
    //分割方法：随机采样法
    seg.setMethodType (pcl::SAC_RANSAC);
    //设置误差容忍范围，也就是我说过的阈值
    seg.setDistanceThreshold (0.01);
//    seg.setMaxIterations(500);
//    seg.setAxis(Eigen::Vector3f(0, 0, 1));
//    seg.setEpsAngle(45.0f * (M_PI / 180.0f));
    //输入点云
    seg.setInputCloud (temp_cloud);
    //分割点云
    seg.segment (*inliers, *coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*temp_cloud, *inliers,*inlierPoints);

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;
    std::cerr << "ax" << seg.getAxis() << std::endl;

    pcl::toPCLPointCloud2(*inlierPoints,cloud_out);
    sensor_msgs::PointCloud2 point_output;

    pcl_conversions::fromPCL(cloud_out, point_output);
    points_pub.publish(point_output);

  }

int main(int argc, char** argv)
  {
     ros::init(argc, argv, "realcam_node");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe (pointstopic, 1, pointsdo);
     points_pub=nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_filter", 1);
     ros::spin();
     return 0;
  }
