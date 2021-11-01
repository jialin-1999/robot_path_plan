1、先安装realsense的驱动，具体可参考https://gitee.com/csc105/dashboard/projects/sensors_and_external_devices_drive/realsense_modify/code/

2、从上述位置克隆相应的功能包，主要是用realsense_modify/realsense2_camera/launch/low_cpu_2pointcloud.launch 只开启点云（最低分辨率，6HZ）。cp占用为15%.

3、这里对launch文件进行了一点修改，主要是原先使用的是两个相机，arg需要调整

4、先通过roslaunch realsense2_camera low_cpu_2pointcloud.launch 开启点云， 话题为/camera1/depth/color/points，数据类型为sensor_msgs::PointCloud2，基于参考系为/camera1_depth_optical_frame

5、相机自身的坐标系下z轴朝前，y轴朝下，x轴朝右，通过/camera1_depth_optical_frame与/camera1_link的tf进行初步的矫正，使z轴朝上，x轴朝前，y轴朝左

6、rosrun test_pcl3 test_pcl3_node 

7、获得基于地面的点云，话题为/camera/depth/points_trans，数据类型为sensor_msgs::PointCloud2，基于参考系为/base_link,可以通过base_link与camera1_depth_optical_frame的tf关系获得地面点云和相机点云的位姿变换


总流程：
roslaunch realsense2_camera low_cpu_2pointcloud.launch
rosrun test_pcl3 test_pcl3_node
 

可以在开启上述节点的情况下，
rosrun rqt_reconfigure rqt_reconfigure
来动态调节地面和矫正过的相机坐标/camera1_link之间的tf
