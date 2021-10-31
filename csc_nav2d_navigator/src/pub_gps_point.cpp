#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<serial/serial.h>
#include<string>

using namespace std;
class pub_gps_point
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher gps_points_pub;
        ros::Subscriber gps_point_sub;
        nav_msgs::Path gps_points;

        /* data */
    public:

        pub_gps_point(/* args */)
        {
            
            gps_points_pub = nh_.advertise<nav_msgs::Path>("goalGPS",1);
            gps_point_sub = nh_.subscribe("/gps_points", 1, &pub_gps_point::gps_point_CB, this);
            
        };

        void gps_point_CB(const geometry_msgs::PoseConstPtr &msg)
        {
            geometry_msgs::PoseStamped  gps_point;
            gps_point.pose.position.x = msg -> position.x;
            gps_point .pose.position.y = msg -> position.y;
            gps_points.poses.push_back(gps_point);
            gps_points_pub.publish(gps_points);
            ROS_INFO("accept point");
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_gps_point");
    pub_gps_point pub_gps_points;
    ros::spin();
}