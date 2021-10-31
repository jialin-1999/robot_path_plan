#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>
#include <csc_nav2d_navigator/GridMap.h>
#include <csc_nav2d_navigator/commands.h>
#include <csc_nav2d_navigator/MapInflationTool.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <costmap_2d/costmap_2d_ros.h>

#include <queue>

class localRobotNavigator
{
public:
    #define MIN_ANGLE 0.03
    localRobotNavigator();
    ~localRobotNavigator();

    bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void receiveMoveGoal(const geometry_msgs::Pose2DConstPtr &msg);
    void receiveWaypoint(const nav_msgs::PathConstPtr &msg);
    int waypoint_ind;
    int waypoint_count;
    std::vector<geometry_msgs::PoseStamped> waypoint_list;

private:
    bool setLocalCurrentPosition();
    bool getLocalMap(double originX, double originY, unsigned int width, unsigned int height);//xjh add
    void stop();
    void pause();
    bool generateLocalNearTargetCommand(double goal_theta, double targetAngleTolerance);
    bool generateLocalCommand();
    bool createLocalMap();
    bool createLocalPlan();
    bool createLocalGoal(double worldInMeterX, double worldInMeterY);
    void publishLocalPlan();
    void publishDJGridMap();
    bool RayTrackisClean(unsigned int endPointIndex);

    // Everything related to ROS
    tf::TransformListener* mTfListener;

    ros::Subscriber mGoalSubscriber;
    ros::Publisher mLocalPlanPublisher;
    ros::Publisher mCommandPublisher;
    ros::Publisher mLocalGridMapPublisher;
    ros::Publisher mLocalDJGridMapPublisher;
    ros::Publisher mLocalMarkerPublisher;
    ros::Publisher mLocalDirectionMarkerPublisher;
    ros::Publisher mFinishPublisher;
    ros::ServiceServer mStopServer;
    ros::ServiceServer mPauseServer;

    ros::Subscriber mWayPointSubscriber;
    ros::Publisher mWayPointPublisher;

    std::string mMapFrame;
    std::string mRobotFrame;

    // Current status and goals
    bool mIsPaused;
    bool mIsStopped;
    bool mIsNavPaused;
    int mLocalPlanMissCounter;
    int mStatus;
    bool mLocalHasNewMap;

    unsigned int mLocalGoalPoint;
    unsigned int mLocalStartPoint;

    //在计算地图点是否是free时会用到mRobotRadius mInflationRadius，要求机器人半径比障碍半径小!
    double mInflationRadius;  //障碍物膨胀半径 米
    double mRobotRadius;  //机器人半径 米
    unsigned int mCellInflationRadius;  //障碍物膨胀半径对应costmap的格子数
    unsigned int mCellRobotRadius;  //机器人半径对应costmap的格子数

    char mCostObstacle;
    char mCostLethal;
    char mCostRayTrack;  // 用于优化局部路径规划

    double mNavigationGoalDistance;    // goal点的容忍误差 距离值 米
    double mNavigationGoalAngle;       // goal点的容忍误差 角度值 rad
    double mNavigationHomingDistance;  // 到达goal周围时，发布的cmd不带避障
    double mMinReplanningPeriod;       // 默认3秒，从mapserver获得新全局地图，基于全局地图计算costmap（mCurrentPlan）
    double mMaxReplanningPeriod;

    double mUpdateFrequency;  // 导航规划算法的时间限制频率
    double mRasterSize;
    double mDebug_show_height;
    double mTotalDistance;    //

    costmap_2d::Costmap2DROS* mLocalMap;
    costmap_2d::Costmap2D* mCostmap;
    MapInflationTool mLocalInflationTool;

    GridMap mLocalCurrentMap;
    GridMap mLocalLastMap;//mLocalCurrentMap更新后并不代表就能找到局部路径，当局部路径查找失败时，还想让车体运动，就必须保存最近一次规划成功的map
    double* mLocalCurrentPlan;
    double* mLocalLastPlan;
    std::vector<unsigned int> mLocalPlanPointsInWorldCell;

    double mSpeedMin;
    double mSpeedMax;
    double mRotateThresholdDeg;
    double mRayTrackSafetyValue;
    double mPathFollowRadius;
    double mLastPathPenalty;

    double mCurrentDirection;
    double mlast_angle;
};
