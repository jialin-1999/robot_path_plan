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

    //???????????????????????????free????????????mRobotRadius mInflationRadius??????????????????????????????????????????!
    double mInflationRadius;  //????????????????????? ???
    double mRobotRadius;  //??????????????? ???
    unsigned int mCellInflationRadius;  //???????????????????????????costmap????????????
    unsigned int mCellRobotRadius;  //?????????????????????costmap????????????

    char mCostObstacle;
    char mCostLethal;
    char mCostRayTrack;  // ??????????????????????????????

    double mNavigationGoalDistance;    // goal?????????????????? ????????? ???
    double mNavigationGoalAngle;       // goal?????????????????? ????????? rad
    double mNavigationHomingDistance;  // ??????goal?????????????????????cmd????????????
    double mMinReplanningPeriod;       // ??????3?????????mapserver????????????????????????????????????????????????costmap???mCurrentPlan???
    double mMaxReplanningPeriod;

    double mUpdateFrequency;  // ???????????????????????????????????????
    double mRasterSize;
    double mDebug_show_height;
    double mTotalDistance;    //

    costmap_2d::Costmap2DROS* mLocalMap;
    costmap_2d::Costmap2D* mCostmap;
    MapInflationTool mLocalInflationTool;

    GridMap mLocalCurrentMap;
    GridMap mLocalLastMap;//mLocalCurrentMap???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????map
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
