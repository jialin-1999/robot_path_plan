#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <csc_nav2d_navigator/localRobotNavigator.h>

#include <set>
#include <map>

#define PI 3.14159265

using namespace ros;
using namespace tf;
using namespace std;

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

/**
 * @brief localRobotNavigator::localRobotNavigator
 * 总的导航算法
 */
localRobotNavigator::localRobotNavigator()
{	
    NodeHandle robotNode;

    mLocalGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localGridMap",1);
    mLocalDJGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localDJGridMap",1);
    mCommandPublisher = robotNode.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    mGoalSubscriber = robotNode.subscribe<geometry_msgs::Pose2D>("target_pose", 10, &localRobotNavigator::receiveMoveGoal, this);
    mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &localRobotNavigator::receiveStop, this);
    mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &localRobotNavigator::receivePause, this);
    mFinishPublisher = robotNode.advertise<std_msgs::Bool>("mission_finish",10);
    mWayPointSubscriber = robotNode.subscribe<nav_msgs::Path>("waypoint",1,&localRobotNavigator::receiveWaypoint,this);
    mWayPointPublisher = robotNode.advertise<geometry_msgs::PoseStamped>("waypoint_pub",10);
    // Get global parameters
    robotNode.param("map_frame", mMapFrame, std::string("map"));
    robotNode.param("robot_frame", mRobotFrame, std::string("base_link"));


    NodeHandle navigatorNode("~/");
    mLocalPlanPublisher = navigatorNode.advertise<sensor_msgs::PointCloud>("localPlan", 5);
    mLocalMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("localGoalMarkers", 1, true);
    mLocalDirectionMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("LocalDirectionMarkers", 1, true);

    // Get private parameters
    navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
    navigatorNode.param("robot_radius", mRobotRadius, 0.4);
    navigatorNode.param("navigation_goal_distance", mNavigationGoalDistance, 1.0);
    navigatorNode.param("navigation_goal_angle", mNavigationGoalAngle, 0.1);
    navigatorNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);
    navigatorNode.param("SpeedMax", mSpeedMax, 0.6);
    navigatorNode.param("SpeedMin", mSpeedMin, 0.3);
    navigatorNode.param("rotate_threshold_deg", mRotateThresholdDeg, 60.0);
    navigatorNode.param("RayTrackSafetyValue", mRayTrackSafetyValue, 0.2);
    navigatorNode.param("pathFollowRadius", mPathFollowRadius, 1.25);
    navigatorNode.param("last_path_penalty", mLastPathPenalty, 1.0);

    mTfListener = new tf::TransformListener(robotNode);
    // 创建一个costmap
    mLocalMap = new costmap_2d::Costmap2DROS("local_costmap", *mTfListener);
    mRasterSize = mLocalMap->getCostmap()->getResolution();

    mCellInflationRadius = mInflationRadius / mRasterSize;  //变成了格子的占用数
    mCellRobotRadius = mRobotRadius / mRasterSize;
    mLocalInflationTool.computeCaches(mCellInflationRadius);

    mCostObstacle = 100;
    mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;  //阈值，用于判断某个地图点是否是free，必须要求机器人半径小于膨胀半径！
    mCostRayTrack = mCostLethal * mRayTrackSafetyValue;

    // Apply tf_prefix to all used frame-id's
    mRobotFrame = mTfListener->resolve(mRobotFrame);
    mMapFrame = mTfListener->resolve(mMapFrame);

    mLocalCurrentPlan = NULL;
    mLocalLastPlan = NULL;

    mIsStopped = false;
    mIsPaused = false;
    mIsNavPaused = false;
    mStatus = NAV_ST_IDLE;
    mCellInflationRadius = 0;
    mLocalPlanMissCounter = 0;
    mDebug_show_height = 0.0;
    mUpdateFrequency = 10;
    mlast_angle = 0;
    waypoint_ind = 0;


}
  void localRobotNavigator::receiveWaypoint(const nav_msgs::PathConstPtr &msg)
  {
    waypoint_list.clear();
    geometry_msgs::PoseStamped temp_pose;
    waypoint_count =msg->poses.size();
    ROS_INFO("Received waypoint");
    ROS_INFO("%d",waypoint_count);
    for(int i = 0; i < msg->poses.size(); i++)
    {
      temp_pose.header.frame_id = msg->header.frame_id;
      temp_pose.header.seq = msg ->header.seq;
      temp_pose.header.stamp = msg->header.stamp;
      temp_pose.pose.position.x =msg->poses.at(i).pose.position.x;
      temp_pose.pose.position.y =msg->poses.at(i).pose.position.y;
      waypoint_list.push_back(temp_pose);
    }
    if(waypoint_count==1)
    {
        
        mWayPointPublisher.publish(waypoint_list[0]);
        ROS_INFO("jhsjkla");
    }   
  }
localRobotNavigator::~localRobotNavigator()
{
    delete[] mLocalCurrentPlan;
    delete[] mLocalLastPlan;
}

/**
 * @brief localRobotNavigator::receiveStop
 * 停止标志，可配合手柄
 * @param req
 * @param res
 * @return
 */
bool localRobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    mIsStopped = true;
    res.success = true;
    res.message = "Navigator received stop signal.";
    return true;
}

/**
 * @brief localRobotNavigator::receivePause
 * 暂停，按一次暂停导航，再按一次继续导航。
 * @param req
 * @param res
 * @return
 */
bool localRobotNavigator::receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if(mIsPaused)
    {
        mIsPaused = false;
        res.success = false;
        res.message = "Navigator continues.";
    }else
    {
        mIsPaused = true;
        geometry_msgs::Twist stopMsg;
        stopMsg.linear.x = 0;
        stopMsg.angular.z = 0;
        mCommandPublisher.publish(stopMsg);
        res.success = true;
        res.message = "Navigator pauses.";
    }
    return true;
}

/**
 * @brief localRobotNavigator::getLocalMap
 * 基于机器人当前位置，局部costmap，全局mCurrentMap，
 * 在局部costmap 范围 中搜索mCurrentMap中的最小值，如果这个值没有被占用设为目标最小，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
 * 如果被占用了，则搜索costmap 范围以外的一个mCurrentMap中的最小值，设为目标，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
 * @param originX
 * @param originY
 * @param widthInCostMap
 * @param heightInCostMap
 * @return 输出 bool mLocalHasNewMap; GridMap mLocalCurrentMap; double* mLocalCurrentPlan;
 */
bool localRobotNavigator::getLocalMap(double originX, double originY, unsigned int widthInCostMap, unsigned int heightInCostMap)
{
    mCostmap = mLocalMap->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

    //ROS_INFO("originX:%f, originY:%f, widthInCell:%d, heightInCell:%d", originX,originY,widthInCostMap,heightInCostMap);

    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(widthInCostMap*heightInCostMap);
    tempMap.info.height = heightInCostMap;
    tempMap.info.width = widthInCostMap;
    tempMap.info.resolution = mCostmap->getResolution();
    tempMap.info.origin.position.x = originX;
    tempMap.info.origin.position.y = originY;
    tempMap.info.origin.position.z = mDebug_show_height;
    tempMap.header.frame_id = mMapFrame.c_str();

    mLocalCurrentMap.update(tempMap);

    double wx, wy;
    int idLocalX, idLocalY;

//    double costmap_originX = - mCostmap->getSizeInMetersX()/2.0;
//    double costmap_originY = - mCostmap->getSizeInMetersY()/2.0;
//    tf::StampedTransform trans;
//    try
//    {
//        mTfListener->lookupTransform("odom", "base_link", ros::Time(0), trans);
//    }
//    catch(tf::TransformException ex)
//    {
//        ROS_ERROR("Could not get robot position: %s", ex.what());
//        return false;
//    }

//    Eigen::Quaterniond qu;
//    qu.x() = trans.getRotation().getX();
//    qu.y() = trans.getRotation().getY();
//    qu.z() = trans.getRotation().getZ();
//    qu.w() = trans.getRotation().getW();

//    Eigen::Vector3d t = Eigen::Vector3d(trans.getOrigin().getX(),
//                                        trans.getOrigin().getY(),
//                                        trans.getOrigin().getZ());
    // 将costmap中的动态障碍物加入局部地图中
    for(unsigned int mx=0; mx<mCostmap->getSizeInCellsX(); mx++)
    {
        for(unsigned int my=0; my<mCostmap->getSizeInCellsY(); my++)
        {
            if(mCostmap->getCost(mx,my) == costmap_2d::LETHAL_OBSTACLE )
            {
                mCostmap->mapToWorld(mx, my, wx, wy);//这个句话是将costmap中的某个点的id坐标转化到meter坐标。全局map坐标系下
//                Eigen::Vector3d tw = Eigen::Vector3d(wx,wy,0);
//                Eigen::Vector3d t1;
//                t1 = qu.inverse() * (tw - t);
//                idLocalX = t1(0)/mLocalCurrentMap.getResolution();
//                idLocalY = t1(1)/mLocalCurrentMap.getResolution();
                idLocalX = (wx-originX)/mLocalCurrentMap.getResolution();
                idLocalY = (wy-originY)/mLocalCurrentMap.getResolution();
                mLocalCurrentMap.setData(idLocalX, idLocalY, mCostObstacle);
            }
        }
    }

    if(mLocalCurrentPlan) delete[] mLocalCurrentPlan;
    mLocalCurrentPlan = new double[mLocalCurrentMap.getSize()];

    mLocalCurrentMap.setLethalCost(mCostLethal);//刷新阈值，用于判断某个地图点是否是free

    return true;
}




/**
 * @brief localRobotNavigator::createLocalMap
 * 局部规划使用1.5倍大小的局部地图，其中叠加1倍大小的局部障碍物，换言之 靠边上的0.5倍的区域不存在局部障碍物，只存在全局障碍物 用于导航规划。
 * @return
 */
bool localRobotNavigator::createLocalMap()
{
    //必须要加这两句话 否则会返回空的costmap！
    mCostmap = mLocalMap->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

    //注意 这个是costmap中的cell 他和mapserver发布的分辨率会不同
    unsigned int localMapWidthInCell =  (mCostmap->getSizeInMetersX()*1.5 + mRobotRadius*2) / mCostmap->getResolution();
    unsigned int localMapHeightInCell = (mCostmap->getSizeInMetersY()*1.5 + mRobotRadius*2) / mCostmap->getResolution();

    tf::StampedTransform trans;
    try
    {
      mTfListener->lookupTransform(mMapFrame, mRobotFrame, ros::Time(0), trans);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Could not get robot position: %s", ex.what());
      return false;
    }

    double localMapStartInMeterX = trans.getOrigin().x() - (localMapWidthInCell) * mCostmap->getResolution() / 2.0;
    double localMapStartInMeterY = trans.getOrigin().y() - (localMapHeightInCell) * mCostmap->getResolution() / 2.0;

    // 使用costmap的动态障碍物数据构造局部地图
    if(!getLocalMap(localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell))
    {
        ROS_WARN("Could not get a new local map, trying to go with the old one...");
        return false;
    }
    //ROS_INFO("mCurrentPositionX=%f, mCurrentPositionY=%f, MeterX=%f , MeterY=%f , WidthInCell=%d, HeightInCell=%d",mCurrentPositionX,mCurrentPositionY,localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell);
    // 设置机器人在局部地图中的位置，设置mLocalStartPoint
    setLocalCurrentPosition();

    // Clear robot footprint in map
    //实际上因为全局定位会有偏差，导致在狭窄的地方，机器人被定位到了全局map的障碍物点上，导致无法找到局部goal。
    //把机器人在局部地图中的区域设置为free
    unsigned int x = 0, y = 0;
    if(mLocalCurrentMap.getCoordinates(x, y, mLocalStartPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
                mLocalCurrentMap.setData(x+i, y+j, 0);

    mLocalInflationTool.inflateMap(&mLocalCurrentMap);  // 膨胀地图
    return true;
}


/**
 * @brief localRobotNavigator::createLocalGoal
 * 在创建全局地图时会清除机器人区域，然后做全局plan计算
 * 当机器人要获得localplan时要根据全局plan找到localGoal
 * 但是由于全局plan只在初始化时会清除机器人区域，之后不会在地图中清除对应的区域，因此在做局部规划时，将当前机器人放进去算会导致在计算局部goal时卡死，周围没有空闲区域。
 * 因为全局定位难免有偏差，这个问题会出现在特别狭窄的地方。
 * @return
 */
bool localRobotNavigator::createLocalGoal(double worldInMeterX, double worldInMeterY)
{
    unsigned int goalLocalX, goalLocalY;  // 局部目标点(在局部地图中)的栅格坐标

    goalLocalX = (worldInMeterX - mLocalCurrentMap.getOriginX()) / mLocalCurrentMap.getResolution();
    goalLocalY = (worldInMeterY - mLocalCurrentMap.getOriginY()) / mLocalCurrentMap.getResolution();

    // 获得局部goal的id号，用于下一步的局部规划，即mLocalGoalPoint
    if(!mLocalCurrentMap.getIndex(goalLocalX, goalLocalY, mLocalGoalPoint))
    {
        ROS_ERROR("Couldn't get goal point index!");
        return false;
        //ROS_INFO("ready to LocalPlan! goalLocalX=%d goalLocalY=%d", goalLocalX, goalLocalY);
    }

    // 把目标点周围的区域设置为free
    unsigned int x = 0, y = 0;
    if(mLocalCurrentMap.getCoordinates(x, y, mLocalGoalPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius*2; j < (int)mCellRobotRadius*2; j++)
                mLocalCurrentMap.setData(x+i, y+j, 0);

    mLocalGridMapPublisher.publish(mLocalCurrentMap.getMap());  // 发布局部地图
    // 发布局部规划的目标点，绿色
    visualization_msgs::Marker marker;
    marker.header.frame_id = mMapFrame.c_str();
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mLocalCurrentMap.getOriginX() + ((double)goalLocalX * mLocalCurrentMap.getResolution());
    marker.pose.position.y = mLocalCurrentMap.getOriginY() + ((double)goalLocalY * mLocalCurrentMap.getResolution());
    marker.pose.position.z = mDebug_show_height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mLocalCurrentMap.getResolution() * 1.0;
    marker.scale.y = mLocalCurrentMap.getResolution() * 1.0;
    marker.scale.z = 3.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mLocalMarkerPublisher.publish(marker);

    //ROS_INFO("mLocalGoalPoint=%d, goal_x=%d, goal_y=%d", mLocalGoalPoint, goal_x, goal_y);

    return true;
}

/**
 * @brief localRobotNavigator::createLocalPlan
 * 计算局部路径
 * @return
 */
bool localRobotNavigator::createLocalPlan()
{
    Queue queue;

    // Reset the plan
    int mapSize = mLocalCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mLocalCurrentPlan[i] = -1;
    }

    if(mLocalCurrentMap.isFree(mLocalGoalPoint))
    {
        queue.insert(Entry(0.0, mLocalGoalPoint));
        mLocalCurrentPlan[mLocalGoalPoint] = 0;
    }
    else {
        //在局部地图中，这个条件一般情况不会成立
        //因为全局规划路线时就不会将被占用的点当作路径点
        //但是当靠近goal，goal处于局部costmap区域中（mLocalCurrentMap是1.5倍的costmap大小）就会出现这个问题
        //如果目标点，以及周围1m内都被障碍物占用了，这里的写法是有bug的。
        // Initialize the queue with area around the goal point

        int reach = mCellRobotRadius + (0.5 / mLocalCurrentMap.getResolution());
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(mLocalGoalPoint, reach);

        if(neighbors.size() > 0)
        {
            for(unsigned int i = 0; i < neighbors.size(); i++)
            {
                queue.insert(Entry(0.0, neighbors[i]));
                mLocalCurrentPlan[neighbors[i]] = 0;
            }
        }
        else
        {
            ROS_WARN("local goal is not free!");
            return false;//目标点附近没有空闲区域
        }

        ROS_WARN("local goal is not free! find free neighbors number %d, reach=%d", (int)neighbors.size(), reach);
        //如果goal被占用，则寻找周围车身半径+1m范围的区域内所有的空闲区域，都设置为0的代价。

        //        ROS_WARN("local goal is not free!");
        //        return false;//不修改目标点，直接规划失败
    }

    //将上一次的最短路径转化到当前map坐标下。
    std::vector<unsigned int> lastLocalPlanInd;
    for(unsigned int i=0; i<mLocalPlanPointsInWorldCell.size(); i++)
    {
        lastLocalPlanInd.push_back(mLocalPlanPointsInWorldCell[i]);
    }

    Queue::iterator next;
    double distance;
    unsigned int x, y, index;
    unsigned int start_x=0, start_y=0;

    if(!mLocalCurrentMap.getCoordinates(start_x, start_y, mLocalStartPoint)) return false;

    double linear = mLocalCurrentMap.getResolution();
    double diagonal = std::sqrt(2.0) * linear;

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        distance = next->first;
        index = next->second;
        queue.erase(next);

        //当地图中的点已经有代价，并且这个点代价比distance小（因为被其他相邻节点更新了），就不需要再更新这个节点。
        //if(mLocalCurrentPlan[index] >= 0 && mLocalCurrentPlan[index] < distance) continue;

        //注释掉这个break是为了防止车子在导航中途因为避障碍，开向其他地方后无法再找到周围的最优迭代方向
        //个人觉得在局部地图中规划时，这个break可以加
        if(index == mLocalStartPoint) break;

        // Add all adjacent cells
        if(!mLocalCurrentMap.getCoordinates(x, y, index)) continue;

        std::vector<unsigned int> ind;
        ind = mLocalCurrentMap.getNeighbors(index, true);

        // 遍历当前栅格周围的8个栅格，分别计算各自的的代价
        for(unsigned int it = 0; it < ind.size(); it++)
        {
            unsigned int i = ind[it];
            if(mLocalCurrentMap.isFree(i))  // 若该邻居栅格没有被搜索过
            {
                double delta = (it < 4) ? linear : diagonal;

                // 增加上一帧的最优路径奖励
                auto itt = find(lastLocalPlanInd.begin(),lastLocalPlanInd.end(), i);
                if (itt == lastLocalPlanInd.end())
                {
                    // 对非上一帧最优路径上的点增加代价
                    delta *= mLastPathPenalty;
                }

                double newDistance = mLocalCurrentPlan[index] + delta + \
                        (10 * delta * (double)mLocalCurrentMap.getData(i) / (double)mCostObstacle);//最后一项，在基本的DJ算法上，增加cost的代价。(double)mCurrentMap.getData(i) / (double)mCostObstacle是0-1的值，所以放大10倍

                if(mLocalCurrentPlan[i] == -1 || newDistance < mLocalCurrentPlan[i])
                {
                    if(!mLocalCurrentMap.getCoordinates(x, y, i)) continue;

                    // 增加启发函数
                    int dx = std::abs((int)x-(int)start_x);
                    int dy = std::abs((int)y-(int)start_y);
                    double heuristic = linear*(dx + dy) + (diagonal - 2*linear) * std::min(dx, dy);

                    double priority = newDistance + heuristic;

                    queue.insert(Entry(priority, i));

                    mLocalCurrentPlan[i] = newDistance;
                }
            }
        }
    }
    //DJ结束以后检查初始位置是否是可行的
    if(mLocalCurrentPlan[mLocalStartPoint] < 0)
    {
        ROS_WARN("In createLocalPlan. No way between robot and goal!");
        return false;
    }
    return true;
}

/**
 * @brief localRobotNavigator::publishLocalPlan
 * 发布局部路径
 *
 */
void localRobotNavigator::publishLocalPlan()
{
    unsigned int index = mLocalStartPoint;
    std::vector<std::pair<double, double> > points;

    mLocalPlanPointsInWorldCell.clear();
    mLocalPlanPointsInWorldCell.push_back(index);  // 局部规划的路径点(以全局地图中的索引号存储)

    double linear = mLocalCurrentMap.getResolution();  // 用于计算路径的真实长度
    double diagonal = std::sqrt(2.0) * linear;

    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
            points.push_back(std::pair<double, double>(
                                 ((x+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginX(),
                                 ((y+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginY()
                                 ));

        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        double delta = 0;  // 路径上相邻两个cell之间的距离
        unsigned int next_index = index;

        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index]) {
                next_index = neighbors[i];
                delta = (i < 4) ? linear : diagonal;
            }
        }

        mTotalDistance += delta;

        if(index == next_index) break;//这个条件，暂时想不出来有什么可能会落入
        index = next_index;

        mLocalPlanPointsInWorldCell.push_back(index);  // 局部规划的路径点(以全局地图中的索引号存储)
    }
    //这里是局部地图坐标下的点要转化到全局坐标进行发布

    sensor_msgs::PointCloud plan_msg;
    plan_msg.header.frame_id = mMapFrame.c_str();
    plan_msg.header.stamp = Time::now();

    sensor_msgs::ChannelFloat32 temp;
    geometry_msgs::Point32 temp2;
    temp.name = "intensity";

    double wx, wy;
    for(unsigned int i = 0; i < points.size(); i++)
    {
        temp2.x = points[i].first;
        temp2.y = points[i].second;
        temp2.z = mDebug_show_height;
        plan_msg.points.push_back(temp2);
        temp.values.push_back(i);
    }
    plan_msg.channels.push_back(temp);
    mLocalPlanPublisher.publish(plan_msg);
}

/**
 * @brief localRobotNavigator::RayTrackisClean
 * 射线清除法，优化局部目标点
 * 在先前的基础上 基于DJST计算出来的路径 从近处开始向远处查询，
 * 计算mLocalStartPoint到第i个路径点连成的直线，遍历这个直线上所有的栅格，如果大于某一个占用阈值，就认为失败否则就是可行点
 * @param endPointIndex
 * @return
 */
bool localRobotNavigator::RayTrackisClean(unsigned int endPointIndex)
{
    if(endPointIndex == mLocalStartPoint)
        return true;

    unsigned int startX = 0, startY = 0;
    unsigned int endX = 0, endY = 0;

    mLocalCurrentMap.getCoordinates(startX, startY, mLocalStartPoint);
    mLocalCurrentMap.getCoordinates(endX, endY, endPointIndex);

    int deltaX = abs(int(startX)-int(endX));
    int deltaY = abs(int(startY)-int(endY));
    //ROS_INFO("startX%d, startY%d endX%d, endY%d deltaX%d deltaY %d",startX, startY,endX, endY,deltaX,deltaY);
    //因为一开始排除了同一个点的情况 所以deltaX deltaY 不可能同时为0 ，所以除数不会为0
    if(deltaX > deltaY)
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaX;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaX;
        //ROS_INFO("stepX%f stepY%f", stepX, stepY);

        for(int i=1; i<= deltaX; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            //ROS_INFO("cellX %d  cellY %d, mCostRayTrack%d cell%d startCell%d ", cellX, cellY, mCostRayTrack, mLocalCurrentMap.getData(cellX, cellY), mLocalCurrentMap.getData(startX, startY));
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostRayTrack)
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaY;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaY;

        for(int i=1; i<= deltaY; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostRayTrack)
                return false;
        }
        return true;
    }
}

/**
 * @brief localRobotNavigator::publishDJGridMap
 * 调试使用，发布DJST地图
 */
void localRobotNavigator::publishDJGridMap()
{
    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(mLocalCurrentMap.getSize());
    tempMap.info.height = mLocalCurrentMap.getHeight();
    tempMap.info.width = mLocalCurrentMap.getWidth();
    tempMap.info.resolution = mLocalCurrentMap.getResolution();
    tempMap.info.origin.position.x = mLocalCurrentMap.getOriginX();
    tempMap.info.origin.position.y = mLocalCurrentMap.getOriginY();
    tempMap.info.origin.position.z = mDebug_show_height;//故意提高 看的清楚

    double max_value=0.0;
    for(int i=0; i<mLocalCurrentMap.getSize(); i++)
    {
        if(mLocalCurrentPlan[i] > max_value)
            max_value = mLocalCurrentPlan[i];
        //ROS_INFO("value = %f", mLocalCurrentPlan[i]);
    }
    //ROS_INFO("CurrentPlan max_value = %f", max_value);

    for(int id=0; id<mLocalCurrentMap.getSize(); id++)
    {
        tempMap.data[id] = (mLocalCurrentPlan[id]/max_value)*255;
    }
    mLocalDJGridMapPublisher.publish(tempMap);
}

/**
 * @brief localRobotNavigator::stop
 * 停止
 */
void localRobotNavigator::stop()
{
  geometry_msgs::Twist stopMsg;
  stopMsg.linear.x = 0;
  stopMsg.angular.z = 0;
  mCommandPublisher.publish(stopMsg);
  mStatus = NAV_ST_IDLE;
  mIsPaused = false;
  mIsStopped = false;
}

/**
 * @brief localRobotNavigator::generateLocalNearTargetCommand
 * @return
 */
bool localRobotNavigator::generateLocalNearTargetCommand(double goal_theta, double targetAngleTolerance)
{
    // Do nothing when paused
    /*if(mIsPaused || mIsNavPaused)
    {
        ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
        return false;
    }

    if(mStatus != NAV_ST_NAVIGATING )
    {
        ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
        return false;
    }*/

    double angle = goal_theta / 180.0 * PI;
    if(angle < -PI) angle += 2*PI;
    if(angle > PI) angle -= 2*PI;

    geometry_msgs::Twist msg;

    if(fabs(angle) > targetAngleTolerance)
    {
        msg.angular.z = angle;
        msg.linear.x = 0;
        mCommandPublisher.publish(msg);
        return false;
    }
    else {
        msg.angular.z = 0;
        msg.linear.x = 0;
        mCommandPublisher.publish(msg);
        return true;
    }

}
/**
 * @brief localRobotNavigator::generateLocalCommand
 * 运动输出
 * 注意，这里是使用射线跟随的方式进行局部方向优化。
 * @return
 */
bool localRobotNavigator::generateLocalCommand()
{
    // Do nothing when paused
    /*if(mIsPaused || mIsNavPaused)
    {
        ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
        return true;
    }*/

    /*if(mStatus != NAV_ST_NAVIGATING )
    {
        ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
        return false;
    }*/

    // Generate direction command from plan
    unsigned int current_x = 0, current_y = 0;
    if(!mLocalCurrentMap.getCoordinates(current_x, current_y, mLocalStartPoint)) // || !mCurrentMap.isFree(mStartPoint))
    {
        ROS_ERROR("Plan execution failed, robot not in map!");
        return false;
    }

    unsigned int targetInSize = mLocalStartPoint;
    //前瞻距离，从当前位置开始搜索，向外mPathFollowRadiusm范围内找可行的目标点。
    int steps = mPathFollowRadius / mLocalCurrentMap.getResolution();//看mPathFollowRadius 米范围内的目标点，
    //室内mPathFollowRadius比较好，狭窄环境能通过
    for(int i = 0; i < steps; i++)
    {
        unsigned int bestPoint = targetInSize;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(targetInSize);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            if(mLocalCurrentPlan[neighbors[i]] >= (unsigned int)0 && \
                    mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[bestPoint])
                bestPoint = neighbors[i];
        }
        targetInSize = bestPoint;
    }

    //raytrack优化局部目标
    //首先获得局部地图中的最优路径点
    unsigned int index = mLocalStartPoint;
    std::vector<unsigned int> pointsId;
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
        {
            pointsId.push_back(index);
        }
        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index])
                next_index = neighbors[i];
        }
        if(index == next_index) break;
        index = next_index;
    }

    unsigned int track_target = mLocalStartPoint;
    if(pointsId.size() > 1)//raytrack有效时
    {
        int i;
        //根据射线法，找到最远的目标点
        for(i=1; i<pointsId.size(); i++)
        {
            if(RayTrackisClean(pointsId[i]))
            {
                track_target = pointsId[i];
            }
            else
                break;
        }
        //之后判断 是否在mPathFollowRadius距离以内，mPathFollowRadius是室内运动的一个比较好的前瞻值，射线法应该找到比mPathFollowRadius更远的点
        if(mLocalCurrentPlan[track_target] > mLocalCurrentPlan[targetInSize])
        {
            track_target = targetInSize;//raytrack 追踪的节点没有1.25范围内找到的更加靠近goal
            ROS_INFO("raytrackGoal == localTargetGoal!");
        }
    }
    else
    {
        track_target = targetInSize;
    }

    // 计算局部跟踪的目标点与车体中心的距离
    unsigned int targetX, targetY, startX, startY;
    mLocalCurrentMap.getCoordinates(startX, startY, mLocalStartPoint);

    if(!mLocalCurrentMap.getCoordinates(targetX, targetY, track_target))
    {
        ROS_ERROR("Plan execution failed, track_target pose not in map!");
        return false;
    }

    double target_lenth = sqrt(pow(((int)startX-(int)targetX)*mLocalCurrentMap.getResolution(),2) + \
                               pow(((int)startY-(int)targetY)*mLocalCurrentMap.getResolution(),2));

    double map_angle = atan2((double)targetY - current_y, (double)targetX - current_x);

    double target_angle = map_angle - mCurrentDirection;
    if(target_angle < -PI) target_angle += 2*PI;
    if(target_angle >  PI) target_angle -= 2*PI;

    // Create the command message
    geometry_msgs::Twist msg;
    msg.angular.z = (180.0/mRotateThresholdDeg) * target_angle / PI;  //角速度与偏差的角度成正比

    //这儿需要考虑狭窄和极限情况，当局部目标点在车身范围内，车子只能原地自转！或者给负的速度？
    //也会出现局部目标点在车身后面，或者和车身90度垂直但是远离车身的位置。

    //速度输出在0.3-0.6之间
    //角度和目标点位置决定当前速度输出，角度小速度大  目标点远速度大
    if(mLocalCurrentPlan[mLocalStartPoint] > mNavigationHomingDistance || mLocalCurrentPlan[mLocalStartPoint] < 0)
    {

        if(abs(msg.angular.z) >= 1)
        {
            msg.linear.x = 0;
        }
        else
        {
            double speed_vel = (mSpeedMax - (mSpeedMax-mSpeedMin)*fabs(msg.angular.z));//考虑转角
            //目标点在1.5m-3m之间系数线性变换。考虑距离
            double target_lenth_temp = target_lenth;
            if(target_lenth_temp > 3)
                target_lenth_temp = 3;
            if(target_lenth_temp < 1.5)
                target_lenth_temp = 1.5;
            double scale = 1;//(target_lenth_temp-1.5)*0.3333+0.5;

            speed_vel = speed_vel * scale;

            if(speed_vel < mSpeedMin)
                speed_vel = mSpeedMin;
            if(speed_vel > mSpeedMax)
                speed_vel = mSpeedMax;

            msg.linear.x = speed_vel;
        }
    }
    else//在距离目标3m左右开始变慢
    {
        msg.linear.x = mSpeedMin;
        if(abs(msg.angular.z) >=1)
        {
            msg.linear.x = 0.0;
        }
    }

    // 对四足机器人，最后直接发送偏差角度
   msg.angular.z = msg.angular.z/2;
   if(msg.angular.z - mlast_angle > MIN_ANGLE)
   {
       msg.angular.z = mlast_angle + MIN_ANGLE;
   }
   else if(msg.angular.z - mlast_angle < -MIN_ANGLE)
   {
       msg.angular.z = mlast_angle - MIN_ANGLE;
   }
    
    mlast_angle = msg.angular.z;
    mCommandPublisher.publish(msg);

    //调试使用 发布局部方向点的marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = mMapFrame.c_str();
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mLocalCurrentMap.getOriginX() + (((double)targetX+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.y = mLocalCurrentMap.getOriginY() + (((double)targetY+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.z = mDebug_show_height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mLocalCurrentMap.getResolution() * 1.0;
    marker.scale.y = mLocalCurrentMap.getResolution() * 1.0;
    if(track_target == targetInSize)
        marker.scale.z = 3.0;
    else
        marker.scale.z = 6.0;//使用raytrack 用更长的mark表示
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mLocalDirectionMarkerPublisher.publish(marker);

    return true;
}

/**
 * @brief localRobotNavigator::receiveMoveGoal
 * 目标位置的服务程序，请求这个服务以后，在这个程序中会开始进行导航规划，直到达到目标点位置
 * @param goal
 */
void localRobotNavigator::receiveMoveGoal(const geometry_msgs::Pose2DConstPtr &goal)
{
    // Start navigating according to the generated plan
    static bool reached = false;
    mTotalDistance = 0.0;

    //构建局部地图，以机器人为中心1.5倍golbalCostMap的大小，中间叠加了带障碍物信息的局部costmap
    if(!createLocalMap())
    {
        ROS_ERROR("Local planning failed!");
        return;
    }

    //基于全局路径找到局部地图里最边缘上的局部goal，设置mLocalGoalPoint
    if(!createLocalGoal(goal->x, goal->y))
    {
        ROS_ERROR("Prepare localGoal failed!");
        return;
    }

    if(!createLocalPlan())
    {
        ROS_WARN("Prepare LocalPlan failed!");
        //publishDJGridMap();
        mLocalPlanMissCounter = 5;
        mIsNavPaused = true;
        geometry_msgs::Twist stopMsg;
        stopMsg.linear.x = 0;
        stopMsg.angular.z = 0;
        mCommandPublisher.publish(stopMsg);
        return;
    }
    else
    {
        if(mLocalPlanMissCounter>0)
        {
            mLocalPlanMissCounter--;
            ROS_INFO("mLocalPlanMissCounter=%d", mLocalPlanMissCounter);
        }
        else
        {
            mIsNavPaused = false;
        }

        //        publishDJGridMap();
        publishLocalPlan();
    }

    ROS_INFO("Distance: %f", mTotalDistance);
    // Are we already close enough?

    if(mTotalDistance <= mNavigationHomingDistance
            && mTotalDistance > 0)
    {
        ROS_INFO("Reached track_target, now turning to desired direction: %.2f", goal->theta-mCurrentDirection);
        generateLocalNearTargetCommand(goal->theta-mCurrentDirection, mNavigationGoalAngle);

        reached = true;
        if(waypoint_ind < waypoint_count)
        {

            waypoint_ind++;
            mWayPointPublisher.publish(waypoint_list[waypoint_ind]);
        }
    }
    else {
        generateLocalCommand();
        reached = false;
    }
    std_msgs::Bool finish;
    finish.data = reached;
    mFinishPublisher.publish(finish);
}

/**
 * @brief localRobotNavigator::setLocalCurrentPosition
 * 根据全局位置设置局部坐标位置
 * @return
 */
bool localRobotNavigator::setLocalCurrentPosition()
{
  tf::StampedTransform trans;
  try
  {
    mTfListener->lookupTransform(mMapFrame, mRobotFrame, ros::Time(0), trans);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
    unsigned int current_x = (trans.getOrigin().x() - mLocalCurrentMap.getOriginX()) / mLocalCurrentMap.getResolution();
    unsigned int current_y = (trans.getOrigin().y() - mLocalCurrentMap.getOriginY()) / mLocalCurrentMap.getResolution();

    mLocalCurrentMap.getIndex(current_x, current_y, mLocalStartPoint);
    //ROS_INFO("CurrentX:%d, CurrentY:%d, LocalStartPoint:%d", current_x, current_y, mLocalStartPoint);

    mCurrentDirection = getYaw(trans.getRotation());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localNavigator");
    ros::NodeHandle n;

    localRobotNavigator robNav;

    ros::spin();

    // ros::Rate loop(1.0);

    // while(ros::ok()) {
        
        ROS_INFO("%d",robNav.waypoint_count);

    //robNav.receiveMoveGoal();
//         loop.sleep();
//   }
// ros::spinOnce();
    return 0;
}
