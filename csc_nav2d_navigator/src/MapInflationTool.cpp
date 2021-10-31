#include <csc_nav2d_navigator/MapInflationTool.h>

CellData::CellData(double d, double i, unsigned int sx, unsigned int sy):distance(d), index(i), sx(sx), sy(sy){
}

MapInflationTool::MapInflationTool()
{
	mInflationMarkers = NULL;
	mCachedCosts = NULL;
	mCachedDistances = NULL;
	mCostObstacle = 100;
}

MapInflationTool::~MapInflationTool()
{
	delete[] mInflationMarkers;
	delete[] mCachedCosts;
	delete[] mCachedDistances;
}

// This method calculates two different lookup-tables:
// 1. The distance of cells arount an obstacle towards this obstacle
// 2. The cost that is applied to these cells around an obstacle
void MapInflationTool::computeCaches(unsigned int radius)
{
	mCellInflationRadius = radius;
	
	mCachedCosts = new char*[mCellInflationRadius + 2];
	mCachedDistances = new double*[mCellInflationRadius + 2];
	
	for(unsigned int i = 0; i < mCellInflationRadius + 2; i++)
	{
		mCachedCosts[i] = new char[mCellInflationRadius + 2];
		mCachedDistances[i] = new double[mCellInflationRadius + 2];
		for(unsigned int j = 0; j < mCellInflationRadius + 2; j++)
		{
			double d = sqrt(i*i + j*j);
			mCachedDistances[i][j] = d;
			d /= mCellInflationRadius;
			if(d > 1) d = 1;
			mCachedCosts[i][j] = (1.0 - d) * mCostObstacle;
		}
	}
}

// Method used to query the cache for a distance.
inline double MapInflationTool::distanceLookup(int mx, int my, int src_x, int src_y)
{
	unsigned int dx = abs(mx - src_x);
	unsigned int dy = abs(my - src_y);
	if(dx > mCellInflationRadius + 1 || dy > mCellInflationRadius + 1)
	{
		ROS_ERROR("Error in distanceLookup! Asked for (%d, %d), but CellInflationRadius is %d!", dx, dy, mCellInflationRadius);
		return 50.0;
	}
	return mCachedDistances[dx][dy];
}

// Method used to query the cache for the cell cost.
inline char MapInflationTool::costLookup(int mx, int my, int src_x, int src_y)
{
	unsigned int dx = abs(mx - src_x);
	unsigned int dy = abs(my - src_y);
	if(dx > mCellInflationRadius + 1 || dy > mCellInflationRadius + 1)
	{
		ROS_ERROR("Error in costLookup! Asked for (%d, %d), but CellInflationRadius is %d!", dx, dy, mCellInflationRadius);
		return 50.0;
	}
	return mCachedCosts[dx][dy];
}

// Main method to start the inflation on the given map in-place.
void MapInflationTool::inflateMap(GridMap* map)//
{
	ROS_DEBUG("Started map inflation ...");
	
	// 0. Do some initialization
	mGridMap = map;
	int mapSize = mGridMap->getSize();
	
	if(mInflationMarkers) delete[] mInflationMarkers;
	mInflationMarkers = new unsigned char[mapSize];
	for(int i = 0; i < mapSize; i++)
	{
		mInflationMarkers[i] = 0;
	}
	
	// 1. Put all real obstacles in a queue
	while(!mInflationQueue.empty()) mInflationQueue.pop();
	
	for(int index = 0; index < mapSize; index++)
	{
    if(mGridMap->getData(index) > 0)//大于0为占用，因为地图是表示成[-128 127] -1是未知，0是free 127是占用，这里的地图还只具备3个值
		{
			unsigned int sx, sy;
			mGridMap->getCoordinates(sx, sy, index);
      enqueueObstacle(index, sx, sy);//这个用法是偷懒，这个函数真正的作用是在下一步，只是刚好也可以用来标记地图中的真实障碍物。
    }else if(mGridMap->getData(index) == -1)//未知的部分是-1
		{
      mInflationMarkers[index] = 1;//1是标志位，代表有障碍物
		} 
	}
	
  //此时队列中有静态障碍物的点，之后会基于这些点，向上下左右蔓延，蔓延时会判断是否cost为0（看的是distance）,不为0则继续添加队列，直到蔓延结束。
	// 2. Inflate them by the given inflation radius
	int count = 0;
	while(!mInflationQueue.empty())
	{
		CellData cell = mInflationQueue.top();
		mInflationQueue.pop();
		unsigned int x,y;
		if(!mGridMap->getCoordinates(x, y, cell.index)) continue;
		if(x >= 1)
			enqueueObstacle(cell.index-1, cell.sx, cell.sy);
		if(x < mGridMap->getWidth() - 1)	
			enqueueObstacle(cell.index+1, cell.sx, cell.sy);
		if(y >= 1)
			enqueueObstacle(cell.index-mGridMap->getWidth(), cell.sx, cell.sy);
		if(y < mGridMap->getHeight() - 1)
			enqueueObstacle(cell.index+mGridMap->getWidth(), cell.sx, cell.sy);
		count++;
	}
	
	ROS_DEBUG("Finished inflation. (%d cells)", count);
}

void MapInflationTool::enqueueObstacle(unsigned int index, unsigned int sx, unsigned int sy)
{
	unsigned int mx, my;
	if(!mGridMap->getCoordinates(mx, my, index)) return;
  if(mInflationMarkers[index] != 0) return;//0是默认值，其他值就代表被标记过了
	
	double distance = distanceLookup(mx, my, sx, sy);
	if(distance == 50)
		ROS_INFO("Tried to add cell (%u, %u) -> (%u, %u) to inflation queue!", sx, sy, mx, my);
	
	if(distance > mCellInflationRadius) return;
		
	CellData cell(distance, index, sx, sy);
  mInflationQueue.push(cell);//障碍物队列
  mInflationMarkers[index] = 1;//障碍物标志位
  char value = costLookup(mx, my, sx, sy);//这个会变成1-100的cost，以便后面设置阈值
	mGridMap->setData(index, value);
//	ROS_DEBUG("Set cell %d cost to %d", index, value);
}
