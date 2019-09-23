#include "Globals.hpp"
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace cv;

struct Loop {
	std::vector<Point2i> PointsL;
	int Counter;

	Loop() :Counter(0)
	{
	}

	Loop(std::vector<Point2i> _points)
		: PointsL(_points), Counter(0)
	{
	}
};
typedef std::vector<Loop> LoopList;

class CollisionDetector
{
public:

	CollisionDetector();
	~CollisionDetector();

	LoopList& checkCollision(TrackingObjectList& _tobjs);
	double calcCollisionArea(TrackingObject* _tObj, Loop& _loop);
	void addLoop(const Loop& _loop);

private:
	LoopList mLoops;
};


