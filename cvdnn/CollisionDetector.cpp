#include "TrackingObject.h"
#include "CollisionDetector.h"
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace cv;
using boost::geometry::append;
using boost::geometry::make;
using boost::geometry::correct;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

CollisionDetector::CollisionDetector()
{
}

CollisionDetector::~CollisionDetector()
{
}


LoopList& CollisionDetector::checkCollision(TrackingObjectList& _tobjs)
{
	for (TrackingObjectList::iterator it = _tobjs.begin(); it != _tobjs.end(); it++)
	{
		TrackingObject* to = *it;

		if (!to->Counted)
		{
			Loop* loopMaxIntersect = nullptr;
			int loopMaxIntersectArea = 0;
			for (LoopList::iterator it_ll = mLoops.begin(); it_ll != mLoops.end(); it_ll++)
			{
				double area = calcCollisionArea(to, (*it_ll));
				if (area > 0 && (!loopMaxIntersect || loopMaxIntersectArea < area))
				{
					loopMaxIntersect = &(*it_ll);
					loopMaxIntersectArea = area;
				}
			}
			if (loopMaxIntersect)
			{
				to->Counted = true;
				loopMaxIntersect->Counter++;
			}
		}
	}
	return mLoops;
}

double CollisionDetector::calcCollisionArea(TrackingObject* _tObj, Loop& _loop)
{
	polygon_type pBoundingbox;
	append(pBoundingbox, make<point_type>(_tObj->BoundingBox.x, _tObj->BoundingBox.y));
	append(pBoundingbox, make<point_type>(_tObj->BoundingBox.x, _tObj->BoundingBox.y + _tObj->BoundingBox.height));
	append(pBoundingbox, make<point_type>(_tObj->BoundingBox.x + _tObj->BoundingBox.width, _tObj->BoundingBox.y + _tObj->BoundingBox.height));
	append(pBoundingbox, make<point_type>(_tObj->BoundingBox.x + _tObj->BoundingBox.width, _tObj->BoundingBox.y));
	append(pBoundingbox, make<point_type>(_tObj->BoundingBox.x, _tObj->BoundingBox.y));

	polygon_type pLoop;
	for (std::vector<cv::Point>::iterator it = _loop.PointsL.begin(); it != _loop.PointsL.end(); ++it)
	{
		append(pLoop, make<point_type>(it->x, it->y));
	}
	append(pLoop, make<point_type>(_loop.PointsL[0].x, _loop.PointsL[0].y));

	std::deque<polygon_type> intersectionArea;
	boost::geometry::intersection(pBoundingbox, pLoop, intersectionArea);
	return intersectionArea.size() > 0 ? boost::geometry::area(intersectionArea[0]) : 0.0;
}

void CollisionDetector::addLoop(const Loop& _loop)
{
	mLoops.push_back(_loop);
}
