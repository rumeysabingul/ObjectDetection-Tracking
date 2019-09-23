#pragma once

#include "Globals.hpp"
#include <list>
#include <queue>
#include <opencv2/opencv.hpp>

class DetectedObject;

class TrackingObject
{
public:
	TrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId);
	virtual ~TrackingObject();
	virtual void update(double _deltaT, cv::Rect2i *_measuredBoundingBox = nullptr) = 0;

	DetectedObject * possibleMatchForFrame(int _frameId);

	int Id;
	int TypeId;
	cv::Rect2i BoundingBox;
	int FrameId;
	bool Counted;

	std::list<std::pair<DetectedObject *, int>> Matches;
};

typedef std::list<TrackingObject*> TrackingObjectList;