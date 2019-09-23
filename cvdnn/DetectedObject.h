#pragma once

#include "Globals.hpp"

class TrackingObject;

class DetectedObject
{
public:
	DetectedObject();
	~DetectedObject();

	TrackingObject * possibleMatchForFrame(int _frameId);

	float TypeId;
	float Confidence;
	cv::Rect2i BoundingBox;
	int FrameId;

	std::list<std::pair<TrackingObject *, int>> Matches;
};

typedef std::list<DetectedObject*> DetectedObjectList;