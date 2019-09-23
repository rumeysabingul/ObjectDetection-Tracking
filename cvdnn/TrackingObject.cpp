#include "TrackingObject.h"
#include "DetectedObject.h"

TrackingObject::TrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId)
	: Id(_id),
	TypeId(_typeId),
	BoundingBox(_bbox),
	FrameId(_frameId)
{
}

TrackingObject::~TrackingObject()
{
}

DetectedObject * TrackingObject::possibleMatchForFrame(int _frameId)
{
	for (std::list<std::pair<DetectedObject *, int>>::iterator it = Matches.begin(); it != Matches.end(); it++)
	{
		if (it->first->FrameId != _frameId)
			return it->first;
	}

	return nullptr;
}


