#include "DetectedObject.h"
#include "TrackingObject.h"

DetectedObject::DetectedObject()
{
}

DetectedObject::~DetectedObject()
{
}

TrackingObject * DetectedObject::possibleMatchForFrame(int _frameId)
{
	for (std::list<std::pair<TrackingObject *, int>>::iterator it = Matches.begin(); it != Matches.end(); it++)
	{
		if (it->first->FrameId != _frameId)
			return it->first;
	}

	return nullptr;
}