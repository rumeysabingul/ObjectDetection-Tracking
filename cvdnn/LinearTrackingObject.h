#pragma once

#include "TrackingObject.h"

class LinearTrackingObject : public TrackingObject
{
public:
	LinearTrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId, int _historySize = 50);
	~LinearTrackingObject();
	void update(double _deltaT, cv::Rect2i *_measuredBoundingBox = nullptr) override;

	std::vector<cv::Rect2i> BoundingBoxHistory;

private:
	int mHistorySize;
};

