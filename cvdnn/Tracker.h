#pragma once

#include <opencv2/opencv.hpp>
#include "TrackingObject.h"
#include "DetectedObject.h"

class Tracker
{
public:
	Tracker(
		int _lostTrackFrameLimit = 30,
		float _matchDiffThreshold = 0.5f
	);

	~Tracker();

	TrackingObjectList & track(double delta_t, DetectedObjectList & _detectedObjects);

	TrackingObjectList & getTrackingObjects();

	void clearTrackingObjects();

private:
	int 
		mLostTrackFrameLimit,
		mObjectIdCounter,
		mFrameIdCounter;

	float
		mMatchDiffThreshold;

	TrackingObjectList mTrackingObjects;
};