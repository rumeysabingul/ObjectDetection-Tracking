#pragma once

#include "TrackingObject.h"

class KalmanFilterTrackingObject : public TrackingObject
{
public:
	KalmanFilterTrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId);
	~KalmanFilterTrackingObject();
	void update(double _deltaT, cv::Rect2i *_measuredBoundingBox = nullptr) override;

private:
	bool mInitKF;
	cv::KalmanFilter *mKF;
	cv::Mat_<float> *mMeasMat, *mStateMat;
};