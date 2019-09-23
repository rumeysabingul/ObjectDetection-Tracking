#pragma once

#include <string>
#include <opencv2/dnn.hpp>
#include "DetectedObject.h"

class Detector
{
public:
	Detector(
		const std::string &_modelType,
		const std::string &_model,
		const std::string &_modelTxt,
		int _w,
		int _h,
		int _backEnd = 3,
		int _target = 0,
		float _detectConfidenceThreshold = 0.25f,
		float _overlapThreshold = 0.8f
	);
	~Detector();

	DetectedObjectList & detect(cv::Mat & _frame, bool _drawDetections = true);

	DetectedObjectList & getDetectedObjects();

	void clearDetectedObjects();

private:
	cv::dnn::Net mDnnNet;
	DetectedObjectList mDetectedObjects;

	float
		mDetectConfidenceThreshold,
		mOverlapThreshold;

	bool
		mSwapBR;

	int
		mW,
		mH;
};