#include "Detector.h"
#include "Globals.hpp"

using namespace std;
using namespace cv;

Detector::Detector(const string & _modelType, const string & _model, const string & _modelTxt, int _w, int _h, int _backEnd, int _target, float _detectConfidenceThreshold, float _overlapThreshold)
	: mDetectConfidenceThreshold(_detectConfidenceThreshold),
	mOverlapThreshold(_overlapThreshold),
	mSwapBR(false),
	mW(_w),
	mH(_h)
{
	if(_modelType == "opt")
		mDnnNet = cv::dnn::readNetFromModelOptimizer(_model, _modelTxt);
	else if (_modelType == "tensorflow")
	{
		mSwapBR = true;
		mDnnNet = cv::dnn::readNetFromTensorflow(_model, _modelTxt);
	}
	else if (_modelType == "caffe")
	{
		mDnnNet = cv::dnn::readNetFromCaffe(_modelTxt, _model);
	}

	mDnnNet.setPreferableBackend(_backEnd);
	mDnnNet.setPreferableTarget(_target);
}

Detector::~Detector()
{
	clearDetectedObjects();
}

DetectedObjectList & Detector::detect(Mat & _frame, bool _drawDetections)
{
	// set dnn blob input
	mDnnNet.setInput(cv::dnn::blobFromImage(_frame, 1.0, cv::Size(mW, mH), cv::Scalar(), mSwapBR));

	// dnn forward
	Mat detection = mDnnNet.forward();
	Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

	// collect detections by eleminating overlaps
	int tlX = 0, tlY = 0, brX = 0, brY = 0;
	float typeId = 0, cf = 0;
	bool overlap = false;
	for (int i = 0; i < detectionMat.rows; i++)
	{
		overlap = false;
		typeId = detectionMat.at<float>(i, 1);
		cf = detectionMat.at<float>(i, 2);

		tlX = static_cast<int>(detectionMat.at<float>(i, 3) * _frame.cols);
		tlY = static_cast<int>(detectionMat.at<float>(i, 4) * _frame.rows);
		brX = static_cast<int>(detectionMat.at<float>(i, 5) * _frame.cols);
		brY = static_cast<int>(detectionMat.at<float>(i, 6) * _frame.rows);
		Rect2i rect(cv::Point2i(tlX, tlY), cv::Point2i(brX, brY));

		if (cf >= mDetectConfidenceThreshold && rect.area() > 1000)
		{
			for (DetectedObjectList::iterator it = mDetectedObjects.begin(); it != mDetectedObjects.end();)
			{
				DetectedObject * dobj = *it;

				if (calcMatchScore(dobj->BoundingBox, rect, mOverlapThreshold, true) > 0)
				{
					if (dobj->Confidence < cf)
					{
						it = mDetectedObjects.erase(it);
						delete dobj;
						dobj = nullptr;
						continue;
					}
					else
					{
						overlap = true;
						break;
					}
				}

				++it;
			}

			if (!overlap)
			{
				DetectedObject * dobj = new DetectedObject();
				dobj->BoundingBox = rect;
				dobj->Confidence = cf;
				dobj->TypeId = typeId;
				dobj->FrameId = -1;

				if (_drawDetections)
					rectangle(_frame, rect, Scalar(0, 255, 0), 2);

				mDetectedObjects.push_back(dobj);
			}
		}
	}

	return mDetectedObjects;
}

DetectedObjectList & Detector::getDetectedObjects()
{
	return mDetectedObjects;
}

void Detector::clearDetectedObjects()
{
	for (DetectedObjectList::iterator it_dt = mDetectedObjects.begin(); it_dt != mDetectedObjects.end();)
	{
		DetectedObject * dobj = *it_dt;

		it_dt = mDetectedObjects.erase(it_dt);
		delete dobj;
	}
}