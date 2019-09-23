#include <iostream>
#include <opencv2/opencv.hpp>
#include <deque>
//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>
//#include <boost/foreach.hpp>
#include "Detector.h"
#include "Tracker.h"
#include "CollisionDetector.h"

int main(int argc, char* argv[])
{
#pragma region Init
	CollisionDetector collision;
	Loop L1;
	L1.PointsL.push_back(cv::Point2i(13, 630));
	L1.PointsL.push_back(cv::Point2i(235, 522));
	L1.PointsL.push_back(cv::Point2i(238, 548));
	L1.PointsL.push_back(cv::Point2i(206, 603));
	L1.PointsL.push_back(cv::Point2i(13, 630));
	Loop L2;
	L2.PointsL.push_back(cv::Point2i(290, 620));
	L2.PointsL.push_back(cv::Point2i(336, 495));
	L2.PointsL.push_back(cv::Point2i(452, 526));
	L2.PointsL.push_back(cv::Point2i(459, 603));
	L2.PointsL.push_back(cv::Point2i(290, 620));
	Loop L3;
	L3.PointsL.push_back(cv::Point2i(480, 620));
	L3.PointsL.push_back(cv::Point2i(548, 562));
	L3.PointsL.push_back(cv::Point2i(598, 596));
	L3.PointsL.push_back(cv::Point2i(655, 630));
	L3.PointsL.push_back(cv::Point2i(480, 620));
	collision.addLoop(L1);
	collision.addLoop(L2);
	collision.addLoop(L3);
#pragma endregion

	cv::VideoCapture vcap(argc >= 2 ? argv[1] : "../data/video.mp4");

	if (!vcap.isOpened())
	{
		std::cout << "Video file couldn't opened. " << argv[1] << std::endl;
		return -1;
	}


	cv::namedWindow("cvdnn");

	Detector detector(
		argc >= 3 ? argv[2] : "opt",
		argc >= 5 ? argv[3] : "../data/graph.xml",
		argc >= 5 ? argv[4] : "../data/graph.bin",
		argc >= 7 ? std::stoi(argv[5]) : 300,
		argc >= 7 ? std::stoi(argv[6]) : 300,
		argc >= 8 ? std::stoi(argv[7]) : cv::dnn::DNN_BACKEND_INFERENCE_ENGINE,
		argc >= 9 ? std::stoi(argv[8]) : cv::dnn::DNN_TARGET_CPU
	);

	Tracker tracker;

	cv::Mat f;

	int64 tS = 0;
	while (true)
	{
		double preTicks = (double)tS;
		tS = cv::getTickCount();
		double deltaT = (tS - preTicks) / cv::getTickFrequency();
		//---

		vcap >> f;

		DetectedObjectList detectedObjects = detector.detect(f);

		TrackingObjectList trackingObjects = tracker.track(deltaT, detectedObjects);

		LoopList loops = collision.checkCollision(trackingObjects);
		detector.clearDetectedObjects();

		for (TrackingObjectList::iterator it = trackingObjects.begin(); it != trackingObjects.end(); it++)
		{
			TrackingObject* to = *it;
			cv::rectangle(f, (*it)->BoundingBox, cv::Scalar(0, 0, 255), 1);
			cv::putText(f, "#" + std::to_string(to->Id) + " T:" + std::to_string(to->TypeId), to->BoundingBox.tl(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 120, 80), 2);
			cv::putText(f, "#" + std::to_string(to->Id) + " T:" + std::to_string(to->TypeId), to->BoundingBox.tl(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(50, 0, 0), 1);
		}

		for (LoopList::iterator it_ll = loops.begin(); it_ll != loops.end(); it_ll++)
		{
			const Point* pts = (const cv::Point*) Mat(it_ll->PointsL).data;
			int koseS = Mat(it_ll->PointsL).rows;
			cv::polylines(f, &pts, &koseS, 1, false, Scalar(255, 255, 0), 2);
			cv::putText(f, std::to_string(it_ll->Counter), it_ll->PointsL[0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 255), 2);
		}
		
		//---
		cv::putText(f, "FPS: " + std::to_string(cv::getTickFrequency() / (double)(cv::getTickCount() - tS)), cv::Point2i(5, f.rows - 5), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
		cv::imshow("cvdnn", f);
		cv::waitKey(1);
	}

	vcap.release();
	return 0;
}