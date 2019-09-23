#include "KalmanFilterTrackingObject.h"

KalmanFilterTrackingObject::KalmanFilterTrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId) : TrackingObject(_deltaT, _id, _typeId, _bbox, _frameId)
{
	Counted = false;

	mKF = new cv::KalmanFilter(6, 4);
	mStateMat = new cv::Mat_<float>(6, 1);	// [x,y,v_x,v_y,w,h]
	mMeasMat = new cv::Mat_<float>(4, 1);	// [z_x,z_y,z_w,z_h]
	//cv::Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	cv::setIdentity(mKF->transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	mKF->measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
	mKF->measurementMatrix.at<float>(0) = 1.0f;
	mKF->measurementMatrix.at<float>(7) = 1.0f;
	mKF->measurementMatrix.at<float>(16) = 1.0f;
	mKF->measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	mKF->processNoiseCov.at<float>(0) = 1e-2f;
	mKF->processNoiseCov.at<float>(7) = 1e-2f;
	mKF->processNoiseCov.at<float>(14) = 5.0f;
	mKF->processNoiseCov.at<float>(21) = 5.0f;
	mKF->processNoiseCov.at<float>(28) = 1e-2f;
	mKF->processNoiseCov.at<float>(35) = 1e-2f;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(mKF->measurementNoiseCov, cv::Scalar(1e-1));

	mInitKF = true;
	update(_deltaT, &_bbox);
}

KalmanFilterTrackingObject::~KalmanFilterTrackingObject()
{
	if(mKF)
		delete mKF;

	if(mMeasMat)
		delete mMeasMat;

	if (mStateMat)
		delete mStateMat;
}

void KalmanFilterTrackingObject::update(double _deltaT, cv::Rect2i *_measuredBoundingBox)
{
	mKF->transitionMatrix.at<float>(2) = (float)_deltaT;
	mKF->transitionMatrix.at<float>(9) = (float)_deltaT;

	(*mStateMat) = mKF->predict();

	if (_measuredBoundingBox)
	{
		mMeasMat->at<float>(0) = (float)_measuredBoundingBox->x;
		mMeasMat->at<float>(1) = (float)_measuredBoundingBox->y;
		mMeasMat->at<float>(2) = (float)_measuredBoundingBox->width;
		mMeasMat->at<float>(3) = (float)_measuredBoundingBox->height;

		if (mInitKF)
		{
			mKF->errorCovPre.at<float>(0) = 1; // px
			mKF->errorCovPre.at<float>(7) = 1; // px
			mKF->errorCovPre.at<float>(14) = 1;
			mKF->errorCovPre.at<float>(21) = 1;
			mKF->errorCovPre.at<float>(28) = 1; // px
			mKF->errorCovPre.at<float>(35) = 1; // px

			mStateMat->at<float>(0) = mMeasMat->at<float>(0);
			mStateMat->at<float>(1) = mMeasMat->at<float>(1);
			mStateMat->at<float>(2) = 0;
			mStateMat->at<float>(3) = 0;
			mStateMat->at<float>(4) = mMeasMat->at<float>(2);
			mStateMat->at<float>(5) = mMeasMat->at<float>(3);

			mKF->statePost = *mStateMat;

			mInitKF = false;
		}
		else
		{
			(*mStateMat) = mKF->correct(*mMeasMat);
		}
	}

	BoundingBox.x = (int)mStateMat->at<float>(0);
	BoundingBox.y = (int)mStateMat->at<float>(1);
	BoundingBox.width = (int)mStateMat->at<float>(4);
	BoundingBox.height = (int)mStateMat->at<float>(5);
}