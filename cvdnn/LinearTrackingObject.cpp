#include "LinearTrackingObject.h"

LinearTrackingObject::LinearTrackingObject(double _deltaT, int _id, int _typeId, cv::Rect2i _bbox, int _frameId, int _historySize) 
	: TrackingObject(_deltaT, _id, _typeId, _bbox, _frameId),
	mHistorySize(_historySize)
{
}

LinearTrackingObject::~LinearTrackingObject()
{
}

void LinearTrackingObject::update(double _deltaT, cv::Rect2i *_measuredBoundingBox)
{
	BoundingBoxHistory.push_back(BoundingBox);

	if (_measuredBoundingBox)
	{
		BoundingBox = *_measuredBoundingBox;
	}
	else
	{
		if (BoundingBoxHistory.size() <= 1)
			return;
		else if (BoundingBoxHistory.size() > mHistorySize)
			BoundingBoxHistory.erase(BoundingBoxHistory.begin(), BoundingBoxHistory.end() - mHistorySize);

		double x_tl = .0, y_tl = .0, x_br = .0, y_br = .0, mul = 1.0;
		for (size_t i = 1; i < BoundingBoxHistory.size(); i++)
		{
			mul = 1.0 - ((double)i / (double)BoundingBoxHistory.size() / 1.5);
			cv::Rect2i & rect = BoundingBoxHistory.at(i);
			cv::Rect2i & rectOld = BoundingBoxHistory.at(i - 1);

			x_tl += (double)(rect.tl().x - rectOld.tl().x) * mul;
			y_tl += (double)(rect.tl().y - rectOld.tl().y) * mul;
			x_br += (double)(rect.br().x - rectOld.br().x) * mul;
			y_br += (double)(rect.br().y - rectOld.br().y) * mul;
		}

		double d = (double)BoundingBoxHistory.size();
		BoundingBox = cv::Rect2i(
			BoundingBox.tl() + cv::Point2i((int)(x_tl / d), (int)(y_tl / d)),
			BoundingBox.br() + cv::Point2i((int)(x_br / d), (int)(y_br / d))
		);
	}
}