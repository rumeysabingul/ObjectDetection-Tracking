#pragma once

#include <list>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

static float calcMatchScore(cv::Rect2i & _rectA, cv::Rect2i & _rectB, float _threshold, bool _centerPointCheck = false)
{
	float score = 0;

	cv::Point2i centerA(_rectA.x + (_rectA.width / 2), _rectA.y + (_rectA.height / 2));
	cv::Point2i centerB(_rectB.x + (_rectB.width / 2), _rectB.y + (_rectB.height / 2));

	if (!_centerPointCheck
		|| (_centerPointCheck && _rectA.contains(centerB) && _rectB.contains(centerA))
		)
	{
		cv::Rect2i rectIntersect = _rectA & _rectB;
		float areaRatio = ((float)std::min(_rectA.area(), _rectB.area()) / (float)std::max(_rectA.area(), _rectB.area()));

		if (rectIntersect.area() > 0
			&& areaRatio > _threshold
			)
		{
			score = ((float)rectIntersect.area() / (float)((_rectA.area() + _rectB.area()) - rectIntersect.area())) * 1000.0f;
			score -= (float)norm(centerA - centerB) / 10.0f;
			score -= (float)norm(_rectA.tl() - _rectB.tl()) / 100.0f;
			score -= (float)norm(_rectA.br() - _rectB.br()) / 100.0f;
		}
	}

	return score;
}

template <typename T> static bool matchPairComparer(std::pair<T *, int> & _first, std::pair<T *, int>  & _second)
{
	return _first.second > _second.second;
}