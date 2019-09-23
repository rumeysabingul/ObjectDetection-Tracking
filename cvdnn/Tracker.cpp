#include "Tracker.h"
#include "LinearTrackingObject.h"
#include "KalmanFilterTrackingObject.h"
#include "DetectedObject.h"

using namespace std;
using namespace cv;

Tracker::Tracker(int _lostTrackFrameLimit, float _matchDiffThreshold)
	: mLostTrackFrameLimit(_lostTrackFrameLimit),
	mMatchDiffThreshold(_matchDiffThreshold),
	mObjectIdCounter(0),
	mFrameIdCounter(0)
{
}

Tracker::~Tracker()
{
}

TrackingObjectList & Tracker::track(double _deltaT, DetectedObjectList & _detectedObjects)
{
	int
		frameId = ++mFrameIdCounter;
		
	float
		sc = 0;

	// prepare detect <-> track match list
	for (DetectedObjectList::iterator it_dt = _detectedObjects.begin(); it_dt != _detectedObjects.end();++it_dt)
	{
		DetectedObject * dobj = *it_dt;

		for (TrackingObjectList::iterator it_tr = mTrackingObjects.begin(); it_tr != mTrackingObjects.end(); ++it_tr)
		{
			TrackingObject * tobj = *it_tr;

			if ((sc = calcMatchScore(tobj->BoundingBox, dobj->BoundingBox, mMatchDiffThreshold)) > 0)
			{
				dobj->Matches.push_back(std::make_pair(tobj, sc));
				tobj->Matches.push_back(std::make_pair(dobj, sc));

				if (tobj->Matches.size() >= 2)
					tobj->Matches.sort(matchPairComparer<DetectedObject>);
			}
		}

		if (dobj->Matches.size() >= 2)
			dobj->Matches.sort(matchPairComparer<TrackingObject>);
	}

	// match by track & update tracks with measurement
	for (TrackingObjectList::iterator it_tr = mTrackingObjects.begin(); it_tr != mTrackingObjects.end(); ++it_tr)
	{
		TrackingObject * tobj = *it_tr;

		DetectedObject * dobj = tobj->possibleMatchForFrame(frameId);
		if (dobj && (dobj->possibleMatchForFrame(frameId) == tobj))
		{
			tobj->TypeId = (int)dobj->TypeId;
			tobj->FrameId = dobj->FrameId = frameId;
			tobj->update(_deltaT, &dobj->BoundingBox);
		}
	}

	// match by detect & update tracks with measurement
	for (DetectedObjectList::iterator it_dt = _detectedObjects.begin(); it_dt != _detectedObjects.end(); ++it_dt)
	{
		DetectedObject * dobj = *it_dt;

		TrackingObject * tobj = dobj->possibleMatchForFrame(frameId);
		if (tobj && (tobj->possibleMatchForFrame(frameId) == dobj))
		{
			tobj->TypeId = (int)dobj->TypeId;
			tobj->FrameId = dobj->FrameId = frameId;
			tobj->update(_deltaT, &dobj->BoundingBox);
		}
	}

	// update or delete tracks wihout measurement
	for (TrackingObjectList::iterator it_tr = mTrackingObjects.begin(); it_tr != mTrackingObjects.end();)
	{
		TrackingObject * tobj = *it_tr;

		tobj->Matches.clear();

		if (tobj->FrameId != frameId)
		{
			if ((frameId - tobj->FrameId) >= mLostTrackFrameLimit)
			{
				it_tr = mTrackingObjects.erase(it_tr);
				delete tobj;
				tobj = nullptr;

				continue;
			}
			else
			{
				tobj->update(_deltaT);
			}
		}

		++it_tr;
	}

	// add new tracks by detects and clear detects
	for (DetectedObjectList::iterator it_dt = _detectedObjects.begin(); it_dt != _detectedObjects.end(); ++it_dt)
	{
		DetectedObject * dobj = *it_dt;

		if (dobj->FrameId != frameId)
		{
			TrackingObject * tobj = new KalmanFilterTrackingObject(_deltaT, ++mObjectIdCounter, (int)dobj->TypeId, dobj->BoundingBox, frameId);
			mTrackingObjects.push_back(tobj);
			tobj->Matches.clear();
		}
	}

	return mTrackingObjects;
}

TrackingObjectList & Tracker::getTrackingObjects()
{
	return mTrackingObjects;
}

void Tracker::clearTrackingObjects()
{
	for (TrackingObjectList::iterator it_tr = mTrackingObjects.begin(); it_tr != mTrackingObjects.end();)
	{
		TrackingObject * tobj = *it_tr;

		it_tr = mTrackingObjects.erase(it_tr);
		delete tobj;
	}
}
