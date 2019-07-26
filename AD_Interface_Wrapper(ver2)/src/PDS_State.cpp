#include "PDS_State.h"
#include <assert.h>

PDS_State::PDS_State() 
{
	// Initialize the time variables to 0
	prevTime = 0.0;
	currTime = 0.0;
}

void PDS_State::allocate(int totalNodes)
{
	currInflows.resize(totalNodes * 3, 0.0);
	prevInflows.resize(totalNodes * 3, 0.0);
}


void PDS_State::initializeHubState(Vector_3D hubPosition,
	Vector_3D hubOrientation,
	Vector_3D hubVelocity,
	Vector_3D hubRotationalVelocity)
{
	currHubPosition = hubPosition;
	currHubOrientation = hubOrientation;
	currHubVelocity = hubVelocity;
	currHubRotationalVelocity = hubRotationalVelocity;
}

void PDS_State::initializeInflows(const std::vector<double>& inflows)
{
	currInflows = inflows;
}

void PDS_State::updateStates(double time, 
	Vector_3D hubPosition,
	Vector_3D hubOrientation,
	Vector_3D hubVelocity,
	Vector_3D hubRotationVelocity,
	const std::vector<double>& inflows)
{
	prevInflows = currInflows;
	currInflows = inflows;

	prevHubPosition = currHubPosition;
	prevHubOrientation = currHubOrientation;
	prevHubVelocity = currHubVelocity;
	prevHubRotationalVelocity = currHubRotationalVelocity;

	currHubPosition = hubPosition;
	currHubOrientation = hubOrientation;
	currHubVelocity = hubVelocity;
	currHubRotationalVelocity = hubRotationVelocity;

	prevTime = currTime;
	currTime = time;
}

Vector_3D PDS_State::getCurrentHubPosition() const
{
	return currHubPosition;
}

Vector_3D PDS_State::getCurrentHubOrientation() const
{
	return currHubOrientation;
}
Vector_3D PDS_State::getCurrentHubVelocity() const
{
	return currHubVelocity;
}
Vector_3D PDS_State::getCurrentHubRotationalVelocity() const
{
	return currHubRotationalVelocity;
}

Vector_3D PDS_State::interpolateSpatialDirection(double time, const Vector_3D& prev, const Vector_3D& curr) const
{
	assert(time >= prevTime && time <= currTime);

	double oneOverDeltaTime = 1.0 / (prevTime - currTime);
	return prev + (time - prevTime) * (oneOverDeltaTime * (prev - curr));
}

Vector_3D PDS_State::interpolateEulerAngles(double time, const Vector_3D& prev, const Vector_3D& curr) const
{
	// Currently not sure how to interpolate between Euler angles. If interpolating this is important, maybe
	// just convert to Quaternions. But for now just return prev Euler Angles.
	assert(time >= prevTime && time <= currTime);

	return prev;
}

Vector_3D PDS_State::interpolateHubPosition(double time) const
{
	return interpolateSpatialDirection(time, prevHubPosition, currHubPosition);
}

Vector_3D PDS_State::interpolateHubOrientation(double time) const
{
	return interpolateEulerAngles(time, prevHubOrientation, currHubOrientation);
}

Vector_3D PDS_State::interpolateHubVelocity(double time) const
{
	return interpolateSpatialDirection(time, prevHubVelocity, currHubVelocity);
}

Vector_3D PDS_State::interpolateHubRotationalVelocity(double time) const
{
	return interpolateEulerAngles(time, prevHubRotationalVelocity, currHubRotationalVelocity);
}