#pragma once

#include "Vector_3D.h"
#include <vector>


class PDS_State {
public:
	PDS_State();

	void allocate(int totalNodes);

	void initializeHubState(
		Vector_3D hubPosition,
		Vector_3D hubOrientation,
		Vector_3D hubVelocity,
		Vector_3D hubRotationalVelocity);

	void initializeInflows(const std::vector<double>& inflows);


	// Above here, PDS will have used the hub state to find the node positions, and then used
	// those node positions to find inflows (by passing hub state to AD and getting node positions).
	void updateStates(double time,
		Vector_3D hubPosition,
		Vector_3D hubOrientation,
		Vector_3D hubVelocity,
		Vector_3D hubRotationVelocity,
		const std::vector<double>& inflows);

	Vector_3D getCurrentHubPosition() const;
	Vector_3D getCurrentHubOrientation() const;
	Vector_3D getCurrentHubVelocity() const;
	Vector_3D getCurrentHubRotationalVelocity() const;

	Vector_3D interpolateHubPosition(double time) const;
	Vector_3D interpolateHubOrientation(double time) const;
	Vector_3D interpolateHubVelocity(double time) const;
	Vector_3D interpolateHubRotationalVelocity(double time) const;

private:
	Vector_3D interpolateSpatialDirection(double time, const Vector_3D& prev, const Vector_3D& curr) const;
	Vector_3D interpolateEulerAngles(double time, const Vector_3D& prev, const Vector_3D& curr) const;

	double prevTime, currTime;

	std::vector<double> prevInflows, currInflows;

	Vector_3D prevHubPosition, currHubPosition;
	Vector_3D prevHubOrientation, currHubOrientation;
	Vector_3D prevHubVelocity, currHubVelocity;
	Vector_3D prevHubRotationalVelocity, currHubRotationalVelocity;
};
