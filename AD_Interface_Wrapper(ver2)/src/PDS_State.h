#pragma once

#include "Vector_3D.h"
#include <vector>


class PDS_State {
public:
	PDS_State();

	void allocate(int totalNodes);

	// Above here, PDS will have used the hub state to find the node positions, and then used
	// those node positions to find inflows (by passing hub state to AD and getting node positions).
	void updateStates(double time,
		const double hubPosition[3],
		const double hubOrientation[3],
		const double hubVelocity[3],
		const double hubRotationalVelocity[3],
		const std::vector<double>& inflows);

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
