#pragma once

#include "Vector_AD.h"
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
		const vector<double>& inflows);

	Vector_AD interpolateHubPosition(double time) const;
	Vector_AD interpolateHubOrientation(double time) const;
	Vector_AD interpolateHubVelocity(double time) const;
	Vector_AD interpolateHubRotationalVelocity(double time) const;

private:
	double prevTime, currTime;

	std::vector<double> prevInflows, currInflows;

	Vector_AD prevHubPosition, currHubPosition;
	Vector_AD prevHubOrientation, currHubOrientation;
	Vector_AD prevHubVelocity, currHubVelocity;
	Vector_AD prevHubRotationalVelocity, currHubRotationalVelocity;
};
