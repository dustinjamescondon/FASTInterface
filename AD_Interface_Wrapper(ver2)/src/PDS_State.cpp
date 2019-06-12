#include "PDS_State.h"

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

void PDS_State::updateStates(double time, 
	const double hubPositions[3],
	const double hubOrientation[3],
	const double hubVelocity[3],
	const double hubRotationVelocity[3],
	const std::vector<double>& inflows)
{
	prevInflows = currInflows;
	currInflows = inflows;

	prevTime = currTime;
	currTime = time;
}