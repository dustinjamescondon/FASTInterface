#include "..\..\AD_Interface_Wrapper(ver2)\src\PDS_OpenFAST_Wrapper.h"
#include <iostream>
#include <vector>

void updateHubOrientation(Vector_3D& hubOri, Vector_3D hubRotVel, double dt);
void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed);

int main(int argc, char *argv[])
{	
	//------------------------
	// Parameters
	double simulationTime = 5.0; // the amount of time to be simulated (in seconds)
	double shaftSpeed = 2.0;     // in rads/sec
	double dt = 0.01;            
	double bladePitch = 0.0;
	double inflowSpeed = 8.7;    // in meters/sec
	//-------------------------
	// Local variables
	int nSteps = (int)ceil(simulationTime / dt);
	double time = 0.0;

	Vector_3D hubPos(0.0, 0.0, 0.0);
	Vector_3D hubOri(0.0, 0.0, 0.0);
	Vector_3D hubVel(0.0, 0.0, 0.0);
	Vector_3D hubRotVel(shaftSpeed, 0.0, 0.0);

	int nBlades, nNodes;
	int totalNodes = 0;

	Vector_3D force, moment;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	std::vector<double> inflows;
	//-------------------------
	// Initialization

	// Create instance of the wrapper class
	PDS_AD_Wrapper adWrapper;

	totalNodes = adWrapper.initHub("input/ad_driver_example.inp", hubPos, hubOri, hubVel, hubRotVel,
		shaftSpeed, bladePitch, &nBlades, &nNodes);

	inflows.resize(totalNodes * 3, 0.0);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	createInflows(inflows, totalNodes, inflowSpeed);

	adWrapper.initInflows(inflows);

	std::cout << "Simulating..." << std::endl;
	
	//-------------------------------
	// Simulation loop
	for (int i = 0; i <= nSteps; i++)
	{
		// this would be where ProteusDS would take its time step, updating the hub kinematics.
		time = i * dt;
		updateHubOrientation(hubOri, hubRotVel, dt);

		// send Aerodyn the hub kinematics.
		adWrapper.updateHubState(time, hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch);

		// then would usually request the current node positions from Aerodyn by calling 
		// getBladeNodePos(...)
		// and then figure out what the inflows would be, but our inflow is just constant anyway,
		// so we just leave the inflow as constant.

		// then we call solve, which will make Aerodyn step forward and simulate up to 'time', and return the 
		// force moment at that time.
		adWrapper.solve(inflows, force, moment, massMatrix, addedMassMatrix);
	}

	return 0;
}

void updateHubOrientation(Vector_3D& hubOri, Vector_3D hubRotVel, double dt)
{
	hubOri.x() += hubRotVel.x() * dt;
}

void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed)
{
	for (int i = 0; i < totalNodes; i++)
	{
		inflows[i * 3] = inflowSpeed;
	}
}
