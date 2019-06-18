#include "..\..\AD_Interface_Wrapper(ver2)\src\PDS_OpenFAST_Wrapper.h"
#include <iostream>

#include <vector>
#include <string.h>  
#include <fstream>

void updateHubOrientation(Vector_3D& hubOri, Vector_3D hubRotVel, double dt);
void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed);

int main(int argc, char *argv[])
{	
	// parameters
	double simulationTime = 5.0; // in seconds
	double shaftSpeed = 2.0;     // in rads/sec
	double dt = 0.01;
	double bladePitch = 0.0;
	double inflowSpeed = 8.7;

	// local variables
	int nSteps = (int)(simulationTime / dt);
	double time = 0.0;

	std::ofstream outfile("loads.out", std::ofstream::out);
	outfile << "Forces(x,y,z)     Moments(Mx, My, Mz) \n";

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

	PDS_AD_Wrapper adWrapper;

	totalNodes = adWrapper.initHub("input/ad_driver_example.inp", hubPos, hubOri, hubVel, hubRotVel,
		shaftSpeed, bladePitch, &nBlades, &nNodes);

	inflows.resize(totalNodes * 3, 0.0);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	createInflows(inflows, totalNodes, inflowSpeed);

	adWrapper.initInflows(inflows);
	
	// Testing loop
	for (int i = 0; i <= nSteps; i++)
	{
		time = i * dt;
		updateHubOrientation(hubOri, hubRotVel, dt);

		adWrapper.updateHubState(time, hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch);
		adWrapper.solve(inflows, force, moment, massMatrix, addedMassMatrix);

		outfile << "( " << force.x() << ", " << force.y() << ", " << force.z() << " ); ( "
			<< moment.x() << ", " << moment.y() << ", " << moment.z() << " )" << std::endl;


	}
	outfile.close();

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
