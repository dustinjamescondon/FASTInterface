#include "..\..\AD_Interface_Wrapper(ver2)\src\PDS_OpenFAST_Wrapper.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <windows.h>  
#include <stdlib.h>  
#include <string.h>  
#include <tchar.h>  

using namespace std;

void updateHubOrientation(Vector_3D& hubOri, Vector_3D hubRotVel, double dt);

int main(int argc, char *argv[])
{	

	double shaftSpeed = 2.0;
	double time = 0.0;
	double dt = 0.01;

	Vector_3D hubPos(0.0, 0.0, 0.0);
	Vector_3D hubOri(0.0, 0.0, 0.0);
	Vector_3D hubVel(0.0, 0.0, 0.0);
	Vector_3D hubRotVel(shaftSpeed, 0.0, 0.0);

	double bladePitch = 0.0;
	double inflowSpeed = 8.7;
	int nBlades, nNodes;

	Vector_3D force, moment;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	std::vector<double> inflows;

	PDS_AD_Wrapper adWrapper;

	adWrapper.init_inputFiles("input/ad_driver_example.inp", hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch,
		&nBlades, &nNodes);

	inflows.resize(nBlades * nNodes * 3, 0.0);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	for (int i = 0; i < nBlades; i++)
		for (int j = 0; j < nNodes; j++)
			inflows[3 * nNodes * i + 3 * j + 0] = inflowSpeed;  // m/s flow rate in x direction

	adWrapper.init_inflows(inflows);

	ofstream outfile;
	outfile.open("Forces.out");
	outfile << "Forces(x,y,z)     Moments(Mx, My, Mz) \n";
	
	// Testing loop
	for(int i=1; i<=8; i++)
	{
		time = i * dt;
		updateHubOrientation(hubOri, hubRotVel, dt);

		adWrapper.updateHubState(time, hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch);
		adWrapper.solve(inflows, force, moment, massMatrix, addedMassMatrix);
	
	}

	outfile.close();

	return 0;
}

void updateHubOrientation(Vector_3D& hubOri, Vector_3D hubRotVel, double dt)
{
	hubOri.x() += hubRotVel.x() * dt;
}

