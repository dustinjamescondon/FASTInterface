#pragma once

// Note, this is defined in the project preprocessor section
#ifdef AD_INTERFACE_WRAPPER_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

#include "PDS_State.h"
#include <vector> // for vector data type used by ProteusDS

class PDS_AD_Wrapper {
public:
	DECLDIR PDS_AD_Wrapper();

	// Initialize AeroDyn by loading input files and setting initial hub state. Returns the total amount of 
	// nodes used in the simulation.
	int DECLDIR initHub(
		const char* inputFilename,
		Vector_3D hubPosition,
		Vector_3D hubOrientation,
		Vector_3D hubVelocity,
		Vector_3D hubRotationalVelocity,
		double shaftSpeed, // rotional speed of the shaft in rads/sec
		double bladePitch,
		int* nBlades_out,  // number of blades, to be assigned upon calling the function
		int* nNodes_out);  // number of nodes per blade, to be assigned upon calling the function

	// Initialize the inflows. The format expected is (in global coordinate system):
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc
	void DECLDIR initInflows(std::vector<double>& inflows);

	// call this before calling solve, 
	void DECLDIR updateHubState(double time,
		Vector_3D hubPosition,
		Vector_3D hubOrientation,
		Vector_3D hubVelocity,
		Vector_3D hubRotationalVelocity,
		double shaftSpeed,
		double bladePitch);

	// once updateHubState has been called, we call this to get where that input put the blade nodes
	void DECLDIR getBladeNodePositions(std::vector<double>& bladeNodePositions);
	
	//  then we call this, passing the inflows at "time", and we will get back the loads and moments from 
	// previous time up until this current time
	void DECLDIR solve(
		std::vector<double>& inflows,
		Vector_3D& force_out,
		Vector_3D& moment_out,
		double massMatrix_out[6][6],
		double addedMassMatrix[6][6]);

private:

	int nBlades;
	int nNodes;
};