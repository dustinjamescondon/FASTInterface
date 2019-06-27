/* 
Wrapper for AD_Interface, which is written in FORTRAN. Main purpose is to provide Aerodyn with inputs, for which it 
can return loads and moment on the hub. 

The expected order of function calls is follows:

1) initHub, passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
2) getBladeNodePositions, receiving the positions of the blade nodes relative to the hub coordinate system (must confirm this)
3) initInflows, passing the inflows at time=0
----------- simulation loop -------------
4) setHubState, passing hubstate at time = 0
5) getBladeNodePositions, ...
6) solve, passing the inflow at time = 0
---next iteration---
7) setHubState, passing hubstate at time = t_1
8) getBladeNodePositions, ...
9) solve, passing the inflow at time = t_1

etc.
*/

#pragma once

// Note, this is defined in the project preprocessor section
#ifdef AD_INTERFACE_WRAPPER_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

#include "Vector_3D.h"
#include <vector> // for vector data type used by ProteusDS

class PDS_AD_Wrapper {
public:
	DECLDIR PDS_AD_Wrapper();

	// Initialize AeroDyn by loading input files and setting initial hub state. Returns the total amount of 
	// nodes used in the simulation.
	int DECLDIR initHub(
		const char* inputFilename,		 // filename (including path) of the main driver input file
		Vector_3D hubPosition,			 // in meters
		Vector_3D hubOrientation,        // in Euler angles, radians
		Vector_3D hubVelocity,			 // in meters/sec
		Vector_3D hubRotationalVelocity, // in Euler angles, radians/sec
		double shaftSpeed,               // rotional speed of the shaft in radians/sec
		double bladePitch,               // pitch of the blade in radians
		int* nBlades_out,                // number of blades, to be assigned upon calling the function
		int* nNodes_out);				 // number of nodes per blade, to be assigned upon calling the function

	// Initialize the inflows
	void DECLDIR initInflows(const std::vector<double>& inflows);
	// The format expected is(in global coordinate system or hub coordinate system?)
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc

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
	std::vector<double> transformInflows_PDStoAD(const std::vector<double>& pdsInflows) const;

	int totalNodes;
	int nBlades;
	int nNodes;
};