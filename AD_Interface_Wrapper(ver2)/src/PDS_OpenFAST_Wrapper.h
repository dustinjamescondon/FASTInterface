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
6) simulate, passing the inflow at time = 0
---next iteration---
7) setHubState, passing hubstate at time = t_1
8) getBladeNodePositions, ...
9) simulate, passing the inflow at time = t_1

etc.
*/

#pragma once

// Note, this is defined in the project preprocessor section
#ifdef AD_INTERFACE_WRAPPER_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

#include <vector>

class PDS_AD_Wrapper {
public:
	DECLDIR PDS_AD_Wrapper();

	// Initialize AeroDyn by loading input files and setting initial hub state. Returns the total amount of 
	// nodes used in the simulation.
	int DECLDIR initHub(
		const char* inputFilename,		 // filename (including path) of the main driver input file
		double fluidDensity,             // 
		double kinematicFluidVisc,       //
		const double hubPosition[3],			 // in meters
		const double hubOrientation[3],        // in Euler angles, radians
		const double hubVelocity[3],			 // in meters/sec
		const double hubRotationalVelocity[3], // axis-angle in global coordinate system
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
	void DECLDIR updateHubState(
		double time,                    // the moment in time that the inputs describe (seconds)
		const double hubPosition[3],          // position of the hub in global coordinate system (meters)
		const double hubOrientation[3],       // euler angles describing orientation of the hub of the hub (radians)
		const double hubVelocity[3],          // velocity of the hub in the global coordinate system (meters/sec)
		const double hubRotationalVelocity[3],// rotational velocity of the hub in global coordinate system (axis-angle?)
		double shaftSpeed,              // rotational speed of the hub along its axis (radians/sec)
		double bladePitch);

	// once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	void DECLDIR getBladeNodePositions(std::vector<double>& bladeNodePositions);
	
	// then we call this, passing the inflows at "time", and we will get back the load and moment
	// at time passed to updateHubState
	void DECLDIR simulate(
		std::vector<double>& inflows,
		double force_out[3],
		double moment_out[3],
		double power_out[3],
		double massMatrix_out[6][6],
		double addedMassMatrix[6][6]);

private:
	// updates aerodynInflows with transformed pdsInflows
	void transformInflows_PDStoAD(const std::vector<double>& pdsInflows);

	std::vector<double> aerodynInflows;
	int totalNodes;
	int nBlades;
	int nNodes;
};