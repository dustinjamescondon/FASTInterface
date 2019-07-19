/* 
Wrapper for AeroDyn_Interface, which is written in FORTRAN. Main purpose is to provide Aerodyn with inputs, for which it 
can return hub load and moment, as well as power. 

The expected order of function calls is follows:

1) initAerodyn, passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
2) getBladeNodePositions, receiving the positions of the blade nodes in global coordinate system
3) initInflows, passing the inflows at time=0
----------- simulation loop -------------
4) setHubMotion, passing hubstate at time = 0
5) getBladeNodePositions, ...
6) simulate, passing the inflow at time = 0
---next iteration---
7) setHubMotion, passing hubstate at time = t_1
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

	DECLDIR ~PDS_AD_Wrapper();

	// Initializes AeroDyn by loading input files and setting initial hub state. Returns the total amount of 
	// nodes used in the simulation.
	int DECLDIR initAerodyn(
		const char* inputFilename,		       // filename (including path) of the main driver input file
		double fluidDensity,                   // kg/m^3
		double kinematicFluidVisc,             // m^2/sec
		double hubRadius,                      // metres
		const double hubPosition[3],		   // metres
		const double hubOrientation[3],        // Euler angles (in radians)
		const double hubVelocity[3],		   // metres/sec
		const double hubRotationalVelocity[3], // axis-angle form in global coordinate system
		double bladePitch,                     // radians
		int* nBlades_out,                      // number of blades, to be assigned upon calling the function
		int* nNodes_out);				       // number of nodes per blade, to be assigned upon calling the function

	// Initializes the inflows. Note, inflow velocities are in global coordinate system
	void DECLDIR initInflows(const std::vector<double>& inflows);
	// The format expected is (in global coordinate system)
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc

	// Updates the hub motion variables, changing the positions of the nodes accordingly. Call
	// this before getBladeNodePositions(...)
	void DECLDIR updateHubMotion(
		double time,                          // the moment in time that the inputs describe (seconds)
		const double hubPosition[3],          // position of the hub in global coordinate system (meters)
		const double hubOrientation[3],       // euler angles describing orientation of the hub of the hub (radians)
		const double hubVelocity[3],          // velocity of the hub in the global coordinate system (meters/sec)
		const double hubRotationalVelocity[3],// rotational velocity of the hub in global coordinate system (axis-angle?)
		double bladePitch);

	// Once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	void DECLDIR getBladeNodePositions(std::vector<double>& bladeNodePositions);
	// The format expected is the same as initInflows. So indices 0,1,2 correspond to the x,y,z 
	// position of node 0; and inflow indices 0,1,2 represent the x,y,z inflow velocity for that 
	// node.
	
	// Then we call this, passing the inflow velocities at the time passed to updateHubMotion(...)
	// Returns the resulting force, moment, and power at that same time
	void DECLDIR simulate(
		std::vector<double>& inflows,
		double force_out[3],
		double moment_out[3],
		double* power_out,
		double massMatrix_out[6][6],
		double addedMassMatrix_out[6][6]);

private:
	// updates aerodynInflows with transformed pdsInflows
	void transformInflows_PDStoAD(const std::vector<double>& pdsInflows);

	// the turbine index/number for the current instance of the class
	int turbineIndex;

	std::vector<double> aerodynInflows;
	int totalNodes;
	int nBlades;
	int nNodes;
};