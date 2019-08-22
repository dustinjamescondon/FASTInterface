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
#include <stdexcept>

// exception class for any other AeroDyn error
class ADError : public std::runtime_error {
public:
	inline ADError(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file cannot be found
class ADInputFileNotFound : public std::runtime_error {
public: 
	inline ADInputFileNotFound(const char* errMsg) : std::runtime_error(errMsg) {}
};

// exception class for when the AeroDyn input file has invalid contents
class ADInputFileContents : public std::runtime_error {
public:
	inline ADInputFileContents(const char* errMsg) : std::runtime_error(errMsg) {}
};

class PDS_AD_Wrapper {
public:
	DECLDIR PDS_AD_Wrapper();

	DECLDIR ~PDS_AD_Wrapper();

	// Initializes AeroDyn by loading input files and setting initial hub state. Returns the total amount of 
	// nodes used in the simulation.
	void DECLDIR InitAerodyn(
		const char* inputFilename,		       // filename (including path) of the main driver input file
		double fluidDensity,                   // kg/m^3
		double kinematicFluidVisc,             // m^2/sec
		double hubRadius,                      // metres (must be non-zero)
		const double hubPosition[3],		   // metres
		const double hubOrientation[3],        // Euler angles (in radians)
		const double hubVelocity[3],		   // metres/sec
		const double hubRotationalVelocity[3], // axis-angle form in global coordinate system
		double bladePitch);                    // radians

	// Initializes the inflows. Note, inflow velocities are in global coordinate system
	void DECLDIR InitInflows(const std::vector<double>& inflows);
	// The format expected is (in global coordinate system)
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc

	// Updates the hub motion variables, changing the positions of the nodes accordingly. Call
	// this before getBladeNodePositions(...)
	void DECLDIR UpdateHubMotion(
		double time,                          // the moment in time that the inputs describe (seconds)
		const double hubPosition[3],          // position of the hub in global coordinate system (meters)
		const double hubOrientation[3],       // euler angles describing orientation of the hub of the hub (radians)
		const double hubVelocity[3],          // velocity of the hub in the global coordinate system (meters/sec)
		const double hubRotationalVelocity[3],// rotational velocity of the hub in global coordinate system (axis-angle)
		double bladePitch);

	// Once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	void DECLDIR GetBladeNodePositions(std::vector<double>& bladeNodePositions);
	// The format expected is the same as initInflows. So indices 0,1,2 correspond to the x,y,z 
	// position of node 0; and inflow indices 0,1,2 represent the x,y,z inflow velocity for that 
	// node.

	// Returns the total number of nodes (call after InitAerodyn)
	int DECLDIR GetNumNodes() const;

	// Returns the number of blades (call after InitAerodyn)
	int DECLDIR GetNumBlades() const;

	double DECLDIR GetTurbineDiameter() const;
	
	// Then we call this, passing the inflow velocities at the time passed to updateHubMotion(...)
	// Returns the resulting force, moment, and power at that same time
	void DECLDIR Simulate(
		std::vector<double>& inflows,
		double force_out[3],
		double moment_out[3],
		double* power_out,
		double* tsr_out,
		double massMatrix_out[6][6],
		double addedMassMatrix_out[6][6]);

private:
	// updates aerodynInflows with transformed pdsInflows
	void TransformInflows_PDStoAD(const std::vector<double>& pdsInflows);

	// the turbine instance pointer for the current instance of the class (points to a FORTRAN type)
	void* simulationInstance;

	std::vector<double> aerodynInflows;
	double turbineDiameter;
	int totalNodes;
	int nBlades;
	int nNodes; // number of nodes per blade
};