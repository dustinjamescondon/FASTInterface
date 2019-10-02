/* 
Wrapper for AeroDyn_Interface, which is written in FORTRAN. Main purpose is to provide Aerodyn with inputs, for which it 
can return hub load and moment, as well as power. 

The expected order of function calls is follows:

1) InitAerodyn, passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
2) GetBladeNodePositions, receiving the positions of the blade nodes in global coordinate system
3) InitInflows, passing the inflows at time=0
----------- simulation loop -------------
4) SetHubMotion, passing hubstate at time = 0
5) GetBladeNodePositions, ...
6) Simulate, passing the inflow at time = 0
---next iteration---
7) SetHubMotion, passing hubstate at time = t_1
8) GetBladeNodePositions, ...
9) Simulate, passing the inflow at time = t_1

etc.
*/

#pragma once

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

class AeroDyn_Interface_Wrapper {
public:
	AeroDyn_Interface_Wrapper();

	~AeroDyn_Interface_Wrapper();

	// Initializes AeroDyn by loading input files and setting initial hub state
	void InitAerodyn(
		const char* inputFilename,		       // filename (including path) of the main driver input file
		double fluidDensity,                   // kg/m^3
		double kinematicFluidVisc,             // m^2/sec
		const double hubPosition[3],		   // metres
		const double hubOrientation[3],        // Euler angles (in radians)
		const double hubVelocity[3],		   // metres/sec
		const double hubRotationalVelocity[3], // axis-angle form in global coordinate system
		double bladePitch);                    // radians

	// Initializes the inflows. Note, inflow velocities are in global coordinate system
	void InitInflows(const std::vector<double>& inflows);
	// The format expected is (in global coordinate system)
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc

	// Updates the hub motion variables, changing the positions of the nodes accordingly. Call
	// this before getBladeNodePositions(...)
	void SetHubMotion(
		double time,                          // the moment in time that the inputs describe (seconds)
		const double hubPosition[3],          // position of the hub in global coordinate system (meters)
		const double hubOrientation[3],       // euler angles describing orientation of the hub of the hub (radians)
		const double hubVelocity[3],          // velocity of the hub in the global coordinate system (meters/sec)
		const double hubRotationalVelocity[3],// rotational velocity of the hub in global coordinate system (axis-angle)
		double bladePitch,
		bool isRealStep = true);

	// Removed this from UpdateStates, so now this should be called before UpdateStates
	void SetInflowVelocities(const std::vector<double>& inflow, bool isRealStep=true);

	// Once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	void GetBladeNodePositions(
		std::vector<double>& bladeNodePositions,
		bool isRealStep = true);
	// The format expected is the same as initInflows. So indices 0,1,2 correspond to the x,y,z 
	// position of node 0; and inflow indices 0,1,2 represent the x,y,z inflow velocity for that 
	// node.

	// Returns the total number of nodes (call after InitAerodyn)
	int GetNumNodes() const;

	// Returns the number of blades (call after InitAerodyn)
	int GetNumBlades() const;

	// Returns the diameter of the turbine (call after InitAeroDyn)
	double GetTurbineDiameter() const;
	
	// Then we call this, passing the inflow velocities at the time passed to UpdateHubMotion(...)
	// Returns the resulting force, moment, and power at that same time
	void UpdateStates(
		double force_out[3],
		double moment_out[3],
		double* power_out,
		double* tsr_out,
		double massMatrix_out[6][6],
		double addedMassMatrix_out[6][6],
		bool isRealStep = true);

	double GetTorque() const; // Returns the torque resulting from the last call to UpdateStates
	void GetForce(double[3]) const; 
	void GetMoment(double[3]) const;
	double GetBladePitch() const; // Returns the last actual pitch value (ones that have been assigned via isRealStep == true).

private:
	// updates aerodynInflows with transformed pdsInflows
	void TransformInflows_PDStoAD(const std::vector<double>& pdsInflows);

	// the turbine instance pointer for the current instance of the class (points to a FORTRAN type)
	void* simulationInstance;

	std::vector<double> aerodynInflows;
	double turbineDiameter;
	double force[3];
	double moment[3];
	double pitch;
	int totalNodes;
	int nBlades;
	int nNodes; // number of nodes per blade
};