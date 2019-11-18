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
#include "FASTTurbineExceptions.h"
#include "Eigen/SparseCore"

// Define this for the mass matrix
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class AeroDyn_Interface_Wrapper {
public:

	struct HubReactionLoads {
		Vector3d force;
		Vector3d moment;
		double power;
		double tsr;
		Matrix6d massMatrix;
		Matrix6d addedMassMatrix;
	};

	AeroDyn_Interface_Wrapper();

	~AeroDyn_Interface_Wrapper();

	// Initializes AeroDyn by loading input files and setting initial hub state
	void InitAerodyn(
		const char* inputFilename,		       // filename (including path) of the main driver input file
		double fluidDensity,                   // kg/m^3
		double kinematicFluidVisc,             // m^2/sec
		bool useAddedMass,                     // have AeroDyn include added mass effects
		const Vector3d& hubPosition,		   // metres
		const Matrix3d& hubOrientation,        // Euler angles (in radians)
		const Vector3d& hubVelocity,		   // metres/sec
		const Vector3d& hubRotationalVelocity, // axis-angle form in global coordinate system
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
		double time,                    // the moment in time that the inputs describe (seconds)
		const Vector3d& hubPosition,			// position of the hub in global coordinate system (meters)
		const Matrix3d& hubOrientation,		// euler angles describing orientation of the hub of the hub (radians)
		const Vector3d& hubVelocity,			// velocity of the hub in the global coordinate system (meters/sec)
		const Vector3d& hubAngularVelocity,    // angular velocity of the hub in global coordinate system (axis-angle)
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
	
	// Then we call this, which returns the reaction loads (and other things) from AeroDyn
	HubReactionLoads UpdateStates(bool isRealStep = true);

	double GetTSR() const;
	double GetTorque() const; // Returns the torque resulting from the last call to UpdateStates
	Vector3d GetForce() const; 
	Vector3d GetMoment() const;
	double GetBladePitch() const; // Returns the last actual pitch value (ones that have been assigned via isRealStep == true).

private:
	Vector3d Transform_PDStoAD(const Vector3d& v) const;

	Vector3d Transform_ADtoPDS(const Vector3d& v) const;

	Matrix6d Transform_ADtoPDS_MassMatrix(const Matrix6d& m) const;

	// Transforms the global to local orientation matrix from z-down to z-up coordinate system
	// Note 
	Matrix3d TransformOrientation(const Matrix3d& orientation) const;

	// updates aerodynInflows with transformed pdsInflows
	void TransformInflows_PDStoAD(const std::vector<double>& pdsInflows);

	void TransformHubKinematics_PDStoAD(Vector3d& hubPos, Matrix3d& hubOri, Vector3d& hubVel, Vector3d& hubRotVel);

	// the turbine instance pointer for the current instance of the class (points to a FORTRAN type)
	void* simulationInstance;

	HubReactionLoads hubReactionLoads; // Saved reaction loads from last call to UpdateStates
	std::vector<double> aerodynInflows;
	double turbineDiameter;
	double pitch;
	int totalNodes;
	int nBlades;
	int nNodes; // number of nodes per blade
};