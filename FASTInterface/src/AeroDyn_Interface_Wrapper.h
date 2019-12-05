/* 
Description:
------------

This is a wrapper for AeroDyn_Interface, which is written in FORTRAN. Its main purpose is to provide Aerodyn with inputs,
for which it can return reaction loads on the hub. It also changes from ProteusDS' z-positive-down coordinate 
system to AeroDyn's z-positive-up coordinate system. 


Notes:
------

This interface actually manages two sets of AeroDyn instances:
one is the main instance, and the other is a temporary copy which can be updated without affecting the main states.
The temporary states are accessed whenever the flag "isRealStep" is passed as false. The instances 
are defined in the FORTRAN layer of the code; that layer uses <FUNCTIONNAME>_FAKE to get/set 
the temporary AeroDyn instance, and <FUNCTIONNAME> to get/set the main AeroDyn instance.

When SetHubMotion is called with isRealStep=false, the main states are copied, and the copy's input is updated with the rest of the 
parameters of SetHubMotion.

Example:
--------

The expected order of function calls is follows:

1) InitAerodyn, passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
2) GetBladeNodePositions, receiving the positions of the blade nodes in global coordinate system
3) InitInflows, passing the inflows at time=0
----------- simulation loop -------------
4) SetHubMotion, passing hubstate at time = 0
5) GetBladeNodePositions, ...
6) SetInflowVelocities(...)
7) Simulate, passing the inflow at time = 0
---next iteration---
8) SetHubMotion, passing hubstate at time = t_1
9) GetBladeNodePositions, ...
10) SetInflowVelocities(...)
11) Simulate, passing the inflow at time = t_1

etc.
*/

#pragma once

#include <vector>
#include "FASTInterfaceExceptions.h"
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
		const Matrix3d& hubOrientation,        // 3x3 orientation matrix
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
		double time,                        // the moment in time that the inputs describe (seconds)
		const Vector3d& hubPosition,        // position of the hub in global coordinate system (meters)
		const Matrix3d& hubOrientation,		// orientation matrix in ProteusDS' z-positive-down coordinate system
		const Vector3d& hubVelocity,	    // velocity of the hub in the global coordinate system (meters/sec)
		const Vector3d& hubAngularVelocity, // angular velocity of the hub in global coordinate system (axis-angle)
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

	// Returns the TSR of the last call to UpdateStates (whether it was with isRealStep == true or false)
	double GetTSR() const;

	// Returns the torque resulting from the last call to UpdateStates
	double GetTorque() const; 

	// Returns the force resulting from the last call to UpdateStates
	Vector3d GetForce() const; 

	// Returns the moment resulting from the last call to UpdateStates
	Vector3d GetMoment() const;

	// Returns the last actual pitch value (the one that have been assigned via isRealStep == true).
	double GetBladePitch() const; 

private:

	// Transforms a vector expressed in a z-positive-down coordinate system, to a z-positive-up coordinate system
	Vector3d Transform_PDStoAD(const Vector3d& v) const;

	// Transforms a vector expressed in a z-positive-up coordinate system, to a z-positive-down coordinate system
	Vector3d Transform_ADtoPDS(const Vector3d& v) const;

	// Transforms a vector expressed in a z-positive-down coordinate system, to a z-positive-up coordinate system
	Matrix6d Transform_ADtoPDS_MassMatrix(const Matrix6d& m) const;

	// Transforms the global to local orientation matrix from z-down to z-up coordinate system
	// Note 
	Matrix3d TransformOrientation(const Matrix3d& orientation) const;

	// updates aerodynInflows with transformed pdsInflows
	void TransformInflows_PDStoAD(const std::vector<double>& pdsInflows);

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