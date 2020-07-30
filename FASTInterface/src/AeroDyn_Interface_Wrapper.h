/*

TODO REWRITE THIS DESCRIPTION
Description:
------------

This is a wrapper for AeroDyn_Interface, which is written in FORTRAN. Its main purpose is to provide Aerodyn with inputs,
for which it can return reaction loads on the hub. It also changes from ProteusDS' z-positive-down coordinate
system to AeroDyn's z-positive-up coordinate system.


Notes:
------
TODO if they're going to work for the project's purposes, describe save states_pred and restore states_pred function

Example:
--------

The expected order of function calls is follows:

1) InitAerodyn, passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
2) GetBladeNodePositions, receiving the positions of the blade nodes in global coordinate system
3) InitInputs_Inflow, passing the inflows at time_act=0
----------- simulation loop -------------
4) Set_Inputs_Hub, passing hubstate at time_act = 0
5) GetBladeNodePositions, ...
6) SetInflowVelocities(...)
7) Simulate, passing the inflow at time_act = 0
---next iteration---
8) Set_Inputs_Hub, passing hubstate at time_act = t_1
9) GetBladeNodePositions, ...
10) SetInflowVelocities(...)
11) Simulate, passing the inflow at time_act = t_1

etc.
*/

#pragma once

#include <vector>
#include "FASTInterfaceExceptions.h"
#include "Eigen/SparseCore"

using Eigen::Matrix3d;
using Eigen::Vector3d;

class AeroDyn_Interface_Wrapper {
public:

	struct HubReactionLoads {
		Vector3d force;
		Vector3d moment;
		double power;
		double tsr;
	};

	AeroDyn_Interface_Wrapper();

	~AeroDyn_Interface_Wrapper();

	// Initializes AeroDyn by loading input files and setting initial hub state
	void InitAerodyn(
		const char* inputFilename,			   // filename (including path) of the AeroDyn input file
		const char* outputFilename,			   // root name of the output file to be generated
		double timestep,                       // the global_timestep that AeroDyn will take
		int numBlades,                         //
		double hubRadius,                      //
		double precone,                        // in radians
		bool useAddedMass,                     // have AeroDyn include added mass effects
		double coeffAddedMass,                 // coefficient of added mass (only used when useAddedMass is true)
		const Vector3d& hubPosition,	       // metres
		const Matrix3d& hubOrientation,        // 3x3 orientation matrix
		const Vector3d& hubVel,	               // metres/sec
		const Vector3d& hubAcc,		       // metres/sec^2
		const Vector3d& hubRotationalVel,      // axis-angle form in global coordinate system
		const Vector3d& hubRotationalAcc,      // axis-angle form in global coordinate system	
		double bladePitch);                    // radians

		// Initializes the inflows. Note, inflow velocities are in global coordinate system
	void InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);
	// The format expected is (in global coordinate system)
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc

	void SaveCurrentStates();

	void RestoreSavedStates();

	// Updates the hub motion variables, changing the positions of the nodes accordingly. Call
	// this before getBladeNodePositions(...)
	void Set_Inputs_Hub(
		double time,                        // the moment in time_act that the inputs describe (seconds)
		const Vector3d& hubPosition,        // position of the hub in global coordinate system (meters)
		const Matrix3d& hubOrientation,	    // orientation matrix in ProteusDS' z-positive-down coordinate system
		const Vector3d& hubVel,	    // velocity of the hub in the global coordinate system (meters/sec)
		const Vector3d& hubAcc,             // ^ (m/sec^2)
		const Vector3d& hubAngularVel, // angular velocity of the hub in global coordinate system (axis-angle)
		const Vector3d& hubAngularAcc,      // angular acceleration ^ 
		double bladePitch);

	// This is just like Set_Inputs_Hub, but it doesn't advance the input window and only sets the accelerations. 
	// The expected use of this function is to use to calculate the partial derivatives 
	void Set_Inputs_HubAcceleration(
		const Vector3d& hubAcc,
		const Vector3d& hubAngularAcc);

	// This should be called before UpdateStates
	void Set_Inputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);

	// TODO
	HubReactionLoads CalcOutput();

	HubReactionLoads GetOutput() const;

	void Advance_InputWindow();

	// Only need these for now (used in the input solver in the coordinating class)
	Vector3d GetInput_HubAcc() const { return input.hubAcc; }
	Vector3d GetInput_HubRotAcc() const { return input.hubRotAcc; }
	Matrix3d GetInput_HubOrient() const { return input.hubOrient; }

	// Once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	void GetBladeNodePositions(
		std::vector<double>& bladeNodePositions);
	// The format expected is the same as initInflows. So indices 0,1,2 correspond to the x,y,z 
	// position of node 0; and inflow indices 0,1,2 represent the x,y,z inflow velocity for that 
	// node.

	// Returns the total number of nodes (call after InitAerodyn)
	int GetNumNodes() const;

	// Returns the number of blades (call after InitAerodyn)
	int GetNumBlades() const;

	// Returns the diameter of the turbine (call after InitAeroDyn)
	double GetTurbineDiameter() const;
	
	// TODO
	void UpdateStates();

	// TODO
	void CopyStates_Pred_to_Curr();

	// TODO
	void PrintOutputLine();
	
	HubReactionLoads GetHubReactionLoads() const;

	// Returns the TSR of the last call to UpdateStates (whether it was with isRealStep == true or false)
	double GetTSR() const;

	// Returns the torque resulting from the last call to UpdateStates
	double GetTorque() const;

	// Returns the force resulting from the last call to UpdateStates
	Vector3d GetForce() const;

	// Returns the moment resulting from the last call to UpdateStates
	Vector3d GetMoment() const;

	// Returns the last actual pitch value
	double GetBladePitch() const;

private:

	struct Input {
		Vector3d hubPos, hubVel, hubRotVel, hubAcc, hubRotAcc;
		Matrix3d hubOrient;
		double bladePitch;
	};

	// Transforms a vector expressed in a z-positive-down coordinate system, to a z-positive-up coordinate system
	Vector3d Transform_PDStoAD(const Vector3d& v) const;

	// Transforms a vector expressed in a z-positive-up coordinate system, to a z-positive-down coordinate system
	Vector3d Transform_ADtoPDS(const Vector3d& v) const;

	// Transforms the local to global orientation matrix from z-down to z-up coordinate system
	// Note 
	Matrix3d TransformOrientation(const Matrix3d& orientation) const;

	// updates aerodynInflows with transformed pdsInflows
	void TransformInflows_PDStoAD(const std::vector<double>& pdsInflowVel, const std::vector<double>& pdsInflowAcc);

	// the turbine instance pointer for the current instance of the class (points to a FORTRAN type)
	void* simulationInstance;

	HubReactionLoads hubReactionLoads, hubReactionLoads_saved; // Saved reaction loads from last call to UpdateStates

	// Save the inputs 
	Input input, input_saved;
	std::vector<double> aerodynInflowVel, aerodynInflowVel_saved;
	std::vector<double> aerodynInflowAcc, aerodynInflowAcc_saved;


	double turbineDiameter;
	double pitch, pitch_saved;
	int totalNodes;
	int nBlades;
	int nNodes; // number of nodes per blade
};
