
#include "AeroDyn_Interface_Wrapper.h"
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>

// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INITAERODYN(const char* inputFilename, int* fname_len, bool* useAddedMass, double* fluidDensity, double* kinematicFluidVisc,
	    double* hubPos, double* hubOri, double* hubVel, double* hubRotVel, 
	    double* bladePitch, int* nBlades_out, int* nNodes_out, double* turbDiameter_out, void** simulationInstance_out,
		int* errStat, char* errMsg);

	void INTERFACE_INITINFLOWS(void* simulationInstance, int* nBlades, int* nNodes, const double inflows[]);

	void INTERFACE_SETHUBMOTION(void* simulationInstance, double* time, double hubPos[3], double hubOri[3*3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_SETHUBMOTION_FAKE(void* simulationInstance, double* time, double hubPos[3], double hubOri[3*3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_SETINFLOWS(void* simulationInstance, int* nBlades, int* nNodes, double* inflows);

	void INTERFACE_SETINFLOWS_FAKE(void* simulationInstance, int* nBlades, int* nNodes, double* inflows);

	void INTERFACE_UPDATESTATES(void* simulationInstance, double* force_out,
		double* moment_out, double* power_out, double* tsr_out, double massMatrix_out[6*6], double addedMassMatrix_out[6*6]);

	void INTERFACE_UPDATESTATES_FAKE(void* simulationInstance, double* force_out,
		double* moment_out, double* power_out, double* tsr_out, double massMatrix_out[6*6], double addedMassMatrix_out[6*6]);

	void INTERFACE_GETBLADENODEPOS(void* simulationInstance, double* nodePos);

	void INTERFACE_GETBLADENODEPOS_FAKE(void* simulationInstance, double* nodePos);

	void INTERFACE_END(void* simulationInstance);
}


Vector3d AeroDyn_Interface_Wrapper::Transform_PDStoAD(const Vector3d& v) const
{
	return Vector3d(v.x(), -v.y(), -v.z());
}


Vector3d AeroDyn_Interface_Wrapper::Transform_ADtoPDS(const Vector3d& v) const
{
	return Vector3d(v.x(), -v.y(), -v.z());
}

Matrix3d AeroDyn_Interface_Wrapper::TransformOrientation(const Matrix3d& orientation) const
{
	Matrix3d trans;
	trans.row(0) = orientation.row(0);
	trans.row(1) = -1.0 * orientation.row(1);
	trans.row(2) = -1.0 * orientation.row(2);

	return trans;
}

Matrix6d AeroDyn_Interface_Wrapper::Transform_ADtoPDS_MassMatrix(const Matrix6d& m) const
{
	// TODO figure out how to transform a mass matrix from positive-up to positive-down (roll pi radians)
	return m;
}

// Returns the inflows transformed from PDS' coordinate system to that of AD's
void AeroDyn_Interface_Wrapper::TransformInflows_PDStoAD(const std::vector<double>& pdsInflows)
{
	// iterate through all the x, y, z components of the inflows, negating the y and z components
	for (int i = 0; i < totalNodes; ++i) {
		aerodynInflows[(i * 3) + 0] = pdsInflows[(i * 3) + 0];
		aerodynInflows[(i * 3) + 1] = -pdsInflows[(i * 3) + 1];
		aerodynInflows[(i * 3) + 2] = -pdsInflows[(i * 3) + 2];
	}
}

AeroDyn_Interface_Wrapper::AeroDyn_Interface_Wrapper()
{
	nBlades = nNodes = totalNodes = 0;

	hubReactionLoads.tsr = 0.0;
	hubReactionLoads.power = 0.0;

	simulationInstance = 0;
}

AeroDyn_Interface_Wrapper::~AeroDyn_Interface_Wrapper()
{
	INTERFACE_END(simulationInstance);
}

void AeroDyn_Interface_Wrapper::InitAerodyn(
	const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	bool useAddedMass,
	const Vector3d& hubPosition,
	const Matrix3d& hubOrientation,
	const Vector3d& hubVelocity,
	const Vector3d& hubAngularVel,
	double bladePitch)
{
	// Holds the error status returned from AeroDyn_Interface's initialization function
	int errStat = 0;
	// Holds any error message related to the error status (the max length of an AeroDyn 
	// error message is 1024, so we add one to account for the null character).
	char errMsg[1025];

	// get the length of the string to pass to the FORTRAN subroutine (FORTRAN needs the length, because it 
	// doesn't recognize null-terminated character as the end of the string)
	int fname_len = strlen(inputFilename);

	// transform them to Aerodyn's global coordinate system
	Vector3d hubPosition_trans = Transform_PDStoAD(hubPosition);
	Vector3d hubVelocity_trans = Transform_PDStoAD(hubVelocity);
	Vector3d hubAngularVel_trans = Transform_PDStoAD(hubAngularVel);
	Matrix3d hubOrientation_trans = TransformOrientation(hubOrientation);

	// call the initialization subroutine in the FORTRAN DLL
	INTERFACE_INITAERODYN(inputFilename, &fname_len, &useAddedMass, &fluidDensity, &kinematicFluidVisc,
	    hubPosition_trans.data(), hubOrientation_trans.data(), hubVelocity_trans.data(), hubAngularVel_trans.data(),
		&bladePitch, &nBlades, &nNodes, &turbineDiameter, &simulationInstance, &errStat,
		errMsg);

	// check the error status number 
	if (errStat == 4) {
		throw ADErrorException(errMsg);
	}

	// return the total amount of nodes used in the simulation
	totalNodes = nBlades * nNodes;

	// resize our internal vector so it can hold the velocity components of each node
	aerodynInflows.resize(totalNodes * 3);

	// Save the pitch at this time for later
	pitch = bladePitch;
}

// Transform each inflow from PDS' coordinate system (positive-down) to AD's (positive-up), and send them 
// to AD for its initialization.
void AeroDyn_Interface_Wrapper::InitInflows(const std::vector<double>& pdsInflows)
{
	// update member variable aerodynInflow with transformed inflows passed to this function
	TransformInflows_PDStoAD(pdsInflows);

	// call inflow initialization subroutine in FORTRAN DLL with these transformed inflows
	INTERFACE_INITINFLOWS(simulationInstance, &nBlades, &nNodes, &aerodynInflows[0]);
}

void AeroDyn_Interface_Wrapper::SetHubMotion(double time,
	const Vector3d& hubPosition,
	const Matrix3d& hubOrientation,
	const Vector3d& hubVelocity,
	const Vector3d& hubAngularVel,
	double bladePitch,
	bool isRealStep)
{

	// transform them to Aerodyn's global coordinate system 
	Vector3d hubPos_trans = Transform_PDStoAD(hubPosition);
	Vector3d hubVel_trans = Transform_PDStoAD(hubVelocity);
	Vector3d hubAngVel_trans = Transform_PDStoAD(hubAngularVel);
	Matrix3d hubOri_trans = TransformOrientation(hubOrientation);

	// If this is a fake-step, then we don't want this to be permanent, so we call the fake version
	if (!isRealStep) {
		INTERFACE_SETHUBMOTION_FAKE(simulationInstance, &time, hubPos_trans.data(), hubOri_trans.data(), hubVel_trans.data(),
			hubAngVel_trans.data(), &bladePitch);
	}
	// Otherwise we call the real version, which perminantly changes the inputs
	else {
		INTERFACE_SETHUBMOTION(simulationInstance, &time, hubPos_trans.data(), hubOri_trans.data(), hubVel_trans.data(),
			hubAngVel_trans.data(), &bladePitch);
	}

	pitch = bladePitch;
}

void AeroDyn_Interface_Wrapper::SetInflowVelocities(const std::vector<double>& inflows, bool isRealStep)
{
	// Assigns the transformed inflows to aeroDynInflows
	TransformInflows_PDStoAD(inflows);

	if (isRealStep) {
		INTERFACE_SETINFLOWS(simulationInstance, &nBlades, &nNodes, aerodynInflows.data());
	}
	else
	{
		INTERFACE_SETINFLOWS_FAKE(simulationInstance, &nBlades, &nNodes, aerodynInflows.data());
	}
}

// If isRealStep == false, the temporary instance of AeroDyn is updated; otherwise, the main instance of AeroDyn is 
// updated. Once that is done, the resulting reaction loads are returned.
AeroDyn_Interface_Wrapper::HubReactionLoads AeroDyn_Interface_Wrapper::UpdateStates(bool isRealStep)
{
	// If this is a fake-step, then we don't want this to be permanent, so we call the fake version
	if ( !isRealStep ) {
		// Note, we're passing our transformed inflows here, not the inflows from the 
		// function parameter.
		INTERFACE_UPDATESTATES_FAKE(simulationInstance, hubReactionLoads.force.data(), hubReactionLoads.moment.data(), &hubReactionLoads.power,
			&hubReactionLoads.tsr, hubReactionLoads.massMatrix.data(), hubReactionLoads.addedMassMatrix.data());
	}
	// Otherwise we call the real version, which permanently changes the states for this turbine
	else {
		// Note, we're passing our transformed inflows here, not the inflows from the 
		// function parameter.
		INTERFACE_UPDATESTATES(simulationInstance, hubReactionLoads.force.data(), hubReactionLoads.moment.data(), &hubReactionLoads.power,
			&hubReactionLoads.tsr, hubReactionLoads.massMatrix.data(), hubReactionLoads.addedMassMatrix.data());
	}

	// Transform the results into PDS' coordinate system
	hubReactionLoads.force = Transform_ADtoPDS(hubReactionLoads.force);
	hubReactionLoads.moment = Transform_ADtoPDS(hubReactionLoads.moment);
	hubReactionLoads.massMatrix = Transform_ADtoPDS_MassMatrix(hubReactionLoads.massMatrix);
	hubReactionLoads.addedMassMatrix = Transform_ADtoPDS_MassMatrix(hubReactionLoads.addedMassMatrix);

	return hubReactionLoads;
}

// Communicates blade node positions to ProteusDS. This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
void AeroDyn_Interface_Wrapper::GetBladeNodePositions(std::vector<double>& nodePos, bool isRealStep)
{
	// If we're in the process of performing a fake step, then we call the fake version, which returns the 
	// node positions according to the last call to the fake UpdateHubMotion
	if ( !isRealStep ) {
		// fills nodePos with node positions. Note! It assumes enough elements have been allocated
		INTERFACE_GETBLADENODEPOS_FAKE(simulationInstance, nodePos.data());
	} 
	// Otherwise, return the node positions according to the last call to normal UpdateHubMotion
	else {
		// fills nodePos with node positions. Note! It assumes enough elements have been allocated
		INTERFACE_GETBLADENODEPOS(simulationInstance, nodePos.data());
	}

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int i = 0; i < totalNodes; ++i) {
		nodePos[(i * 3) + 0] =  nodePos[(i * 3) + 0]; // x position of node (m)
		nodePos[(i * 3) + 1] = -nodePos[(i * 3) + 1]; // y position of node (m) flipped coordinates
		nodePos[(i * 3) + 2] = -nodePos[(i * 3) + 2]; // z position of node (m) flipped coordinates
	}
}

int AeroDyn_Interface_Wrapper::GetNumBlades() const
{
	return nBlades;
}

int AeroDyn_Interface_Wrapper::GetNumNodes() const
{
	return totalNodes;
}

double AeroDyn_Interface_Wrapper::GetBladePitch() const
{
	return pitch;
}

double AeroDyn_Interface_Wrapper::GetTurbineDiameter() const
{
	return turbineDiameter;
}

double AeroDyn_Interface_Wrapper::GetTSR() const
{
	return hubReactionLoads.tsr;
}

double AeroDyn_Interface_Wrapper::GetTorque() const
{
	return hubReactionLoads.moment.x();
}

Vector3d AeroDyn_Interface_Wrapper::GetForce() const
{
	return hubReactionLoads.force;
}

Vector3d AeroDyn_Interface_Wrapper::GetMoment() const
{
	return hubReactionLoads.moment;
}


