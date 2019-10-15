
#include "AeroDyn_Interface_Wrapper.h"
#include "..//eigen/Eigen/SparseCore"
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>

// this is for writing the input log to disk so we can read what ProteusDS is sending our AD interface
std::ofstream f;

void logHeader()
{
	f << "time \t pos \t eulerAngles \t vel \t angularVel \t bladePitch" << std::endl;
}

void logVector(const double* v)
{
	f << " ( " << v[0] << ", " << v[1] << ", " << v[2] << " ) \t";
}

void logInput(double time, const double* pos, const double* eulerAngles, const double* vel, const double* angularVel, double bladePitch)
{
	f << time << "\t";
	logVector(pos);
	logVector(eulerAngles);
	logVector(vel);
	logVector(angularVel);
	f << bladePitch << std::endl;
}

// Create a 6x6 matrix from Eigen3's template for transforming the mass matrix
using Eigen::Matrix;
using Eigen::Vector3d;

typedef Matrix<double, 6, 6> Matrix6d;

// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INITAERODYN(const char* inputFilename, int* fname_len, double* fluidDensity, double* kinematicFluidVisc,
	    double* hubPos, double* hubOri, double* hubVel, double* hubRotVel, 
	    double* bladePitch, int* nBlades_out, int* nNodes_out, double* turbDiameter_out, void** simulationInstance_out,
		int* errStat, char* errMsg);

	void INTERFACE_INITINFLOWS(void* simulationInstance, int* nBlades, int* nNodes, const double inflows[]);

	void INTERFACE_SETHUBMOTION(void* simulationInstance, double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_SETHUBMOTION_FAKE(void* simulationInstance, double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_SETINFLOWS(void* simulationInstance, double* inflows, int* nBlades, int* nNodes);

	void INTERFACE_SETINFLOWS_FAKE(void* simulationInstance, double* inflows, int* nBlades, int* nNodes);

	void INTERFACE_UPDATESTATES(void* simulationInstance, double* force_out,
		double* moment_out, double* power_out, double* tsr_out, double massMatrix_out[6][6], double addedMassMatrix_out[6][6]);

	void INTERFACE_UPDATESTATES_FAKE(void* simulationInstance, double* force_out,
		double* moment_out, double* power_out, double* tsr_out, double massMatrix_out[6][6], double addedMassMatrix_out[6][6]);

	void INTERFACE_GETBLADENODEPOS(void* simulationInstance, double* nodePos);

	void INTERFACE_GETBLADENODEPOS_FAKE(void* simulationInstance, double* nodePos);

	void INTERFACE_END(void* simulationInstance);
}


void Transform_PDStoAD(double v[3])
{
	v[1] *= -1.0;
	v[2] *= -1.0;
}

void Transform_ADtoPDS(double v[3])
{
	v[1] *= -1.0;
	v[2] *= -1.0;
}

void TransformHubKinematics_PDStoAD(double hubPos[3], double hubOri[3], double hubVel[3], double hubRotVel[3])
{
	Transform_PDStoAD(hubPos);
	Transform_PDStoAD(hubOri);
	Transform_PDStoAD(hubVel);
	Transform_PDStoAD(hubRotVel);
}

Matrix6d Transform_ADtoPDS(const Matrix6d& m)
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

	force[0] = force[1] = force[2] = 0.0;
	moment[0] = moment[1] = moment[2] = 0.0;

	simulationInstance = 0;

	f.open("InputLog.txt", std::ofstream::out);
	logHeader();
}

AeroDyn_Interface_Wrapper::~AeroDyn_Interface_Wrapper()
{
	INTERFACE_END(simulationInstance);
	f.close();
}

void AeroDyn_Interface_Wrapper::InitAerodyn(
	const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	const double hubPosition[3],
	const double hubOrientation[3],
	const double hubVelocity[3],
	const double hubRotationalVelocity[3],
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

	// copy all the vectors into their own local static arrays so we can transform them for Aerodyn's 
	// coordinate system
	double _hubPos[3];
	double _hubOri[3];
	double _hubVel[3];
	double _hubRotVel[3];

	// I know you wouldn't usually do this in c++... but I want to keep it simple with primary data types
	memcpy(_hubPos, hubPosition, 3 * sizeof(double));
	memcpy(_hubOri, hubOrientation, 3 * sizeof(double));
	memcpy(_hubVel, hubVelocity, 3 * sizeof(double));
	memcpy(_hubRotVel, hubRotationalVelocity, 3 * sizeof(double));

	// transform them to Aerodyn's global coordinate system 
	TransformHubKinematics_PDStoAD(_hubPos, _hubOri, _hubVel, _hubRotVel);

	// call the initialization subroutine in the FORTRAN DLL
	INTERFACE_INITAERODYN(inputFilename, &fname_len, &fluidDensity, &kinematicFluidVisc,
	    _hubPos, _hubOri, _hubVel, _hubRotVel,
		&bladePitch, &nBlades, &nNodes, &turbineDiameter, &simulationInstance, &errStat,
		errMsg);

	// check the error status number 
	if (errStat == 4) {
		throw ADErrorException(errMsg);
	}
	if (errStat == 5) {
		throw ADInputFileNotFoundException(errMsg);
	} 
	else if (errStat == 6) {
		throw ADInputFileContentsException(errMsg);
	}

	// return the total amount of nodes used in the simulation
	totalNodes = nBlades * nNodes;

	// resize our internal vector so it can hold the velocity components of each node
	aerodynInflows.resize(totalNodes * 3);

	logInput(-1.0, hubPosition, hubOrientation, hubVelocity, hubRotationalVelocity, bladePitch);

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
	const double hubPosition[3],
	const double hubOrientation[3],
	const double hubVelocity[3],
	const double hubRotationalVelocity[3],
	double bladePitch,
	bool isRealStep)
{
	// copy all the vectors into their own local static arrays so we can transform them for Aerodyn's 
	// coordinate system
	double _hubPos[3];
	double _hubOri[3];
	double _hubVel[3];
	double _hubRotVel[3];

	// I know you wouldn't usually do this in c++... but I want to keep it simple with primary data types
	memcpy(_hubPos, hubPosition, 3 * sizeof(double));
	memcpy(_hubOri, hubOrientation, 3 * sizeof(double));
	memcpy(_hubVel, hubVelocity, 3 * sizeof(double));
	memcpy(_hubRotVel, hubRotationalVelocity, 3 * sizeof(double));

	// transform them to Aerodyn's global coordinate system 
	TransformHubKinematics_PDStoAD(_hubPos, _hubOri, _hubVel, _hubRotVel);

	// If this is a fake-step, then we don't want this to be permanent, so we call the fake version
	if (!isRealStep) {
		INTERFACE_SETHUBMOTION_FAKE(simulationInstance, &time, _hubPos, _hubOri, _hubVel,
			_hubRotVel, &bladePitch);
	}
	// Otherwise we call the real version, which perminantly changes the inputs
	else {
		INTERFACE_SETHUBMOTION(simulationInstance, &time, _hubPos, _hubOri, _hubVel,
			_hubRotVel, &bladePitch);
	}

	logInput(time, hubPosition, hubOrientation, hubVelocity, hubRotationalVelocity, bladePitch);
	pitch = bladePitch;
}

void AeroDyn_Interface_Wrapper::SetInflowVelocities(const std::vector<double>& inflows, bool isRealStep)
{
	// Assigns the transformed inflows to aeroDynInflows
	TransformInflows_PDStoAD(inflows);

	if (isRealStep) {
		INTERFACE_SETINFLOWS(simulationInstance, aerodynInflows.data(), &nBlades, &nNodes);
	}
	else
	{
		INTERFACE_SETINFLOWS_FAKE(simulationInstance, aerodynInflows.data(), &nBlades, &nNodes);
	}
}

void AeroDyn_Interface_Wrapper::UpdateStates(
	double* force_out,
	double* moment_out,
	double* power_out,
	double* tsr_out,
	double massMatrix_out[6][6],
	double addedMassMatrix_out[6][6],
	bool isRealStep)
{
	static const int VectorSize = sizeof(double) * 3;


	// If this is a fake-step, then we don't want this to be permanent, so we call the fake version
	if ( !isRealStep ) {
		// Note, we're passing our transformed inflows here, not the inflows from the 
		// function parameter.
		INTERFACE_UPDATESTATES_FAKE(simulationInstance, force_out, moment_out, power_out,
			tsr_out, massMatrix_out, addedMassMatrix_out);
	}
	// Otherwise we call the real version, which permanently changes the states for this turbine
	else {
		// Note, we're passing our transformed inflows here, not the inflows from the 
		// function parameter.
		INTERFACE_UPDATESTATES(simulationInstance, force_out, moment_out, power_out,
			tsr_out, massMatrix_out, addedMassMatrix_out);
	}

	// Save the resulting forces and moments
	memcpy(force, force_out, VectorSize);
	memcpy(moment, moment_out, VectorSize);
}

// Communicates blade node positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
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

double AeroDyn_Interface_Wrapper::GetTorque() const
{
	return moment[0];
}

void AeroDyn_Interface_Wrapper::GetForce(double force_out[3]) const
{
	force_out[0] = force[0];
	force_out[1] = force[1];
	force_out[2] = force[2];
}

void AeroDyn_Interface_Wrapper::GetMoment(double moment_out[3]) const
{
	moment_out[0] = moment[0];
	moment_out[1] = moment[1];
	moment_out[2] = moment[2];
}


