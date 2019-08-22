
#include "PDS_OpenFAST_Wrapper.h"
#include "..//eigen/Eigen/SparseCore"
#include <assert.h>
#include <iostream>
#include <stdlib.h>

// Create a 6x6 matrix from Eigen3's template for transforming the mass matrix
using Eigen::Matrix;
using Eigen::Vector3d;

typedef Matrix<double, 6, 6> Matrix6d;

// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INITAERODYN(const char* inputFilename, int* fname_len, double* fluidDensity, double* kinematicFluidVisc,
		double* hubRad, double* hubPos, double* hubOri, double* hubVel, double* hubRotVel, 
	    double* bladePitch, int* nBlades_out, int* nNodes_out, double* turbDiameter_out, void** simulationInstance_out,
		int* errStat, char* errMsg);

	void INTERFACE_INITINFLOWS(void* simulationInstance, int* nBlades, int* nNodes, const double inflows[]);

	void INTERFACE_SETHUBMOTION(void* simulationInstance, double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_ADVANCESTATES(void* simulationInstance, int* nBlades, int* nNodes, double bladeNodeInflow[], double* force_out,
		double* moment_out, double* power_out, double* tsr_out, double massMatrix_out[6][6], double addedMassMatrix_out[6][6]);

	void INTERFACE_GETBLADENODEPOS(void* simulationInstance, double* nodePos);

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
void PDS_AD_Wrapper::TransformInflows_PDStoAD(const std::vector<double>& pdsInflows)
{
	// iterate through all the x, y, z components of the inflows, negating the y and z components
	for (int i = 0; i < totalNodes; ++i) {
		aerodynInflows[(i * 3) + 0] = pdsInflows[(i * 3) + 0];
		aerodynInflows[(i * 3) + 1] = -pdsInflows[(i * 3) + 1];
		aerodynInflows[(i * 3) + 2] = -pdsInflows[(i * 3) + 2];
	}
}

PDS_AD_Wrapper::PDS_AD_Wrapper()
{
	nBlades = nNodes = totalNodes = 0;

	simulationInstance = 0;
}

PDS_AD_Wrapper::~PDS_AD_Wrapper()
{
	INTERFACE_END(simulationInstance);
}

void PDS_AD_Wrapper::InitAerodyn(
	const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	double hubRad,
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
		&hubRad, _hubPos, _hubOri, _hubVel, _hubRotVel,
		&bladePitch, &nBlades, &nNodes, &turbineDiameter, &simulationInstance, &errStat,
		errMsg);

	// check the error status number 
	if (errStat == 4) {
		throw ADError(errMsg);
	}
	if (errStat == 5) {
		throw ADInputFileNotFound(errMsg);
	} 
	else if (errStat == 6) {
		throw ADInputFileContents(errMsg);
	}

	// return the total amount of nodes used in the simulation
	totalNodes = nBlades * nNodes;

	// resize our internal vector so it can hold the velocity components of each node
	aerodynInflows.resize(totalNodes * 3);
}

// Transform each inflow from PDS' coordinate system (positive-down) to AD's (positive-up), and send them 
// to AD for its initialization.
void PDS_AD_Wrapper::InitInflows(const std::vector<double>& pdsInflows)
{
	// update member variable aerodynInflow with transformed inflows passed to this function
	TransformInflows_PDStoAD(pdsInflows);

	// call inflow initialization subroutine in FORTRAN DLL with these transformed inflows
	INTERFACE_INITINFLOWS(simulationInstance, &nBlades, &nNodes, &aerodynInflows[0]);
}

void PDS_AD_Wrapper::UpdateHubMotion(double time,
	const double hubPosition[3],
	const double hubOrientation[3],
	const double hubVelocity[3],
	const double hubRotationalVelocity[3],
	double bladePitch)
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

	INTERFACE_SETHUBMOTION(simulationInstance, &time, _hubPos, _hubOri, _hubVel,
		_hubRotVel, &bladePitch);
}

void PDS_AD_Wrapper::Simulate(
	std::vector<double>& inflows,
	double force_out[3],
	double moment_out[3],
	double* power_out,
	double* tsr_out,
	double massMatrix_out[6][6],
	double addedMassMatrix_out[6][6])
{
	TransformInflows_PDStoAD(inflows);

	// Note, we're passing out transformed inflows here, not the inflows from the 
	// function parameter.
	INTERFACE_ADVANCESTATES(simulationInstance, &nBlades, &nNodes, &aerodynInflows[0], force_out, moment_out, power_out,
		tsr_out, massMatrix_out, addedMassMatrix_out);
}

// Communicates blade node positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
void PDS_AD_Wrapper::GetBladeNodePositions(std::vector<double>& nodePos)
{
	// fills nodePos with node positions. Note! It assumes enough elements have been allocated
	INTERFACE_GETBLADENODEPOS(simulationInstance, nodePos.data());

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int i = 0; i < totalNodes; ++i) {
		nodePos[(i * 3) + 0] =  nodePos[(i * 3) + 0]; // x position of node (m)
		nodePos[(i * 3) + 1] = -nodePos[(i * 3) + 1]; // y position of node (m) flipped coordinates
		nodePos[(i * 3) + 2] = -nodePos[(i * 3) + 2]; // z position of node (m) flipped coordinates
	}
}

int PDS_AD_Wrapper::GetNumBlades() const
{
	return nBlades;
}

int PDS_AD_Wrapper::GetNumNodes() const
{
	return totalNodes;
}

double PDS_AD_Wrapper::GetTurbineDiameter() const
{
	return turbineDiameter;
}