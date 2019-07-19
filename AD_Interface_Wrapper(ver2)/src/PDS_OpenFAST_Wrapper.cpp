
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
	    double* bladePitch, int* nBlades_out, int* nNodes_out, int* turbineIndex_out);

	void INTERFACE_INITINFLOWS(int* turbineIndex, int* nBlades, int* nNodes, const double inflows[]);

	void INTERFACE_SETHUBMOTION(int* turbineIndex, double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* bladePitch);

	void INTERFACE_ADVANCESTATES(int* turbineIndex, int* nBlades, int* nNodes, double bladeNodeInflow[], double* force_out,
		double* moment_out, double* power_out, double massMatrix_out[6][6], double addedMassMatrix_out[6][6]);

	void INTERFACE_GETBLADENODEPOS(int* turbineIndex, double* nodePos);

	void INTERFACE_END(int* turbineIndex);
}


void transform_PDStoAD(double v[3])
{
	v[1] *= -1.0;
	v[2] *= -1.0;
}

void transform_ADtoPDS(double v[3])
{
	v[1] *= -1.0;
	v[2] *= -1.0;
}

void transformHubKinematics_PDStoAD(double hubPos[3], double hubOri[3], double hubVel[3], double hubRotVel[3])
{
	transform_PDStoAD(hubPos);
	transform_PDStoAD(hubOri);
	transform_PDStoAD(hubVel);
	transform_PDStoAD(hubRotVel);
}

Matrix6d transform_ADtoPDS(const Matrix6d& m)
{
	// TODO figure out how to transform a mass matrix from positive-up to positive-down (roll pi radians)
	return m;
}

// Returns the inflows transformed from PDS' coordinate system to that of AD's
void PDS_AD_Wrapper::transformInflows_PDStoAD(const std::vector<double>& pdsInflows)
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

	turbineIndex = 0;
}

PDS_AD_Wrapper::~PDS_AD_Wrapper()
{
	INTERFACE_END(&turbineIndex);
}

int PDS_AD_Wrapper::initAerodyn(
	const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	double hubRad,
	const double hubPosition[3],
	const double hubOrientation[3],
	const double hubVelocity[3],
	const double hubRotationalVelocity[3],
	double bladePitch,
	int* nBlades_out,  // number of blades, to be assigned upon calling the function
	int* nNodes_out)  // number of nodes per blade, to be assigned upon calling the function
{
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
	transformHubKinematics_PDStoAD(_hubPos, _hubOri, _hubVel, _hubRotVel);

	// call the initialization subroutine in the FORTRAN DLL
	INTERFACE_INITAERODYN(inputFilename, &fname_len, &fluidDensity, &kinematicFluidVisc,
		&hubRad, _hubPos, _hubOri, _hubVel,
		_hubRotVel, &bladePitch, &nBlades, &nNodes, &turbineIndex);
	*nBlades_out = nBlades;
	*nNodes_out = nNodes;

	// return the total amount of nodes used in the simulation
	totalNodes = nBlades * nNodes;

	aerodynInflows.resize(totalNodes * 3);
	return totalNodes;
}

// Transform each inflow from PDS' coordinate system (positive-down) to AD's (positive-up), and send them 
// to AD for its initialization.
void PDS_AD_Wrapper::initInflows(const std::vector<double>& pdsInflows)
{
	// update member variable aerodynInflow with transformed inflows passed to this function
	transformInflows_PDStoAD(pdsInflows);

	// call inflow initialization subroutine in FORTRAN DLL with these transformed inflows
	INTERFACE_INITINFLOWS(&turbineIndex, &nBlades, &nNodes, &aerodynInflows[0]);
}

void PDS_AD_Wrapper::updateHubMotion(double time,
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
	transformHubKinematics_PDStoAD(_hubPos, _hubOri, _hubVel, _hubRotVel);

	INTERFACE_SETHUBMOTION(&turbineIndex, &time, _hubPos, _hubOri, _hubVel,
		_hubRotVel, &bladePitch);
}

void PDS_AD_Wrapper::simulate(
	std::vector<double>& inflows,
	double force_out[3],
	double moment_out[3],
	double power_out[3],
	double massMatrix_out[6][6],
	double addedMassMatrix_out[6][6])
{
	INTERFACE_ADVANCESTATES(&turbineIndex, &nBlades, &nNodes, &inflows[0], force_out, moment_out, power_out,
		massMatrix_out, addedMassMatrix_out);
}


// Communicates blade node positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
void PDS_AD_Wrapper::getBladeNodePositions(std::vector<double>& nodePos)
{
	// fills nodePos with node positions. Note! It assumes enough elements have been allocated
	INTERFACE_GETBLADENODEPOS(&turbineIndex, nodePos.data());

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int i = 0; i < totalNodes; ++i) {
		nodePos[(i * 3) + 0] =  nodePos[(i * 3) + 0]; // x position of node (m)
		nodePos[(i * 3) + 1] = -nodePos[(i * 3) + 1]; // y position of node (m) flipped coordinates
		nodePos[(i * 3) + 2] = -nodePos[(i * 3) + 2]; // z position of node (m) flipped coordinates
	}
}

