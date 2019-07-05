
#include "PDS_OpenFAST_Wrapper.h"
#include "..//eigen/Eigen/Dense"
#include <assert.h>
#include <iostream>
#include <stdlib.h>

// Create a 6x6 matrix from Eigen3's template for transforming the mass matrix
using Eigen::Matrix;
typedef Matrix<double, 6, 6> Matrix6d;


// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INIT(const char* inputFilename, int* fname_len, double* fluidDensity, double* kinematicFluidVisc,
		double* hubPos, double* hubOri, double* hubVel, double* hubRotVel, double* shaftSpeed, 
		double* bladePitch, int* nBlades, int* nNodes);

	void INTERFACE_INITINFLOWS(int* nBlades, int* nNodes, const double inflows[]);

	void INTERFACE_SETHUBSTATE(double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* shaftSpeed, double* bladePitch);

	void INTERFACE_ADVANCESTATES(int* nBlades, int* nNodes, double bladeNodeInflow[], double* force_out,
		double* moment_out, double massMatrix[6][6], double addedMassMatrix[6][6]);

	void INTERFACE_GETBLADENODEPOS(double* nodePos);
	void INTERFACE_CLOSE();
}


Vector_3D transform_PDStoAD(const Vector_3D& v)
{
	return Vector_3D(v.x(), -v.y(), -v.z());
}

Vector_3D transform_ADtoPDS(const Vector_3D& v)
{
	return Vector_3D(v.x(), -v.y(), -v.z());
}

void transformHubKinematics_PDStoAD(Vector_3D& hubPos, Vector_3D& hubOri, Vector_3D& hubVel,
	Vector_3D& hubRotVel)
{
	hubPos = transform_PDStoAD(hubPos);
	hubOri = transform_PDStoAD(hubOri);
	hubVel = transform_PDStoAD(hubVel);
	hubRotVel = transform_PDStoAD(hubRotVel);
}

Matrix6d transform_ADtoPDS(const Matrix6d& m)
{
	// TODO, figure out how to transform a mass matrix from positive-up to positive-down (roll pi radians)
	return m;
}

// Returns the inflows transformed from PDS' coordinate system to that of AD's
std::vector<double> PDS_AD_Wrapper::transformInflows_PDStoAD(const std::vector<double>& pdsInflows) const
{
	std::vector<double> adInflows(totalNodes * 3);

	// iterate through all the x, y, z components of the inflows, negating the y and z components
	for (int i = 0; i < totalNodes; ++i) {
		adInflows[(i * 3) + 0] = pdsInflows[(i * 3) + 0];
		adInflows[(i * 3) + 1] = pdsInflows[(i * 3) + 1];
		adInflows[(i * 3) + 2] = pdsInflows[(i * 3) + 2];
	}
	return adInflows;
}

PDS_AD_Wrapper::PDS_AD_Wrapper()
{
	nBlades = nNodes = totalNodes = 0;
}

int PDS_AD_Wrapper::initHub(
	const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	Vector_3D hubPos,
	Vector_3D hubOri,
	Vector_3D hubVel,
	Vector_3D hubRotVel,
	double shaftSpeed, // rotional speed of the shaft in rads/sec
	double bladePitch,
	int* nBlades_out,  // number of blades, to be assigned upon calling the function
	int* nNodes_out)  // number of nodes per blade, to be assigned upon calling the function
{
	// Parameters are passed by reference, and are therefore transformed.
	//transformHubKinematics_PDStoAD(hubPos, hubOri, hubVel, hubRotVel);

	int fname_len = strlen(inputFilename);

	transformHubKinematics_PDStoAD(hubPos, hubOri, hubVel, hubRotVel);

	INTERFACE_INIT(inputFilename, &fname_len, &fluidDensity, &kinematicFluidVisc, hubPos.getCArray(), hubOri.getCArray(), hubVel.getCArray(),
		hubRotVel.getCArray(), &shaftSpeed, &bladePitch, &nBlades, &nNodes);
	*nBlades_out = nBlades;
	*nNodes_out = nNodes;

	// return the total amount of nodes used in the simulation
	totalNodes = nBlades * nNodes;
	return totalNodes;
}

// Transform each inflow from PDS' coordinate system (positive-down) to AD's (positive-up), and send them 
// to AD for its initialization.
void PDS_AD_Wrapper::initInflows(const std::vector<double>& pdsInflows)
{
	std::vector<double> adInflows = transformInflows_PDStoAD(pdsInflows);

	INTERFACE_INITINFLOWS(&nBlades, &nNodes, &adInflows[0]);
}

void PDS_AD_Wrapper::updateHubState(double time,
	Vector_3D hubPosition,
	Vector_3D hubOrientation,
	Vector_3D hubVelocity,
	Vector_3D hubRotationalVelocity,
	double shaftSpeed,
	double bladePitch)
{
	transformHubKinematics_PDStoAD(hubPosition, hubOrientation, hubVelocity, hubRotationalVelocity);

	INTERFACE_SETHUBSTATE(&time, hubPosition.getCArray(), hubOrientation.getCArray(), hubVelocity.getCArray(),
		hubRotationalVelocity.getCArray(), &shaftSpeed, &bladePitch);
}

void PDS_AD_Wrapper::simulate(
	std::vector<double>& inflows,
	Vector_3D& force_out,
	Vector_3D& moment_out,
	double massMatrix_out[6][6],
	double addedMassMatrix_out[6][6])
{
	INTERFACE_ADVANCESTATES(&nBlades, &nNodes, &inflows[0], force_out.getCArray(), moment_out.getCArray(),
		massMatrix_out, addedMassMatrix_out);
}


// Communicates blade nods positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
void PDS_AD_Wrapper::getBladeNodePositions(std::vector<double>& nodePos)
{

	// fills nodePos with node positions. Note! It assumes enough elements have been allocated
	INTERFACE_GETBLADENODEPOS(&nodePos[0]);

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int i = 0; i < totalNodes; ++i) {
		nodePos[(i * 3) + 0] =  nodePos[(i * 3) + 0]; // x position of node (m)
		nodePos[(i * 3) + 1] = -nodePos[(i * 3) + 1]; // y position of node (m) flipped coordinates
		nodePos[(i * 3) + 2] = -nodePos[(i * 3) + 2]; // z position of node (m) flipped coordinates
	}

}

