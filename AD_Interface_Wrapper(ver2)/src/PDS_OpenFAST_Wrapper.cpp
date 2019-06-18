
#include "PDS_OpenFAST_Wrapper.h"
#include "ISO_Fortran_binding.h"
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <iostream>

// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INIT(const char* inputFilename, int* fname_len, double* hubPos, double* hubOri, double* hubVel,
		double* hubRotVel, double *shaftSpeed, double* bladePitch, int* nBlades, int* nNodes);
	void INTERFACE_INITINFLOWS(int* nBlades, int* nNodes, double inflows[]);

	void INTERFACE_SETHUBSTATE(double* time, double hubPos[3], double hubOri[3], double hubVel[3],
		double hubRotVel[3], double* shaftSpeed, double* bladePitch);

	void INTERFACE_ADVANCESTATES(int* nBlades, int* nNodes, double bladeNodeInflow[], double* force_out,
		double* moment_out, double massMatrix[6][6], double addedMassMatrix[6][6]);

	void INTERFACE_GETBLADENODEPOS(double* nodePos);
	void INTERFACE_CLOSE();
}

PDS_AD_Wrapper::PDS_AD_Wrapper()
{
	nBlades = nNodes = 0;
}

int PDS_AD_Wrapper::initHub(
	const char* inputFilename,
	Vector_3D hubPos,
	Vector_3D hubOri,
	Vector_3D hubVel,
	Vector_3D hubRotVel,
	double shaftSpeed, // rotional speed of the shaft in rads/sec
	double bladePitch,
	int* nBlades_out,  // number of blades, to be assigned upon calling the function
	int* nNodes_out)  // number of nodes per blade, to be assigned upon calling the function
{
	int fname_len = strlen(inputFilename);

	INTERFACE_INIT(inputFilename, &fname_len, hubPos.getCArray(), hubOri.getCArray(), hubVel.getCArray(),
		hubRotVel.getCArray(), &shaftSpeed, &bladePitch, &nBlades, &nNodes);
	*nBlades_out = nBlades;
	*nNodes_out = nNodes;

	assert(nBlades > 2);
	assert(nNodes > 0);

	// return the total amount of nodes used in the simulation
	return nBlades * nNodes;
}

void PDS_AD_Wrapper::initInflows(std::vector<double>& inflows)
{
	INTERFACE_INITINFLOWS(&nBlades, &nNodes, &inflows[0]);
}

void PDS_AD_Wrapper::updateHubState(double time,
	Vector_3D hubPosition,
	Vector_3D hubOrientation,
	Vector_3D hubVelocity,
	Vector_3D hubRotationalVelocity,
	double shaftSpeed,
	double bladePitch)
{
	INTERFACE_SETHUBSTATE(&time, hubPosition.getCArray(), hubOrientation.getCArray(), hubVelocity.getCArray(),
		hubRotationalVelocity.getCArray(), &shaftSpeed, &bladePitch);
}

void PDS_AD_Wrapper::solve(
	std::vector<double>& inflows,
	Vector_3D& force_out,
	Vector_3D& moment_out,
	double massMatrix_out[6][6],
	double addedMassMatrix_out[6][6])
{
	INTERFACE_ADVANCESTATES(&nBlades, &nNodes, &inflows[0], force_out.getCArray(), moment_out.getCArray(),
		massMatrix_out, addedMassMatrix_out);
}

/*
// Combination of set_inputs, advance, and get_outputs
int DECLDIR Turbine_solve(double time, int RK_flag, double hubKinematics[6][3], double shaftSpeed,
	vector<double>& forceAndMoment, vector< vector<double> >& massMatrix,
	vector< vector<double> >& addedMassMatrix, double* genTorque)
{
	// step (integer): current step number in the simulation
	// hubState (double [18]): serialized vector of the state variables for the rotor hub in ProteusDS.  
	// Vector includes, in order:
	//   Position (x, y, z) (m)
	//   Orientation (roll, pitch, yaw) (rad)
	//   Velocity (Vx, Vy, Vz) (m/s)
	//   Angular Velocity (Vroll, Vpitch, Vyaw) (rad/s)
	//   Acceleration (Ax, Ay, Az) (m/s^2)
	//   Angular Acceleration (Aroll, Apitch, Ayaw) (rad/s^2)
	// shaftSpeed (double): Rotational speed of the low speed shaft
	// bladeNodeInflow (double [6*n*b], n = number of nodes per blade, b = number of blades): water particle velocities and accelerations at the turbine blade nodes.  Vector ordered as ( Vx,b1,n1, Vy,b1,n1, Vz,b1,n1, Ax,b1,n1, Ay,b1,n1, Az,b1,n1, Vx,b1,n2, Vy,b1,n2, Vz,b1,n2, etc.)

	//   step (integer): current step number in the simulation
	//   dt_PDS (double): Time-step size used internally by ProteusDS
	//   dt_C (double): coupling time-step size, for frequency of coupling calls

   //   forceAndMoment (double [6]): Forces and moments at the turbine rotor hub.  Stored as a vector (Fx, Fy, Fz, Mx, My, Mz).
	//   massMatrix (double [36]): The sum of all inertial terms.  Includes the total mass and inertia for the turbine as well as the added mass.  Ordered as ( mxx, mxy, mxz, mx,roll, mx,pitch, mx,yaw, myx, myy etc.)
	//   generatorTorque (double): Torque exerted on the low speed rotor shaft by the generator.  Equal and opposite to the body reaction torque.

	//cout << "interpolating flow " <<endl;

   // interpolate inflow data for the current time!
	for (int i = 0; i < nBlades * nNodes * 3; i++) { // loop through all inflow data
		// @dustin: We're going to forget about interpolation until we know things are working with a constant flow
		//inflowInterp[i] = inflowOld[i] + (time - inflowTimeOld) * (inflow[i] - inflowOld[i]) / (inflowTime - inflowTimeOld);  // linear interpolation
	
		inflowInterp[i] = inflow[i];
	}


	// create hub kinematics vector from the one passed from ProteusDS, including coordinate system conversion
	// FAST convention: 
	// ProteusDS convention: 

	// @dustin: just took out coordinate flips until later
	/*
	for (int i = 0; i < 6; ++i) {
		hubKinematics[i][0] = hubState[i][0]; // x
		hubKinematics[i][1] = -hubState[i][1]; // y
		hubKinematics[i][2] = -hubState[i][2]; // z
	}*/

	//hubKinematics[0] = hubState[0]; // positions (m), adjusted for coordinates change
	//hubKinematics[1] = -hubState[1];
	//hubKinematics[2] = -hubState[2];
	//hubKinematics[3] = hubState[3]; // rotations (rad), adjusted for coordinates change
	//hubKinematics[4] = -hubState[4];
	//hubKinematics[5] = -hubState[5];
	//hubKinematics[6] = hubState[6]; // velocities (m/s), adjusted for coordinates change
	//hubKinematics[7] = -hubState[7];
	//hubKinematics[8] = -hubState[8];
	//hubKinematics[9] = hubState[9]; // angular rates (rad/s), adjusted for coordinates change (should include rotor rotation!)
	//hubKinematics[10] = -hubState[10];
	//hubKinematics[11] = -hubState[11];
	//hubKinematics[12] = 0.0;  // accelerations (m/s^2) - not yet used
	//hubKinematics[13] = 0.0;
	//hubKinematics[14] = 0.0;
	//hubKinematics[15] = 0.0; // anglar accelerations (rad/s^2) - not yet used
	//hubKinematics[16] = 0.0;
	//hubKinematics[17] = 0.0;
/*



	// declare arrays to be filled by the DLL's function
	double    forceAndMoment2[6];
	double    massMatrix2[36];
	double    addedMassMatrix2[36];
	double genTorque2 = 0.0;  // ooh, could also optionally implement the control on genTorque HERE! <<<<<<<
	int RKflag = 1;     // need to set this up at some point! <<<<<<<


	cout << "calling AD step advance states, with shaft speed " << shaftSpeed << endl;

	// now that things have been converted in terms of both data type and coordinate system, call the DLL function
	INTERFACE_ADVANCESTATES(&time, &RKflag, hubKinematics, &shaftSpeed, &nBlades, &nNodes, inflowInterp, 
		forceAndMoment2, massMatrix2, addedMassMatrix2, &genTorque2);

	// copy outputs into arrays passed in from ProteusDS (do we need to also convert coordinates?)
	for (int i = 0; i < 6; i++)
		forceAndMoment[i] = forceAndMoment2[i];

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			massMatrix[i][j] = massMatrix2[6 * i + j];
			addedMassMatrix[i][j] = addedMassMatrix2[6 * i + j];  // need to check the order of these <<<<<<
		}
	}

	return 1;
} */

// Description:  Communicates blade nods positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
void PDS_AD_Wrapper::getBladeNodePositions(std::vector<double>& nodePos)
{
	// step (integer): current step number in the simulation
	// nodePos(double [3*n*b], n = number of nodes per blade, b = number of blades): Current positions of the blade nodes.  Stored as a serialized vector ordered as ( xb1,n1, yb1,n1, zb1,n1, xb1,n2, yb1,n2, zb1,n2, etc. )
	//

	// fills nodePos with node positions. Note! It assumes enough elements have been allocated
	INTERFACE_GETBLADENODEPOS(&nodePos[0]);

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int iB = 0; iB < nBlades; iB++) // loop through blades
	{
		for (int iN = 0; iN < nNodes; iN++) // loop through nodes on the blade
		{
			//cout << "reading from index (+0,1,2) of "<< iB*nNodes*3 + iN*3 << endl;

			nodePos[iB * nNodes * 3 + iN * 3 + 0] = nodePos[iB * nNodes * 3 + iN * 3 + 0]; // x position of node (m)
			nodePos[iB * nNodes * 3 + iN * 3 + 1] = -1 * nodePos[iB * nNodes * 3 + iN * 3 + 1]; // y position of node (m) flipped coordinates
			nodePos[iB * nNodes * 3 + iN * 3 + 2] = -1 * nodePos[iB * nNodes * 3 + iN * 3 + 2]; // z position of node (m) flipped coordinates
		}
	}

}

