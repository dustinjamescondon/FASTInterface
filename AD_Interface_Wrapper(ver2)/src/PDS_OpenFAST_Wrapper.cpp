// Wrapper DLL that facilitates ProteusDS interacting with the AD_Interface DLL.
// It converts between ProtuesDS's vector data types and regular arrays,
// and it handles coordinate system conversions since ProteusDS uses positive-down.

// Matt Hall, Patrick Connolly - 2018-07

//  #include whatever stuff you need here (this isn't a working example)

#include "PDS_OpenFAST_Wrapper.h"
#include <iostream>
#include <windows.h>  
#include <stdlib.h>  
#include <string.h>  
#include <tchar.h>  
#include <iostream>
#include <memory> // for creating and deleting variable-size arrays at runtime


using namespace std;


// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
	void INTERFACE_INIT(int* nBlades, int* nNodes);
	void INTERFACE_SETSTATES(double* time, double hubState[], double* shaftSpeed);
	void INTERFACE_ADVANCESTATES(double* time, int* rkFlag, double hubKinematics[], double* shaftSpeed, double bladeNodeInflow[],
		double forceAndMoment[], double massMatrix[], double addedMassMatrix[], double* genTorque);

	void INTERFACE_GETOUTPUTS(double* time, double* forceAndMoment[], double* generatorTorque, double massMatrix[], double addedMassMatrix[]);
	void INTERFACE_SETBLADEINFLOW(double* time, double* bladeNodeInflow);
	void INTERFACE_GETBLADENODEPOS(double* time, double* nodePos);
	void INTERFACE_CLOSE();
}

// ------------------------ some global variables -----------------------------
// pointers to arrays that will hold the variables passed from AD_Interface
double* inflow;          // latest flow data 
double* inflowOld;       // previous flow data   
double* inflowInterp;       // flow data interpolated for current coupling instant
double* nodePos2;
double* nodePos2Old;
double inflowTime;   // time that inflow data corresponds to
double inflowTimeOld;


double* forceAndMoment2;
double* massMatrix2;
double* addedMassMatrix2;

int nBlades;
int nNodes;



// This is the initialization function. It loads the AD_interface DLL, initializes the model, etc.
int DECLDIR Turbine_init(int* nBladesOut, int* nNodesOut)
{

	// SetErrorMode(SEM_FAILCRITICALERRORS);    // @mht: what does this do ??

	cout << "All done.  Now try calling some functions..." << endl;

	cout << "Finished loading DLL and functions" << endl;

	// call to initialize AeroDyn driver, which includes reading in important parameters

	INTERFACE_INIT(&nBlades, &nNodes);

	// allocate some arrays based on number of blades and nodes
	inflow = new double[6 * nBlades * nNodes];  // dynamically allocate the array to hold inflow data passed to AD_Interface
	inflowInterp = new double[6 * nBlades * nNodes];  // dynamically allocate the array to hold inflow data passed to AD_Interface
	inflowOld = new double[6 * nBlades * nNodes];  // dynamically allocate the array to hold inflow data passed to AD_Interface
	nodePos2 = new double[3 * nBlades * nNodes];  // dynamically allocate the array to hold node coordinates passed from AD_Interface
	nodePos2Old = new double[3 * nBlades * nNodes];  // dynamically allocate the array to hold node coordinates passed from AD_Interface
	
	cout << "There are " << nBlades << " blades with " << nNodes << " nodes each." << endl;

	// save to passed in pointers for use by calling program

	*nBladesOut = nBlades;
	*nNodesOut = nNodes;

	return 1; // just return this for now because they've declared the function this way
}


// Combination of set_inputs, advance, and get_outputs
int DECLDIR Turbine_solve(double time, int RK_flag, const vector<double>& hubState, double shaftSpeed,
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

	for (int i = 0; i < 6 * nBlades * nNodes; i++) // loop through all inflow data
		inflowInterp[i] = inflowOld[i] + (time - inflowTimeOld) * (inflow[i] - inflowOld[i]) / (inflowTime - inflowTimeOld);  // linear interpolation


	// create hub kinematics vector from the one passed from ProteusDS, including coordinate system conversion
	// FAST convention: 
	// ProteusDS convention: 


	 //cout << "copying hub kinematics" <<endl;

	double hubKinematics[18];

	hubKinematics[0] = hubState[0]; // positions (m), adjusted for coordinates change
	hubKinematics[1] = -hubState[1];
	hubKinematics[2] = -hubState[2];
	hubKinematics[3] = hubState[3]; // rotations (rad), adjusted for coordinates change
	hubKinematics[4] = -hubState[4];
	hubKinematics[5] = -hubState[5];
	hubKinematics[6] = hubState[6]; // velocities (m/s), adjusted for coordinates change
	hubKinematics[7] = -hubState[7];
	hubKinematics[8] = -hubState[8];
	hubKinematics[9] = hubState[9]; // angular rates (rad/s), adjusted for coordinates change (should include rotor rotation!)
	hubKinematics[10] = -hubState[10];
	hubKinematics[11] = -hubState[11];
	hubKinematics[12] = 0.0;  // accelerations (m/s^2) - not yet used
	hubKinematics[13] = 0.0;
	hubKinematics[14] = 0.0;
	hubKinematics[15] = 0.0; // anglar accelerations (rad/s^2) - not yet used
	hubKinematics[16] = 0.0;
	hubKinematics[17] = 0.0;




	// declare arrays to be filled by the DLL's function
	double    forceAndMoment2[6];
	double    massMatrix2[36];
	double    addedMassMatrix2[36];
	double genTorque2 = 0.0;  // ooh, could also optionally implement the control on genTorque HERE! <<<<<<<
	int RKflag = 1;     // need to set this up at some point! <<<<<<<


	cout << "calling AD step advance states, with shaft speed " << shaftSpeed << endl;

	//double time2 = 3.14;
	//double shaftSpeed2 = 3.0; //1.0*shaftSpeed;

	// now that things have been converted in terms of both data type and coordinate system, call the DLL function
	INTERFACE_ADVANCESTATES(&time, &RKflag, hubKinematics, &shaftSpeed, inflowInterp, forceAndMoment2, massMatrix2, addedMassMatrix2, &genTorque2);



	//cout << "copying results " <<endl;

	// copy outputs into arrays passed in from ProteusDS (do we need to also convert coordinates?)
	for (int i = 0; i < 6; i++)
		forceAndMoment[i] = forceAndMoment2[i];


	//cout << "... " <<endl;

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			massMatrix[i][j] = massMatrix2[6 * i + j];
			addedMassMatrix[i][j] = addedMassMatrix2[6 * i + j];  // need to check the order of these <<<<<<
		}
	}

	//cout << "done " <<endl;

	return 1;
}


// Changed: stores flow data passed from calling program 
int DECLDIR Turbine_setBladeInflow(double time, const vector<double>& bladeNodeInflow)
{
	// step (integer): current step number in the simulation
	// nodePos(double [3*n*b], n = number of nodes per blade, b = number of blades): Current positions of the blade nodes.  Stored as a serialized vector ordered as ( xb1,n1, yb1,n1, zb1,n1, xb1,n2, yb1,n2, zb1,n2, etc. )



   // STORE previous data

	memcpy(inflowOld, inflow, 6 * nBlades * nNodes * sizeof(double)); // save copy of previous inflow data
	memcpy(nodePos2Old, nodePos2, 3 * nBlades * nNodes * sizeof(double)); // save copy of previous node position data

	inflowTimeOld = inflowTime;


	// STORE LATEST DATA NOW

	 //cout << "Turbine_setBladeInflow: size of bladeNodeInflow is "<< bladeNodeInflow.size() << endl;

	 // create flow kinematics array from the one passed from ProteusDS, including coordinate system conversion    
	for (int iB = 0; iB < nBlades; iB++) // loop through blades
	{
		for (int iN = 0; iN < nNodes; iN++) // loop through nodes on the blade
		{
			//cout << "writing to index (+0:5) of "<< iB*nNodes*6 + iN*6 << endl;

			inflow[iB * nNodes * 6 + iN * 6 + 0] = bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 0]; // x velocity (m/s)
			inflow[iB * nNodes * 6 + iN * 6 + 1] = -1 * bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 1]; // y velocity (m/s) flipped coordinates
			inflow[iB * nNodes * 6 + iN * 6 + 2] = -1 * bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 2]; // z velocity (m/s) flipped coordinates
			inflow[iB * nNodes * 6 + iN * 6 + 3] = bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 3]; // x acceleration (m/s^2)
			inflow[iB * nNodes * 6 + iN * 6 + 4] = -1 * bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 4]; // y acceleration (m/s^2) flipped coordinates
			inflow[iB * nNodes * 6 + iN * 6 + 5] = -1 * bladeNodeInflow[iB * nNodes * 6 + iN * 6 + 5]; // z acceleration (m/s^2) flipped coordinates
		}
	}

	inflowTime = time;



	// WE WILL JUST STORE THIS DATA IN THIS DLL UNTIL THE NEXT COUPLING CALL, THEN IT WILL BE USED TO INTERPOLATE FROM

	 //cout<< inflow << endl;
	 //cout<<"-------------------"<<endl;

	 // call DLL's function to set inflows
	 //int success = setBladeInflow(&time, inflow);

	return 1;
}


// Description:  Communicates blade nods positions to ProteusDS.  This needs to be separate from the other outputs so that it can be used to get inflow values at the current time step.
int DECLDIR Turbine_getBladeNodePos(double time, vector<double>& nodePos)
{
	// step (integer): current step number in the simulation
	// nodePos(double [3*n*b], n = number of nodes per blade, b = number of blades): Current positions of the blade nodes.  Stored as a serialized vector ordered as ( xb1,n1, yb1,n1, zb1,n1, xb1,n2, yb1,n2, zb1,n2, etc. )

	// call DLL's function to get node coordinates filled in
	INTERFACE_GETBLADENODEPOS(&time, nodePos2);

	//cout << "Turbine_getBladeNodePos: size of nodePos is "<< nodePos.size() << endl;

	// copy node positions into the vector<double> for ProteusDS, including coordinate system conversion
	for (int iB = 0; iB < nBlades; iB++) // loop through blades
	{
		for (int iN = 0; iN < nNodes; iN++) // loop through nodes on the blade
		{
			//cout << "reading from index (+0,1,2) of "<< iB*nNodes*3 + iN*3 << endl;

			nodePos[iB * nNodes * 3 + iN * 3 + 0] = nodePos2[iB * nNodes * 3 + iN * 3 + 0]; // x position of node (m)
			nodePos[iB * nNodes * 3 + iN * 3 + 1] = -1 * nodePos2[iB * nNodes * 3 + iN * 3 + 1]; // y position of node (m) flipped coordinates
			nodePos[iB * nNodes * 3 + iN * 3 + 2] = -1 * nodePos2[iB * nNodes * 3 + iN * 3 + 2]; // z position of node (m) flipped coordinates
		}
	}

	return 1;
}


// @dustin TODO: free the rest of the allocated variables
// @mth: todo: need a close/cleanup function to delete inflow and nodePos arrays
int DECLDIR Turbine_close()
{
	delete inflow;
	delete nodePos2;

	return 0;
}
