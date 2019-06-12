// example of dynamic linking to a DLL (i.e. loading a DLL at runtime; not using it at all at compiletime)
// Matt Hall - 2018-07-11

//  #include whatever stuff you need here (this isn't a working example)

//#include "PDS_OpenFAST_Wrapper.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <windows.h>  
#include <stdlib.h>  
#include <string.h>  
#include <tchar.h>  

using namespace std;

// ------------ define some pointers to the dll's functions ---------------

// first make types for the functions (just need to specify the parameter types and return type, I also include parameter variable names as a reminder)
typedef int(*Init_DLL)();
typedef int(*Turbine_init)(double hubKinematics[6][3], double shaftSpeed, int* nBlades, int* nNodes);
typedef int(*Turbine_initInflows)(vector<double>& inflows);
typedef int(*Turbine_setInflows)(vector<double>& inflows);
typedef int(*Turbine_solve)(double time, int RK_flag, double hubKinematics[6][3], double shaftSpeed,
	  vector<double> &forceAndMoment, vector< vector<double> > &massMatrix,
	  vector< vector<double> > &addedMassMatrix, double* genTorque);
typedef int(*Turbine_setBladeInflow)(double time, vector<double> &bladeNodeInflow);
typedef int(*Turbine_getBladeNodePos)(double time, vector<double> &nodePos);
typedef int(*Turbine_close)();

// now make the actual function handle variables using the new types

Init_DLL		checkDLL;
Turbine_init		init;
Turbine_initInflows initInflows;
Turbine_setInflows setInflows;
Turbine_solve		solve;
Turbine_setBladeInflow	setBladeInflow;
Turbine_getBladeNodePos	getBladeNodePos;
Turbine_close		close;

int nNodes;
int nBlades;

// -------------------------- main function -------------------------------
int main(int argc, char *argv[])
{
	
	SetErrorMode(SEM_FAILCRITICALERRORS);
	
	// -------------------- LOAD AeroDyn DLL ----------------------------
	
	HINSTANCE hInstLibraryTurbine = 0;
	
	try
	{
		hInstLibraryTurbine = LoadLibraryA("AD_Interface_Wrapper.dll");		// a handle to the instance of the dll
	}
	catch (...)
	{
		cout << "yikes";
	}

	if (hInstLibraryTurbine == NULL)
	{
		cout << "Could not load the wrapper DLL!" << endl;
		exit(1);
	}
	cout << "Loaded the C++ wrapper DLL!" << endl;

	// get addresses to each function within the DLL
	//checkDLL  = 	(Init_DLL)GetProcAddress(hInstLibraryTurbine, "Init_DLL");
	init = (Turbine_init)GetProcAddress(hInstLibraryTurbine, "Turbine_init");
	initInflows = (Turbine_initInflows)GetProcAddress(hInstLibraryTurbine, "Turbine_initInflows");
	setInflows = (Turbine_setInflows)GetProcAddress(hInstLibraryTurbine, "Turbine_setInflows");
	solve = (Turbine_solve)GetProcAddress(hInstLibraryTurbine, "Turbine_solve");
	setBladeInflow = (Turbine_setBladeInflow)GetProcAddress(hInstLibraryTurbine, "Turbine_setBladeInflow");
	getBladeNodePos = (Turbine_getBladeNodePos)GetProcAddress(hInstLibraryTurbine, "Turbine_getBladeNodePos");
	close = (Turbine_close)GetProcAddress(hInstLibraryTurbine, "Turbine_close");

	// check for success at finding each function in the DLL
	//if (checkDLL)	  	cout << " Got address for init" <<  endl;
	//else			  cout << " ERROR: Failed to get address for init" <<  endl;

	if (init)	  	cout << " Got address for turbine init" << endl;
	else			  cout << " ERROR: Failed to get address for turbine init" << endl;

	if (initInflows) cout << " Got address for init inflows" << endl;
	else             cout << "ERROR: Failed to get address for init inflows" << endl;

	if (setInflows) cout << " Got address for set infows" << endl;
	else            cout << "Error: Failed to get address for set inflows" << endl;

	if (solve)	  	cout << " Got address for turbine solve" << endl;
	else			  cout << " ERROR: Failed to get address for turbine solve" << endl;


	if (setBladeInflow)	  	cout << " Got address for setBladeInflow" << endl;
	else			  cout << " ERROR: Failed to get address for setBladeInflow" << endl;

	if (getBladeNodePos)	cout << " Got address for getBladeNodePos" << endl;
	else			  cout << " ERROR: Failed to get address for getBladeNodePos" << endl;

	if (close)	  	cout << " Got address for close" << endl;
	else			  cout << " ERROR: Failed to get address for close" << endl;


	cout << "All done.  Now try calling some functions of the C++ wrapper functions..." << endl;

	double shaftSpeed = 1.66504; // in rad/s should translate to around 20.0 rpm
	double genTorque = 0;

	// initialize the hubState
	double hubState[6][3];      // 6 position DOFS (x,y,z,roll,pitch,yaw) then velocities, then accelerations
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 3; j++) {
			hubState[i][j] = 0.0;
		}
	}

	hubState[3][0] = shaftSpeed; // set shaft rotational velocity

	vector<double> forceAndMoment(6, 0.0); // 6 DOF reaction forces/moments returned from rotor

	// call the wrapper DLL function that initializes the model
	init(hubState, shaftSpeed, &nBlades, &nNodes);

	// initialize vectors based on rotor discretization
	vector<double> nodePos(3 * nNodes * nBlades, 0.0);
	vector<double> inflows(3 * nNodes * nBlades, 0.0);

	// create a steady flow situation in +x direction
	for (int i = 0; i < nBlades; i++)
		for (int j = 0; j < nNodes; j++)
			inflows[3 * nNodes * i + 3 * j + 0] = 8.7;  // m/s flow rate in x direction

	// PDS: get the node positions so you can pass in the appropriate inflows

	initInflows(inflows);
	
	cout << " DONE initializing " << endl;
	
	double t_i = 0.0;
	double dt = 0.01;
	
	cout << "There are " << nBlades << " blades with " << nNodes << " nodes each." << endl;
	
	vector< vector<double> > massMatrix( 6, vector<double>(6,  0.0) );
	vector< vector<double> > addedMassMatrix( 6, vector<double>(6,  0.0) );
	
		
	//cout << "THIS IS nodePOS as created ";
	//for (int i=0; i<nodePos.size(); i++) 	 cout<< nodePos[i];
	//cout << endl;
	
	ofstream outfile;
	outfile.open("Forces.out");
	outfile << "Forces(x,y,z)     Moments(Mx, My, Mz) \n";
	
	// Testing loop
	for(int i=0; i<8000; i++)
	{
		t_i = i*dt;
		
		cout << endl << " -------------- TIME STEP " << i << " --------------" << endl;
				
		cout << "get blade node positions from the model" << endl;
		getBladeNodePos(t_i, nodePos);
		
		cout << nodePos[0] << " " << nodePos[1] << " " << nodePos[2] << " " << nodePos[3] << " " << endl;
		
		
		cout << "send flow info to the model" << endl;
		setBladeInflow(t_i, inflows);
		
		// prescribe the hub (coupling point) rotation for now:
		hubState[3][0] = shaftSpeed; // this is rotational vel
		hubState[1][0] = (hubState[1][0] + shaftSpeed*dt); // this is orientation
		
		
		cout << "call the time stepping function of the model" << endl;
		int RK_flag = 0;
		
		solve(t_i, RK_flag, hubState, shaftSpeed, forceAndMoment, massMatrix, addedMassMatrix, &genTorque);

		outfile << "( " << forceAndMoment[0] << ", " << forceAndMoment[1] << ", " << forceAndMoment[2] << " ); ( "
			<< forceAndMoment[3] << ", " << forceAndMoment[4] << ", " << forceAndMoment[5] << " )" << endl;
	
	}

	outfile.close();
	close();
}
