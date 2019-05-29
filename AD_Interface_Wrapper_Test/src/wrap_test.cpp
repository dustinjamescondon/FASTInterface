// example of dynamic linking to a DLL (i.e. loading a DLL at runtime; not using it at all at compiletime)
// Matt Hall - 2018-07-11

//  #include whatever stuff you need here (this isn't a working example)

//#include "PDS_OpenFAST_Wrapper.h"
#include <iostream>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <windows.h>  
#include <stdlib.h>  
#include <string.h>  
#include <tchar.h>  
#include <iostream>

using namespace std;

// ------------ define some pointers to the dll's functions ---------------

// first make types for the functions (just need to specify the parameter types and return type, I also include parameter variable names as a reminder)
typedef int(*Init_DLL)();
typedef int(*Turbine_init)(int* nBlades, int* nNodes);
typedef int(*Turbine_solve)(double time, int RK_flag, const vector<double>& hubState, double shaftSpeed,
	  vector<double> &forceAndMoment, vector< vector<double> > &massMatrix,
	  vector< vector<double> > &addedMassMatrix, double* genTorque);
typedef int(*Turbine_setBladeInflow)(double time, vector<double> &bladeNodeInflow);
typedef int(*Turbine_getBladeNodePos)(double time, vector<double> &nodePos);
typedef int(*Turbine_close)();

// now make the actual function handle variables using the new types

Init_DLL		checkDLL;
Turbine_init		init;
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
	solve = (Turbine_solve)GetProcAddress(hInstLibraryTurbine, "Turbine_solve");
	setBladeInflow = (Turbine_setBladeInflow)GetProcAddress(hInstLibraryTurbine, "Turbine_setBladeInflow");
	getBladeNodePos = (Turbine_getBladeNodePos)GetProcAddress(hInstLibraryTurbine, "Turbine_getBladeNodePos");
	close = (Turbine_close)GetProcAddress(hInstLibraryTurbine, "Turbine_close");

	// check for success at finding each function in the DLL
	//if (checkDLL)	  	cout << " Got address for init" <<  endl;
	//else			  cout << " ERROR: Failed to get address for init" <<  endl;

	if (init)	  	cout << " Got address for turbine init" << endl;
	else			  cout << " ERROR: Failed to get address for turbine init" << endl;

	if (solve)	  	cout << " Got address for turbine solve" << endl;
	else			  cout << " ERROR: Failed to get address for turbine solve" << endl;


	if (setBladeInflow)	  	cout << " Got address for setBladeInflow" << endl;
	else			  cout << " ERROR: Failed to get address for setBladeInflow" << endl;

	if (getBladeNodePos)	cout << " Got address for getBladeNodePos" << endl;
	else			  cout << " ERROR: Failed to get address for getBladeNodePos" << endl;

	if (close)	  	cout << " Got address for close" << endl;
	else			  cout << " ERROR: Failed to get address for close" << endl;


	cout << "All done.  Now try calling some functions of the C++ wrapper functions..." << endl;

	// call the wrapper DLL function that initializes the model
	init(&nBlades, &nNodes);
	
	cout << " DONE initializing " << endl;
	
	double t_i = 0.0;
	double dt = 0.01;
	
	cout << "There are " << nBlades << " blades with " << nNodes << " nodes each." << endl;
	
	// initialize vectors based on rotor discretization
	vector<double> nodePos(3*nNodes*nBlades, 0.0);
	vector<double> inflows(6*nNodes*nBlades, 0.0);
	
	// TODO: this is extremely hacky
	// create a steady flow situation in +x direction
	for (int i=0; i<nBlades; i++)
		for (int j=0; j<nNodes; j++)
			inflows[6*nNodes*i + 6*j + 0] = 2.0;  // 2 m/s flow rate in x direction
	
	
	// initialize other vectors	
	vector<double> hubState(18, 0.0);      // 6 position DOFS (x,y,z,roll,pitch,yaw) then velocities, then accelerations
	vector<double> forceAndMoment(6, 0.0); // 6 DOF reaction forces/moments returned from rotor
	vector< vector<double> > massMatrix( 6, vector<double>(6,  0.0) );
	vector< vector<double> > addedMassMatrix( 6, vector<double>(6,  0.0) );

	double shaftSpeed = 0;
	double genTorque = 1;
	
		
	//cout << "THIS IS nodePOS as created ";
	//for (int i=0; i<nodePos.size(); i++) 	 cout<< nodePos[i];
	//cout << endl;
	
	
	
	// Testing loop
	for(int i=0; i<8; i++)
	{
		t_i = i*dt;
		
		cout << endl << " -------------- TIME STEP " << i << " --------------" << endl;
		
		// Calls? need to test these
		
		cout << "get blade node positions from the model" << endl;
		getBladeNodePos(t_i, nodePos);
		
		cout << nodePos[0] << " " << nodePos[1] << " " << nodePos[2] << " " << nodePos[3] << " " << endl;
		
		
		cout << "send flow info to the model" << endl;
		setBladeInflow(t_i, inflows);
		
		// prescribe the hub (coupling point) rotation for now:
		hubState[9] = shaftSpeed;
		hubState[3] = hubState[3] + shaftSpeed*dt;
		
		
		cout << "call the time stepping function of the model" << endl;
		int RK_flag = 0;
		shaftSpeed = 2.0; // in rad/s
		solve(t_i, RK_flag, hubState, shaftSpeed, forceAndMoment, massMatrix, addedMassMatrix, &genTorque);
		
		//cout << " generator torque: " << genTorque << endl;

	}
	close();
	
	/* // Test method calls 
	int step = 10;
	double state [9] = {0,1,2,3,4,5,6,7,8};
	
	double FM [6];
	
	
	Advance_AD_states(0, 0.1, 0.1, 1, 10);
	
	cout << "yeet" << endl;
		
	
	get_AD_bladeNodePos(step);
	
	get_AD_hubForceandMoment(step, FM);
	
	// PDS
	
	set_AD_hubstate(step, state);
	
	Advance_AD_states(1, 0.1, 0.1, 1, 10); */
	
	
}
