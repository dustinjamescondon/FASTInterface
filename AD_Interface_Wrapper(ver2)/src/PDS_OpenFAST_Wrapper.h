// Dummy header file for the wrapper DLL (wrapper.dll) that facilitates ProteusDS interacting with the AD_Interface DLL.
// It converts between ProtuesDS's vector data types and regular arrays, and it handles coordinate system conversions since ProteusDS uses positive-down.

// Matt Hall, Patrick Connolly - 2018-07


#pragma once

// Note, this is defined in the project preprocessor section
#ifdef AD_INTERFACE_WRAPPER_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

#include "PDS_State.h"
#include <vector> // for vector data type used by ProteusDS

class PDS_AD_Wrapper {
public:
	// PDS_FAST is a singleton class, so this returns a pointer to the only instance of the class
	static PDS_AD_Wrapper* DECLDIR getInstance();

	// Initialize the Aerodyn and it's interface
	// ---
	// hubKinematics - two dimensional array constaining the following data about the hub
	//               hubKinematics[0][0-2] position
	//                         ...[1][0-2] orientation
	//                         ...[2][0-2] velocity
	//                         ...[3][0-2] rotational velocity
	//                         ...[4][0-2] acceleration
	//                         ...[5][0-2] rotational acceleration
	//				 shaftSpeed - the rotional speed of the shaft in rads/sec
	//				 nBlades_out - the number of blades, to be assigned upon calling the function
	//				 nNodes_out  - the number of nodes per blade, to be assigned upon calling the function
	int DECLDIR init_inputFiles(double hubKinematics[6][3], double shaftSpeed, int* nBlades_out, int* nNodes_out);

	// Initialize the inflows. The format expected is (in global coordinate system):
	// inflows[0] inflow velocity in x direction at node 0
	//     ...[1] inflow velocity in y direction at node 0
	//     ...[2] inflow velocity in z direction at node 0
	//     ...[3] inflow velocity in x direction at node 1
	// ... etc
	int DECLDIR init_inflows(const vector<double>& inflows);

	// This 
	int DECLDIR updateState(double time,
							const double hubPosition[3],
							const double hubOrientation[3],
							const double hubVelocity[3],
							const double hubRotationalVelocity[3],
							const vector<double>& inflows);

	//
	int DECLDIR solve(double time, double hubKinematics[6][3], vector<double>& forceAndMoment_out,
		vector< vector<double> >& massMatrix_out, vector< vector<double> >& addedMassMatrix);

	// Note, does not actually set the hub orientation and position in AD, it just passes these to 
	// AD so it can do the calculation of where the nodes are.
	void DECLDIR getBladeNodePositions(double hubPosition[3], double hubOrientation[3],
		vector<double>& bladeNodePos_out);




private:
	PDS_State pds_state;

	int nBlades;
	int nNodes;


};

extern "C" {


	using namespace std;


	// Initialization - loads the AD_interface DLL, initializes the model, etc.
	int DECLDIR Turbine_init(double hubKinematics[6][3], double shaftSpeed, int* nBladesOut, int* nNodesOut);
	// nBladesOut - returns number of blades
	// nNodesOut - returns number of nodes per blade

	int DECLDIR Turbine_initInflows(vector<double>& inflows);

	int DECLDIR Turbine_setInflows(vector<double>& inflows);


// Send hub kinematics and get turbine reaction forces - this is the coupling function
	int DECLDIR Turbine_solve(double time, int RK_flag, double hubKinematics[6][3], double shaftSpeed,
		vector<double>& forceAndMoment, vector< vector<double> >& massMatrix,
		vector< vector<double> >& addedMassMatrix, double* genTorque);
	// time: current simulation time in s
   // RK_flag (integer): stage in the RK45 integration process (i.e. which sub-step number) [work in progress]
	// hubState (double [18]): serialized vector of the statevaria bles for the rotor hub in ProteusDS. The vector includes, in order:
	// Position (x, y, z) (m)
	// Orientation (roll, pitch, yaw) (rad)
	// Velocity (Vx, Vy, Vz) (m/s)
	// Angular Velocity (Vroll, Vpitch, Vyaw) (rad/s)
	// Acceleration (Ax, Ay, Az) (m/s^2)
	// Angular Acceleration (Aroll, Apitch, Ayaw) (rad/s^2)
	// shaftSpeed (double): Rotational speed of the low speed shaft (rad/s)
	// forceAndMoment (double [6]): Forces and moments at the turbine rotor hub.  Stored as a vector (Fx, Fy, Fz, Mx, My, Mz).
	// massMatrix (double [36]): The sum of all inertial terms of the turbine.  Ordered as ( mxx, mxy, mxz, mx,roll, mx,pitch, mx,yaw, myx, myy etc.)
	// addedMassMatrix (double [36]): The sum of all hydrodynamic added mass terms. Ordered as ( mxx, mxy, mxz, mx,roll, mx,pitch, mx,yaw, myx, myy etc.)
	// genTorque (double): Torque exerted on the low speed rotor shaft by the generator.  Equal and opposite to the body reaction torque.


// @dustin: made bladeNodeInflow pass by reference so the vector doesn't need to be copied
// Receives water kinematics at blade node points
	int DECLDIR Turbine_setBladeInflow(double time, const vector<double>& bladeNodeInflow);
	// time: current simulation time in s
	// nodePos(double [3*n*b], n = number of nodes per blade, b = number of blades): Current positions of the blade nodes.  Stored as a serialized vector ordered as ( xb1,n1, yb1,n1, zb1,n1, xb1,n2, yb1,n2, zb1,n2, etc. )

// Returns blade node positions 
	int DECLDIR Turbine_getBladeNodePos(double time, vector<double>& nodePos);
	// time: current simulation time in s
	// nodePos(double) [3*n*b], n = number of nodes per blade, b = number of blades): Current positions of the blade nodes.  
	//                        Stored as a serialized vector ordered as ( xb1,n1, yb1,n1, zb1,n1, xb1,n2, yb1,n2, zb1,n2, etc. )

// Close things, free memory
	int DECLDIR Turbine_close();


}