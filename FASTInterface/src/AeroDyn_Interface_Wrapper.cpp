
#include "AeroDyn_Interface_Wrapper.h"
#include <assert.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>

// declare the existence of the FORTRAN subroutines which are in the DLL
extern "C" {
    void INTERFACE_INITAERODYN(const char* inputFilename, int* inputFile_len, const char* outputFilename, int* outputFile_len, double* timestep, bool* useAddedMass, 
		double* coeffAddedMass, double* hubPos, double* hubOri, double* hubVel, double* hubAcc, double* hubRotVel, 
		double* hubRotAcc, int*nBlades, double* bladePitch, double* hubRadius, double* precone, int* nNodes_out, double* turbDiameter_out, 
		void** simulationInstance_out, int* errStat, char* errMsg);

    void INTERFACE_INITINPUTS_INFLOW(void* simulationInstance, int* nBlades, int* nNodes, const double inflowVel[], const double inflowAcc[]);

	void INTERFACE_SAVECURRENTSTATES(void* simulationInstance);

	void INTERFACE_RESTORESAVEDSTATES(void* simulationInstance);

    void INTERFACE_SETINPUTS_HUB(void* simulationInstance, double* time, double hubPos[3], double hubOri[3*3], double hubVel[3], double hubAcc[3],
				double hubRotVel[3], double hubRotAcc[3], double* bladePitch);

    void INTERFACE_SETINPUTS_INFLOW(void* simulationInstance, int* nBlades, int* nNodes, double* inflowVel, double* inflowAcc);

	void INTERFACE_ADVANCE_INPUTWINDOW(void* simulationInstance);

	void INTERFACE_COPYSTATES_PRED_TO_CURR(void* simulationInstance);

	void INTERFACE_SETINPUTS_HUBACCELERATION(void* simulationInstance, const double linearAcc[3], const double rotationAcc[3]);

	void INTERFACE_CALCOUTPUT(void* simulationInstance, double* force_out, double* moment_out, double* power, double* tsr);

	void INTERFACE_WRITEOUTPUTLINE(void* simulationInstance);

    void INTERFACE_UPDATESTATES(void* simulationInstance);

    void INTERFACE_GETBLADENODEPOS(void* simulationInstance, double* nodePos);

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

// Returns the inflows transformed from PDS' coordinate system to that of AD's
void AeroDyn_Interface_Wrapper::TransformInflows_PDStoAD(const std::vector<double>& pdsInflowVel, 
														 const std::vector<double>& pdsInflowAcc)
{
    // iterate through all the x, y, z components of the inflows, negating the y and z components
    for (int i = 0; i < totalNodes; ++i) {
	aerodynInflowVel[(i * 3) + 0] = pdsInflowVel[(i * 3) + 0];
	aerodynInflowVel[(i * 3) + 1] = -pdsInflowVel[(i * 3) + 1];
	aerodynInflowVel[(i * 3) + 2] = -pdsInflowVel[(i * 3) + 2];

	aerodynInflowAcc[(i * 3) + 0] =  pdsInflowAcc[(i * 3) + 0];
	aerodynInflowAcc[(i * 3) + 1] = -pdsInflowAcc[(i * 3) + 1];
	aerodynInflowAcc[(i * 3) + 2] = -pdsInflowAcc[(i * 3) + 2];
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
	const char* outputFilename,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
    bool useAddedMass,
	double coeffAddedMass,
    const Vector3d& hubPosition,
    const Matrix3d& hubOrientation,
    const Vector3d& hubVel,
    const Vector3d& hubAcc,
    const Vector3d& hubAngVel,
    const Vector3d& hubAngAcc,
    double bladePitch)
{
    // Holds the error status returned from AeroDyn_Interface's initialization function
    int errStat = 0;
    // Holds any error message related to the error status (the max length of an AeroDyn 
    // error message is 1024, so we add one to account for the null character).
    char errMsg[1025];

    // get the length of the string to pass to the FORTRAN subroutine (FORTRAN needs the length, because it 
    // doesn't recognize null-terminated character as the end of the string)
    int inputFile_len = strlen(inputFilename);
	int outputFile_len = strlen(outputFilename);

    // transform them to Aerodyn's global coordinate system
    Vector3d hubPosition_trans = Transform_PDStoAD(hubPosition);
    Vector3d hubVel_trans = Transform_PDStoAD(hubVel);
    Vector3d hubAcc_trans = Transform_PDStoAD(hubAcc);
    Vector3d hubAngVel_trans = Transform_PDStoAD(hubAngVel);
    Vector3d hubAngAcc_trans = Transform_PDStoAD(hubAngAcc);  
    Matrix3d hubOrientation_trans = TransformOrientation(hubOrientation);

	nBlades = numBlades;

    // call the initialization subroutine in the FORTRAN DLL
    INTERFACE_INITAERODYN(inputFilename, &inputFile_len, outputFilename, &outputFile_len, &timestep, &useAddedMass, &coeffAddedMass,
		hubPosition_trans.data(), hubOrientation_trans.data(), hubVel_trans.data(), hubAcc_trans.data(),
		hubAngVel_trans.data(), hubAngAcc_trans.data(), &numBlades, &bladePitch, &hubRadius, &precone, &nNodes, &turbineDiameter,
		&simulationInstance, &errStat, errMsg);

    // check the error status number 
    if (errStat == 4) {
		throw ADErrorException(errMsg);
    }

    // return the total amount of nodes used in the simulation
    totalNodes = nBlades * nNodes;

    // resize our internal vector so it can hold the velocity components of each node
    aerodynInflowVel.resize(totalNodes * 3);
    aerodynInflowAcc.resize(totalNodes * 3);

    // Save the pitch at this time_act for later
    pitch = bladePitch;
}

// Transform each inflow from PDS' coordinate system (positive-down) to AD's (positive-up), and send them 
// to AD for its initialization.
void AeroDyn_Interface_Wrapper::InitInflows(const std::vector<double>& pdsInflowVel, const std::vector<double>& pdsInflowAcc)
{
    // update member variable aerodynInflow with transformed inflows passed to this function
    TransformInflows_PDStoAD(pdsInflowVel, pdsInflowAcc);
    
    // call inflow initialization subroutine in FORTRAN DLL with these transformed inflows
    INTERFACE_INITINPUTS_INFLOW(simulationInstance, &nBlades, &nNodes, &aerodynInflowVel[0], &aerodynInflowAcc[0]);

	// TODO confirm this is a good idea - want to be able to get the loads right away
	CalcOutput();
}

void AeroDyn_Interface_Wrapper::SaveCurrentStates()
{
	INTERFACE_SAVECURRENTSTATES(simulationInstance);
	
	pitch_saved = pitch;
	input_saved = input;
	hubReactionLoads_saved = hubReactionLoads;
	aerodynInflowVel_saved = aerodynInflowVel;
	aerodynInflowAcc_saved = aerodynInflowAcc;
}

void AeroDyn_Interface_Wrapper::RestoreSavedStates()
{
	INTERFACE_RESTORESAVEDSTATES(simulationInstance);

	pitch = pitch_saved;
	input = input_saved;
	hubReactionLoads = hubReactionLoads_saved;
	aerodynInflowVel = aerodynInflowVel_saved;
	aerodynInflowAcc = aerodynInflowAcc_saved;
}

void AeroDyn_Interface_Wrapper::Set_Inputs_Hub(double time,
					     const Vector3d& hubPosition,
					     const Matrix3d& hubOrientation,
					     const Vector3d& hubVel,
					     const Vector3d& hubAcc,
					     const Vector3d& hubAngularVel,
					     const Vector3d& hubAngularAcc,
					     double bladePitch)
{
	// Save the inputs before transforming them.
	// Note: this is for the get functions, so the inputs can be retrieved in the 
	//		 same coordinate system that they were set in
	input.hubPos = hubPosition;
	input.hubOrient = hubOrientation;
	input.hubVel = hubVel;
	input.hubRotVel = hubAngularVel;
	input.hubAcc = hubAcc;
	input.hubRotAcc = hubAngularAcc;
	input.bladePitch = bladePitch;

    // transform them to Aerodyn's global coordinate system 
    Vector3d hubPos_trans = Transform_PDStoAD(hubPosition);
    Vector3d hubVel_trans = Transform_PDStoAD(hubVel);
    Vector3d hubAcc_trans = Transform_PDStoAD(hubAcc);
    Vector3d hubAngVel_trans = Transform_PDStoAD(hubAngularVel);
    Vector3d hubAngAcc_trans = Transform_PDStoAD(hubAngularAcc);
    Matrix3d hubOri_trans = TransformOrientation(hubOrientation);

	INTERFACE_SETINPUTS_HUB(simulationInstance, &time, hubPos_trans.data(), hubOri_trans.data(), hubVel_trans.data(),
				    hubAcc_trans.data(), hubAngVel_trans.data(), hubAngAcc_trans.data(), &bladePitch);
  

    pitch = bladePitch;
}

// This is just like Set_Inputs_Hub, but it only sets the accelerations. 
// The expected use of this function is to use to calculate the partial derivatives of AeroDyn's CalcSpringForce function
void AeroDyn_Interface_Wrapper::Set_Inputs_HubAcceleration(const Vector3d& hubAcc, const Vector3d& hubAngularAcc)
{
	Vector3d hubAcc_trans = Transform_PDStoAD(hubAcc);
	Vector3d hubAngAcc_trans = Transform_PDStoAD(hubAngularAcc);
	
	// Save the inputs so they can be retrieved by the get routine
	input.hubAcc = hubAcc;
	input.hubRotAcc = hubAngularAcc;

	INTERFACE_SETINPUTS_HUBACCELERATION(simulationInstance, hubAcc.data(), hubAngularAcc.data());
}

void AeroDyn_Interface_Wrapper::Set_Inputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
    // Assigns the transformed inflows to aeroDynInflows
    TransformInflows_PDStoAD(inflowVel, inflowAcc);

	// Note we're passing in the transformed inflows and not the ones passed directly as parameters
	INTERFACE_SETINPUTS_INFLOW(simulationInstance, &nBlades, &nNodes, aerodynInflowVel.data(), aerodynInflowAcc.data());
}

void AeroDyn_Interface_Wrapper::Advance_InputWindow()
{
	INTERFACE_ADVANCE_INPUTWINDOW(simulationInstance);
}

AeroDyn_Interface_Wrapper::HubReactionLoads AeroDyn_Interface_Wrapper::CalcOutput()
{
	HubReactionLoads r;

	Vector3d force_AD;
	Vector3d moment_AD;
	INTERFACE_CALCOUTPUT(simulationInstance, force_AD.data(), moment_AD.data(), &r.power, &r.tsr);

	r.force = Transform_ADtoPDS(force_AD);
	r.moment = Transform_ADtoPDS(moment_AD);

	// Save the results locally
	hubReactionLoads = r;
	return hubReactionLoads;
}

AeroDyn_Interface_Wrapper::HubReactionLoads AeroDyn_Interface_Wrapper::GetOutput() const
{
	return hubReactionLoads;
}

void AeroDyn_Interface_Wrapper::UpdateStates()
{
	INTERFACE_UPDATESTATES(simulationInstance);
}

void AeroDyn_Interface_Wrapper::CopyStates_Pred_to_Curr()
{
	INTERFACE_COPYSTATES_PRED_TO_CURR(simulationInstance);
}

void AeroDyn_Interface_Wrapper::PrintOutputLine()
{
	INTERFACE_WRITEOUTPUTLINE(simulationInstance);
}

// Communicates blade node positions to ProteusDS. This needs to be separate from the other outputs so that it can be used to get inflow values at the current time_act step.
void AeroDyn_Interface_Wrapper::GetBladeNodePositions(std::vector<double>& nodePos)
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
AeroDyn_Interface_Wrapper::HubReactionLoads AeroDyn_Interface_Wrapper::GetHubReactionLoads() const {
	return hubReactionLoads;
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