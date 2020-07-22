#include "FASTInterface.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
#include "AeroDynTurbine.h"
#include <Eigen/Dense>
#include "InputFile.h"
#include <fstream>

// If this is defined, then output files will be written to which log activity
// To silence any logging, just comment out this define preprocessor statement
// #define LOG_ACTIVITY

// Used for representing vectors and matrices
using namespace Eigen;

// Takes Euler angles and returns global to local rotation matrix
Matrix3d EulerConstruct(const Vector3d& theta);
// Takes global to local rotation matrix and returns Euler angles
Vector3d EulerExtract(const Matrix3d& m);

// The implementation class for the turbine model. Anything that would be in the private section of 
// the turbine class has been placed here.
class FASTInterface::PImp : public AeroDynTurbine
{
public:

#ifdef LOG_ACTIVITY
	//-----------------------------------------------
	// Debug
	std::ofstream fout_input;
	std::ofstream fout_output;
	std::ofstream fout_funcCalls;

	void WriteHeader() {
		fout_input << "time\torient_roll\torient_pitch\torient_yaw\tang_vel_x\tang_vel_y\tang_vel_z" << std::endl;
		fout_output << "time\tnacforce_x\tnacforce_y\tnacforce_z\tnacmoment_x\tnacmoment_y\tnacmoment_z" << std::endl;
	}
	void LogInput(double time, FASTInterface::PImp::NacelleMotion nm) {
		fout_input << time << '\t' << nm.EulerAngles[0] << '\t' << nm.EulerAngles[1] << '\t' << nm.EulerAngles[2] << '\t' << nm.angularVel[0] << '\t' << nm.angularVel[1] << '\t'
			<< nm.angularVel[2] << '\t';
	}
	void LogOutput(double time, FASTInterface::NacelleReactionLoads rf) {
		fout_output << time << '\t' << rf.force[0] << '\t' << rf.force[1] << '\t' << rf.force[2] << '\t' << rf.moment[0] << '\t'
			<< rf.moment[1] << '\t' << rf.moment[2] << std::endl;
	}
	void LogInflows(double x) {
		fout_input << x << '\t' << std::endl;
	}
#endif
};

FASTInterface::FASTInterface() :
	p_imp(new PImp)
{

}

FASTInterface::~FASTInterface()
{

}

//-----------------------------------------------------
// Initialization methods

// This takes the place of initializing the drive train and controllers
// \constantRotorSpeed: in rad/sec
// \constantBladePitch: in rads
void FASTInterface::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->InitWithConstantRotorSpeedAndPitch(constantRotorSpeed, constantBladePitch);
}

// Must initialize the drivetrain first
// \rotorMOI: rotor mass moment of inertia about the rotor shaft
// \genMOI: generator mass moment of inertia about the generator shaft
// \stiffness: the stiffness coefficient of the drive train
// \damping: the damping coefficient of the drive train
// \gearboxRatio: represented as "1 : \gearboxRatio", where LHS is the rotor shaft speed, and RHS is generator shaft speed
// \initialRotorSpeed: the initial rotation speed of the rotor shaft in rad/sec
void FASTInterface::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed)
{
	p_imp->InitDriveTrain(rotorMOI, genMOI, stiffness, damping, gearboxRatio, initRotorSpeed);
 }

// Must initialize one of the controller types second
// (Bladed-style DLL controller initialization)
// \bladed_dll_fname: the filename and path to the bladed-style DLL
// \initialBladePitch: The initial blade pitch in radians
void FASTInterface::InitControllers_BladedDLL(int numBlades, const std::string& bladed_dll_fname, double initialBladePitch)
{
	p_imp->InitControllers_BladedDLL(numBlades, bladed_dll_fname, initialBladePitch);
}

// Must initialize AeroDyn last. This is because AeroDyn's initialization requires the rotor speed 
// from the drive train and blade pitch command from the controller.
// \inputFilename: the filename of the AeroDyn input file
// \fluidDensity: 
// \kinematicFluidVisc: 
// \nacellePos: the initial position of the nacelle
// \nacelleEulerAngles: the initial orientation in Euler angles 
// \nacelleVel: the initial nacelle velocity
// \nacelleAngularVel: the initial nacelle angular velocity of the nacelle represented as axis-angle vector
void FASTInterface::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
	bool useAddedMass,
	double coeffAddedMass,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
	double fluidDensity,
	double kinematicFluidVisc,
	const double nacellePos[3],
	const double nacelleEulerAngles[3],
	const double nacelleVel[3],
	const double nacelleAcc[3],
	const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3])
{
	Vector3d nacellePos_v(nacellePos);
	Vector3d nacelleEuler_v(nacelleEulerAngles);
	Vector3d nacelleVel_v(nacelleVel);
	Vector3d nacelleAcc_v(nacelleAcc);
	Vector3d nacelleAngVel_v(nacelleAngularVel);
	Vector3d nacelleAngAcc_v(nacelleAngularAcc);

	p_imp->InitAeroDyn(inputFilename, outputFilename, useAddedMass, coeffAddedMass, timestep, numBlades, hubRadius,
		precone, fluidDensity, kinematicFluidVisc, nacellePos_v, nacelleEuler_v,
		nacelleVel_v, nacelleAcc_v, nacelleAngVel_v, nacelleAngAcc_v);
}


void FASTInterface::InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->InitInputs_Inflow(inflowVel, inflowAcc);
}
//-----------------------------------------------------------

// This will simulate from the last simulation time_act up until the time_act passed to SetNacelleStates(...)
// and return the nacelle reaction loads at that time_act.
// If isRealStep was false for SetNacelleStates, then this doesn't update states_pred permanently;
// if true, then this does update states_pred perminantely.
FASTInterface::NacelleReactionLoads FASTInterface::AdvanceStates()
{
	NacelleReactionLoads r;
	
	AeroDynTurbine::NacelleReactionLoads_Vec t = p_imp->AdvanceStates();

	memcpy(r.force, t.force.data(), 3 * sizeof(double));
	memcpy(r.moment, t.moment.data(), 3 * sizeof(double));
	r.power = t.power;
	r.tsr = t.tsr;

	return r;
}

// Returns the blade node postion at the \time_act which was passed to SetNacelleMotion(...)
// Sets the pass-by-reference parameters with the blade node positions. Assumes enough space has been allocated.
void FASTInterface::GetBladeNodePositions(std::vector<double>& nodePos_out)
{
	p_imp->GetBladeNodePositions(nodePos_out);
}

double FASTInterface::GetAerodynamicTorque() const
{
	return p_imp->GetHubReactionLoads().moment.x();
}

// Returns the reaction force on the nacelle
void FASTInterface::GetNacelleForce(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleReactionForce().data(), 3 * sizeof(double));
}

// Returns the moment of the nacelle
void FASTInterface::GetNacelleMoment(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleReactionMoment().data(), 3 * sizeof(double));
}

// Returns the tip speed ratio
double FASTInterface::GetTSR() const
{
	return p_imp->GetTSR();
}

// Get reaction force at hub in hub coordinate system
void FASTInterface::GetHubForce(double out[3]) const
{
	memcpy(out, p_imp->GetHubReactionLoads().force.data(), 3 * sizeof(double));
}

// Get moment of hub in hub coordinate system
void FASTInterface::GetHubMoment(double out[3]) const
{
	memcpy(out, p_imp->GetHubReactionLoads().moment.data(), 3 * sizeof(double));
}

// Returns the total number of nodes 
int FASTInterface::GetNumNodes() const
{
	return p_imp->GetNumNodes();
}

// Returns the number of blades for the turbine loaded from the AeroDyn input file
int FASTInterface::GetNumBlades() const
{
	return p_imp->GetNumBlades();
}

double FASTInterface::GetTurbineDiameter() const
{
	return p_imp->GetTurbineDiameter();
}

// Returns the current blade pitch
double FASTInterface::GetBladePitch() const
{
	return p_imp->GetBladePitch();
}

// Returns the current generator torque
double FASTInterface::GetGeneratorTorque() const
{
	return p_imp->GetGenTorque();
}

// Returns the current generator shaft speed from the drive train
double FASTInterface::GetGeneratorSpeed() const
{
	return p_imp->GetGenSpeed();
}

// Returns the current rotor shaft speed from the drive train
double FASTInterface::GetRotorSpeed() const
{
	return p_imp->GetRotorSpeed();
}

// Returns the current angular displacement of the rotor shaft 
double FASTInterface::GetRotorAngularDisp() const
{
	return p_imp->GetRotorShaftState().theta;
}

// Returns the current angular displacement of the generator shaft
double FASTInterface::GetGeneratorAngularDisp() const
{
	return p_imp->GetGenShaftState().theta;
}

// Begin an update to simulation states_pred to bring them to \time_act
// \time_act: the time_act to update to
// \nacellePos: the position of the nacelle
// \nacelleEulerAngles: the orientation in Euler angles 
// \nacelleVel: the nacelle velocity
// \nacelleAngularVel: the nacelle angular velocity of the nacelle represented as axis-angle vector	
// \isRealStep: if true, the turbines states_pred will be permanently updated to \time_act; if false, 
//              the turbines states_pred will only be temporarily updated to \time_act. In both cases
//				the reaction forces at /time_act are reported
void FASTInterface::SetNacelleStates(
	double time,
	const double nacellePos[3],
	const double nacelleEulerAngles[3],
	const double nacelleVel[3],
	const double nacelleAcc[3],
	const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3],
	bool isTempUpdate)
{
	using namespace Eigen;
	
	// TODO just setting time isn't enough because PDS doesnt take consistent steps
	p_imp->SetInputs_Nacelle(time, Vector3d(nacellePos), Vector3d(nacelleEulerAngles), Vector3d(nacelleVel),
		Vector3d(nacelleAcc), Vector3d(nacelleAngularVel), Vector3d(nacelleAngularAcc), isTempUpdate);
}

// Sets the inflow velocities at the \time_act passed to SetNacelleMotion(...)
void FASTInterface::SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->SetInputs_Inflow(inflowVel, inflowAcc);
}

void FASTInterface::SetCalcOutputCallback(std::function<void(const double*, const double*, double*, double*)> calcOutput) 
{
	p_imp->SetCallback_CalcOutput(calcOutput);
}
