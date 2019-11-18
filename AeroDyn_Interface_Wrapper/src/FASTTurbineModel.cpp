#include "FASTTurbineModel.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
#include <Eigen/Dense>
#include "InputFile.h"
#include <fstream>

// Used for representing vectors and matrices
using namespace Eigen;

// Takes Euler angles and returns global to local rotation matrix
Matrix3d EulerConstruct(const Vector3d& theta);
// Takes global to local rotation matrix and returns Euler angles
Vector3d EulerExtract(const Matrix3d& m);

// The implementation class for the turbine model. Anything that would be in the private section of 
// the turbine class has been placed here.
class FASTTurbineModel::PImp
{
public:
	struct HubMotion {
		Vector3d position, velocity, angularVel;
		Matrix3d orientation;
	};

	PImp();

	DriveTrain::ModelStates IntegrateDriveTrain_Euler(double time, const FASTTurbineModel::NacelleMotion&);
	DriveTrain::ModelStates IntegrateDriveTrain_RK4(double time, const FASTTurbineModel::NacelleMotion&, const std::vector<double>& inflows);

	
	NacelleReactionForces UpdateAeroDynStates(bool isRealStep);
	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);
	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation);

	bool onRealStep;
	double time, targetTime;
	FASTTurbineModel::NacelleMotion nacelleMotion;
	Matrix3d nacelleOrient, hubOrient;
	Vector3d nacelleForce, nacelleMoment;
	DriveTrain::ModelStates drivetrainStates_hold; // Holds drivetrain states on non-real state updates
	std::vector<double> inflows; // holds the blade node inflows
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain				  drivetrain;
	DriveTrain::ModelStates   dt_resultStates;
	MasterController		  mcont;

	//-----------------------------------------------
	// Debug
	std::ofstream fout_input;
	std::ofstream fout_output;

	void WriteHeader() {
		fout_input << "time\torient_roll\torient_pitch\torient_yaw\tang_vel_x\tang_vel_y\tang_vel_z" << std::endl;
		fout_output << "time\tnacforce_x\tnacforce_y\tnacforce_z\tnacmoment_x\tnacmoment_y\tnacmoment_z" << std::endl;
	}
	void LogInput(double time, FASTTurbineModel::NacelleMotion nm) {
		fout_input << time << '\t' << nm.eulerAngles[0] << '\t' << nm.eulerAngles[1] << '\t' << nm.eulerAngles[2] << '\t' << nm.angularVel[0] << '\t' << nm.angularVel[1] << '\t'
			<< nm.angularVel[2] << '\t';
	}
	void LogOutput(double time, FASTTurbineModel::NacelleReactionForces rf) {
		fout_output << time << '\t' << rf.force[0] << '\t' << rf.force[1] << '\t' << rf.force[2] << '\t' << rf.moment[0] << '\t'
			<< rf.moment[1] << '\t' << rf.moment[2] << std::endl;
	}
	void LogInflows(double x) {
		fout_input << x << '\t' << std::endl;
	}
	//---------------------------------------------------
};

FASTTurbineModel::PImp::PImp()
{
	time = targetTime = 0.0;
	onRealStep = false;

	fout_input.open("LogTurbineInput.out", std::ofstream::out);
	fout_output.open("LogTurbineOutput.out", std::ofstream::out);
	fout_input.precision(5);
	fout_output.precision(5);
	WriteHeader();
}

FASTTurbineModel::FASTTurbineModel() : p_imp(new PImp)
{

}

FASTTurbineModel::~FASTTurbineModel() = default;

// TODO send AeroDyn the actual hub orientation matrix to save the extra calculation of the Euler angles
// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
FASTTurbineModel::PImp::HubMotion FASTTurbineModel::PImp::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs)
{
	HubMotion hm;

	hm.position = Vector3d(nm.position);
	hm.velocity = Vector3d(nm.velocity);
	
	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	nacelleOrient = EulerConstruct(Vector3d(nm.eulerAngles));

	Vector3d bodyX = nacelleOrient.row(0);
	Vector3d bodyY = nacelleOrient.row(1);
	Vector3d bodyZ = nacelleOrient.row(2);

	AngleAxisd angleaxis(rs.theta, bodyX);

	Vector3d bodyY_prime = angleaxis * bodyY;
	Vector3d bodyZ_prime = angleaxis * bodyZ;

	hubOrient.row(0) = bodyX;
	hubOrient.row(1) = bodyY_prime;
	hubOrient.row(2) = bodyZ_prime;

	// Create rotation matrix for the rotor angle
	// TODO make sure this is rotating the right way
	//Matrix3d rotorRotation = EulerConstruct(Vector3d(rs.theta, 0.0, 0.0));

	// Combine the two rotation matrices
	// R'_x R_z R_y R_x (Don't know why, but this is the order of multiplication and it gives the 
	// correct result. I would assume that the R_x's would have to next to each other)
	//hubOrient = rotorRotation * nacelleOrient;

	hm.orientation = hubOrient;

	hm.angularVel = rs.vel * hubOrient.row(0); // rotation axis in the global coordinate system

	return hm;
}


void FASTTurbineModel::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->drivetrain.Init(constantRotorSpeed);
	p_imp->mcont.Init(constantBladePitch);
	p_imp->drivetrainStates_hold = p_imp->drivetrain.GetStates();
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void FASTTurbineModel::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	// TODO get Andre to change PDS to allow us to specify the initial rotor vel
	// In the meantime because PDS isn't letting us set the initial rotor vel, I've hard-coded it here
	initialRotorVel = 0.9529497705;
	//--------------------------------------------------------------------------------------

	p_imp->drivetrain.Init(initialRotorVel, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
	p_imp->drivetrainStates_hold = p_imp->drivetrain.GetStates();
}

void FASTTurbineModel::InitDriveTrain(const char* drivetrainDefFile, double initRotorSpeed)
{
	InputFile inputfile;
	inputfile.Load(drivetrainDefFile);

	double rotorMOI = inputfile.ReadDouble("RotorMomentOfInertia");
	double genMOI = inputfile.ReadDouble("GeneratorMomentOfInertia");
	double stiffness = inputfile.ReadDouble("DriveTrainStiffness");
	double damping = inputfile.ReadDouble("DriveTrainDamping");
	double gearboxRatio = inputfile.ReadDouble("GearboxRatio");
	
	p_imp->drivetrain.Init(initRotorSpeed, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
}

void FASTTurbineModel::InitControllers_BladedDLL(const char* bladed_dll_fname)
{
	p_imp->mcont.Init_BladedDLL(bladed_dll_fname);
}

void FASTTurbineModel::InitControllers_InputFile(const char* inputfilename)
{
	InputFile inputfile;
	inputfile.Load(inputfilename);

	std::string genControllerInputFile = inputfile.ReadString("GeneratorControllerFile");
	std::string pitchControllerInputFile = inputfile.ReadString("PitchControllerFile");
	p_imp->mcont.Init_InputFile(genControllerInputFile.c_str(), p_imp->drivetrain.GetGenShaftSpeed());
}

// Note, this must be called after the drive train and controllers have been initialized because it 
// requires information gathered from both: rotor speed and blade pitch command.
void FASTTurbineModel::InitAeroDyn(const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	const NacelleMotion& nm)
{
	// Process the nacelle motions to find the hub motions
	DriveTrain::States rotorState = p_imp->drivetrain.GetRotorStates();
	FASTTurbineModel::PImp::HubMotion hm = p_imp->CalculateHubMotion(nm, rotorState);

	// Hard-code this to false because added mass in AeroDyn isn't complete
	bool useAddedMass = false;

	p_imp->aerodyn.InitAerodyn(inputFilename,
		fluidDensity,
		kinematicFluidVisc,
		useAddedMass,
		hm.position,
		hm.orientation,
		hm.velocity,
		hm.angularVel,
		p_imp->mcont.GetBladePitchCommand());

	// Resize our container for the inflow velocities to the appropriate size
	p_imp->inflows.resize(p_imp->aerodyn.GetNumNodes() * 3, 0.0);

	// Do the initial call for the controller
	p_imp->mcont.UpdateController(p_imp->time, p_imp->drivetrain.GetGenShaftSpeed(), p_imp->aerodyn.GetBladePitch());
}

void FASTTurbineModel::InitInflows(const std::vector<double>& inflows)
{
	p_imp->aerodyn.InitInflows(inflows);
}

// Begin updating the model's state by first setting the nacelle states at point in future
void FASTTurbineModel::SetNacelleStates(double time, const NacelleMotion& nm, bool isRealStep) 
{
	// Debug
	if (isRealStep)
		p_imp->LogInput(time, nm);

	p_imp->targetTime = time; // Save the target time to update to
	p_imp->onRealStep = isRealStep; // Save whether this is a call for a real update or not

	// Save the nacelle motion for the UpdateStates function
	p_imp->nacelleMotion = nm;

	double dt = time - p_imp->time; 

	// First we estimate the orientation and velocity of the rotor shaft so we can predict around where the nodes will be 
	DriveTrain::ModelStates dtStates_n = p_imp->drivetrain.GetStates();
	double aerodynamicTorque = p_imp->aerodyn.GetTorque();
	double genTorque = p_imp->mcont.GetGeneratorTorqueCommand();
	double bladePitch = p_imp->mcont.GetBladePitchCommand();

	DriveTrain::ModelStates dtStates_withAcc = p_imp->drivetrain.CalcDeriv(dtStates_n, aerodynamicTorque, genTorque);
	
	// Integrate the drive train states using Euler's method for our estimate of the rotor state
	dtStates_withAcc.rotor.theta += dtStates_withAcc.rotor.vel * dt;
	dtStates_withAcc.rotor.vel += dtStates_withAcc.rotor.acc * dt;

	dtStates_withAcc.gen.theta += dtStates_withAcc.gen.vel * dt;
	dtStates_withAcc.gen.vel += dtStates_withAcc.gen.acc * dt;

	// Send these estimated values to AeroDyn so the caller can get the blade node positions
	PImp::HubMotion hm = p_imp->CalculateHubMotion(nm, dtStates_withAcc.rotor);
	p_imp->aerodyn.SetHubMotion(time, hm.position, hm.orientation, hm.velocity, hm.angularVel, bladePitch, false);
}

DriveTrain::ModelStates FASTTurbineModel::PImp::IntegrateDriveTrain_Euler(double t, const FASTTurbineModel::NacelleMotion& nm)
{
	double dt = t - time;  // Get this from somewhere

	// First we update the drive train so we can get the rotor angular displacement 
	DriveTrain::ModelStates dtStates_n = drivetrain.GetStates();
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	DriveTrain::ModelStates s = drivetrain.CalcDeriv(dtStates_n, aerodynamicTorque, genTorque);

	// Integrate the drive train states using Euler's method
	DriveTrain::ModelStates dtStates_np1 = dtStates_n;

	dtStates_np1.rotor.theta += dtStates_n.rotor.vel * dt;
	dtStates_np1.rotor.vel += s.rotor.acc * dt;

	dtStates_np1.gen.theta += dtStates_n.gen.vel * dt;
	dtStates_np1.gen.vel += s.gen.acc * dt;

	// Save this so that we can retrieve it with get methods
	drivetrainStates_hold = dtStates_np1;

	// Return updated states
	return dtStates_np1;
}

// Uses aerodyn to calculate updated drivtrain states. Doesn't update drivetrain states, nor Aerodyn's states
DriveTrain::ModelStates FASTTurbineModel::PImp::IntegrateDriveTrain_RK4(double t, const FASTTurbineModel::NacelleMotion& nm, const std::vector<double>& inflows)
{
	double dt = t - time;

	// First we update the drive train so we can get the rotor angular displacement 
	DriveTrain::ModelStates state_n = drivetrain.GetStates();
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	double bladePitch = mcont.GetBladePitchCommand();

	// K1
	DriveTrain::ModelStates s = drivetrain.CalcDeriv(state_n, aerodynamicTorque, genTorque);

	double gen_k_1v = s.gen.acc * dt;
	double gen_k_1x = state_n.gen.vel * dt;

	double rot_k_1v = s.rotor.acc * dt;
	double rot_k_1x = state_n.rotor.vel * dt;

	DriveTrain::ModelStates state_n1;
	state_n1.gen.vel = state_n.gen.vel + 0.5 * gen_k_1v;
	state_n1.gen.theta = state_n.gen.theta + 0.5 * gen_k_1x;

	state_n1.rotor.vel = state_n.rotor.vel + 0.5 * rot_k_1v;
	state_n1.rotor.theta = state_n.rotor.theta + 0.5 * rot_k_1x;

	HubMotion hm = CalculateHubMotion(nm, state_n1.rotor);

	aerodyn.SetHubMotion(time + 0.5 * dt, hm.position, hm.orientation, hm.velocity, hm.angularVel, bladePitch, false);
	aerodyn.SetInflowVelocities(inflows, false);

	// Hub reaction loads
	AeroDyn_Interface_Wrapper::HubReactionLoads hrl = aerodyn.UpdateStates(false);


	// K2
	s = drivetrain.CalcDeriv(state_n1, hrl.moment.x(), genTorque);

	double gen_k_2v = s.gen.acc * dt;
	double gen_k_2x = s.gen.vel * dt;

	double rot_k_2v = s.rotor.acc * dt;
	double rot_k_2x = s.rotor.vel * dt;

	DriveTrain::ModelStates state_n2;
	state_n2.gen.vel = state_n.gen.vel + 0.5 * gen_k_2v;
	state_n2.gen.theta = state_n.gen.theta + 0.5 * gen_k_2x;
	state_n2.rotor.vel = state_n.rotor.vel + 0.5 * rot_k_2v;
	state_n2.rotor.theta = state_n.rotor.theta + 0.5 * rot_k_2x;

	hm = CalculateHubMotion(nm, state_n2.rotor);

	aerodyn.SetHubMotion(time + 0.5 * dt, hm.position, hm.orientation, hm.velocity, hm.angularVel, bladePitch, false);
	aerodyn.SetInflowVelocities(inflows, false);
	hrl = aerodyn.UpdateStates(false);

	// K3
	s = drivetrain.CalcDeriv(state_n2, hrl.moment.x(), genTorque);

	double gen_k_3v = s.gen.acc * dt;
	double gen_k_3x = s.gen.vel * dt;

	double rot_k_3v = s.rotor.acc * dt;
	double rot_k_3x = s.rotor.vel * dt;

	DriveTrain::ModelStates state_n3;
	state_n3.gen.vel = state_n.gen.vel + gen_k_3v;
	state_n3.gen.theta = state_n.gen.theta + gen_k_3x;
	state_n3.rotor.vel = state_n.rotor.vel + rot_k_3v;
	state_n3.rotor.theta = state_n.rotor.theta + rot_k_3x;

	hm = CalculateHubMotion(nm, state_n3.rotor);
	aerodyn.SetHubMotion(time + dt, hm.position, hm.orientation, hm.velocity, hm.angularVel, bladePitch, false);
	hrl = aerodyn.UpdateStates(false);

	// K4
	s = drivetrain.CalcDeriv(state_n3, hrl.moment.x(), genTorque);

	double gen_k_4v = s.gen.acc * dt;
	double gen_k_4x = s.gen.vel * dt;

	double rot_k_4v = s.rotor.acc * dt;
	double rot_k_4x = s.rotor.vel * dt;

	// Weighted average
	DriveTrain::ModelStates state_fin;
	state_fin.gen.vel = state_n.gen.vel + (1.0 / 6.0) * (gen_k_1v + 2.0 * gen_k_2v + 2.0 * gen_k_3v + gen_k_4v);
	state_fin.gen.theta = state_n.gen.theta + (1.0 / 6.0) * (gen_k_1x + 2.0 * gen_k_2x + 2.0 * gen_k_3x + gen_k_4x);
	state_fin.rotor.vel = state_n.rotor.vel + (1.0 / 6.0) * (rot_k_1v + 2.0 * rot_k_2v + 2.0 * rot_k_3v + rot_k_4v);
	state_fin.rotor.theta = state_n.rotor.theta + (1.0 / 6.0) * (rot_k_1x + 2.0 * rot_k_2x + 2.0 * rot_k_3x + rot_k_4x);

	// Save this so that we can retrieve it with get methods
	drivetrainStates_hold = state_fin;

	return state_fin;
}


void FASTTurbineModel::GetBladeNodePositions(std::vector<double>& p)
{
	p_imp->aerodyn.GetBladeNodePositions(p, false);
}

void FASTTurbineModel::SetInflowVelocities(const std::vector<double>& inflows)
{ 
	// Don't actually update Aerodyn with these inflows, just save them
	for (unsigned int i = 0; i < p_imp->inflows.size(); ++i)
	{
		p_imp->inflows[i] = inflows[i];
	}

	if(p_imp->onRealStep)
		p_imp->LogInflows(inflows[0]);
}

FASTTurbineModel::NacelleReactionForces FASTTurbineModel::UpdateStates()
{
	// Integrate to find the drive train state
	DriveTrain::ModelStates states = p_imp->IntegrateDriveTrain_RK4(p_imp->targetTime, p_imp->nacelleMotion, p_imp->inflows);
	//DriveTrain::ModelStates states = p_imp->IntegrateDriveTrain_Euler(p_imp->targetTime, p_imp->nacelleMotion);
	
	double bladePitch = p_imp->mcont.GetBladePitchCommand();

	// Use drive train state to update aerodyn
	PImp::HubMotion hm = p_imp->CalculateHubMotion(p_imp->nacelleMotion, states.rotor);
	p_imp->aerodyn.SetHubMotion(p_imp->time, hm.position, hm.orientation, hm.velocity,
		hm.angularVel, bladePitch, p_imp->onRealStep);

	p_imp->aerodyn.SetInflowVelocities(p_imp->inflows, p_imp->onRealStep);
	FASTTurbineModel::NacelleReactionForces rf = p_imp->UpdateAeroDynStates(p_imp->onRealStep);

	// If we're taking an actual step, then save the updated drivetrain state and simulation time
	if (p_imp->onRealStep) {
		p_imp->drivetrain.SetStates(states);
		p_imp->time = p_imp->targetTime;
		p_imp->mcont.UpdateController(p_imp->time, p_imp->drivetrain.GetGenShaftSpeed(), p_imp->aerodyn.GetBladePitch());
	}

	// Overwrite the nacelle x moment to be the generator torque
	/* 
	  Note that only when onRealStep == true is the generator torque reported at current time;
	  if onRealStep == false, then UpdateController isn't called and the generator torque given by
	  GetGeneratorTorqueCommand() is at the previous time.

	  I do see this as a problem, but I think it may be unavoidable with the given Bladed-style DLL because
	  it is written to be loosely coupled to, meaning all calls to it are permanent, and therefore we 
	  can't call it when onRealStep == false
	*/
	rf.moment[0] = p_imp->mcont.GetGeneratorTorqueCommand();

	//----------------------------------
	// Debug
	if(p_imp->onRealStep)
		p_imp->LogOutput(p_imp->time, rf);

	// TODO PDS currently is applying the moments as forces: get Andre to correct this
	rf.moment[0] = rf.force[0];
	rf.moment[1] = rf.force[1];
	rf.moment[2] = rf.force[2];
	
	// Return the reaction forces
	return rf;
}

// Updates AeroDyn's state up to time + dt and will return the forces and moments at that time
FASTTurbineModel::NacelleReactionForces FASTTurbineModel::PImp::UpdateAeroDynStates(bool isRealStep)
{
	// Hub reaction loads
	AeroDyn_Interface_Wrapper::HubReactionLoads hrl = aerodyn.UpdateStates(isRealStep);

	// Transform the force and moment in from the hub coordinate system to the nacelle coordinate system
	Vector3d trans_force_vec = TransformHubToNacelle(hrl.force, nacelleOrient, hubOrient);
	Vector3d trans_moment_vec = TransformHubToNacelle(hrl.moment, nacelleOrient, hubOrient);

	// Get the results into the NacelleReactionForces structure so we can return it
	NacelleReactionForces r;
	r.power = hrl.power;
	r.tsr = hrl.tsr;
	memcpy(r.force, trans_force_vec.data(), 3 * sizeof(double));
	memcpy(r.moment, trans_moment_vec.data(), 3 * sizeof(double));

	// Save the transformed reaction loads
	nacelleForce = Vector3d(r.force);
	nacelleMoment = Vector3d(r.moment);

	return r;
}

int FASTTurbineModel::GetNumNodes() const
{
	return p_imp->aerodyn.GetNumNodes();
}

int FASTTurbineModel::GetNumBlades() const
{
	return p_imp->aerodyn.GetNumBlades();
}

double FASTTurbineModel::GetTurbineDiameter() const
{
	return p_imp->aerodyn.GetTurbineDiameter();
}

double FASTTurbineModel::GetBladePitch() const
{
	return p_imp->aerodyn.GetBladePitch();
}

double FASTTurbineModel::GetGeneratorTorque() const
{
	return p_imp->mcont.GetGeneratorTorqueCommand();
}

double FASTTurbineModel::GetGeneratorSpeed() const
{
	return p_imp->drivetrain.GetGenShaftSpeed();
}

double FASTTurbineModel::GetRotorSpeed() const
{
	return p_imp->drivetrain.GetRotorShaftSpeed();
}

double FASTTurbineModel::GetRotorAngularDisp() const
{
	return p_imp->drivetrainStates_hold.rotor.theta;
}

double FASTTurbineModel::GetGeneratorAngularDisp() const
{
	return p_imp->drivetrainStates_hold.gen.theta;
}

// All three of GetForce, GetMoment, and GetAerodynamicTorque get their values from member
// variables in AeroDyn_Interface. The value returned is the last value 
// returned by AeroDyn, whether it be from a real step or a temporary step for the RK4 
// integration.

double FASTTurbineModel::GetAerodynamicTorque() const
{
	return p_imp->aerodyn.GetTorque();
}

void FASTTurbineModel::GetNacelleForce(double v[3]) const
{
	memcpy(v, p_imp->nacelleForce.data(), 3 * sizeof(double));
}

// The x component of the moment is equal to the aerodynamic torque
void FASTTurbineModel::GetNacelleMoment(double v[3]) const
{
	v[0] = p_imp->mcont.GetGeneratorTorqueCommand();
	v[1] = p_imp->nacelleMoment.y();
	v[2] = p_imp->nacelleMoment.z();
}

double FASTTurbineModel::GetTSR() const
{
	return p_imp->aerodyn.GetTSR();
}

void FASTTurbineModel::GetHubForce(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetForce().data(), 3 * sizeof(double));
}

void FASTTurbineModel::GetHubMoment(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetMoment().data(), 3 * sizeof(double));
}


Vector3d FASTTurbineModel::PImp::TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOri,
	const Matrix3d& hubOri)
{
	return nacelleOri * hubOri.transpose() * v;
}

// taken from Aerodyn's subroutine of the same name
Matrix3d EulerConstruct(const Vector3d& theta)
{
	double cx = cos(theta(0));
	double sx = sin(theta(0));

	double cy = cos(theta(1));
	double sy = sin(theta(1));

	double cz = cos(theta(2));
	double sz = sin(theta(2));

	Matrix3d result;
	result(0, 0) = cy * cz;
	result(1, 0) = -cy * sz;
	result(2, 0) = sy;
	result(0, 1) = cx * sz + sx * sy * cz;
	result(1, 1) = cx * cz - sx * sy * sz;
	result(2, 1) = -sx * cy;
	result(0, 2) = sx * sz - cx * sy * cz;
	result(1, 2) = sx * cz + cx * sy * sz;
	result(2, 2) = cx * cy;

	return result;
}

// a build-in subroutine in FORTRAN. Multiply abs(a) by the sign of b
double sign(double a, double b)
{
	if (b > 0) return a;
	else return -a;
}

// taken from Aerodyn's subroutine of the same name
Vector3d EulerExtract(const Matrix3d& m)
{
	static const double epsilon = 1.0e-5;

	double cx, cy, cz, sx, sy, sz;
	Vector3d theta;

	cy = sqrt(pow(m(0, 0), 2) + pow(m(1, 0), 2));

	if (cy < epsilon) {

		theta(1) = atan2(m(2, 0), cy);
		theta(2) = 0.0;
		theta(0) = atan2(m(1, 2), m(1, 1));
	}
	else {
		theta(2) = atan2(-m(1, 0), m(0, 0));
		cz = cos(theta(2));
		sz = sin(theta(2));

		if (cz < epsilon) {
			cy = sign(cy, -m(1, 0) / sz);
		}

		else {
			cy = sign(cy, m(0, 0) / cz);

		}
		theta(1) = atan2(m(2, 0), cy);

		cz = cos(theta(2));
		sz = sin(theta(2));

		cx = sz * m(0, 1) + cz * m(1, 1);
		sx = sz * m(0, 2) + cz * m(1, 2);

		theta(0) = atan2(sx, cx);
	}

	return theta;
}