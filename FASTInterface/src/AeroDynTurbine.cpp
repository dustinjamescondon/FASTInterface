#include "AeroDynTurbine.h"

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
AeroDynTurbine::HubMotion AeroDynTurbine::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs)
{
	using namespace Eigen;

	HubMotion hm;

	hm.position = Vector3d(nm.position);
	hm.velocity = Vector3d(nm.velocity);

	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	nacelleOrient = AngleAxisd(nm.EulerAngles[0], Vector3d::UnitX())
		* AngleAxisd(nm.EulerAngles[1], Vector3d::UnitY())
		* AngleAxisd(nm.EulerAngles[2], Vector3d::UnitZ());

	// Create rotation matrix for the rotor angle
	Matrix3d rotorRotation = AngleAxisd(rs.theta, Vector3d::UnitX()).toRotationMatrix();

	// Combine the two rotation matrices
	hubOrient = nacelleOrient * rotorRotation;

	hm.orientation = hubOrient;

	hm.angularVel = rs.vel * hubOrient.col(0); // rotation axis in the global coordinate system

	HubAcc ha = CalculateHubAcc(nm.acceleration, nm.angularAcc, nacelleOrient, rs.acc);

	return hm;
}

// Calculates the appropriate hub accelerations according to the nacelle and rotor shaft accelerations
AeroDynTurbine::HubAcc AeroDynTurbine::CalculateHubAcc(
	const Vector3d& nacAcc,
	const Vector3d& nacRotAcc,
	const Matrix3d& nacOrient,
	double rotorShaftAcc)
{
	HubAcc ha;

	ha.acceleration = nacAcc;

	ha.angularAcc = rotorShaftAcc * hubOrient.col(0); // rotation axis in the global coordinate system

	return ha;
}


void AeroDynTurbine::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	drivetrain.Init(constantRotorSpeed);
	mcont.Init(constantBladePitch);
	drivetrainStates_pred = drivetrain.GetStates();
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void AeroDynTurbine::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	drivetrain.Init(initialRotorVel, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
	drivetrainStates_pred = drivetrain.GetStates();
}

void AeroDynTurbine::InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch)
{
	mcont.Init_BladedDLL(bladed_dll_fname.c_str(), initialBladePitch);
}

// Note, this must be called after the drive train and controllers have been initialized because it 
// requires information gathered from both: rotor speed and blade pitch command.
void AeroDynTurbine::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
	double fluidDensity,
	double kinematicFluidVisc,
	const Vector3d& nacellePos,
	const Vector3d& nacelleEulerAngles,
	const Vector3d& nacelleVel,
	const Vector3d& nacelleAcc,
	const Vector3d& nacelleAngularVel,
	const Vector3d& nacelleAngularAcc)
{
	// Put the parameters in nacelle motion structure
	NacelleMotion nm;
	nm.position = nacellePos;
	nm.EulerAngles = nacelleEulerAngles;
	nm.velocity = nacelleVel;
	nm.acceleration = nacelleAcc;
	nm.angularVel = nacelleAngularVel;
	nm.angularAcc = nacelleAngularAcc;

	// Process the nacelle motions to find the hub motions
	DriveTrain::States rotorState = drivetrain.GetRotorStates();
	AeroDynTurbine::HubMotion hm = CalculateHubMotion(nm, rotorState);

	// Hard-code this to false because added mass in AeroDyn isn't complete
	bool useAddedMass = false;

	aerodyn.InitAerodyn(
		inputFilename.c_str(),
		outputFilename.c_str(),
		timestep,
		numBlades,
		hubRadius,
		precone,
		fluidDensity,
		kinematicFluidVisc,
		useAddedMass,
		hm.position,
		hm.orientation,
		hm.velocity,
		hm.acceleration,
		hm.angularVel,
		hm.angularAcc,
		mcont.GetBladePitchCommand());

	// Resize our container for the inflow velocities to the appropriate size
	inflowVel.resize(aerodyn.GetNumNodes() * 3, 0.0);
	inflowAcc.resize(aerodyn.GetNumNodes() * 3, 0.0);

	// Do the initial call for the controller
	mcont.UpdateController(time, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());
}

void AeroDynTurbine::InitInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	aerodyn.InitInflows(inflowVel, inflowAcc);
}

// Begin updating the model's state by first setting the nacelle states at point in future
void AeroDynTurbine::SetInputs_Nacelle(double time, const Vector3d& nacellePos, const Vector3d& nacelleEulerAngles,
	const Vector3d& nacelleVel, const Vector3d& nacelleAcc, const Vector3d& nacelleAngularVel,
	const Vector3d& nacelleAngularAcc, bool isRealStep)
{
	NacelleMotion nm;
	nm.position = nacellePos;
	nm.EulerAngles = nacelleEulerAngles;
	nm.velocity = nacelleVel;
	nm.acceleration = nacelleAcc;
	nm.angularVel = nacelleAngularVel;
	nm.angularAcc = nacelleAngularAcc;

	targetTime = time; // Save the target time to update to
	onRealStep = isRealStep; // Save whether this is a call for a real update or not

	// Save the nacelle motion for the UpdateStates function
	nacelleMotion = nm;

	double dt = time - time;

	// First we estimate the orientation and velocity of the rotor shaft so we can predict around where the nodes will be 
	DriveTrain::ModelStates dtStates_n = drivetrain.GetStates();
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	double bladePitch = mcont.GetBladePitchCommand();

	DriveTrain::ModelStates dtStates_withAcc = drivetrain.CalcDeriv(dtStates_n, aerodynamicTorque, genTorque);

	// Integrate the drive train states using Euler's method for our estimate of the rotor state
	dtStates_withAcc.rotor.theta += dtStates_withAcc.rotor.vel * dt;
	dtStates_withAcc.rotor.vel += dtStates_withAcc.rotor.acc * dt;

	dtStates_withAcc.gen.theta += dtStates_withAcc.gen.vel * dt;
	dtStates_withAcc.gen.vel += dtStates_withAcc.gen.acc * dt;

	// Send these estimated values to AeroDyn so the caller can get the blade node positions
	HubMotion hm = CalculateHubMotion(nm, dtStates_withAcc.rotor);
	aerodyn.Set_Inputs_Hub(time, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch, false);
}

void AeroDynTurbine::SetInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	// Don't actually update Aerodyn with these inflows yet, just save them
	for (unsigned int i = 0; i < inflowVel.size(); ++i)
	{
		this->inflowVel[i] = inflowVel[i];
		this->inflowAcc[i] = inflowAcc[i];
	}

}

AeroDynTurbine::NacelleReactionLoads AeroDynTurbine::AdvanceStates()
{
	// Integrate to find the drive train state
	//-------------------------------------------
	DriveTrain::ModelStates states;

	// If we're performing a permanent step, then take more care by using the RK4 integrator
	if (onRealStep) {
		states = IntegrateDriveTrain_RK4(targetTime, nacelleMotion, inflowVel, inflowAcc);
	}
	// Otherwise just use Euler to save some time
	else {
		states = IntegrateDriveTrain_Euler(targetTime, nacelleMotion);
	}

	double bladePitch = mcont.GetBladePitchCommand();

	// Use drive train state to update aerodyn
	HubMotion hm = CalculateHubMotion(nacelleMotion, states.rotor);
	aerodyn.Set_Inputs_Hub(time, hm.position, hm.orientation, hm.velocity, hm.acceleration,
		hm.angularVel, hm.angularAcc, bladePitch, onRealStep);

	aerodyn.Set_Inputs_Inflow(inflowVel, inflowAcc, onRealStep);
	NacelleReactionLoads rf = UpdateAeroDynStates(onRealStep);

	// If we're taking an actual step, then save the updated drivetrain state and simulation time
	if (onRealStep) {
		drivetrain.SetStates(states);
		time = targetTime;
		mcont.UpdateController(time, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());
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
	rf.moment[0] = mcont.GetGeneratorTorqueCommand();

	return rf;
}

// Calculates the outputs from AeroDyn and translates them in terms of the nacelle
AeroDynTurbine::NacelleReactionLoads AeroDynTurbine::CalcOutput() {
	Vector3d force, moment;
	aerodyn.CalcOutput(force, moment, onRealStep);

	auto hrl = aerodyn.GetHubReactionLoads();
	NacelleReactionLoads nrl;
	nrl.force = force;
	nrl.moment = moment;
	nrl.power = hrl.power;
	nrl.tsr = hrl.tsr;

	return nrl;
}

// Updates AeroDyn's state up to time + dt and will return the forces and moments at that time
AeroDynTurbine::NacelleReactionLoads AeroDynTurbine::UpdateAeroDynStates(bool isRealStep)
{
	// Hub reaction loads
	AeroDyn_Interface_Wrapper::HubReactionLoads hrl = aerodyn.UpdateStates(isRealStep);

	// Transform the force and moment in from the hub coordinate system to the nacelle coordinate system
	Vector3d trans_force_vec = TransformHubToNacelle(hrl.force, nacelleOrient, hubOrient);
	Vector3d trans_moment_vec = TransformHubToNacelle(hrl.moment, nacelleOrient, hubOrient);

	// Get the results into the NacelleReactionForces structure so we can return it
	NacelleReactionLoads r;
	r.power = hrl.power;
	r.tsr = hrl.tsr;
	r.force = trans_force_vec;
	r.moment = trans_moment_vec;

	return r;
}

// Uses aerodyn to calculate drive train states at t. Doesn't set drivetrain or Aerodyn's states permanently
DriveTrain::ModelStates AeroDynTurbine::IntegrateDriveTrain_Euler(double t, const NacelleMotion& nm)
{
	double dt = t - time;

	// First we get the drive train shaft's accelerations using CalcDeriv
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
	drivetrainStates_pred = dtStates_np1;

	// Return updated states
	return dtStates_np1;
}

// Uses aerodyn to calculate drive train states at t. Doesn't set drivetrain or Aerodyn's states permenantly
DriveTrain::ModelStates AeroDynTurbine::IntegrateDriveTrain_RK4(double t, const NacelleMotion& nm,
	const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
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

	aerodyn.Set_Inputs_Hub(time + 0.5 * dt, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch, false);
	aerodyn.Set_Inputs_Inflow(inflowVel, inflowAcc, false);

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

	aerodyn.Set_Inputs_Hub(time + 0.5 * dt, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch, false);
	aerodyn.Set_Inputs_Inflow(inflowVel, inflowAcc, false);
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
	aerodyn.Set_Inputs_Hub(time + dt, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch, false);
	aerodyn.Set_Inputs_Inflow(inflowVel, inflowAcc, false);
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
	drivetrainStates_pred = state_fin;

	return state_fin;
}

Matrix3d AeroDynTurbine::InterpExtrapOrientation(double target_time, const Matrix3d& orient_1, double time_1,
	const Matrix3d& orient_2, double time_2) const
{
	// TODO
	return Matrix3d();
}

Vector3d AeroDynTurbine::InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2) const
{
	// TODO
	return Vector3d();
}

// 
Vector3d AeroDynTurbine::TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOri, const Matrix3d& hubOri)
{
	return nacelleOrient.transpose() * hubOrient * v;
}
