#include "FASTTurbine_Interface.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
#include <Eigen/Dense>

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
		Vector3d position, velocity, orientation, angularVel;
	};

	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);
	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation);

	double time, dt; // dt of the current round of calling k_(...) functions
	Matrix3d nacelleOrient, hubOrient;
	Vector3d nacelleForce, nacelleMoment;
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain				  drivetrain;
	DriveTrain::ModelStates   dt_resultStates;
	MasterController		  mcont;
	bool usingConstantRotorVel;
};

FASTTurbineModel::FASTTurbineModel() : p_imp(new PImp)
{

}

FASTTurbineModel::~FASTTurbineModel() = default;

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
FASTTurbineModel::PImp::HubMotion FASTTurbineModel::PImp::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs)
{
	HubMotion hm;

	hm.position = Vector3d(nm.position);
	hm.velocity = Vector3d(nm.velocity);
	
	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	nacelleOrient = EulerConstruct(Vector3d(nm.eulerAngles));

	// Create rotation matrix for the rotor angle
	Matrix3d rotorRotation = EulerConstruct(Vector3d(rs.theta, 0.0, 0.0));

	// Combine the two rotation matrices (this is kinda indirect - maybe rewrite it to be clearer)
	hubOrient = rotorRotation * nacelleOrient;

	hm.orientation = EulerExtract(hubOrient);

	hm.angularVel = rs.vel * hubOrient.row(0); // rotation axis in the global coordinate system

	return hm;
}


void FASTTurbineModel::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->drivetrain.Init(constantRotorSpeed);
	p_imp->mcont.Init(constantBladePitch);
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void FASTTurbineModel::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	p_imp->drivetrain.Init(initialRotorVel, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
}

void FASTTurbineModel::InitControllers(const char* bladed_dll_fname)
{
	p_imp->mcont.Init(bladed_dll_fname);
}

void FASTTurbineModel::InitControllers(const char* gen_fname, const char* pitch_fname, double cornerFreq)
{
	p_imp->mcont.Init(gen_fname, pitch_fname, cornerFreq, p_imp->drivetrain.GetGenShaftSpeed());
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

	p_imp->aerodyn.InitAerodyn(inputFilename,
		fluidDensity,
		kinematicFluidVisc,
		hm.position.data(),
		hm.orientation.data(),
		hm.velocity.data(),
		hm.angularVel.data(),
		p_imp->mcont.GetBladePitchCommand());
}

void FASTTurbineModel::InitInflows(const std::vector<double>& inflows)
{
	p_imp->aerodyn.InitInflows(inflows);
}

// Pass the nacelle state at t + dt/2; begins calculation of temporary states at t + dt/2
void FASTTurbineModel::Step1(const FASTTurbineModel::NacelleMotion& s, double time, double dt)
{
	// Save the current time and the total time-step length for use in the proceeding K functions
	p_imp->time = time;
	p_imp->dt = dt;

	// Get the state of the drivetrain at current time
	DriveTrain::ModelStates drivetrain_states = p_imp->drivetrain.GetModelStates();

	// Update the pitch and generator controllers
	p_imp->mcont.UpdateController(time, drivetrain_states.gen.vel, p_imp->aerodyn.GetBladePitch());
	double genTorque = p_imp->mcont.GetGeneratorTorqueCommand();
	double bladePitch = p_imp->mcont.GetBladePitchCommand();

	// Temporary generator speed at time + dt/2
	// Have to save this for the input of the next call to K2
	p_imp->dt_resultStates = p_imp->drivetrain.K1(dt, p_imp->aerodyn.GetTorque(), genTorque);

	FASTTurbineModel::PImp::HubMotion hm = p_imp->CalculateHubMotion(s, p_imp->dt_resultStates.rotor);

	// Temporary update to the hub motion in AeroDyn
	p_imp->aerodyn.SetHubMotion(time + 0.5 * dt, hm.position.data(), hm.orientation.data(), hm.velocity.data(),
		hm.angularVel.data(), bladePitch, false);
}

// Pass nacelle state at t + dt/2; begins calculation of temporary states at t + dt/2
void FASTTurbineModel::Step2(const FASTTurbineModel::NacelleMotion& s)
{
	double bladePitch = p_imp->mcont.GetBladePitchCommand();
	double genTorque = p_imp->mcont.GetGeneratorTorqueCommand();

	// Save drivetrain states for call to K3
	p_imp->dt_resultStates = p_imp->drivetrain.K2(p_imp->dt, p_imp->aerodyn.GetTorque(), genTorque);

	FASTTurbineModel::PImp::HubMotion hm = p_imp->CalculateHubMotion(s, p_imp->dt_resultStates.rotor);

	p_imp->aerodyn.SetHubMotion(p_imp->time + 0.5 * p_imp->dt, hm.position.data(), hm.orientation.data(), hm.velocity.data(),
		hm.angularVel.data(), bladePitch, false);
}

// Pass nacelle state at t + dt; begins calculation of temporary states at t + dt
void FASTTurbineModel::Step3(const FASTTurbineModel::NacelleMotion& s)
{
	//double filteredGenSpeed = genSpeedLPF.CalcEstimation(dt_resultStates.gen.vel, 0.5 * dt);

	double bladePitch = p_imp->mcont.GetBladePitchCommand();
	double genTorque = p_imp->mcont.GetGeneratorTorqueCommand();

	p_imp->dt_resultStates = p_imp->drivetrain.K3(p_imp->dt, p_imp->aerodyn.GetTorque(), genTorque);

	FASTTurbineModel::PImp::HubMotion hm = p_imp->CalculateHubMotion(s, p_imp->dt_resultStates.rotor);

	p_imp->aerodyn.SetHubMotion(p_imp->time + p_imp->dt, hm.position.data(), hm.orientation.data(), hm.velocity.data(),
		hm.angularVel.data(), bladePitch, false);
}

// Pass nacelle state at
void FASTTurbineModel::Step4(const FASTTurbineModel::NacelleMotion& s)
{
	double genTorque = p_imp->mcont.GetGeneratorTorqueCommand();

	p_imp->dt_resultStates = p_imp->drivetrain.K4(p_imp->dt, p_imp->aerodyn.GetTorque(), genTorque);

	// Don't need to UpdateStates in AeroDyn... ( I don't think so right now at least )
}

// Pass actual nacelle states at t + dt
void FASTTurbineModel::CompleteStep(const FASTTurbineModel::NacelleMotion& nac_states)
{
	// Calculate final drive train states, saving them
	DriveTrain::ModelStates dt_states = p_imp->drivetrain.UpdateStates();

	// --Use both nacelle states and drivetrain states to update AeroDyn--
	FASTTurbineModel::PImp::HubMotion hm = p_imp->CalculateHubMotion(nac_states, dt_states.rotor);

	double bladePitch = p_imp->mcont.GetBladePitchCommand();

	p_imp->aerodyn.SetHubMotion(p_imp->time + p_imp->dt, hm.position.data(), hm.orientation.data(), hm.velocity.data(),
		hm.angularVel.data(), bladePitch, true);
}

void FASTTurbineModel::GetBladeNodePositions(std::vector<double>& p)
{
	p_imp->aerodyn.GetBladeNodePositions(p, true);
}

void FASTTurbineModel::GetBladeNodePositions_Tmp(std::vector<double>& p)
{
	p_imp->aerodyn.GetBladeNodePositions(p, false);
}

void FASTTurbineModel::SetInflowVelocities(const std::vector<double>& inflows)
{
	p_imp->aerodyn.SetInflowVelocities(inflows, true);
}

void FASTTurbineModel::SetInflowVelocities_Tmp(const std::vector<double>& inflows)
{
	p_imp->aerodyn.SetInflowVelocities(inflows, false);
}

// This temporarily updates AeroDyn's states and returns the reaction forces and other results from the 
// step.
FASTTurbineModel::NacelleReactionForces FASTTurbineModel::UpdateAeroDynStates_Tmp()
{
	double force[3];
	double moment[3];
	double power;
	double tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	p_imp->aerodyn.UpdateStates(force, moment, &power, &tsr, massMat, addedMassMat, false);

	// the force and moment in the hub coordinate system
	Vector3d force_vec(force);
	Vector3d moment_vec(moment);

	// the force and moment in the nacelle coordinate system
	Vector3d trans_force_vec = p_imp->TransformHubToNacelle(force_vec, p_imp->nacelleOrient, p_imp->hubOrient);
	Vector3d trans_moment_vec = p_imp->TransformHubToNacelle(moment_vec, p_imp->nacelleOrient, p_imp->hubOrient);

	NacelleReactionForces r;
	r.power = power;
	r.tsr = tsr;
	memcpy(r.force, trans_force_vec.data(), 3 * sizeof(double));
	memcpy(r.moment, trans_moment_vec.data(), 3 * sizeof(double));	

	return r;
}

// Updates AeroDyn's state up to time + dt and will return the forces and moments at that time
FASTTurbineModel::NacelleReactionForces FASTTurbineModel::UpdateAeroDynStates()
{
	double force[3];
	double moment[3];
	double power;
	double tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	p_imp->aerodyn.UpdateStates(force, moment, &power, &tsr, massMat, addedMassMat, true);

	// For now just return the force and moment as is without doing any other calculations
	// Note these forces are therefore in the hub coordinate system.

	Vector3d force_vec(force);
	Vector3d moment_vec(moment);

	Vector3d trans_force_vec = p_imp->TransformHubToNacelle(force_vec, p_imp->nacelleOrient, p_imp->hubOrient);
	Vector3d trans_moment_vec = p_imp->TransformHubToNacelle(moment_vec, p_imp->nacelleOrient, p_imp->hubOrient);

	NacelleReactionForces r;
	r.power = power;
	r.tsr = tsr;
	memcpy(r.force, trans_force_vec.data(), 3 * sizeof(double));
	memcpy(r.moment, trans_moment_vec.data(), 3 * sizeof(double));

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

double FASTTurbineModel::GetAerodynamicTorque() const
{
	return p_imp->aerodyn.GetTorque();
}

void FASTTurbineModel::GetForce(double v[3]) const
{
	memcpy(v, p_imp->nacelleForce.data(), 3 * sizeof(double));
}

void FASTTurbineModel::GetMoment(double v[3]) const
{
	memcpy(v, p_imp->nacelleMoment.data(), 3 * sizeof(double));
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