#include "FASTTurbine_Interface.h"
#include <Eigen/Dense>

using namespace Eigen;
Matrix3d EulerConstruct(const Vector3d& theta);
Vector3d EulerExtract(const Matrix3d& m);

// Stores the results from ProcessNacelleEulerAnglesForAeroDyn(...)
struct EulerAngleProcessingResults
{
	Vector3d hubEulerAngles;
	Vector3d hubRotationAxis;
};

struct NacelleMotion {
	Vector3d pos, vel, eulerAngles, angVel;
};

struct HubMotion {
	Vector3d pos, vel, eulerAngles, angVel;
};

// v is vector to be rotated around e by theta radians (right-hand rule applies)
// Implements Rodrigues' rotation formula
Vector3d axisAngleRotation(const Vector3d& v, const Vector3d& e, double theta)
{
	Vector3d result = cos(theta) * v + sin(theta) * e.cross(v) + (1 - cos(theta)) * e.dot(v) * e;

	return result;
}

EulerAngleProcessingResults ProcessNacelleEulerAnglesForAeroDyn(const Vector3d& nacelleEulerAngles, const DriveTrain::States& rotorStates)
{
	
	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	Matrix3d nacelleOrientation = EulerConstruct(nacelleEulerAngles);

	// Create rotation matrix for the rotor angle
	Matrix3d rotorRotation = EulerConstruct(Vector3d(rotorStates.theta, 0.0, 0.0));

	// Combine the two rotation matrices
	Matrix3d hubOrientation = rotorRotation * nacelleOrientation;

	// Calculate the Euler angles for this final orientation matrix for the hub
	Vector3d hubEulerAngles = EulerExtract(hubOrientation);

	EulerAngleProcessingResults r;
	r.hubEulerAngles = hubEulerAngles;
	r.hubRotationAxis = hubOrientation.row(0); // rotation axis in the global coordinate system

	return r;
}

FASTTurbineModel::FASTTurbineModel() : genSpeedLPF()
{
	// Set this here for now, but probably will have it set elsewhere later
	drivetrain.SetInitialRotorSpeed(0.01);
}

FASTTurbineModel::~FASTTurbineModel()
{
}

void FASTTurbineModel::InitGenController(const char *fname)
{
	gencont.LoadCSVFile(fname);
}

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
HubMotion CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs)
{
	HubMotion hm;

	hm.pos = nm.pos;
	hm.vel = nm.vel;
	
	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	Matrix3d nacelleOrientation = EulerConstruct(nm.eulerAngles);

	// Create rotation matrix for the rotor angle
	Matrix3d rotorRotation = EulerConstruct(Vector3d(rs.theta, 0.0, 0.0));

	// Combine the two rotation matrices
	Matrix3d hubOrientation = nacelleOrientation * rotorRotation;

	hm.eulerAngles = EulerExtract(hubOrientation);

	hm.angVel = rs.vel * hubOrientation.row(0); // rotation axis in the global coordinate system

	return hm;
}

void FASTTurbineModel::InitAeroDyn(const char* inputFilename,
	double fluidDensity,
	double kinematicFluidVisc,
	const double nacellePosition[3],
	const double nacelleOrientation[3],
	const double nacelleVelocity[3],
	const double nacelleAngularVelocity[3],
	double bladePitch)
{
	NacelleMotion nm;
	nm.pos = Vector3d(nacellePosition);
	nm.vel = Vector3d(nacelleVelocity);
	nm.angVel = Vector3d(nacelleAngularVelocity);
	nm.eulerAngles = Vector3d(nacelleOrientation);

	// Process the nacelle motions to find the hub motions
	HubMotion hm = CalculateHubMotion(nm, drivetrain.GetRotorStates());

	aerodyn.InitAerodyn(inputFilename,
		fluidDensity,
		kinematicFluidVisc,
		hm.pos.data(),
		hm.eulerAngles.data(),
		hm.vel.data(),
		hm.angVel.data(),
		bladePitch);
}

void FASTTurbineModel::InitInflows(const std::vector<double>& inflows)
{
	aerodyn.InitInflows(inflows);
}

void FASTTurbineModel::SetLPFCornerFreq(double c) {
	genSpeedLPF.SetCornerFreq(c);
}

void FASTTurbineModel::SetDriveTrainDamping(double c)
{
	drivetrain.SetDampingCoeff(c);
}

void FASTTurbineModel::SetDriveTrainStiffness(double c)
{
	drivetrain.SetStiffnessCoeff(c);
}

void FASTTurbineModel::SetRotorMassMOI(double m)
{
	drivetrain.SetRotorMassMomentOfInertia(m);
}

void FASTTurbineModel::SetGenMassMOI(double m)
{
	drivetrain.SetGenMassMomentOfInertia(m);
}

void FASTTurbineModel::SetGearboxRatio(double r)
{
	drivetrain.SetGearboxRatio(r);
}

void FASTTurbineModel::SetInitialRotorSpeed(double s)
{
	// Set the drive train's rotor speed, which will accordingly set the generator speed based 
	// on the gearbox ratio
	drivetrain.SetInitialRotorSpeed(s);

	// Initialize low pass filter to have this value as well
	genSpeedLPF.InitFilterVal(drivetrain.GetGenShaftSpeed());
}

// Pass the nacelle state at t + dt/2; begins calculation of temporary states at t + dt/2
void FASTTurbineModel::K1(const FASTTurbineModel::NacelleState& s, double time, double dt)
{
	static const double bladePitch = 0.0f;

	// Save the current time and the total time-step length for use in the proceeding K functions
	this->time = time;
	this->dt = dt;

	// Get the state of the drivetrain at current time
	DriveTrain::ModelStates drivetrain_states = drivetrain.GetModelStates();

	// dt as the difference from last call the filter.
	double filteredGenSpeed = genSpeedLPF.UpdateEstimate(drivetrain_states.gen.vel, dt);

	// Temporary generator speed at time + dt/2
	// Have to save this for the input of the next call to K2
	dt_resultStates = drivetrain.K1(dt, aerodyn.GetTorque(), gencont.GetTorque(filteredGenSpeed));
	
	// Calculate the Euler angles for this final orientation matrix of the hub
	EulerAngleProcessingResults r = ProcessNacelleEulerAnglesForAeroDyn(Vector3d(s.eulerAngles), drivetrain_states.rotor);
	
	// Calulate the axis-angle vector for the hub angular velocity
	Vector3d hubAngVel = drivetrain.GetRotorShaftSpeed() * r.hubRotationAxis;
	
	// Temporary update to the hub motion in AeroDyn
	aerodyn.SetHubMotion(time + 0.5 * dt, s.position, r.hubEulerAngles.data(), s.velocity, 
		hubAngVel.data(), bladePitch, false);
}

// Pass nacelle state at t + dt/2; begins calculation of temporary states at t + dt/2
void FASTTurbineModel::K2(const FASTTurbineModel::NacelleState& s)
{
	static const double bladePitch = 0.0f;

	// Using the temporary generator speed at t + dt/2
	double filteredGenSpeed = genSpeedLPF.CalcEstimation(dt_resultStates.gen.vel, 0.5 * dt);

	// Save drivetrain states for call to K3
	dt_resultStates = drivetrain.K2(dt, aerodyn.GetTorque(), gencont.GetTorque(filteredGenSpeed));

	EulerAngleProcessingResults r = ProcessNacelleEulerAnglesForAeroDyn(Vector3d(s.eulerAngles), dt_resultStates.rotor);

	Vector3d hubAngVel = dt_resultStates.rotor.vel * r.hubRotationAxis;

	aerodyn.SetHubMotion(time + 0.5 * dt, s.position, r.hubEulerAngles.data(), s.velocity,
		hubAngVel.data(), bladePitch, false);
}

// Pass nacelle state at t + dt; begins calculation of temporary states at t + dt
void FASTTurbineModel::K3(const FASTTurbineModel::NacelleState& s)
{
	static const double bladePitch = 0.0f;

	double filteredGenSpeed = genSpeedLPF.CalcEstimation(dt_resultStates.gen.vel, 0.5 * dt);

	dt_resultStates = drivetrain.K3(dt, aerodyn.GetTorque(), gencont.GetTorque(filteredGenSpeed));

	EulerAngleProcessingResults r = ProcessNacelleEulerAnglesForAeroDyn(Vector3d(s.eulerAngles), dt_resultStates.rotor);

	Vector3d hubAngVel = dt_resultStates.rotor.vel * r.hubRotationAxis;

	aerodyn.SetHubMotion(time + dt, s.position, r.hubEulerAngles.data(), s.velocity,
		hubAngVel.data(), bladePitch, false);
}

// Pass nacelle state at 
void FASTTurbineModel::K4(const FASTTurbineModel::NacelleState& s)
{
	static const double bladePitch = 0.0f;

	double filteredGenSpeed = genSpeedLPF.CalcEstimation(dt_resultStates.gen.vel, dt);

	dt_resultStates = drivetrain.K4(dt, aerodyn.GetTorque(), gencont.GetTorque(filteredGenSpeed));

	// Don't need to UpdateStates in AeroDyn... ( I don't think so right now at least )
}

// Pass actual nacelle states at t + dt
void FASTTurbineModel::K_Final(const FASTTurbineModel::NacelleState& nac_states)
{
	static const double bladePitch = 0.0;
	// Calculate final drive train states, saving them
	DriveTrain::ModelStates dt_states = drivetrain.UpdateStates();

	// --Use both nacelle states and drivetrain states to update AeroDyn--

	EulerAngleProcessingResults r = ProcessNacelleEulerAnglesForAeroDyn(Vector3d(nac_states.eulerAngles), dt_states.rotor);

	// Note, currently this doesn't include any angular velocity of the nacelle 
	// TODO implement this inclusion
	Vector3d hubAngVel = dt_states.rotor.vel * r.hubRotationAxis;

	aerodyn.SetHubMotion(time + dt, nac_states.position, r.hubEulerAngles.data(), nac_states.velocity, 
		hubAngVel.data(), bladePitch, true);
}

void FASTTurbineModel::GetBladeNodePositions_Final(std::vector<double>& p)
{
	aerodyn.GetBladeNodePositions(p, true);
}

void FASTTurbineModel::GetBladeNodePositions(std::vector<double>& p)
{
	aerodyn.GetBladeNodePositions(p, false);
}

void FASTTurbineModel::SetInflowVelocities_Final(const std::vector<double>& inflows)
{
	aerodyn.SetInflowVelocities(inflows, true);
}

void FASTTurbineModel::SetInflowVelocities(const std::vector<double>& inflows)
{
	aerodyn.SetInflowVelocities(inflows, false);
}

// this will return the forces and moments from AeroDyn 
FASTTurbineModel::NacelleReactionForces FASTTurbineModel::UpdateAeroDynStates()
{
	NacelleReactionForces r;

	double power;
	double tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	aerodyn.UpdateStates(r.force, r.moment, &power, &tsr, massMat, addedMassMat, false);

	// For now just return the force and moment as is without doing any other calculations
	// Note these forces are therefore in the hub coordinate system.
	return r;
}

// Updates AeroDyn's state up to time + dt and will return the forces and moments at that time
FASTTurbineModel::NacelleReactionForces FASTTurbineModel::UpdateAeroDynStates_Final()
{
	NacelleReactionForces r;

	double power;
	double tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	aerodyn.UpdateStates(r.force, r.moment, &power, &tsr, massMat, addedMassMat, true);

	// For now just return the force and moment as is without doing any other calculations
	// Note these forces are therefore in the hub coordinate system.
	return r;
}

int FASTTurbineModel::GetNumNodes() const
{
	return aerodyn.GetNumNodes();
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
