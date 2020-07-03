#include "FASTInterface.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
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
class FASTInterface::PImp
{
public:
	struct HubMotion {
		Vector3d position, velocity, acceleration, angularVel, angularAcc;
		Matrix3d orientation;
	};

	struct HubAcc {
		Vector3d acceleration, angularAcc;
	};

	// Structure to hold the state of the nacelle
	struct NacelleMotion {
		double position[3];
		double velocity[3];
		double acceleration[3];
		double EulerAngles[3];
		double angularVel[3];
		double angularAcc[3];
	};

	PImp();

	DriveTrain::ModelStates IntegrateDriveTrain_Euler(double time, const FASTInterface::PImp::NacelleMotion&);
	DriveTrain::ModelStates IntegrateDriveTrain_RK4(double time, const FASTInterface::PImp::NacelleMotion&, const std::vector<double>& inflowVel,
		const std::vector<double>& inflowAcc);

	Vector<double, 12> CalcResidual(const Vector<double, 12>& y, const Vector<double, 12>& u);

	NacelleReactionLoads UpdateAeroDynStates(bool isRealStep);
	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);
	HubAcc CalculateHubAcc(const Vector3d& nacAcc, const Vector3d& nacRotAcc, const Matrix3d& nacOri, double rotorShaftAcc);

	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation);
	Matrix3d InterpExtrapOrientation(double target_time, const Matrix3d& orient_1, double time_1, const Matrix3d& orient_2, double time_2) const;
	Vector3d InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1, const Vector3d& vect_2, double time_2) const;

	bool onRealStep;
	double time, targetTime;
	FASTInterface::PImp::NacelleMotion nacelleMotion;
	Matrix3d nacelleOrient, hubOrient;
	Vector3d nacelleForce, nacelleMoment;
	DriveTrain::ModelStates drivetrainStates_pred; // Holds drivetrain states on non-real state updates
	std::vector<double> inflowVel, inflowAcc; // holds the blade node inflows
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain		      drivetrain;
	DriveTrain::ModelStates   dt_resultStates;
	MasterController	      mcont;
	std::function<void(const double*, const double*, double*, double*)> CalcOutput_callback;

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

FASTInterface::PImp::PImp()
{
	time = targetTime = 0.0;
	onRealStep = false;

#ifdef LOG_ACTIVITY
	fout_input.open("LogTurbineInput.out", std::ofstream::out);
	fout_output.open("LogTurbineOutput.out", std::ofstream::out);
	fout_funcCalls.open("LogFunctionCalls.out", std::ofstream::out);
	fout_input.precision(5);
	fout_output.precision(5);
	WriteHeader();
#endif
}

FASTInterface::FASTInterface() : p_imp(new PImp)
{

}

FASTInterface::~FASTInterface() = default;

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
FASTInterface::PImp::HubMotion FASTInterface::PImp::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs)
{
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

	return hm;
}

// Calculates the appropriate hub accelerations according to the nacelle and rotor shaft accelerations
FASTInterface::PImp::HubAcc FASTInterface::PImp::CalculateHubAcc(
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


void FASTInterface::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->drivetrain.Init(constantRotorSpeed);
	p_imp->mcont.Init(constantBladePitch);
	p_imp->drivetrainStates_pred = p_imp->drivetrain.GetStates();

	// Debug
#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitWithiConstantRotorSpeedAndPitch( " << constantRotorSpeed << ", " << constantBladePitch << " );" << std::endl;
#endif
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void FASTInterface::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	p_imp->drivetrain.Init(initialRotorVel, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
	p_imp->drivetrainStates_pred = p_imp->drivetrain.GetStates();

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitDriveTrain( " << rotorMOI << ", " << genMOI << ", " << stiffness << ", " << damping << ", " << gearboxRatio << ", " << initialRotorVel << " );" << std::endl;
#endif
}

void FASTInterface::InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch)
{
	p_imp->mcont.Init_BladedDLL(bladed_dll_fname.c_str(), initialBladePitch);

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitControllers_BladedDLL( " << bladed_dll_fname << " );" << std::endl;
#endif
}

// Note, this must be called after the drive train and controllers have been initialized because it 
// requires information gathered from both: rotor speed and blade pitch command.
void FASTInterface::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
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
	// Put the parameters in the HubMotion structure
	PImp::NacelleMotion nm;
	memcpy(nm.position, nacellePos, 3 * sizeof(double));
	memcpy(nm.EulerAngles, nacelleEulerAngles, 3 * sizeof(double));
	memcpy(nm.velocity, nacelleVel, 3 * sizeof(double));
	memcpy(nm.acceleration, nacelleAcc, 3 * sizeof(double));
	memcpy(nm.angularVel, nacelleAngularVel, 3 * sizeof(double));
	memcpy(nm.angularAcc, nacelleAngularAcc, 3 * sizeof(double));

	// Process the nacelle motions to find the hub motions
	DriveTrain::States rotorState = p_imp->drivetrain.GetRotorStates();
	FASTInterface::PImp::HubMotion hm = p_imp->CalculateHubMotion(nm, rotorState);

	// Hard-code this to false because added mass in AeroDyn isn't complete
	bool useAddedMass = false;

	p_imp->aerodyn.InitAerodyn(
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
		p_imp->mcont.GetBladePitchCommand());

	// Resize our container for the inflow velocities to the appropriate size
	p_imp->inflowVel.resize(p_imp->aerodyn.GetNumNodes() * 3, 0.0);
	p_imp->inflowAcc.resize(p_imp->aerodyn.GetNumNodes() * 3, 0.0);

	// Do the initial call for the controller
	p_imp->mcont.UpdateController(p_imp->time, p_imp->drivetrain.GetGenShaftSpeed(), p_imp->aerodyn.GetBladePitch());

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitAeroDyn( );" << std::endl;
#endif
}

void FASTInterface::InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->aerodyn.InitInflows(inflowVel, inflowAcc);

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitInflows();" << std::endl;
#endif
}

// Begin updating the model's state by first setting the nacelle states at point in future
void FASTInterface::SetNacelleStates(double time, const double nacellePos[3], const double nacelleEulerAngles[3],
	const double nacelleVel[3], const double nacelleAcc[3], const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3], bool isRealStep)
{
	// Put the parameters in the HubMotion structure
	PImp::NacelleMotion nm;
	memcpy(nm.position, nacellePos, 3 * sizeof(double));
	memcpy(nm.EulerAngles, nacelleEulerAngles, 3 * sizeof(double));
	memcpy(nm.velocity, nacelleVel, 3 * sizeof(double));
	memcpy(nm.acceleration, nacelleAcc, 3 * sizeof(double));
	memcpy(nm.angularVel, nacelleAngularVel, 3 * sizeof(double));
	memcpy(nm.angularAcc, nacelleAngularAcc, 3 * sizeof(double));

	// Debug
#ifdef LOG_ACTIVITY
	if (isRealStep)
		p_imp->LogInput(time, nm);
#endif

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
	p_imp->aerodyn.Set_Inputs_Hub(time, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch, false);
}


// Uses aerodyn to calculate drive train states at t. Doesn't set drivetrain or Aerodyn's states permanently
DriveTrain::ModelStates FASTInterface::PImp::IntegrateDriveTrain_Euler(double t, const FASTInterface::PImp::NacelleMotion& nm)
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
DriveTrain::ModelStates FASTInterface::PImp::IntegrateDriveTrain_RK4(double t, const FASTInterface::PImp::NacelleMotion& nm,
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


void FASTInterface::GetBladeNodePositions(std::vector<double>& p)
{
	p_imp->aerodyn.GetBladeNodePositions(p, false);
}

void FASTInterface::SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	// Don't actually update Aerodyn with these inflows yet, just save them
	for (unsigned int i = 0; i < p_imp->inflowVel.size(); ++i)
	{
		p_imp->inflowVel[i] = inflowVel[i];
		p_imp->inflowAcc[i] = inflowAcc[i];
	}


#ifdef LOG_ACTIVITY
	if (p_imp->onRealStep)
		p_imp->LogInflows(inflowVel[0]);
#endif
}

void FASTInterface::SetCalcOutputCallback(std::function<void(const double*, const double*, double*, double*)> calcOutput)
{
	p_imp->CalcOutput_callback = calcOutput;
}

// Q: What is the drivetrain state when this should be called??
// A: 
// Note: Uses pointer arithmetic for now just to save on code length
FASTInterface::PDSAccOutputs FASTInterface::SolveForInputs_And_Outputs(const double nacelleAcc[3], const double nacelleRotationAcc[3])
{
	// hard-code here for now
	const int solver_iterations = 2;
	const double perturb = 0.01;

	// Setup serialized vector of inputs
	Vector<double,12> u;
	// Setup serialized vector of outputs
	Vector<double,12> y;

	// Use the function arguments as the initial inputs to start off the 
	// rootfinding process
	
	// Must transfer the given nacelle accelerations into hub accelerations
	PImp::HubAcc ha = p_imp->CalculateHubAcc(
		Vector3d(nacelleAcc), 
		Vector3d(nacelleRotationAcc), 
		p_imp->nacelleOrient,
		p_imp->drivetrain.GetRotorShaftAcc());

	u.segment(6, 3) = ha.acceleration;
	u.segment(9, 3) = ha.angularAcc;

	p_imp->aerodyn.Set_Inputs_HubAcceleration(ha.acceleration, ha.angularAcc, false);
	Vector3d hubForce, hubMoment;

	p_imp->aerodyn.CalcOutput(hubForce, hubMoment, false);

	// Transfer from hub coordinate system to nacelle
	Vector3d nacForce = p_imp->TransformHubToNacelle(hubForce,p_imp->nacelleOrient, p_imp->hubOrient);
	Vector3d nacMoment = p_imp->TransformHubToNacelle(hubMoment, p_imp->nacelleOrient, p_imp->hubOrient);

	u.segment(0, 3) = nacForce;
	u.segment(3, 3) = nacMoment;

	int k = 0;
	while (true) {
		//------------------------------------------------------
		// Calculate the outputs
		//------------------------------------------------------
		p_imp->CalcOutput_callback(u.data() + 0, u.data() + 3, 
								   y.data() + 0, y.data() + 3);

		p_imp->aerodyn.Set_Inputs_HubAcceleration(
			Vector3d(u.data() + 6), 
			Vector3d(u.data() + 9), 
			false);

		// calculate the outputs from AeroDyn
		Vector3d hubForce, hubMoment;
		p_imp->aerodyn.CalcOutput(
			hubForce, 
			hubMoment, 
			false);

		// Pack into the output vector
		y.segment(6, 3) = hubForce;
		y.segment(9, 3) = hubMoment;
		//------------------------------------------------------

		if (k >= solver_iterations)
			break;

		// At what point do we transfer the outputs from one to the 
		// inputs of the other? We do this in the residual function

		//------------------------------------------------------
		// Perturb inputs, calculate partial derivatives, and fill 
		// in the jacobian matrix
		//------------------------------------------------------
		Matrix<double, 12, 12> jacobian;
		Vector<double, 12> u_residual = p_imp->CalcResidual(y, u);
		
		// PDS first
		for (int i = 0; i < 6; i++) {
			Vector<double, 12> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			p_imp->CalcOutput_callback(u_perturb.data() + 0, u_perturb.data() + 3,
				y_perturb.data() + 0, y_perturb.data() + 3);

			Vector<double, 12> u_perturb_residual = p_imp->CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			Vector<double, 12> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}

		// AD second
		for (int i = 6; i < 12; i++) {
			Vector<double, 12> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			p_imp->aerodyn.Set_Inputs_HubAcceleration(
				Vector3d(u_perturb.data() + 6),
				Vector3d(u_perturb.data() + 9),
				false);

			p_imp->aerodyn.CalcOutput(
				hubForce,
				hubMoment,
				false);

			y_perturb.segment(6, 3) = hubForce;
			y_perturb.segment(9, 3) = hubMoment;

			Vector<double, 12> u_perturb_residual = p_imp->CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			Vector<double, 12> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}
		//------------------------------------------------------
		// Do one step of Newton-Raphson (actually Secant method)
		//------------------------------------------------------

	}
	PDSAccOutputs result;
	Vector3d acc(0.0, 0.0, 0.0);
	Vector3d rotAcc(0.0, 0.0, 0.0);
	memcpy(result.nacelleAcc, acc.data(), 3 * sizeof(double));
	memcpy(result.nacelleRotationAcc, rotAcc.data(), 3 * sizeof(double));
	return result;
}

/*
Transfers the outputs from one module to the inputs of the other, then these
resulting inputs are subtracted from the inputs that generated the outputs
*/
Vector<double, 12> FASTInterface::PImp::CalcResidual(const Vector<double, 12>& y, const Vector<double, 12>& u)
{
	//----------------------------------------------------------------
	// Derive the inputs of one module from the outputs of the other
	//................................................................
	/* first start with deriving AD's inputs from ProteusDS's outputs */
	Vector3d nacelleAcc(y.data() + 0);
	Vector3d nacelleRotationAcc(y.data() + 3);
	Vector3d hubRotationAcc = nacelleRotationAcc + (drivetrain.GetGenShaftAcc() * hubOrient.col(0));

	Vector3d hubAcc = nacelleAcc;

	/* second derive PDS's inputs from AD's outputs */
	Vector3d hubForce(y.data() + 6);
	Vector3d hubMoment(y.data() + 9);

	Vector3d nacelleForce = TransformHubToNacelle(
		hubForce, 
		nacelleOrient, 
		hubOrient);
	Vector3d nacelleMoment = TransformHubToNacelle(
		hubMoment,
		nacelleOrient,
		hubOrient);
	//------------------------------------------------------
	// Pack these results in a vector (TODO use .segment(i,n) for these instead)
	//......................................................
	Vector<double, 12> u_derived;
	memcpy(u_derived.data() + 0, nacelleForce.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 3, nacelleMoment.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 6, hubAcc.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 9, hubRotationAcc.data(), 3 * sizeof(double));

	/* Subtract the derived inputs from the original ones */
	return u - u_derived;
}

FASTInterface::NacelleReactionLoads FASTInterface::Simulate()
{
	// Integrate to find the drive train state
	//-------------------------------------------
	DriveTrain::ModelStates states;

	// If we're performing a permanent step, then take more care by using the RK4 integrator
	if (p_imp->onRealStep) {
		states = p_imp->IntegrateDriveTrain_RK4(p_imp->targetTime, p_imp->nacelleMotion, p_imp->inflowVel, p_imp->inflowAcc);
	}
	// Otherwise just use Euler to save some time
	else {
		states = p_imp->IntegrateDriveTrain_Euler(p_imp->targetTime, p_imp->nacelleMotion);
	}

	double bladePitch = p_imp->mcont.GetBladePitchCommand();

	// Use drive train state to update aerodyn
	PImp::HubMotion hm = p_imp->CalculateHubMotion(p_imp->nacelleMotion, states.rotor);
	p_imp->aerodyn.Set_Inputs_Hub(p_imp->time, hm.position, hm.orientation, hm.velocity, hm.acceleration,
		hm.angularVel, hm.angularAcc, bladePitch, p_imp->onRealStep);

	p_imp->aerodyn.Set_Inputs_Inflow(p_imp->inflowVel, p_imp->inflowAcc, p_imp->onRealStep);
	FASTInterface::NacelleReactionLoads rf = p_imp->UpdateAeroDynStates(p_imp->onRealStep);

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
#ifdef LOG_ACTIVITY
	if (p_imp->onRealStep)
		p_imp->LogOutput(p_imp->time, rf);
#endif

	return rf;
}

// Updates AeroDyn's state up to time + dt and will return the forces and moments at that time
FASTInterface::NacelleReactionLoads FASTInterface::PImp::UpdateAeroDynStates(bool isRealStep)
{
	// Hub reaction loads
	AeroDyn_Interface_Wrapper::HubReactionLoads hrl = aerodyn.UpdateStates(isRealStep);

	// Transform the force and moment in from the hub coordinate system to the nacelle coordinate system
	Vector3d trans_force_vec  = TransformHubToNacelle(hrl.force,  nacelleOrient, hubOrient);
	Vector3d trans_moment_vec = TransformHubToNacelle(hrl.moment, nacelleOrient, hubOrient);

	// Get the results into the NacelleReactionForces structure so we can return it
	NacelleReactionLoads r;
	r.power = hrl.power;
	r.tsr = hrl.tsr;
	memcpy(r.force, trans_force_vec.data(), 3 * sizeof(double));
	memcpy(r.moment, trans_moment_vec.data(), 3 * sizeof(double));

	// Save the transformed reaction loads
	nacelleForce = Vector3d(r.force);
	nacelleMoment = Vector3d(r.moment);

	return r;
}

Matrix3d FASTInterface::PImp::InterpExtrapOrientation(double target_time, const Matrix3d& orient_1, double time_1, 
	const Matrix3d& orient_2, double time_2) const
{
	// TODO
	return Matrix3d();
}

Vector3d FASTInterface::PImp::InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1, 
	const Vector3d& vect_2, double time_2) const
{
	// TODO
	return Vector3d();
}

int FASTInterface::GetNumNodes() const
{
	return p_imp->aerodyn.GetNumNodes();
}

int FASTInterface::GetNumBlades() const
{
	return p_imp->aerodyn.GetNumBlades();
}

double FASTInterface::GetTurbineDiameter() const
{
	return p_imp->aerodyn.GetTurbineDiameter();
}

double FASTInterface::GetBladePitch() const
{
	return p_imp->aerodyn.GetBladePitch();
}

double FASTInterface::GetGeneratorTorque() const
{
	return p_imp->mcont.GetGeneratorTorqueCommand();
}

double FASTInterface::GetGeneratorSpeed() const
{
	return p_imp->drivetrain.GetGenShaftSpeed();
}

double FASTInterface::GetRotorSpeed() const
{
	return p_imp->drivetrain.GetRotorShaftSpeed();
}

double FASTInterface::GetRotorAngularDisp() const
{
	return p_imp->drivetrainStates_pred.rotor.theta;
}

double FASTInterface::GetGeneratorAngularDisp() const
{
	return p_imp->drivetrainStates_pred.gen.theta;
}

// All three of GetForce, GetMoment, and GetAerodynamicTorque get their values from member
// variables in AeroDyn_Interface. The value returned is the last value 
// returned by AeroDyn, whether it be from a real step or a temporary step for the RK4 
// integration.

double FASTInterface::GetAerodynamicTorque() const
{
	return p_imp->aerodyn.GetTorque();
}

void FASTInterface::GetNacelleForce(double v[3]) const
{
	memcpy(v, p_imp->nacelleForce.data(), 3 * sizeof(double));
}

// The x component of the moment is equal to the aerodynamic torque
void FASTInterface::GetNacelleMoment(double v[3]) const
{
	v[0] = p_imp->mcont.GetGeneratorTorqueCommand();
	v[1] = p_imp->nacelleMoment.y();
	v[2] = p_imp->nacelleMoment.z();
}

double FASTInterface::GetTSR() const
{
	return p_imp->aerodyn.GetTSR();
}

void FASTInterface::GetHubForce(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetForce().data(), 3 * sizeof(double));
}

void FASTInterface::GetHubMoment(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetMoment().data(), 3 * sizeof(double));
}

// 
Vector3d FASTInterface::PImp::TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOri, const Matrix3d& hubOri)
{
	return nacelleOrient.transpose() * hubOrient * v;
}
