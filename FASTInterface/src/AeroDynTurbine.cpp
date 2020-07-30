#include "AeroDynTurbine.h"
#include <cmath>
#include <iostream>
using namespace Eigen;

// TODO put this and the interpolation functions in another file
const double Epsilon = 1.0e-06;

// a build-in subroutine in FORTRAN. Multiply abs(a) by the sign of b
double sign(double a, double b)
{
	if (b > 0) return a;
	else return -a;
}

Vector3d project(const Vector3d& v1, const Vector3d& v2)
{
	return (v1.dot(v2) / v2.dot(v2)) * v2;
}

// Extracts the Euler angles that will generate the passed orientation matrix
// Note: Copied from openfast's FORTRAN subroutine of the same name

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

// Construct the orientation matrix from the passed Euler angles
// Note: Copied from openfast's FORTRAN subroutine of the same name
Matrix3d EulerConstruct(const Vector3d& theta)
{
	double cx = cos(theta.x());
	double sx = sin(theta.x());

	double cy = cos(theta.y());
	double sy = sin(theta.y());

	double cz = cos(theta.z());
	double sz = sin(theta.z());

	Matrix3d m;
	m(0, 0) = cy * cz;
	m(1, 0) = -cy * sz;
	m(2, 0) = sy;
	
	m(0, 1) = cx * sz + sx * sy*cz;
	m(1, 1) = cx * cz - sx * sy*sz;
	m(2, 1) = -sx * cy;

	m(0, 2) = sx * sz - cx * sy*cz;
	m(1, 2) = sx * cz + cx * sy*sz;
	m(2, 2) = cx * cy;

	return m;
}


AeroDynTurbine::AeroDynTurbine()
{
	// TODO keep on eye on the effects of this
	dvr_time_curr = dvr_time_next = 0.0;
	onTempUpdate = false;
	useAddedMass = false;
}

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
AeroDynTurbine::HubMotion AeroDynTurbine::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs) const
{
	HubMotion hm;

	hm.position = Vector3d(nm.position);
	hm.velocity = Vector3d(nm.velocity);

	// Combine the two rotation matrices
	//Matrix3d hubOrient = nm.orientation * rotorRotation;
	Vector3d hubEulerAngles = nm.EulerAngles;
	hubEulerAngles(0) += rs.theta;
	Matrix3d hubOrient = EulerConstruct(hubEulerAngles).transpose();

	hm.orientation = hubOrient;

	hm.angularVel = rs.vel * hubOrient.col(0); // rotation axis in the global coordinate system

	HubAcc ha = CalculateHubAcc(nm.acceleration, nm.angularAcc, nacelleMotion_at_global_time_next.orientation, rs.acc);
	hm.acceleration = ha.acceleration;
	hm.angularAcc = ha.angularAcc;
	return hm;
}

// Calculates the appropriate hub accelerations according to the nacelle and rotor shaft accelerations
AeroDynTurbine::HubAcc AeroDynTurbine::CalculateHubAcc(
	const Vector3d& nacAcc,
	const Vector3d& nacRotAcc,
	const Matrix3d& nacOrient,
	double rotorShaftAcc) const
{
	HubAcc ha;

	ha.acceleration = nacAcc;

	ha.angularAcc = nacRotAcc + rotorShaftAcc * nacOrient.col(0); // rotation axis in the global coordinate system

	return ha;
}


void AeroDynTurbine::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	drivetrain.Init(constantRotorSpeed);
	mcont.Init(constantBladePitch);
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void AeroDynTurbine::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	drivetrain.Init(initialRotorVel, gearboxRatio, damping, stiffness, rotorMOI, genMOI);
}

void AeroDynTurbine::InitControllers_BladedDLL(int numBlades, const std::string& bladed_dll_fname, double initialBladePitch)
{
	mcont.Init_BladedDLL(numBlades, bladed_dll_fname.c_str(), initialBladePitch);
}

// TODO initialize the _prev inputs and outputs
// Note, this must be called after the drive train and controllers have been initialized because it 
// requires information gathered from both: rotor speed and blade pitch command.
void AeroDynTurbine::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
	bool useAddedMass,
	double coeffAddedMass,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
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
	nm.orientation = CalculateNacelleOrientation(nacelleEulerAngles);

	// Process the nacelle motions to find the hub motions
	DriveTrain::States rotorState = drivetrain.GetRotorStates();
	AeroDynTurbine::HubMotion hm = CalculateHubMotion(nm, rotorState);

	aerodyn.InitAerodyn(
		inputFilename.c_str(),
		outputFilename.c_str(),
		timestep,
		numBlades,
		hubRadius,
		precone,
		useAddedMass,
		coeffAddedMass,
		hm.position,
		hm.orientation,
		hm.velocity,
		hm.acceleration,
		hm.angularVel,
		hm.angularAcc,
		mcont.GetBladePitchCommand());

	this->useAddedMass = useAddedMass;

	// Do the initial call for the controller
	mcont.UpdateController(global_time_next, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());

	nacAccels_at_dvr_time_next.acceleration = nm.acceleration;
	nacAccels_at_dvr_time_next.rotation_acceleration = nm.angularAcc;
	global_timestep = timestep;
	global_time_curr = global_time_next = -global_timestep;

}

void AeroDynTurbine::InitInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	aerodyn.InitInflows(inflowVel, inflowAcc);
}

void AeroDynTurbine::SetCallback_CalcOutput(std::function<void(const double*, const double*, double*, double*)> calcOutput)
{
	CalcOutput_callback = calcOutput;
}

void AeroDynTurbine::SaveCurrentStates()
{
	aerodyn.SaveCurrentStates();
	drivetrain.SaveCurrentStates();

	global_time_curr_saved = global_time_curr;
	global_time_next_saved = global_time_next;

	nacelleReactionLoads_at_global_time_curr_saved = nacelleReactionLoads_at_global_time_curr;
	nacelleReactionLoads_at_global_time_next_saved = nacelleReactionLoads_at_global_time_next;

	nacelleMotion_at_global_time_curr_saved = nacelleMotion_at_global_time_curr;
	nacelleMotion_at_global_time_next_saved = nacelleMotion_at_global_time_next;
	nacelleMotion_at_dvr_time_next_saved = nacelleMotion_at_dvr_time_next;

	// Note: Can't save the controller's states because the Bladed-style DLL stores its states in the DLL
}


void AeroDynTurbine::RestoreSavedStates()
{
	aerodyn.RestoreSavedStates();
	drivetrain.RestoreSavedStates();

	global_time_curr = global_time_curr_saved;
	global_time_next = global_time_next_saved;

	nacelleReactionLoads_at_global_time_curr = nacelleReactionLoads_at_global_time_curr_saved;
	nacelleReactionLoads_at_global_time_next = nacelleReactionLoads_at_global_time_next_saved;

	nacelleMotion_at_global_time_curr = nacelleMotion_at_global_time_curr_saved;
	nacelleMotion_at_global_time_next = nacelleMotion_at_global_time_next_saved;
	nacelleMotion_at_dvr_time_next = nacelleMotion_at_dvr_time_next_saved;
}

// Begin updating the model's state by first setting the nacelle's states in the future. We can't know where the blade nodes will be 
// before actually updating all the states, so we have to provide an estimate.
// Note: The only reason the drivetrain's states are updated here is to get the rotor shaft displacement to send to AeroDyn, which in 
//	     is then used to set AeroDyn's hub state. Finally this is just used to get the blade node positions at dvr_time_next. Both the 
//       drivetrain update and the inputs sent to AeroDyn are temporary and not actually used for anything other than getting the blade
//       node positions.
void AeroDynTurbine::SetInputs_Nacelle(double p_time, const Vector3d& p_nacellePos, const Vector3d& p_nacelleEulerAngles,
	const Vector3d& p_nacelleVel, const Vector3d& p_nacelleAcc, const Vector3d& p_nacelleAngularVel,
	const Vector3d& p_nacelleAngularAcc, bool p_isTempUpdate)
{
	onTempUpdate = p_isTempUpdate;
	if (onTempUpdate) {
		SaveCurrentStates();
	}

	NacelleMotion nm;
	nm.position = p_nacellePos;
	nm.EulerAngles = p_nacelleEulerAngles;
	nm.velocity = p_nacelleVel;
	nm.acceleration = p_nacelleAcc;
	nm.angularVel = p_nacelleAngularVel;
	nm.angularAcc = p_nacelleAngularAcc;
	nm.orientation = CalculateNacelleOrientation(p_nacelleEulerAngles);

	// Advance the time (dvr_time_curr is overwritten by dvr_time_next in the advance states function)
	dvr_time_next = p_time; // Save the target time to update to

	// Save the nacelle motion for the UpdateStates function
	nacelleMotion_at_dvr_time_next = nm;

	// First we estimate the orientation and velocity of the rotor shaft so we can predict around where the nodes will be 
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	double bladePitch = mcont.GetBladePitchCommand();
	drivetrain.SetInputs(dvr_time_next, aerodynamicTorque, genTorque);

	// Note: drivetrain.AdvanceInputWindow() nor drivetrain.CopyStates_Pred_to_Act() is called, so the results of AdvanceStates_By_One_Global_Timestep here
	// are predicted (or temporary)
	drivetrain.UpdateStates();
	auto drivetrain_states = drivetrain.CalcOutput(); // predicted drivetrain states at "global_time_next"

	// Send these estimated values to AeroDyn so the caller can get the blade node positions
	HubMotion hm = CalculateHubMotion(nm, drivetrain_states.rotor);
	aerodyn.Set_Inputs_Hub(dvr_time_next, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch);
}

// Starts off the predictor-corrector inputs
void AeroDynTurbine::SetInputs_Nacelle_At_Global_Time_Next(const NacelleMotion& nm)
{
	// First we estimate the orientation and velocity of the rotor shaft so we can predict around where the nodes will be 
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	double bladePitch = mcont.GetBladePitchCommand();
	drivetrain.SetInputs(global_time_next, aerodynamicTorque, genTorque);

	// Note: drivetrain.AdvanceInputWindow() nor drivetrain.CopyStates_Pred_to_Act() is called, so the results of AdvanceStates_By_One_Global_Timestep here
	// are predicted (or temporary)
	drivetrain.UpdateStates();
	auto drivetrain_states = drivetrain.CalcOutput(); // predicted drivetrain states at "global_time_next"

	HubMotion hm = CalculateHubMotion(nm, drivetrain_states.rotor);
	aerodyn.Set_Inputs_Hub(global_time_next, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch);
}

void AeroDynTurbine::SetInputs_NacelleAcc(const Vector3d& nacelleAcc, const Vector3d& nacelleRotationAcc) 
{
	nacelleMotion_at_global_time_next.acceleration = nacelleAcc;
	nacelleMotion_at_global_time_next.angularAcc = nacelleRotationAcc;
	HubAcc ha = CalculateHubAcc(nacelleAcc, nacelleRotationAcc, nacelleMotion_at_global_time_next.orientation, drivetrain.GetRotorShaftAcc());
	aerodyn.Set_Inputs_HubAcceleration(ha.acceleration, ha.angularAcc);
}

void AeroDynTurbine::SetInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	aerodyn.Set_Inputs_Inflow(inflowVel, inflowAcc);
}

// this takes the substeps needed to reach within global_timestep of dvr_time_next
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::AdvanceStates()
{
	// TODO watch the behavior of this
	while (global_time_curr + global_timestep <= (dvr_time_next + Epsilon)) {

		// Advance the next time by one global_timestep
		global_time_next = global_time_curr + global_timestep;

		// Set the inputs accordingly
		nacelleMotion_at_global_time_next = InterpolateNacelleMotion_AtNextGlobalTime();
		SetInputs_Nacelle_At_Global_Time_Next(nacelleMotion_at_global_time_next);

		nacelleReactionLoads_at_global_time_next = AdvanceStates_By_One_Global_Timestep();
	}

	nacAccels_at_dvr_time_curr = nacAccels_at_dvr_time_next;
	dvr_time_curr = dvr_time_next;

	return nacelleReactionLoads_at_global_time_next;
}

AeroDynTurbine::NacelleMotion AeroDynTurbine::InterpolateNacelleMotion_AtNextGlobalTime() const
{
	NacelleMotion r;

	r.acceleration = InterpExtrapVector(global_time_next, nacelleMotion_at_global_time_curr.acceleration, global_time_curr,
		nacelleMotion_at_dvr_time_next.acceleration, dvr_time_next);
	r.angularAcc = InterpExtrapVector(global_time_next, nacelleMotion_at_global_time_curr.angularAcc, global_time_curr,
		nacelleMotion_at_dvr_time_next.angularAcc, dvr_time_next);
	r.velocity = InterpExtrapVector(global_time_next, nacelleMotion_at_global_time_curr.velocity, global_time_curr,
		nacelleMotion_at_dvr_time_next.velocity, dvr_time_next);
	r.angularVel = InterpExtrapVector(global_time_next, nacelleMotion_at_global_time_curr.angularVel, global_time_curr,
		nacelleMotion_at_dvr_time_next.angularVel, dvr_time_next);
	r.position = InterpExtrapVector(global_time_next, nacelleMotion_at_global_time_curr.position, global_time_curr,
		nacelleMotion_at_dvr_time_next.position, dvr_time_next);
	r.orientation = InterpOrientation(global_time_next, nacelleMotion_at_global_time_curr.orientation, global_time_curr,
		nacelleMotion_at_dvr_time_next.orientation, dvr_time_next);
	r.EulerAngles = EulerExtract(r.orientation); // TODO, not sure if need the Euler Angles, but do it for now anyway

	return r;
}

// Performs the predictor-corrector method, and uses the input-output solver for each corrector iteration
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::AdvanceStates_By_One_Global_Timestep()
{
	const int n_corrector_steps = 2;

	// Predictor Corrector loop
	for (int i = 0; i < n_corrector_steps; i++) {
		// Advance all states
		aerodyn.UpdateStates();
		drivetrain.UpdateStates();

		if (useAddedMass) {
			// Input-output solve: the resulting inputs are set inside AeroDyn and the drivetrain
			nacelleReactionLoads_at_global_time_next = CalcOutputs_And_SolveInputs();
		}
		else
		{
			// The nacelleMotion_time_next variable is seen as the correct output from the Driver,
			// so just calculate the ouputs of AD and DT, and transfer the outputs to inputs
			nacelleReactionLoads_at_global_time_next = CalcOutputs_And_DeriveInputs();
		}

		// Correct (continue in loop)
	}
	// Copy the predicted states (what the predictor-corrector process has landed on) to the actual states
	aerodyn.CopyStates_Pred_to_Curr();
	drivetrain.CopyStates_Pred_to_Curr();

	// TODO eventually remove this along with the output code in AeroDyn because PDS should be in charge of making output files
	// (but right now its good for debugging)
	aerodyn.PrintOutputLine();
	
	aerodyn.Advance_InputWindow();
	drivetrain.AdvanceInputWindow();

	// If this isn't a temporary update, then update the controller too
	if (!onTempUpdate) {
		mcont.UpdateController(global_time_next, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());
	}

	// Overwrite the nacelle x moment to be the generator torque
	//
	//   Note that only when onTempUpdate == false is the generator torque reported at current time_act;
	//   if onTempUpdate == true, then UpdateController isn't called and the generator torque given by
	//   GetGeneratorTorqueCommand() is at the previous time_act.
	// 
	//   I do see this as a problem, but I think it may be unavoidable with the given Bladed-style DLL because
	//   it is written to be loosely coupled to, meaning all calls to it are permanent, and therefore we
	//   can't call it when onTempUpdate == true

	nacelleReactionLoads_at_global_time_curr = nacelleReactionLoads_at_global_time_next;
	nacelleMotion_at_global_time_curr = nacelleMotion_at_global_time_next;
	global_time_curr = global_time_next;

	return nacelleReactionLoads_at_global_time_next;
}

// Calculate the outputs of AeroDyn and the Drive train, and derive the inputs
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::CalcOutputs_And_DeriveInputs()
{
	auto ad_output = aerodyn.CalcOutput();
	auto dt_output = drivetrain.CalcOutput();

	HubMotion ad_input_hub = CalculateHubMotion(nacelleMotion_at_global_time_next, dt_output.rotor);
	double dt_input_rotor_torque = ad_output.moment.x();
	double dt_input_gen_torque = mcont.GetGeneratorTorqueCommand();

	aerodyn.Set_Inputs_Hub(global_time_next, ad_input_hub.position, ad_input_hub.orientation, ad_input_hub.velocity, ad_input_hub.acceleration,
		ad_input_hub.angularVel, ad_input_hub.angularAcc, mcont.GetBladePitchCommand());
	drivetrain.SetInputs(global_time_next, dt_input_rotor_torque, dt_input_gen_torque);

	// Derive the inputs for Dvr at global_time_next
	Vector3d nacelleForce = TransformHubToNacelle(
		ad_output.force,
		nacelleMotion_at_global_time_next.orientation,
		aerodyn.GetInput_HubOrient());
	Vector3d nacelleMoment = TransformHubToNacelle(
		ad_output.moment,
		nacelleMotion_at_global_time_next.orientation,
		aerodyn.GetInput_HubOrient());
	nacelleMoment(0) = mcont.GetGeneratorTorqueCommand();

	NacelleReactionLoads_Vec dvr_input;
	dvr_input.force = nacelleForce;
	dvr_input.moment = nacelleMoment;
	dvr_input.power = ad_output.power;
	dvr_input.tsr = ad_output.tsr;
	
	return dvr_input;
}

AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::CalcOutputs_And_SolveInputs()
{
	// hard-code here for now
	int solver_iterations = 1;
	
	// TODO initialize this somewhere in the constructor
	// Some inputs are more sensitive to perturbations, so perturb differently
	// depending on the input. The acceleration inputs for AD are particularly sensitive.
	SerializedVector perturb_vec;
	perturb_vec.fill(1000.0);
	perturb_vec.segment(U_AD_HUB_ACC, 6).fill(0.00125);

	/* Input vector layout:
	-----------------------
	[0]  Driver    : nacelle force x
	[1]  Driver    : nacelle force y
	[2]  Driver    : nacelle force z
	[3]  Driver    : nacelle moment x
	[4]  Driver    : nacelle moment y
	[5]  Driver    : nacelle moment z
	[6]  AeroDyn   : hub acceleration x
	[7]  AeroDyn   : hub acceleration y
	[8]  AeroDyn   : hub acceleration z
	[9]  AeroDyn   : hub rotation acceleration x
	[10] AeroDyn   : hub rotation acceleration y
	[11] AeroDyn   : hub rotation acceleration z
	[12] Drivetrain: rotor torque

	Output vector layout:
	---------------------
	[0]  Driver    : nacelle acceleration x
	[1]  Driver    : nacelle acceleration y
	[2]  Driver    : nacelle acceleration z
	[3]  Driver    : nacelle rotation acceleration x
	[4]  Driver    : nacelle rotation acceleration y
	[5]  Driver    : nacelle rotation acceleration z
	[6]  AeroDyn   : hub force x
	[7]  AeroDyn   : hub force y
	[8]  AeroDyn   : hub force z
	[9]  AeroDyn   : hub moment x
	[10] AeroDyn   : hub moment y
	[11] AeroDyn   : hub moment z
	[12] Drivetrain: rotor acceleration
	*/

	// Setup serialized vector of inputs
	SerializedVector u; 
	// Setup serialized vector of outputs
	SerializedVector y;


	// Update the nacelle reaction loads stored in this object. It derives it from AeroDyn and the controller.
	CalcNacelleReactionLoads();

	// Pack the initial inputs that were used to generate the current states at global_time_next into the serialized input vector
	u.segment(U_DVR_NAC_FORCE, 3) = nacelleReactionLoads_at_global_time_next.force; // (updated when CalcNacelleReactionLoads() was called above)
	u.segment(U_DVR_NAC_MOMENT, 3) = nacelleReactionLoads_at_global_time_next.moment; // ^
	u.segment(U_AD_HUB_ACC, 3) = aerodyn.GetInput_HubAcc();
	u.segment(U_AD_HUB_ROTACC, 3) = aerodyn.GetInput_HubRotAcc();
	u(U_DT_ROTOR_TORQUE) = drivetrain.GetInput_RotorTorque();

	int k = 0;
	while (true) {
		//------------------------------------------------------
		// Calculate the outputs
		//------------------------------------------------------
		// calculate the outputs from Dvr and pack into output vector
		NacelleAccelerations dvr_output = Dvr_CalcOutput(u.segment(U_DVR_NAC_FORCE, 3), u.segment(U_DVR_NAC_MOMENT, 3));
		//  and pack into the output vector
		y.segment(Y_DVR_NAC_ACC, 3) = dvr_output.acceleration;
		y.segment(Y_DVR_NAC_ROTACC, 3) = dvr_output.rotation_acceleration;

		// calculate the outputs from AeroDyn
		Vector3d hubForce, hubMoment;
		auto ad_output = aerodyn.CalcOutput();
		//	and pack into the output vector
		y.segment(Y_AD_HUB_FORCE, 3) = ad_output.force;
		y.segment(Y_AD_HUB_MOMENT, 3) = ad_output.moment;

		// calculate the outputs from Drivetrain
		auto dt_output = drivetrain.CalcOutput();
		//	and pack into the output vector
		y(Y_DT_ROTOR_ACC) = dt_output.rotor.acc;
		//------------------------------------------------------

		if (k >= solver_iterations)
			break;

		// Q: At what point do we transfer the outputs from one to the 
		// inputs of the other? 
		// A: We do this in the residual function

		//------------------------------------------------------
		// Perturb inputs, calculate partial derivatives, and fill 
		// in the jacobian matrix
		//------------------------------------------------------
		Matrix<double, 13, 13> jacobian;
		SerializedVector u_residual = CalcResidual(y, u);

		// Driver program first
		// Q: Why perturb the nacelle x moment when we can't change it?
		// (the controller isn't updated at all in this process because it's loosely coupled without the ability to take 
		// temporary updates, but also because acclerations don't affect the generator torque)
		for (int i = 0; i < 6; i++) {
			SerializedVector u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			y_perturb = y;
			u_perturb(i) += perturb_vec(i);

			// then calculate the corresponding output
			NacelleAccelerations dvr_output = Dvr_CalcOutput(u_perturb.segment(U_DVR_NAC_FORCE, 3), u_perturb.segment(U_DVR_NAC_MOMENT, 3));
			y_perturb.segment(Y_DVR_NAC_ACC, 3) = dvr_output.acceleration;
			y_perturb.segment(Y_DVR_NAC_ROTACC, 3) = dvr_output.rotation_acceleration;

			// Numerically calculate the partial derivative of wrt this input
			SerializedVector u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			SerializedVector residual_func_partial_u = (u_perturb_residual - u_residual) / perturb_vec(i);

			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;

		}

		// AD second
		for (int i = 6; i < 12; i++) {
			SerializedVector u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			y_perturb = y;
			u_perturb(i) += perturb_vec(i);

			aerodyn.Set_Inputs_HubAcceleration(
				u_perturb.segment(U_AD_HUB_ACC, 3),
				u_perturb.segment(U_AD_HUB_ROTACC, 3));

			// then calculate the corresponding output
			ad_output = aerodyn.CalcOutput();

			y_perturb.segment(Y_AD_HUB_FORCE, 3) = ad_output.force;
			y_perturb.segment(Y_AD_HUB_MOMENT, 3) = ad_output.moment;

			// Numerically calculate the partial derivative of wrt this input
			SerializedVector u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			SerializedVector residual_func_partial_u = (u_perturb_residual - u_residual) / perturb_vec(i);

			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}

		// Drivetrain third
		{
			SerializedVector u_perturb, y_perturb;

			// perturb the last input
			u_perturb = u;
			y_perturb = y;
			u_perturb(U_DT_ROTOR_TORQUE) += perturb_vec(U_DT_ROTOR_TORQUE);

			drivetrain.SetInputs(global_time_next, u_perturb(U_DT_ROTOR_TORQUE), mcont.GetGeneratorTorqueCommand());

			// Calculate the corresponding output
			auto dt_output = drivetrain.CalcOutput();
			y_perturb(Y_DT_ROTOR_ACC) = dt_output.rotor.acc;

			// Numerically calculate the partial derivative of wrt this input
			SerializedVector u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			SerializedVector residual_func_partial_u = (u_perturb_residual - u_residual) / perturb_vec(U_DT_ROTOR_TORQUE);

			// Add this entry into the jacobian as a column
			jacobian.col(Y_DT_ROTOR_ACC) = residual_func_partial_u;
		}

		//------------------------------------------------------
		// Do one step of Newton-Raphson (actually Secant method because of numerical partial derivatives)
		//			
		//			Note: this solves the following, "Jacobian * u_delta + u_residual = 0"
		//			for u_delta
		//------------------------------------------------------
		SerializedVector u_delta = -jacobian.inverse() * u_residual;

		u = u + u_delta;
		
		//------------------------------------------------------
		// Set the inputs of AD and DT with the new approximation
		//------------------------------------------------------
		aerodyn.Set_Inputs_HubAcceleration(u.segment(U_AD_HUB_ACC, 3), u.segment(U_AD_HUB_ROTACC, 3));
		drivetrain.SetInputs(global_time_next, u(U_DT_ROTOR_TORQUE), mcont.GetGeneratorTorqueCommand());
		k++;
	}

	// Just calc this here for debug purposes to see if it's finding the zeros
	SerializedVector u_residual_check = CalcResidual(y, u);

	// TODO, still not sure what is the right thing to return from this function
	NacelleReactionLoads_Vec result;
	result.force = u.segment(U_DVR_NAC_FORCE,3);
	result.moment = u.segment(U_DVR_NAC_MOMENT,3);

	nacelleMotion_at_global_time_next.acceleration = y.segment(Y_DVR_NAC_ACC, 3);
	nacelleMotion_at_global_time_next.angularAcc = y.segment(Y_DVR_NAC_ROTACC, 3);

	return result;
}

/*
Transfers the outputs from one module to the inputs of the other, then these
resulting inputs are subtracted from the inputs that generated the outputs

Note: this is the function that we need to find the root of because if the result 
	  is a zero vector, then that means the inputs are consistent
*/
AeroDynTurbine::SerializedVector AeroDynTurbine::CalcResidual(const SerializedVector& y, const SerializedVector& u) const
{
	//----------------------------------------------------------------
	// Derive the inputs of one module from the outputs of the other
	//................................................................
	/* first start with deriving AD's inputs from ProteusDS's and the drivetrain's outputs */
	Vector3d nacelleAcc(y.data() + Y_DVR_NAC_ACC);
	Vector3d nacelleRotationAcc(y.data() + Y_DVR_NAC_ROTACC);
	Vector3d hubRotationAcc = nacelleRotationAcc + (y(Y_DT_ROTOR_ACC) * nacelleMotion_at_global_time_next.orientation.col(0));
	Vector3d hubAcc = nacelleAcc;

	/* second derive PDS's inputs from AD's outputs */
	Vector3d hubForce(y.data() + Y_AD_HUB_FORCE);
	Vector3d hubMoment(y.data() + Y_AD_HUB_MOMENT);

	Vector3d nacelleForce = TransformHubToNacelle(
		hubForce,
		nacelleMotion_at_global_time_next.orientation,
		aerodyn.GetInput_HubOrient());
	Vector3d nacelleMoment = TransformHubToNacelle(
		hubMoment,
		nacelleMotion_at_global_time_next.orientation,
		aerodyn.GetInput_HubOrient());
	nacelleMoment(0) = mcont.GetGeneratorTorqueCommand();

	/* third derive the drivetrain's input from Aerodyn's output */
	//	 Note: the first element counting from Y_AD_HUB_MOMENT is the x-component
	double rotorTorque = y(Y_AD_HUB_MOMENT);

	//------------------------------------------------------
	// Pack these results in a vector
	//......................................................
	SerializedVector u_derived;
	u_derived.segment(U_DVR_NAC_FORCE, 3) = nacelleForce;
	u_derived.segment(U_DVR_NAC_MOMENT, 3) = nacelleMoment;
	u_derived.segment(U_AD_HUB_ACC, 3) = hubAcc;
	u_derived.segment(U_AD_HUB_ROTACC, 3) = hubRotationAcc;
	u_derived(U_DT_ROTOR_TORQUE) = rotorTorque;

	/* Subtract the derived inputs from the original ones */
	return u - u_derived;
}

// Extrapolates the passed inputs to the current driver time and interpolate the output of CalcOutput_callback to the given time
AeroDynTurbine::NacelleAccelerations AeroDynTurbine::Dvr_CalcOutput(const Vector3d& nacelle_force, const Vector3d& nacelle_moment)
{
	// Extrapolate the inputs to the dvr's next time
	Vector3d nf_extrap = InterpExtrapVector(dvr_time_next, nacelleReactionLoads_at_global_time_curr.force, global_time_curr, nacelle_force, global_time_next);
	Vector3d nm_extrap = InterpExtrapVector(dvr_time_next, nacelleReactionLoads_at_global_time_curr.moment, global_time_curr, nacelle_moment, global_time_next);

	Vector3d nacAcc, nacRotAcc;

	CalcOutput_callback(nf_extrap.data(), nm_extrap.data(), nacAccels_at_dvr_time_next.acceleration.data(), nacAccels_at_dvr_time_next.rotation_acceleration.data());
	//CalcOutput_callback(nacelle_force.data(), nacelle_moment.data(), nacAccels_at_dvr_time_next.acceleration.data(), nacAccels_at_dvr_time_next.rotation_acceleration.data());
	// Interpolate the outputs at global_time_next (the outputs are at dvr_time_next)
	NacelleAccelerations result;
	result.acceleration = InterpExtrapVector(global_time_next, nacAccels_at_dvr_time_curr.acceleration, dvr_time_curr, 
		nacelleMotion_at_global_time_next.acceleration, dvr_time_next);
	result.rotation_acceleration = InterpExtrapVector(global_time_next, nacAccels_at_dvr_time_curr.rotation_acceleration, dvr_time_curr, 
		nacelleMotion_at_global_time_next.angularAcc, dvr_time_next);

	return result;
}


Matrix3d AeroDynTurbine::InterpOrientation(double target_time, const Matrix3d& orient_1, double time_1,
	const Matrix3d& orient_2, double time_2) const
{
	// if the times are nearly equal, then just return the second vector
	if (abs(time_2 - time_1) < Epsilon) {
		return orient_1;
	}
	// if the target time is very close to time_2
	else if (abs(time_2 - target_time) < Epsilon)
	{
		return orient_2;
	}
	// if the target time is very close to time_1
	else if (abs(time_1 - target_time) < Epsilon)
	{
		return orient_1;
	}

	return InterpOrientation_WithoutChecks(target_time, orient_1, time_1, orient_2, time_2);
}

Matrix3d AeroDynTurbine::InterpOrientation_WithoutChecks(double target_time, const Matrix3d& orient1, double time1,
	const Matrix3d& orient2, double time2) const
{
	// normalize the target time, so 0 is at time1, while 1 is at time2
	double t = (target_time - time1) / (time2 - time1);
	Quaterniond q1(orient1);
	Quaterniond q2(orient2);

	Matrix3d result(q1.slerp(t, q2));
	return result;
}

// TODO create version without the checks, so that functions that bulk interpolate/extrapolate a bunch of vectors/orientations
// at one time can do the check themselves once for all of them
Vector3d AeroDynTurbine::InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2) const
{
	// if the times are nearly equal, then just return the second vector
	if (abs(time_2 - time_1) < Epsilon) {
		return vect_2;
	}
	// if the target time is very close to time_2
	else if(abs(time_2 - target_time) < Epsilon)
	{
		return vect_2;
	} 
	// if the target time is very close to time_1
	else if (abs(time_1 - target_time) < Epsilon)
	{
		return vect_1;
	}
	
	return InterpExtrapVector_WithoutChecks(target_time, vect_1, time_1, vect_2, time_2);
}

Vector3d AeroDynTurbine::InterpExtrapVector_WithoutChecks(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2) const
{
	Vector3d m = (vect_2 - vect_1) / (time_2 - time_1);

	return vect_1 + m * (target_time - time_1);
}

// 
Vector3d AeroDynTurbine::TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOri, const Matrix3d& hubOri) const
{
	return nacelleOri.transpose() * hubOri * v;
}

Matrix3d AeroDynTurbine::CalculateNacelleOrientation(const Vector3d& nacelleEulerAngles) const
{
	// Use Nacelle orientation and rotor.theta to update hub orientation for AeroDyn
	Matrix3d nacelleOrient;
	nacelleOrient = EulerConstruct(nacelleEulerAngles).transpose();

	return nacelleOrient;
}

//
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::CalcNacelleReactionLoads()
{
	nacelleReactionLoads_at_global_time_next.force = TransformHubToNacelle(aerodyn.GetForce(), nacelleMotion_at_global_time_next.orientation, aerodyn.GetInput_HubOrient());
	nacelleReactionLoads_at_global_time_next.moment = TransformHubToNacelle(aerodyn.GetMoment(), nacelleMotion_at_global_time_next.orientation, aerodyn.GetInput_HubOrient());

	nacelleReactionLoads_at_global_time_next.moment(0) = mcont.GetGeneratorTorqueCommand(); 
	// ^ - the moment is in nacelle coordinate system, so just 
	// overwriting the x component works
	// -----
	// BUT! If we switch the loads to be expressed in global coordinate system, then we have to do something else.
	// This version fixes it by projection and cancellation
	// Vector3d sub = project(nacelleReactionLoads_next.moment, nacelleMotion_time_next.orientation.col(0));
	// nacelleReactionLoads_next.moment -= sub;
	// nacelleReactionLoads_next.moment += nacelleMotion_time_next.orientation.col(0) * mcont.GetGeneratorTorqueCommand();


	return nacelleReactionLoads_at_global_time_next;
}