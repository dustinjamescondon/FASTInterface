#include "AeroDynTurbine.h"
using namespace Eigen;

AeroDynTurbine::AeroDynTurbine()
{
	time_curr = 0; 
	time_next = 0;
	onTempUpdate = false;
	useAddedMass = false;
}

// Calculates the appropriate hub motion according to the nacelle motion and rotor motion
AeroDynTurbine::HubMotion AeroDynTurbine::CalculateHubMotion(const NacelleMotion& nm, const DriveTrain::States& rs) const
{
	HubMotion hm;

	hm.position = Vector3d(nm.position);
	hm.velocity = Vector3d(nm.velocity);

	// Create rotation matrix for the rotor angle
	Matrix3d rotorRotation = AngleAxisd(rs.theta, Vector3d::UnitX()).toRotationMatrix();

	// Combine the two rotation matrices
	Matrix3d hubOrient = nm.orientation * rotorRotation;

	hm.orientation = hubOrient;

	hm.angularVel = rs.vel * hubOrient.col(0); // rotation axis in the global coordinate system

	HubAcc ha = CalculateHubAcc(nm.acceleration, nm.angularAcc, nacelleMotion.orientation, rs.acc);
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
	nm.orientation = CalculateNacelleOrientation(nacelleEulerAngles);

	// Process the nacelle motions to find the hub motions
	DriveTrain::States rotorState = drivetrain.GetRotorStates();
	AeroDynTurbine::HubMotion hm = CalculateHubMotion(nm, rotorState);

	// Hard-code this to false because added mass in AeroDyn isn't complete
	useAddedMass = false;

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
	mcont.UpdateController(time_next, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());
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

	// Note: Can't save the controller's states because the Bladed-style DLL stores its states in the DLL
}


void AeroDynTurbine::RestoreSavedStates()
{
	aerodyn.RestoreSavedStates();
	drivetrain.RestoreSavedStates();
}

// Begin updating the model's state by first setting the nacelle's states in the future. We can't know where the blade nodes will be 
// before actually updating all the states, so we have to provide an estimate. We could extrapolate some things to get the future blade node
// positions; OR we could just send where the blade nodes are currently at "time" (rather than "time + dt")
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

	// Advance the time (time_curr is overwritten by time_next in the advance states function)
	time_next = p_time; // Save the target time_act to update to

	// Save the nacelle motion for the UpdateStates function
	nacelleMotion = nm;

	// First we estimate the orientation and velocity of the rotor shaft so we can predict around where the nodes will be 
	double aerodynamicTorque = aerodyn.GetTorque();
	double genTorque = mcont.GetGeneratorTorqueCommand();
	double bladePitch = mcont.GetBladePitchCommand();
	drivetrain.SetInputs(p_time, aerodynamicTorque, genTorque);

	// Note: drivetrain.AdvanceInputWindow() nor drivetrain.CopyStates_Pred_to_Act() is called, so the results of AdvanceStates here
	// are predicted (or temporary)
	drivetrain.UpdateStates();
	auto drivetrain_states = drivetrain.CalcOutput(); // predicted drivetrain states at "time_next"

	// Send these estimated values to AeroDyn so the caller can get the blade node positions
	HubMotion hm = CalculateHubMotion(nm, drivetrain_states.rotor);
	aerodyn.Set_Inputs_Hub(time_next, hm.position, hm.orientation, hm.velocity, hm.acceleration, hm.angularVel, hm.angularAcc, bladePitch);
}

void AeroDynTurbine::SetInputs_NacelleAcc(const Vector3d& nacelleAcc, const Vector3d& nacelleRotationAcc) 
{
	nacelleMotion.acceleration = nacelleAcc;
	nacelleMotion.angularAcc = nacelleRotationAcc;
	HubAcc ha = CalculateHubAcc(nacelleAcc, nacelleRotationAcc, nacelleMotion.orientation, drivetrain.GetRotorShaftAcc());
	aerodyn.Set_Inputs_HubAcceleration(ha.acceleration, ha.angularAcc);
}

void AeroDynTurbine::SetInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	// Don't actually update Aerodyn with these inflows yet, just save them
	// TODO explain why we don't actually set them in AeroDyn yet
	for (unsigned int i = 0; i < inflowVel.size(); ++i)
	{
		this->inflowVel[i] = inflowVel[i];
		this->inflowAcc[i] = inflowAcc[i];
	}

}

// Performs the predictor-corrector method, and uses the input-output solver for each corrector iteration
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::AdvanceStates()
{
	const int n_corrector_steps = 2;
	NacelleReactionLoads_Vec nrl;

	// Predictor Corrector loop
	for (int i = 0; i < n_corrector_steps; i++) {
		// Advance all states
		aerodyn.UpdateStates();
		drivetrain.UpdateStates();

		if (useAddedMass) {
			// Input-output solve: the resulting inputs need to be set inside each module
			nrl = CalcOutputs_And_SolveInputs();
		}
		else
		{
			// The nacelleMotion variable is seen as the correct output from the Driver,
			// so just calculate the ouputs of AD and DT, and transfer the outputs to inputs
			nrl = CalcOutputs_And_DeriveInputs();
		}

		// Correct (continue in loop)
	}
	// Copy the predicted states (what the predictor-corrector process has landed on) to the actual states
	aerodyn.CopyStates_Pred_to_Curr();
	drivetrain.CopyStates_Pred_to_Curr();

	aerodyn.PrintOutputLine();
	
	aerodyn.Advance_InputWindow();
	drivetrain.AdvanceInputWindow();

	// If this isn't a temporary update, then update the controller too
	if (!onTempUpdate) {
		mcont.UpdateController(time_next, drivetrain.GetGenShaftSpeed(), aerodyn.GetBladePitch());
	}

	// Overwrite the nacelle x moment to be the generator torque
	//
	//   Note that only when onRealStep == true is the generator torque reported at current time_act;
	//   if onRealStep == false, then UpdateController isn't called and the generator torque given by
	//   GetGeneratorTorqueCommand() is at the previous time_act.

	//   I do see this as a problem, but I think it may be unavoidable with the given Bladed-style DLL because
	//   it is written to be loosely coupled to, meaning all calls to it are permanent, and therefore we
	//   can't call it when onTempUpdate == true
	nrl.moment[0] = mcont.GetGeneratorTorqueCommand();

	time_curr = time_next;

	return nrl;
}

// Calculate the outputs of AeroDyn and the Drive train, and derive the inputs
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::CalcOutputs_And_DeriveInputs()
{
	auto ad_output = aerodyn.CalcOutput();
	auto dt_output = drivetrain.CalcOutput();

	HubMotion ad_input_hub = CalculateHubMotion(nacelleMotion, dt_output.rotor);
	double dt_input_rotor_torque = ad_output.moment.x();
	double dt_input_gen_torque = mcont.GetGeneratorTorqueCommand();

	aerodyn.Set_Inputs_Hub(time_next, ad_input_hub.position, ad_input_hub.orientation, ad_input_hub.velocity, ad_input_hub.acceleration,
		ad_input_hub.angularVel, ad_input_hub.angularAcc, mcont.GetBladePitchCommand());
	drivetrain.SetInputs(time_next, dt_input_rotor_torque, dt_input_gen_torque);

	// Derive the inputs for Dvr at time_next
	Vector3d nacelleForce = TransformHubToNacelle(
		ad_output.force,
		nacelleMotion.orientation,
		aerodyn.GetInput_HubOrient());
	Vector3d nacelleMoment = TransformHubToNacelle(
		ad_output.moment,
		nacelleMotion.orientation,
		aerodyn.GetInput_HubOrient());
	nacelleMoment.x() = mcont.GetGeneratorTorqueCommand();

	NacelleReactionLoads_Vec dvr_input;
	dvr_input.force = nacelleForce;
	dvr_input.moment = nacelleMoment;
	dvr_input.power = ad_output.power;
	dvr_input.tsr = ad_output.tsr;
	
	return dvr_input;
}

// Note: Uses pointer arithmetic for now just to save on code length
AeroDynTurbine::NacelleReactionLoads_Vec AeroDynTurbine::CalcOutputs_And_SolveInputs()
{
	// hard-code here for now
	int solver_iterations = 2;
	const double perturb = 0.01;

	/* Input vector layout:
	-----------------------
	[0]  Driver    : nacelle force x
	[1]  Driver    : nacelle force y
	[2]  Driver    : nacelle force z
	[3]  Driver    : nacelle moment x
	[4]  Driver    : nacelle moment y
	[5]  Driver    : nacelle moment z
	[6]  AeroDyn   : hub acc x
	[7]  AeroDyn   : hub acc y
	[8]  AeroDyn   : hub acc z
	[9]  AeroDyn   : hub rotation acc x
	[10] AeroDyn   : hub rotation acc y
	[11] AeroDyn   : hub rotation acc z
	[12] Drivetrain: rotor torque

	Output vector layout:
	---------------------
	[0]  Driver    : nacelle acc x
	[1]  Driver    : nacelle acc y
	[2]  Driver    : nacelle acc z
	[3]  Driver    : nacelle rotation acc x
	[4]  Driver    : nacelle rotation acc y
	[5]  Driver    : nacelle rotation acc z
	[6]  AeroDyn   : hub force x
	[7]  AeroDyn   : hub force y
	[8]  AeroDyn   : hub force z
	[9]  AeroDyn   : hub moment x
	[10] AeroDyn   : hub moment y
	[11] AeroDyn   : hub moment z
	[12] Drivetrain: rotor acc
	*/

	// Setup serialized vector of inputs
	Vector<double, 13> u;
	// Setup serialized vector of outputs
	Vector<double, 13> y;

	// Pack the initial inputs that were used to generate the current states at time_next into the serialized input vector
	u.segment(U_DVR_NAC_FORCE, 3) = nacelleMotion.acceleration;
	u.segment(U_DVR_NAC_MOMENT, 3) = nacelleMotion.angularAcc;
	u.segment(U_AD_HUB_ACC, 3) = aerodyn.GetInput_HubAcc();
	u.segment(U_AD_HUB_ROTACC, 3) = aerodyn.GetInput_HubRotAcc();
	u(U_DT_ROTOR_TORQUE) = drivetrain.GetInput_RotorTorque();

	int k = 0;
	while (true) {
		//------------------------------------------------------
		// Calculate the outputs
		//------------------------------------------------------
		// calculate the outputs from Dvr and pack into output vector
		CalcOutput_callback(u.data() + U_DVR_NAC_FORCE, u.data() + U_DVR_NAC_MOMENT,
			y.data() + Y_DVR_NAC_ACC, y.data() + Y_DVR_NAC_ROTACC);

		// calculate the outputs from AeroDyn
		Vector3d hubForce, hubMoment;
		auto ad_output = aerodyn.CalcOutput();
		//	and pack into the output vector
		y.segment(Y_AD_HUB_FORCE, 3) = ad_output.force;
		y.segment(Y_AD_HUB_MOMENT, 3) = ad_output.moment;

		// calculate the outputs from Drivetrain
		auto dt_output = drivetrain.CalcOutput();
		//	and pack into output vector
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
		for (int i = 0; i < 6; i++) {
			Vector<double, 13> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			CalcOutput_callback(u_perturb.data() + U_DVR_NAC_FORCE, u_perturb.data() + U_DVR_NAC_MOMENT,
				y_perturb.data() + Y_DVR_NAC_ACC, y_perturb.data() + Y_DVR_NAC_ROTACC);

			SerializedVector u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			SerializedVector residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}

		// AD second
		for (int i = 6; i < 12; i++) {
			Vector<double, 13> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			aerodyn.Set_Inputs_HubAcceleration(
				Vector3d(u_perturb.data() + U_AD_HUB_ACC),
				Vector3d(u_perturb.data() + U_AD_HUB_ROTACC));

			auto ad_output = aerodyn.CalcOutput();

			y_perturb.segment(Y_AD_HUB_FORCE, 3) = ad_output.force;
			y_perturb.segment(Y_AD_HUB_MOMENT, 3) = ad_output.moment;

			Vector<double, 13> u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			Vector<double, 13> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}

		// Drivetrain third
		{
			Vector<double, 13> u_perturb, y_perturb;

			// perturb the last input
			u_perturb = u;
			u_perturb(U_DT_ROTOR_TORQUE) += perturb;

			// Calculate the corresponding output
			drivetrain.SetInputs(time_next, u_perturb(U_DT_ROTOR_TORQUE), mcont.GetGeneratorTorqueCommand());
			auto dt_output = drivetrain.CalcOutput();
			
			y_perturb = y;
			y_perturb(Y_DT_ROTOR_ACC) = dt_output.rotor.acc;
			Vector<double,13> u_perturb_residual = CalcResidual(y_perturb, u_perturb);
			Vector<double, 13> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			jacobian.col(Y_DT_ROTOR_ACC) = residual_func_partial_u;
		}

		//------------------------------------------------------
		// Do one step of Newton-Raphson (actually Secant method)
		//------------------------------------------------------
		FullPivLU<Matrix<double, 13, 13>> lu = jacobian.fullPivLu();

		Vector<double, 13> u_delta = -lu.inverse() * u_residual;

		u = u + u_delta;
		
		//------------------------------------------------------
		// Set the inputs of AD and DT with the new approximation
		//------------------------------------------------------
		aerodyn.Set_Inputs_HubAcceleration(u.segment(U_AD_HUB_ACC, 3), u.segment(U_AD_HUB_ROTACC, 3));
		drivetrain.SetInputs(time_next, u(U_DT_ROTOR_TORQUE), mcont.GetGeneratorTorqueCommand());
	}

	// TODO, still not sure what is the right thing to return from this function
	NacelleReactionLoads_Vec result;
	result.force = u.segment(U_DVR_NAC_FORCE,3);
	result.moment = u.segment(U_DVR_NAC_MOMENT,3);

	return result;
}

/*
Transfers the outputs from one module to the inputs of the other, then these
resulting inputs are subtracted from the inputs that generated the outputs
*/
AeroDynTurbine::SerializedVector AeroDynTurbine::CalcResidual(const SerializedVector& y, const SerializedVector& u) const
{
	//----------------------------------------------------------------
	// Derive the inputs of one module from the outputs of the other
	//................................................................
	/* first start with deriving AD's inputs from ProteusDS's and the drivetrain's outputs */
	Vector3d nacelleAcc(y.data() + Y_DVR_NAC_ACC);
	Vector3d nacelleRotationAcc(y.data() + Y_DVR_NAC_ROTACC);
	Vector3d hubRotationAcc = nacelleRotationAcc + (y(Y_DT_ROTOR_ACC) * aerodyn.GetInput_HubOrient().col(0));
	Vector3d hubAcc = nacelleAcc;

	/* second derive PDS's inputs from AD's outputs */
	Vector3d hubForce(y.data() + Y_AD_HUB_FORCE);
	Vector3d hubMoment(y.data() + Y_AD_HUB_MOMENT);

	Vector3d nacelleForce = TransformHubToNacelle(
		hubForce,
		nacelleMotion.orientation,
		aerodyn.GetInput_HubOrient());
	Vector3d nacelleMoment = TransformHubToNacelle(
		hubMoment,
		nacelleMotion.orientation,
		aerodyn.GetInput_HubOrient());
	nacelleMoment.x() = mcont.GetGeneratorTorqueCommand();
	/* third derive the drivetrain's input from Aerodyn's output */
	//	 Note: the first element counting from Y_AD_HUB_MOMENT is the x-component
	double rotorTorque = y(Y_AD_HUB_MOMENT);

	//------------------------------------------------------
	// Pack these results in a vector (TODO use .segment(i,n) for these instead)
	//......................................................
	SerializedVector u_derived;
	memcpy(u_derived.data() + U_DVR_NAC_FORCE, nacelleForce.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + U_DVR_NAC_MOMENT, nacelleMoment.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + U_AD_HUB_ACC, hubAcc.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + U_AD_HUB_ROTACC, hubRotationAcc.data(), 3 * sizeof(double));

	/* Subtract the derived inputs from the original ones */
	return u - u_derived;
}

Matrix3d AeroDynTurbine::InterpExtrapOrientation(double target_time, const Matrix3d& orient_1, double time_1,
	const Matrix3d& orient_2, double time_2) const
{
	// TODO just return the ealier of the two for now
	return orient_1;
}

Vector3d AeroDynTurbine::InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2) const
{
	// TODO just return the earlier of the two for now
	return vect_1;
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
	nacelleOrient = AngleAxisd(nacelleEulerAngles.x(), Vector3d::UnitX())
		* AngleAxisd(nacelleEulerAngles.y(), Vector3d::UnitY())
		* AngleAxisd(nacelleEulerAngles.z(), Vector3d::UnitZ());

	return nacelleOrient;
}