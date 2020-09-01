#include "DriveTrain.h"

DriveTrain::DriveTrain(double constantRotorSpeed)
{
	Init(constantRotorSpeed);
}

DriveTrain::DriveTrain(double rs, double gbr) {
	states_pred.rotor.vel = rs;
	states_pred.gen.vel = rs * gbr;
	gearbox_ratio = gbr;
}

DriveTrain::DriveTrain() {
	states_pred.rotor.acc = 0.0;
	states_pred.rotor.vel = 0.0;
	states_pred.rotor.theta = 0.0;

	states_pred.gen.acc = 0.0;
	states_pred.gen.vel = 0.0;
	states_pred.gen.theta = 0.0;

	gearbox_ratio = 1.0;
	gen_moi = 1.0;
	rotor_moi = 1.0;

	time = 0.0;
	inputTime[NEXT] = 0.0;
	inputTime[CURR] = -1.0;
}

// Initialize the drive train with a constant rotor speed
void DriveTrain::Init(double constantRotorSpeed)
{
	mode = CONSTANT;

	states_pred.rotor.vel = constantRotorSpeed;
	states_pred.rotor.acc = 0.0;
	states_pred.rotor.theta = 0.0;

	// Don't have the generator shaft move when using constant rotor speed
	states_pred.gen.acc = 0.0;
	states_pred.gen.vel = 0.0;
	states_pred.gen.theta = 0.0;

	CopyStates_Pred_to_Curr();

	// extrapolate back for the inputs
	input[CURR].genTorque = 0;
	input[CURR].rotorTorque = 0;
	input[NEXT].genTorque = 0;
	input[NEXT].rotorTorque = 0;

	// Setting them equal at zero will mean dt in Advance States will be, so the states will remain unchanged for 
	// the first call to Advance States
	inputTime[CURR] = 0.0;
	inputTime[NEXT] = 0.0;
}

// Initialize the drive train using the connected two-mass model
void DriveTrain::Init(double initialRotorSpeed, double gbr, double damping, double stiffness,
	double rotorMOI, double genMOI)
{
	mode = DYNAMIC;

	gearbox_ratio = gbr;
	SetInitialRotorSpeed(initialRotorSpeed);
	damping_coeff = damping;
	stiffness_coeff = stiffness;
	rotor_moi = rotorMOI;
	gen_moi = genMOI;
}

void DriveTrain::SaveCurrentStates()
{
	saved_states = states_curr;
	saved_input[0] = input[0];
	saved_input[1] = input[1];
	saved_time = time;
	saved_inputTime[0] = inputTime[0];
	saved_inputTime[1] = inputTime[1];
}

void DriveTrain::RestoreSavedStates()
{
	states_curr = saved_states;
	input[0] = saved_input[0];
	input[1] = saved_input[1];
	time = saved_time;
	inputTime[0] = saved_inputTime[0];
	inputTime[1] = saved_inputTime[1];
}

// Calculates the accelerations given the states_pred 
DriveTrain::ModelStates DriveTrain::CalcOutput()
{
	states_pred = CalcOutput(states_pred, input[NEXT]);
	return states_pred;
}

// Calculates the accelerations given the states_pred s
DriveTrain::ModelStates DriveTrain::CalcOutput(const ModelStates& s, const Input& u) const
{
	ModelStates result = s;

	if (mode == CONSTANT) {
		result.gen.acc = result.rotor.acc = 0.0;
	}
	else {
		double theta_gen = result.gen.theta;
		double vel_gen = result.gen.vel;
		double theta_rotor = result.rotor.theta;
		double vel_rotor = result.rotor.vel;

		// ODEs from Ben's report
		result.rotor.acc = u.rotorTorque + stiffness_coeff * ((theta_gen / gearbox_ratio) - theta_rotor) +
			damping_coeff * ((vel_gen / gearbox_ratio) - vel_rotor);

		result.rotor.acc /= rotor_moi; // all over rotor_moi

		result.gen.acc = (-1.0 * gearbox_ratio * u.genTorque) + stiffness_coeff * (theta_rotor - (theta_gen / gearbox_ratio)) +
			damping_coeff * (vel_rotor - (vel_gen / gearbox_ratio));

		result.gen.acc /= gen_moi;  // all over gen_moi
	}

	return result;
}

void DriveTrain::UpdateStates()
{
	double dt = inputTime[NEXT] - inputTime[CURR];
	states_pred = states_curr;

	// TODO if Heun's method isn't accurate enough, then switch to RK4
	/*----------------- Heun's method -------------------*/

	//----------------------------------------------------
	// Perform Euler step for first part of Heun's method
	//....................................................

	// Use the torque inputs at the prev time for calculating accelerations
	ModelStates tmp = CalcOutput(states_pred, input[CURR]);

	// rotor states
	tmp.rotor.theta = states_pred.rotor.theta + dt * tmp.rotor.vel;
	tmp.rotor.vel = states_pred.rotor.vel + dt * tmp.rotor.acc;
	// generator states
	tmp.gen.theta = states_pred.gen.theta + dt * tmp.gen.vel;
	tmp.gen.vel = states_pred.gen.vel + dt * tmp.gen.acc;

	//----------------------------------------------------
	// Perform second part of Heun's method
	//.....................................................

	// Use the torque inputs at latest time for calculating the accelerations
	ModelStates tmp2 = CalcOutput(tmp, input[NEXT]);

	tmp2.rotor.theta = states_pred.rotor.theta + dt * tmp2.rotor.vel;
	tmp2.rotor.vel = states_pred.rotor.vel + dt * tmp2.rotor.acc;

	tmp2.gen.theta = states_pred.gen.theta + dt * tmp2.gen.vel;
	tmp2.gen.vel = states_pred.gen.vel + dt * tmp2.gen.acc;
	
	//-----------------------------------------------------
	// Combine to calculate new states
	//......................................................
	states_pred.rotor.theta = 0.5 * (tmp.rotor.theta + tmp2.rotor.theta);
	states_pred.rotor.vel = 0.5 * (tmp.rotor.vel + tmp2.rotor.vel);
	states_pred.gen.theta = 0.5 * (tmp.gen.theta + tmp2.gen.theta);
	states_pred.gen.vel = 0.5 * (tmp.gen.vel + tmp2.gen.vel);
}

void DriveTrain::CopyStates_Pred_to_Curr()
{
	states_curr = states_pred;
}

void DriveTrain::SetInputs(double time, double rotorTorque, double genTorque)
{
	inputTime[NEXT] = time;
	input[NEXT].rotorTorque = rotorTorque;
	input[NEXT].genTorque = genTorque;
}

// Saves the latest input into the previous one, so next time the set inputs function is called,
// the NEXT input can be overwritten
void DriveTrain::AdvanceInputWindow()
{
	input[CURR] = input[NEXT];
	inputTime[CURR] = inputTime[NEXT];
}

// assumes gearbox ratio has already been set and is non-zero
void DriveTrain::SetInitialGenSpeed(double s)
{
	states_pred.gen.vel = s;
	states_pred.rotor.vel = s / gearbox_ratio;
}

void DriveTrain::SetInitialRotorSpeed(double s)
{
	states_pred.rotor.vel = s;
	states_pred.gen.vel = s * gearbox_ratio;
}

void DriveTrain::SetDampingCoeff(double x)
{
	damping_coeff = x;
}

void DriveTrain::SetStiffnessCoeff(double x)
{
	stiffness_coeff = x;
}

void DriveTrain::SetGenMassMomentOfInertia(double x)
{
	gen_moi = x;
}

void DriveTrain::SetRotorMassMomentOfInertia(double x)
{
	rotor_moi = x;
}

void DriveTrain::SetGearboxRatio(double x)
{
	gearbox_ratio = x;
}

DriveTrain::ModelStates DriveTrain::GetStates() const
{
	return states_pred;
}

double DriveTrain::GetInput_RotorTorque() const
{
	return input[NEXT].rotorTorque;
}

double DriveTrain::GetInput_GenTorque() const
{
	return input[NEXT].genTorque;
}

DriveTrain::States DriveTrain::GetRotorStates() const
{
	return states_pred.rotor;
}

DriveTrain::States DriveTrain::GetGenStates() const
{
	return states_pred.gen;
}

double DriveTrain::GetRotorShaftSpeed() const
{
	return states_pred.rotor.vel;
}

double DriveTrain::GetRotorShaftTheta() const
{
	return states_pred.rotor.theta;
}

double DriveTrain::GetGenShaftSpeed() const
{
	return states_pred.gen.vel;
}

double DriveTrain::GetGenShaftTheta() const
{
	return states_pred.gen.theta;
}

double DriveTrain::GetGenShaftAcc() const
{
	return states_pred.gen.acc;
}

double DriveTrain::GetRotorShaftAcc() const
{
	return states_pred.rotor.acc;
}