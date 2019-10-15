#include "DriveTrain.h"

DriveTrain::DriveTrain(double rs, double gbr) {
	states.rotor.vel = rs;
	states.gen.vel = rs * gbr;
	gearbox_ratio = gbr;
}

DriveTrain::DriveTrain() {
	states.rotor.acc = 0.0;
	states.rotor.vel = 0.0;
	states.rotor.theta = 0.0;

	states.gen.acc = 0.0;
	states.gen.vel = 0.0;
	states.gen.theta = 0.0;

	gearbox_ratio = 1.0;
	gen_moi = 1.0;
	rotor_moi = 1.0;

	time = 0.0;
}

void DriveTrain::Init(double constantRotorSpeed)
{
	mode = CONSTANT;

	states.rotor.vel = constantRotorSpeed;
	states.rotor.acc = 0.0;
	states.rotor.theta = 0.0;

	// Don't have the generator shaft move when using constant rotor speed
	states.gen.acc = 0.0;
	states.gen.vel = 0.0;
	states.gen.theta = 0.0;
}

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

// Calculates the accelerations given the states s
DriveTrain::ModelStateDeriv DriveTrain::CalcDeriv(const ModelStates& s, double torque_rotor, double torque_gen) const
{
	DriveTrain::ModelStateDeriv result;

	if (mode == CONSTANT) {
		result.genAcc = result.rotorAcc = 0.0;
	}
	else {
		double theta_gen = s.gen.theta;
		double vel_gen = s.gen.vel;
		double theta_rotor = s.rotor.theta;
		double vel_rotor = s.rotor.vel;

		// ODEs from Ben's report
		result.rotorAcc = torque_rotor + stiffness_coeff * ((theta_gen / gearbox_ratio) - theta_rotor) +
			damping_coeff * ((vel_gen / gearbox_ratio) - vel_rotor);

		result.rotorAcc /= rotor_moi; // all over rotor_moi

		result.genAcc = (-1.0 * gearbox_ratio * torque_gen) + stiffness_coeff * (theta_rotor - (theta_gen / gearbox_ratio)) +
			damping_coeff * (vel_rotor - (vel_gen / gearbox_ratio));

		result.genAcc /= gen_moi;  // all over gen_moi
	}

	return result;
}

void DriveTrain::SetStates(const DriveTrain::ModelStates& s)
{
	states = s;
}

DriveTrain::ModelStates DriveTrain::GetStates() const
{
	return states;
}

// Base states are the states for which the accelerations are calculated
DriveTrain::ModelStates DriveTrain::K(int i, double dt, double dState_coeff, const ModelStates& base_states, double rotor_torque, double gen_torque)
{
	ModelStateDeriv deriv = CalcDeriv(base_states, rotor_torque, gen_torque);
	ModelStates result;

	/* calculate rotor shaft states */
	dStates_k[i].rotor.dVel = deriv.rotorAcc * dt;
	dStates_k[i].rotor.dTheta = states.rotor.vel * dt;

	//result.rotorStates.acc = deriv.rotorAcc; not sure if this is right, but we don't need it anyway
	result.rotor.vel = states.rotor.vel + dState_coeff * dStates_k[i].rotor.dVel;
	result.rotor.theta = states.rotor.theta + dState_coeff * dStates_k[i].rotor.dTheta;

	/* calculate generator shaft states */
	dStates_k[i].gen.dVel = deriv.genAcc * dt;
	dStates_k[i].gen.dTheta = states.gen.vel * dt;

	//result.genStates.acc = deriv.genAcc; not sure if this is right, but we don't need it anyway
	result.gen.vel = states.gen.vel + dState_coeff * dStates_k[i].gen.dVel;
	result.gen.theta = states.gen.theta + dState_coeff * dStates_k[i].gen.dTheta;

	result_k[i] = result;
	return result;
}

// input torques at current time; returns temporary states at t + dt/2
DriveTrain::ModelStates DriveTrain::K1(double dt, double rotor_torque, double gen_torque)
{
	return K(0, dt, 0.5, states, rotor_torque, gen_torque);
}

// input torques at t + dt/2; returns temporary states at t + dt/2
DriveTrain::ModelStates DriveTrain::K2(double dt, double rotor_torque, double gen_torque)
{
	return K(1, dt, 0.5, result_k[0], rotor_torque, gen_torque);
}

// input torques at t + dt/2; returns temporary states at t + dt
DriveTrain::ModelStates DriveTrain::K3(double dt, double rotor_torque, double gen_torque)
{
	return K(2, dt, 1.0, result_k[1], rotor_torque, gen_torque);
}

// input torques at t + dt; returns temporary states at t + dt (not really needed)
void DriveTrain::K4(double dt, double rotor_torque, double gen_torque)
{
	K(3, dt, 1.0, result_k[2], rotor_torque, gen_torque);
}

// Call this only after all K functions have been called in order
DriveTrain::ModelDStates DriveTrain::CalcWeightedAverage(const ModelDStates s[4]) const
{
	ModelDStates result;

	double oneOver6 = 1.0 / 6.0;

	// calculate the weighted average of the rotor dstates
	//		...dvel
	result.rotor.dVel = oneOver6 * (s[0].rotor.dVel + (2.0 * s[1].rotor.dVel)
		+ (2.0 * s[2].rotor.dVel) + s[3].rotor.dVel);

	//		...dtheta
	result.rotor.dTheta = oneOver6 * (s[0].rotor.dTheta + (2.0 * s[1].rotor.dTheta)
		+ (2.0 * s[2].rotor.dTheta) + s[3].rotor.dTheta);

	// calculate the weighted average of the generator dstates
	//		...dvel
	result.gen.dVel = oneOver6 * (s[0].gen.dVel + (2.0 * s[1].gen.dVel)
		+ (2.0 * s[2].gen.dVel) + s[3].gen.dVel);

	//		...dtheta
	result.gen.dTheta = oneOver6 * (s[0].gen.dTheta + (2.0 * s[1].gen.dTheta)
		+ (2.0 * s[2].gen.dTheta) + s[3].gen.dTheta);

	return result;
}

// Uses RK4 results to update states with weighted average and returns them
DriveTrain::ModelStates DriveTrain::UpdateStates()
{
	ModelDStates avg = CalcWeightedAverage(dStates_k);

	states.rotor.vel += avg.rotor.dVel;
	states.rotor.theta += avg.rotor.dTheta;

	states.gen.vel += avg.gen.dVel;
	states.gen.theta += avg.gen.dTheta;

	return states;
}

// Version that solves the ODEs entirely in this single call using RK4
DriveTrain::ModelStates DriveTrain::UpdateStates(double dt, double rotor_torque, double gen_torque)
{

	// If the torque isn't going to change through the whole step, then this isn't any different than the broken
	// down version with K 1 through 4.
	ModelStateDeriv deriv = CalcDeriv(states, rotor_torque, gen_torque);

	K1(dt, rotor_torque, gen_torque);
	K2(dt, rotor_torque, gen_torque);
	K3(dt, rotor_torque, gen_torque);
	K4(dt, rotor_torque, gen_torque);
	UpdateStates();

	return states;
}

// assumes gearbox ratio has already been set and is non-zero
void DriveTrain::SetInitialGenSpeed(double s)
{
	states.gen.vel = s;
	states.rotor.vel = s / gearbox_ratio;
}

void DriveTrain::SetInitialRotorSpeed(double s)
{
	states.rotor.vel = s;
	states.gen.vel = s * gearbox_ratio;
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

DriveTrain::ModelStates DriveTrain::GetModelStates() const
{
	return states;
}

DriveTrain::States DriveTrain::GetRotorStates() const
{
	return states.rotor;
}

DriveTrain::States DriveTrain::GetGenStates() const
{
	return states.gen;
}

double DriveTrain::GetRotorShaftSpeed() const
{
	return states.rotor.vel;
}

double DriveTrain::GetGenShaftSpeed() const
{
	return states.gen.vel;
}

