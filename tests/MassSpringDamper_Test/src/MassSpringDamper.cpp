#include "MassSpringDamper.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <functional>
#include <Eigen/Core>

MassSpringDamper::MassSpringDamper(bool p_enable_added_mass, double p_timestep, double p_mass, double p_stiffness_coeff, double p_damping_coeff, 
	double p_initial_disp, double p_rpm, double p_inflow_speed, std::string p_output_filename)
{
	enable_added_mass = p_enable_added_mass;
	timestep = p_timestep;
	time_curr = 0.0;
	mass = p_mass;
	displacement = p_initial_disp;
	damping_coeff = p_damping_coeff;
	stiffness_coeff = p_stiffness_coeff;
	rpm = p_rpm;
	inflow_speed = p_inflow_speed;
	output_filename = p_output_filename;

	spring_force = CalcSpringForce();
	acceleration = CalcOutput(0.0);

	using std::placeholders::_1;
	using std::placeholders::_2;
	using std::placeholders::_3;
	using std::placeholders::_4;

	std::function<void(const double*, const double*, double*, double*)> f = std::bind(&MassSpringDamper::CalcOutput_Callback, this, _1, _2, _3, _4);
	turb.SetCalcOutputCallback(f);

	InitFASTInterface();
}

void MassSpringDamper::InitFASTInterface()
{
	// Init DriveTrain first
	const double rotorMOI = 38829739.1;
	const double genMOI = 534.116;
	const double stiffness = 867637000.0;
	const double damping = 6215000.0;
	const double gearbox_ratio = 97.0;
	const double initial_rotor_speed = rpm * M_PI / 30;
	const int num_blades = 3;

	//turb.InitDriveTrain(rotorMOI, genMOI, stiffness, damping, gearbox_ratio, initial_rotor_speed);

	// Controller second
	const char* bladed_dll_fname = "Discon_OC3Hywind.dll";
	const double initial_pitch = 0.0;

	//turb.InitControllers_BladedDLL(num_blades, bladed_dll_fname, initial_pitch);
	turb.InitWithConstantRotorSpeedAndPitch(initial_rotor_speed, 0.0);

	// AeroDyn last
	// TODO the input file is non-standard right now
	const char* ad_input_file = "..\\modules\\openfast\\reg_tests\\r-test\\glue-codes\\openfast\\5MW_OC4Semi_WSt_WavesWN\\NRELOffshrBsline5MW_OC3Hywind_AeroDyn15_Water.dat";
	const char* ad_output_file = output_filename.c_str();
	const double added_mass_coeff = 1.0; // TODO ^
	const double hub_radius = 1.5;
	const double precone = 0.0;
	const double fluid_density = 1.225;
	const double kinematic_fluid_visc = 1.4639e-05;
	const double nacelle_pos[3] = { displacement, 0.0, 0.0 };
	const double nacelle_euler_angles[3] = { 0.0, 0.0, 0.0 };
	const double nacelle_vel[3] = { 0.0,0.0,0.0 };
	const double nacelle_acc[3] = { acceleration,0.0,0.0 };
	const double nacelle_rotvel[3] = { 0.0,0.0,0.0 };
	const double nacelle_rotacc[3] = { 0.0,0.0,0.0 };

	turb.InitAeroDyn(ad_input_file, ad_output_file, enable_added_mass, added_mass_coeff, timestep, num_blades,
		hub_radius, precone, nacelle_pos, nacelle_euler_angles, nacelle_vel,
		nacelle_acc, nacelle_rotvel, nacelle_rotacc);

	// Set what the constant inflows should be 
	constant_inflow_vel.resize(3 * turb.GetNumNodes());
	constant_inflow_acc.resize(3 * turb.GetNumNodes());
	for (int i = 0; i < turb.GetNumNodes(); i++) {
		constant_inflow_vel[3 * i + 0] = inflow_speed;
		constant_inflow_vel[3 * i + 1] = 0.0;
		constant_inflow_vel[3 * i + 2] = 0.0;

		constant_inflow_acc[3 * i + 0] = 0.0;
		constant_inflow_acc[3 * i + 1] = 0.0;
		constant_inflow_acc[3 * i + 2] = 0.0;
	}

	// Set the inflows in AeroDyn
	turb.InitInflows(constant_inflow_vel, constant_inflow_acc);

	double tb_output[3];
	turb.GetNacelleForce(tb_output);
	acceleration = (spring_force + tb_output[0]) / mass; // (spring force + aerodynamic force) / mass

}

/* This version doesn't need the temporary update functionality because it's just using the Euler integration method */
void MassSpringDamper::Simulate(double time_next)
{
	using namespace Eigen;

	double dt = time_next - time_curr;

	//-----------------------------------------------------------------------
	// First, get forces at current simulation time by updating FASTInterface
	//.......................................................................

	// Q: do we assume the acceleration is aleady set at time_curr?
	// A: yes

	Vector2d x_curr;
	x_curr(0) = displacement;
	x_curr(1) = velocity;

	Vector2d dx_curr;
	dx_curr(0) = velocity,
	dx_curr(1) = acceleration;

	//-----------------------------------------------------------------------
	// Perform first step of Huen's method
	//.......................................................................
	Vector2d x_tilde_next = x_curr + dt * dx_curr;

	displacement = x_tilde_next(0);
	velocity = x_tilde_next(1);
	spring_force = CalcSpringForce();

	// Calculate dx_tilde at time_next
	double nacelle_disp[3] = { x_tilde_next(0), 0.0, 0.0 };
	double nacelle_vel[3] = { x_tilde_next(1), 0.0, 0.0 };
	double nacelle_acc[3] = { dx_curr(1), 0.0, 0.0 }; // Just use prev acceleration as guess (this changes once the input solver is called)
	double nacelle_Euler_angles[3] = { 0.0, 0.0, 0.0 };
	double nacelle_angular_vel[3] = { 0.0, 0.0, 0.0 };
	double nacelle_angular_acc[3] = { 0.0, 0.0, 0.0 };
	
	// Take make a temporary update from time_curr to time_next
	turb.SetNacelleStates(time_next, nacelle_disp, nacelle_Euler_angles, nacelle_vel, nacelle_acc, nacelle_angular_vel, nacelle_angular_acc, true);
	turb.SetInflows(constant_inflow_vel, constant_inflow_acc);

	turb.AdvanceStates();

	Vector2d dx_tilde_next;

	// If added mass is enabled, then the input solver was run during call to AdvanceStates(). This
	// means the nacelle acceleration has already been solved for, so it doens't need to be calculated 
	// from the nacelle force.
	if(enable_added_mass) {
		double nac_acc[3];
		turb.GetNacelleAcc(nac_acc);
		dx_tilde_next(0) = x_tilde_next(1);
		dx_tilde_next(1) = nac_acc[0];
	}
	// Otherwise, the input solver wasn't used, so we still have to calculate the nacelle acceleration from the 
	// nacelle force.
	else {
		double nac_force[3];
		turb.GetNacelleForce(nac_force);
		dx_tilde_next(0) = x_tilde_next(1);
		dx_tilde_next(1) = (nac_force[0] + CalcSpringForce(x_tilde_next(0), x_tilde_next(1))) / mass;
	}

	Vector2d x_next = x_curr + 0.5 * dt * (dx_curr + dx_tilde_next);

	displacement = x_next(0);
	velocity = x_next(1);

	nacelle_disp[0] = displacement;
	nacelle_vel[0] = velocity;

	turb.SetNacelleStates(time_next, nacelle_disp, nacelle_Euler_angles, nacelle_vel, nacelle_acc, nacelle_angular_vel, nacelle_angular_acc, false);
	turb.SetInflows(constant_inflow_vel, constant_inflow_acc);
	auto nac_force = turb.AdvanceStates();

	Vector2d dx_next;

	// If added mass is enabled, then the input solver was run during call to AdvanceStates(). This
	// means the nacelle acceleration has already been solved for, so it doens't need to be calculated 
	// from the nacelle force.
	if (enable_added_mass) {
		double nac_acc[3];
		turb.GetNacelleAcc(nac_acc);
		acceleration = nac_acc[0];
	}
	// Otherwise, the input solver wasn't used, so we still have to calculate the nacelle acceleration from the 
	// nacelle force.
	else {
		double nac_force[3];
		turb.GetNacelleForce(nac_force);
		acceleration = (nac_force[0] + CalcSpringForce(displacement, velocity)) / mass;
	}

	time_curr = time_next;
}

// Calculates the spring force based on the current state of the spring mass damper
double MassSpringDamper::CalcSpringForce() const 
{
	return -(damping_coeff * velocity + stiffness_coeff * displacement);
}

// Calculates the spring force based on the current state of the spring mass damper
double MassSpringDamper::CalcSpringForce(double p_displacement, double p_velocity) const
{
	return -(damping_coeff * p_velocity + stiffness_coeff * p_displacement);
}

double MassSpringDamper::CalcOutput(double aerodynamic_force) const
{
	return (aerodynamic_force + spring_force) / mass;
}

// The callback function made specifically for the added mass coupling with FASTInterface
// Calculates and returns the acceleration of the object based on the current states and the passed
// inputs of nacelle force and moment.
// ------------------------------------------------------------------------------------------------
// Note: it doesn't set the acceleration inside the mass spring damper object. It doesn't do this because 
//	     the input solver uses this function to calculate its partial derivatives, which involves perturbing
//       each component of the inputs. So if this function actuallys sets the acceleration, it might be set 
//       with a perturbed input and left that way (not very elegant).
void MassSpringDamper::CalcOutput_Callback(const double* nacelle_force, const double* nacelle_moment,
	double* nacelle_acc, double* nacelle_rotacc)
{
	double tmp_spring_force = CalcSpringForce();
	double zero[3] = { 0.0, 0.0, 0.0 };

	// Set nacelle rotation acceleration (always zero)
	memcpy(nacelle_rotacc, zero, 3 * sizeof(double));

	// Set nacelle acceleration (only setting the x component)
	nacelle_acc[0] = (tmp_spring_force + nacelle_force[0]) / mass;
	nacelle_acc[1] = 0.0;
	nacelle_acc[2] = 0.0;
}