#include "MassSpringDamper.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <functional>

MassSpringDamper::MassSpringDamper(bool p_enable_added_mass, double p_timestep, double p_mass, double p_stiffness_coeff, double p_damping_coeff, double p_initial_disp)
{
	enable_added_mass = p_enable_added_mass;
	timestep = p_timestep;
	time_curr = 0.0;
	mass = p_mass;
	displacement = p_initial_disp;
	damping_coeff = p_damping_coeff;
	stiffness_coeff = p_stiffness_coeff;

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
	const double initial_rotor_speed = 3 * M_PI / 30;
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
	const char* ad_output_file = "mass_spring_damper_test";
	const double added_mass_coeff = 0.5; // TODO ^
	const double hub_radius = 1.5;
	const double precone = 0.0;
	const double fluid_density = 1.225;
	const double kinematic_fluid_visc = 1.4639e-05;
	const double inflow_speed = 1.0;
	const double nacelle_pos[3] = { displacement, 0.0, 0.0 };
	const double nacelle_euler_angles[3] = { 0.0, 0.0, 0.0 };
	const double nacelle_vel[3] = { 0.0,0.0,0.0 };
	const double nacelle_acc[3] = { acceleration,0.0,0.0 };
	const double nacelle_rotvel[3] = { 0.0,0.0,0.0 };
	const double nacelle_rotacc[3] = { 0.0,0.0,0.0 };


	turb.InitAeroDyn(ad_input_file, ad_output_file, enable_added_mass, added_mass_coeff, timestep, num_blades,
		hub_radius, precone, fluid_density, kinematic_fluid_visc, nacelle_pos, nacelle_euler_angles, nacelle_vel,
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
}

/* This version doesn't need the temporary update functionality because it's just using the Euler integration method */
void MassSpringDamper::Simulate(double time_next)
{
	//-----------------------------------------------------------------------
	// First, get forces at current simulation time by updating FASTInterface
	//.......................................................................

	// Figure out the nacelle input information and put into 3 component arrays
	double nacelle_pos[3] = { displacement, 0.0, 0.0 };
	double nacelle_euler_angles[3] = { 0.0, 0.0, 0.0 };
	double nacelle_vel[3] = { velocity, 0.0, 0.0 };
	double nacelle_rotvel[3] = { 0.0, 0.0, 0.0 };
	double nacelle_acc[3] = { acceleration, 0.0, 0.0 };
	double nacelle_rotacc[3] = { 0.0, 0.0, 0.0 };

	// Send them to FASTInterface
	turb.SetNacelleStates(time_curr, nacelle_pos, nacelle_euler_angles, nacelle_vel, nacelle_acc, nacelle_rotvel, nacelle_rotacc, false);
	turb.SetInflows(constant_inflow_vel, constant_inflow_acc);

	// Advance its states to time_curr. Now this could either return the loads or the acceleration of the nacelle.
	// Doing the former means we have to calculate the acceleration, so the latter might be more straight-forward
	auto output = turb.AdvanceStates();
	acceleration = (spring_force + output.force[0]) / mass; // (spring force + aerodynamic force) / mass

	// integrate
	double dt = time_next - time_curr;
	displacement += velocity * dt;
	velocity += acceleration * dt;
	/* can't really set the acceleration to what it should be at time_next, because the 
	   input solver can only do that. So just leave it as the acceleration at time_curr,
	   which will be the initial guess for the solver next time step. */
	
	// But we can know the spring force at time_next, so set it now
	spring_force = CalcSpringForce();
	
	time_curr = time_next;
}

// Calculates the spring force based on the current state of the spring mass damper
double MassSpringDamper::CalcSpringForce() const 
{
	return -(damping_coeff * velocity + stiffness_coeff * displacement);
}

double MassSpringDamper::CalcOutput(double aerodynamic_force) const
{
	return aerodynamic_force + spring_force / mass;
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

	// Set nacelle rotation acc (always zero)
	memcpy(nacelle_rotacc, zero, 3 * sizeof(double));

	// Set nacelle acc (only setting the x component)
	nacelle_acc[0] = (tmp_spring_force + nacelle_force[0]) / mass;
	nacelle_acc[1] = 0.0;
	nacelle_acc[2] = 0.0;
}