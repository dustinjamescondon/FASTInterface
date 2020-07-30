#include "FASTInterface.h"
#include "FASTInterfaceExceptions.h"

class MassSpringDamper
{
public:
	MassSpringDamper(bool enable_added_mass, double timestep, double mass, double stiffness_coeff, double damping_coeff, double initial_disp);

	void InitFASTInterface();

	// Bring the simulation's states up to global_time_next
	void Simulate(double time_next);

	double GetDisp() const { return displacement; }
	double GetVel() const { return velocity; }
	double GetRotorDisp() const { return turb.GetRotorAngularDisp(); }
	double GetRotorVel() const { return turb.GetRotorSpeed(); }
	double GetGenDisp() const { return turb.GetGeneratorAngularDisp(); }
	double GetGenVel() const { return turb.GetGeneratorSpeed(); }

private:
	
	double CalcOutput(double aerodynamic_force) const;
	// Calculates the accelerations at current time given the nacelle loads
	void CalcOutput_Callback(const double* in_nacelle_force, const double* in_nacelle_moment, double* out_nacelle_acc, double* out_nacelle_rotacc);
	double CalcSpringForce() const;

	FASTInterface  turb;

	bool enable_added_mass;
	double damping_coeff;
	double stiffness_coeff;
	double displacement;
	double velocity;
	double acceleration;
	double mass;
	double time_curr;
	double timestep;

	double spring_force;

	double inflow_speed;
	std::vector<double> constant_inflow_vel;
	std::vector<double> constant_inflow_acc;
};