/* This test has the turbine attached to the mass of a simple mass spring damper */

#include "MassSpringDamper.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

struct SimulationParameters
{
	bool enable_added_mass;
	float simulation_time; // How much time will be simulated
	float timestep;
	float mass;
	float initial_disp;
	float spring_coeff;
	float damping_coeff;
};

//---------------------------------------------------------
// Forward function declarations
//---------------------------------------------------------
SimulationParameters ParseCommandLineArgs(int argc, char** argv);
void PrintHelpMenu();
void PrintHeader(std::ofstream& p_ofs);
void PrintOutputLine(std::ofstream& p_ofs, double time, double disp, double vel, double rotor_disp, double rotor_vel,
	double gen_disp, double gen_vel);

int main(int argc, char* argv[])
{
	SimulationParameters params = ParseCommandLineArgs(argc, argv);

	double time = 0.0;
	
	MassSpringDamper msd(params.enable_added_mass, params.timestep, params.mass, params.spring_coeff, params.damping_coeff, params.initial_disp);

	// Open an output stream to the output file
	std::ofstream fout("msd_timeseries.out");
	PrintHeader(fout);

	while (time <= params.simulation_time) {
		msd.Simulate(time);
		PrintOutputLine(fout, time, msd.GetDisp(), msd.GetVel(), msd.GetRotorDisp(), msd.GetRotorVel(), msd.GetGenDisp(), msd.GetGenVel());

		time += params.timestep;
	}

	fout.close();

	return 0;
}

//---------------------------------------------------------
// Function definitions
//---------------------------------------------------------
SimulationParameters ParseCommandLineArgs(int argc, char** argv)
{
	const int n_params = 7;

	SimulationParameters r;

	if (argc != n_params + 1) {
		PrintHelpMenu();
		exit(1);
	}

	const char* parameter_names[7] = {
		"added mass enabled",
		"simulation time     ",
		"timestep            ",
		"mass                ",
		"initial displacement",
		"spring coefficient  ",
		"damping coefficient "
	};

	for (int i = 1; i < n_params + 1; i++) {
		std::cout << parameter_names[i - 1] << ": " << argv[i] << std::endl;
	}

	// Assumes the values are valid
	r.enable_added_mass = bool(atoi(argv[1]));
	r.simulation_time = strtof(argv[2], NULL);
	r.timestep = strtof(argv[3], NULL);
	r.mass = strtof(argv[4], NULL);
	r.initial_disp = strtof(argv[5], NULL);
	r.spring_coeff = strtof(argv[6], NULL);
	r.damping_coeff = strtof(argv[7], NULL);

	return r;
}

void PrintHeader(std::ofstream& p_ofs)
{
	p_ofs << "Time\tDisp\tVel\tRotorDisp\tRotorVel\tGenDisp\tGenVel\n";
}

void PrintOutputLine(std::ofstream& p_ofs, double time, double disp, double vel, double rot_disp, double rot_vel, double gen_disp, double gen_vel)
{
	p_ofs << time << '\t' << disp << '\t' << vel << '\t' << rot_disp << '\t' << rot_vel << '\t' << gen_disp << '\t' << gen_vel << '\n';
}

void PrintHelpMenu()
{
	using namespace std;

	cout << "Incorrect command line arguments!" << endl <<
		"Correct input has " << 7 << " parameters:" << endl <<
		"[Added mass enabled] [Simulation time] [timestep] [mass] [displacement] [spring coefficient] [damping coefficient]" <<
		endl;
}