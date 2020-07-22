/* This test has the turbine attached to the mass of a simple mass spring damper */

#include "MassSpringDamper.h"
#include <fstream>
#include <iostream>

struct SimulationParameters
{
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
void PrintOutputLine(std::ofstream& p_ofs, double time, double disp, double vel);

int main(int argc, char* argv[])
{
	SimulationParameters params = ParseCommandLineArgs(argc, argv);

	double time = 0.0;
	double dt = 0.0125;
	double end_time = 10.0;

	double mass = 100000.0; // TODO
	double stiffness = 10000.0;
	double damping = 10000.0;
	double initial_disp = 1.0;
	
	MassSpringDamper msd(params.timestep, params.mass, params.spring_coeff, params.damping_coeff, params.initial_disp);

	// Open an output stream to the output file
	std::ofstream fout("msd_timeseries.out");
	PrintHeader(fout);

	while (time <= end_time) {
		msd.Simulate(time);
		PrintOutputLine(fout, time, msd.GetDisp(), msd.GetVel());

		time += dt;
	}

	fout.close();

	return 0;
}

//---------------------------------------------------------
// Function definitions
//---------------------------------------------------------
SimulationParameters ParseCommandLineArgs(int argc, char** argv)
{
	const int n_params = 6;

	SimulationParameters r;

	if (argc != n_params + 1) {
		PrintHelpMenu();
		exit(1);
	}

	const char* parameter_names[6] = {
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
	r.simulation_time = strtof(argv[1], NULL);
	r.timestep = strtof(argv[2], NULL);
	r.mass = strtof(argv[3], NULL);
	r.initial_disp = strtof(argv[4], NULL);
	r.spring_coeff = strtof(argv[5], NULL);
	r.damping_coeff = strtof(argv[6], NULL);

	return r;
}

void PrintHeader(std::ofstream& p_ofs)
{
	p_ofs << "Time\tDisp\tVel\n";
}

void PrintOutputLine(std::ofstream& p_ofs, double time, double disp, double vel)
{
	p_ofs << time << '\t' << disp << '\t' << vel << '\n';
}

void PrintHelpMenu()
{
	using namespace std;

	cout << "Incorrect command line arguments!" << endl <<
		"Correct input has " << 6 << " parameters:" << endl <<
		"[Simulation time] [timestep] [mass] [displacement] [spring coefficient] [damping coefficient]" <<
		endl;
}
