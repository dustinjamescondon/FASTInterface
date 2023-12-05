#include "FASTInterface.h"
#include "FASTInterfaceExceptions.h"
#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

struct SimulationParams {
	double simTime;
	double timestep;
	double hubRadius;
	int numBlades;
	double precone;
	double pitch;
	double inflowSpeed;
	double rotorSpeed;
};

// Assumes the inflow acceleration is zero
void GenerateInflowVelocities(const std::vector<double>& nodePositions, int totalNodes,
	double inflowSpeed, std::vector<double>& inflowVel, std::vector<double>& inflowAcc);
SimulationParams ParseCmdLineArgs(int argc, char* argv[]);

//----------------------------
// main routine
int main(int argc, char* argv[])
{

	//-------------------------
	// Parse the command line args for the correct simulation parameters
	SimulationParams params = ParseCmdLineArgs(argc, argv);

	//-------------------------
	// Simulation parameters
	static const int NSteps = (int)(params.simTime / params.timestep);

	//-------------------------
	// Local variables
	double time = 0;

	std::vector<double> inflowVel;
	std::vector<double> inflowAcc;
	std::vector<double> bladeNodePositions;

	int totalNodes = 0;
	//--------------------------
	// Outputs from turbine
	FASTInterface::NacelleReactionLoads rf;

	FASTInterface turb;

	// Initialize the nacelle state - it will be constant for this simulation test
	double nacelleAngularVel[3];
	double nacelleAngularAcc[3];
	double nacelleEulerAngles[3];
	double nacellePosition[3];
	double nacelleVel[3];
	double nacelleAcc[3];

	nacelleAngularVel[0] = 0.0;
	nacelleAngularVel[1] = 0.0;
	nacelleAngularVel[2] = 0.0;

	nacelleAngularAcc[0] = 0.0;
	nacelleAngularAcc[1] = 0.0;
	nacelleAngularAcc[2] = 0.0;

	nacelleEulerAngles[0] = 0.0;
	nacelleEulerAngles[1] = 0.0;
	nacelleEulerAngles[2] = 0.0;

	nacellePosition[0] = 0.0;
	nacellePosition[1] = nacellePosition[2] = 0.0;

	nacelleVel[0] = 0.0;
	nacelleVel[1] = 0.0;
	nacelleVel[2] = 0.0;

	nacelleAcc[0] = 0.0;
	nacelleAcc[1] = 0.0;
	nacelleAcc[2] = 0.0;

	try {
		// Use this to intialize the turbine with constant rotor speed and blade pitch
		turb.InitWithConstantRotorSpeedAndPitch(params.rotorSpeed, params.pitch);

		turb.InitAeroDyn("../resources/d5MW_OC4Semi_WSt_WavesWN/NRELOffshrBsline5MW_OC3Hywind_AeroDyn15.dat",
			"output",
			false,
			0.0,
			params.timestep,
			params.numBlades,
			params.hubRadius,
			params.precone,
			nacellePosition,
			nacelleEulerAngles,
			nacelleVel,
			nacelleAcc,
			nacelleAngularVel,
			nacelleAngularAcc);
	}
	catch (FileNotFoundException& e) {
		std::cout << e.what();
		return 0;
	}
	catch (FileContentsException& e) {
		std::cout << e.what();
		return 0;
	}
	catch (ADErrorException& e) {
		std::cout << e.what();
		return 0;
	}
	catch (std::runtime_error& e) {
		std::cout << e.what();
		return 0;
	}

	double turbineDiam = turb.GetTurbineDiameter();

	totalNodes = turb.GetNumNodes();

	// now we know the total number of nodes, so allocate accordingly
	inflowVel.resize(totalNodes * 3);
	inflowAcc.resize(totalNodes * 3);
	bladeNodePositions.resize(totalNodes * 3);

	// get hub positions from AD and then use then to find new inflows
	//		But for this test, just have a constant inflow
	//		create a steady flow in +x direction
	GenerateInflowVelocities(bladeNodePositions, totalNodes, params.inflowSpeed, inflowVel, inflowAcc);

	turb.InitInflows(inflowVel, inflowAcc);
	//-------------------------------
	// Simulation loop
	for (int i = 0; i < NSteps; i++)
	{
		//---------------------------------------------------------------------
		// Using the FASTTurbine

		// Begin a simulation update to time_act by passing nacelle state at time_act
		turb.SetNacelleStates(time,
			nacellePosition,
			nacelleEulerAngles,
			nacelleVel,
			nacelleAcc,
			nacelleAngularVel,
			nacelleAngularAcc,
			false);

		// Set the inflows at those positions
		turb.SetInflows(inflowVel, inflowAcc);

		// And update the states to global_time_next, returning the nacelle reaction forces
		rf = turb.AdvanceStates();

		time += params.timestep;
	}

	return 0;
}

void PrintHelp()
{
	std::cout
		<< "1) simulation time \n"
		<< "2) time step \n"
		<< "3) hub radius \n"
		<< "4) number of blades \n"
		<< "5) precone angle (degrees) \n"
		<< "6) blade pitch (degrees) \n"
		<< "7) inflow speed \n"
		<< "8) rotor speed \n";
}


SimulationParams ParseCmdLineArgs(int argc, char* argv[])
{
	if (argc != 8 + 1) {
		std::cout << "Only provided " << argc - 1 << " arguments! Need 8" << std::endl;
		PrintHelp();
		exit(1);
	}
	
	SimulationParams p;
	p.simTime = strtod(argv[1], NULL);
	p.timestep = strtod(argv[2], NULL);
	p.hubRadius = strtod(argv[3], NULL);
	p.numBlades = atoi(argv[4]);
	p.precone = strtod(argv[5], NULL);
	p.pitch = strtod(argv[6], NULL);
	p.inflowSpeed = strtod(argv[7], NULL);
	p.rotorSpeed = strtod(argv[8], NULL);

	{ // Print out the parameters used
		std::cout 
			<< "Using the following parameters: \n"
			<< "simulation time  : " << p.simTime << std::endl
			<< "time step        : " << p.timestep << std::endl
			<< "hub radius       : " << p.hubRadius << std::endl
			<< "number of blades : " << p.numBlades << std::endl
			<< "precone          : " << p.precone << std::endl
			<< "pitch            : " << p.pitch << std::endl
			<< "inflow speed     : " << p.inflowSpeed << std::endl
			<< "rotor speed      : " << p.rotorSpeed << std::endl;
	}

	{ // convert degrees to radians and RPM to rads/sec
		p.pitch = 180.0 * p.pitch / M_PI;
		p.precone = 180.0 * p.precone / M_PI;

		p.rotorSpeed = p.rotorSpeed * M_PI / 30.0;
	}
	return p;
}

// Assumes the inflow acceleration is zero
void GenerateInflowVelocities(const std::vector<double>& nodePositions, int totalNodes,
	double inflowSpeed, std::vector<double>& inflowVel, std::vector<double>& inflowAcc)
{

	for (int i = 0; i < totalNodes; i++)
	{
		inflowVel[i * 3 + 0] = inflowSpeed;
		inflowVel[i * 3 + 1] = 0.0;
		inflowVel[i * 3 + 2] = 0.0;

		// For this test, just assume the inflows aren't accelerating
		inflowAcc[i * 3 + 0] = 0.0;
		inflowAcc[i * 3 + 1] = 0.0;
		inflowAcc[i * 3 + 2] = 0.0;
	}
}
