#include "..\..\..\FASTInterface\src\FASTInterface.h"
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include "TimePlot.h"
#include <Windows.h>
#include "Driver.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "Graphics.h"

// used to define vectors and matrices in this small example
using namespace Eigen;

// Just a dummy function that always results in zero vectors
void CalcOutput_dummy(const double *, const double *, double *acc_out, double *rotAcc_out)
{
	double zero[3] = { 0.0, 0.0, 0.0 };
	memcpy(acc_out, zero, 3 * sizeof(double));
	memcpy(rotAcc_out, zero, 3 * sizeof(double));
}

//----------------------------
// main routine
int main(int argc, char* argv[])
{	
	sf::RenderWindow window(sf::VideoMode(800, 800), "Node position test");

	//-------------------------
	// Simulation parameters
	static const double EndTime = 100.0;
	int numBlades = 3;
	double hubRadius = 1.5;
	double precone = 0.0;
	static const double dt = 0.0125;
	static const int NSteps = (int)(EndTime / dt);

	static const double InflowSpeed = 10.0;    // in metres/sec
	static const double InitialRotorSpeed = 10 * M_PI / 30; // Converts from RPM to rads/sec
	static const double InitialPitch = 0.0;
	static const double GearboxRatio = 97.0;				// From NREL's OC3 report
	static const double DriveTrainDamping = 6215000.0;      // "
	static const double DriveTrainStiffness = 867637000.0;  // "
	static const double GenMOI = 534.116;					// "
	static const double LPFCornerFreq = 1.570796;			// "
	static const double RotorMOI = 38829739.1;				// " (Used parallel axis theorem to calculate this using NREL's specification of MOI relative to root)
	static const bool useAddedMass = true;
	static const double coeffAddedMass = 0.0;

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

	//--------------------------
	// Initialization

	// Simple real-time_act visualizations of simulation values
	TimePlot pitchPlot(0, 0, 200, 100, 0.0, 2.0);
	TimePlot genPlot(0, 102, 200, 100, 0.0, 47402.91);
	TimePlot genSpeedPlot(0, 204, 200, 100, 0.0, 300.0);
	TimePlot rotorSpeedPlot(0, 306, 200, 100, 0.0, 2);

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
	nacellePosition[1] = nacellePosition[2] = 75.0;

	nacelleVel[0] = 0.0;
	nacelleVel[1] = 0.0; 
	nacelleVel[2] = 0.0;

	nacelleAcc[0] = 0.0;
	nacelleAcc[1] = 0.0;
	nacelleAcc[2] = 0.0;

	try {
		// Option 1) Use these initialization methods to use the Bladed-style DLL
		//turb.InitDriveTrain(RotorMOI, GenMOI, DriveTrainStiffness, DriveTrainDamping, GearboxRatio, InitialRotorSpeed);
		//turb.InitControllers_BladedDLL("C:/Users/dusti/Documents/Work/PRIMED/ControllerDLLs/Discon_OC3Hywind.dll", InitialPitch);

		// Option 2) Use this to intialize the turbine with constant rotor speed and blade pitch
		turb.InitWithConstantRotorSpeedAndPitch(InitialRotorSpeed, InitialPitch);

		turb.InitAeroDyn("../modules/openfast/reg_tests/r-test/glue-codes/openfast/5MW_OC4Semi_WSt_WavesWN/NRELOffshrBsline5MW_OC3Hywind_AeroDyn15.dat",
			"output",
			useAddedMass,
			coeffAddedMass,
			dt,
			numBlades,
			hubRadius,
			precone,
			nacellePosition,
			nacelleEulerAngles,
			nacelleVel,
			nacelleAcc,
			nacelleAngularVel,
			nacelleAngularAcc);
	
		using std::placeholders::_1;
		using std::placeholders::_2;
		using std::placeholders::_3;
		using std::placeholders::_4;

		std::function<void(const double*, const double*, double*, double*)> f = std::bind(&CalcOutput_dummy, _1, _2, _3, _4);
		turb.SetCalcOutputCallback(f);
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
	GenerateInflowVelocities(bladeNodePositions, totalNodes, InflowSpeed, inflowVel, inflowAcc);

	turb.InitInflows(inflowVel, inflowAcc);
	//-------------------------------
	// Simulation loop
	for (int i = 0; i < NSteps; i++)
	{
		// visualization window event handling
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed) {
				window.close();
				return 0;
			}
		}

		bool isTempUpdate = false;
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
			isTempUpdate);

		// Now the rotor orientation has been set, so Aerodyn can report where the node positions are
		turb.GetBladeNodePositions(bladeNodePositions);

		// Set the inflows at those positions
		turb.SetInflows(inflowVel, inflowAcc);

		// And update the states_pred to time_act, returning the nacelle reaction forces
		rf = turb.AdvanceStates();

		//---------------------------------------------------------------------
		// Value visualization code
		RenderBladeNodes(window, bladeNodePositions, Vector3d(nacellePosition), 5.0, totalNodes);

		pitchPlot.plot(0.0, turb.GetBladePitch());
		pitchPlot.draw(window);

		genPlot.plot(0.0, turb.GetGeneratorTorque());
		genPlot.draw(window);

		genSpeedPlot.plot(0.0, turb.GetGeneratorSpeed());
		genSpeedPlot.draw(window);

		rotorSpeedPlot.plot(0.0, turb.GetRotorSpeed());
		rotorSpeedPlot.draw(window);

		window.display();

		time += dt;
	}

	return 0;
}