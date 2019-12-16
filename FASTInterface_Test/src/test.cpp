#include "..\..\FASTInterface\src\FASTInterface.h"
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


//----------------------------
// main routine
int main()
{	
	sf::RenderWindow window(sf::VideoMode(800, 800), "Node position test");

	//-------------------------
	// Simulation parameters
	static const double EndTime = 100.0;
	static const double dt = 0.01;
	static const int NSteps = (int)(EndTime / dt);

	static const double InflowSpeed = 20.0;    // in metres/sec
	static const double FluidDensity = 1.225;
	static const double KinematicFluidVisc = 1.4639e-05;
	static const double InitialRotorSpeed = 1.2;
	static const double InitialPitch = 0.0;
	static const double GearboxRatio = 97.0;				// From NREL's OC3 report
	static const double DriveTrainDamping = 6215000.0;      // "
	static const double DriveTrainStiffness = 867637000.0;  // "
	static const double GenMOI = 534.116;					// "
	static const double LPFCornerFreq = 1.570796;			// "
	static const double RotorMOI = 38829739.1;				// " (Used parallel axis theorem to calculate this using NREL's specification of MOI relative to root)

	//-------------------------
	// Local variables
	double time = 0.0;

	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;

	int totalNodes = 0;
	//--------------------------
	// Outputs from turbine
	FASTInterface::NacelleReactionLoads rf;

	//--------------------------
	// Initialization

	// Simple real-time visualizations of simulation values
	TimePlot pitchPlot(0, 0, 200, 100, 0.0, 2.0);
	TimePlot genPlot(0, 102, 200, 100, 0.0, 47402.91);
	TimePlot genSpeedPlot(0, 204, 200, 100, 0.0, 300.0);
	TimePlot rotorSpeedPlot(0, 306, 200, 100, 0.0, 2);

	FASTInterface turb;

	// Initialize the nacelle state - it will be constant for this simulation test
	double nacelleAngularVel[3];
	double nacelleEulerAngles[3];
	double nacellePosition[3];
	double nacelleVel[3];
	nacelleAngularVel[0] = 0.0;
	nacelleAngularVel[1] = 0.0;
	nacelleAngularVel[2] = 0.0;

	nacelleEulerAngles[0] = 0.0;
	nacelleEulerAngles[1] = 0.0;
	nacelleEulerAngles[2] = 0.0;

	nacellePosition[0] = 0.0;
	nacellePosition[1] = nacellePosition[2] = 75.0;
	nacelleVel[0] = nacelleVel[1] = nacelleVel[2] = 0.0;

	try {
		// Use these initialization methods to use the Bladed-style DLL
		//turb.InitDriveTrain(RotorMOI, GenMOI, DriveTrainStiffness, DriveTrainDamping, GearboxRatio, InitialRotorSpeed);
		//turb.InitControllers_BladedDLL("C:/Users/dusti/Documents/Work/PRIMED/ControllerDLLs/Discon_OC3Hywind.dll", InitialPitch);

		// Use this to intialize the turbine with constant rotor speed and blade pitch
		turb.InitWithConstantRotorSpeedAndPitch(InitialRotorSpeed, InitialPitch);
		turb.InitAeroDyn("C:/Users/dusti/Documents/Work/PRIMED/inputfiles/ad_interface_example4.inp", FluidDensity, KinematicFluidVisc,
			nacellePosition,
			nacelleEulerAngles,
			nacelleVel,
			nacelleAngularVel);
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
	inflows.resize(totalNodes * 3);
	bladeNodePositions.resize(totalNodes * 3);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow in +x direction
	GenerateInflowVelocities(bladeNodePositions, totalNodes, InflowSpeed, inflows);

	float diam = turb.GetTurbineDiameter();

	turb.InitInflows(inflows);
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

		bool isRealStep = true;
		//---------------------------------------------------------------------
		// Using the FASTTurbine

		// Begin a simulation update to time by passing nacelle state at time
		turb.SetNacelleStates(time, 
			nacellePosition,
			nacelleEulerAngles,
			nacelleVel,
			nacelleAngularVel,
			isRealStep);

		// Now the rotor orientation has been set, so Aerodyn can report where the node positions are
		turb.GetBladeNodePositions(bladeNodePositions);

		// Set the inflows at those positions
		turb.SetInflowVelocities(inflows);

		// And update the states to time, returning the nacelle reaction forces
		rf = turb.Simulate();

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