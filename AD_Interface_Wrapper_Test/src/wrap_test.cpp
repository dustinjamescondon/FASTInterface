#include "..\..\AeroDyn_Interface_Wrapper\src\FASTTurbine_Interface.h"
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include "Driver.h"
#include "TimePlot.h"
#include <Windows.h>

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
	static const double EndTime = 40.0;
	static const double dt = 0.0015;
	static const int NSteps = (int)(EndTime / dt);

	static const double InflowSpeed = 10.0;    // in metres/sec
	static const double FluidDensity = 1.225;
	static const double KinematicFluidVisc = 1.4639e-05;
	static const double InitialRotorSpeed = 1.183333233;
	static const double GearboxRatio = 97.0;				// From NREL's OC3 report
	static const double DriveTrainDamping = 6215000.0;      // "
	static const double DriveTrainStiffness = 867637000.0;  // "
	static const double GenMOI = 534.116;					// "
	static const double LPFCornerFreq = 1.570796;			// "
	static const double RotorMOI = 38829739.1;				// " (Used parallel axis theorem to calculate this using NREL's specification of MOI relative to root)


	//-------------------------
	// Local variables
	double time = 0.0;

	// create an axis-angle angular velocity using the x basis in global coordinate system
	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;

	int totalNodes = 0;
	//--------------------------
	// Outputs from turbine
	Vector3d force, moment;
	double bladePitch = 0.0;
	double power;
	double tsr;
	double turbineDiameter;

	//--------------------------
	// Initialization

	// Simple real-time visualizations of simulation values
	TimePlot pitchPlot(0, 0, 200, 100, 0.0, 2.0);
	TimePlot genPlot(0, 102, 200, 100, 0.0, 47402.91);
	TimePlot genSpeedPlot(0, 204, 200, 100, 0.0, 300.0);
	TimePlot rotorSpeedPlot(0, 306, 200, 100, 0.0, 2);

	FASTTurbineModel turb;

	// Initialize the nacelle state - it will be constant for this simulation test
	FASTTurbineModel::NacelleMotion nstate;
	nstate.angularVel[0] = 0.0;
	nstate.angularVel[1] = 0.0;
	nstate.angularVel[2] = 0.0;

	nstate.eulerAngles[0] = nstate.eulerAngles[1] = 0.0;
	nstate.eulerAngles[2] = 0.0;

	nstate.position[0] = 0.0;
	nstate.position[1] = nstate.position[2] = 75.0;

	nstate.velocity[0] = nstate.velocity[1] = nstate.velocity[2] = 0.0;

	try {
		turb.InitDriveTrain(RotorMOI, GenMOI, DriveTrainStiffness, DriveTrainDamping, GearboxRatio, InitialRotorSpeed);
		turb.InitControllers("Discon.dll");
		//turb.InitWithConstantRotorSpeedAndPitch(InitialRotorSpeed, 0.0);
		turb.InitAeroDyn("C:/Users/dusti/Documents/Work/PRIMED/inputfiles/ad_interface_example2.inp", FluidDensity, KinematicFluidVisc,
			nstate);
	}
	catch (ADInputFileNotFoundException& e) {
		std::cout << "Input file couldn't be found" << std::endl;
		std::cout << e.what();
		return 0;
	}

	catch (ADInputFileContentsException& e) {
		std::cout << "Input file has invalid contents" << std::endl;
		std::cout << e.what();
		return 0;
	}

	catch (ADErrorException& e) {
		std::cout << "An error occured in AeroDyn" << std::endl;
		std::cout << e.what();
		return 0;
	}

	catch (std::runtime_error& e)
	{
		std::cout << "An error occured" << std::endl;
		std::cout << e.what();
		return 0;
	}

	totalNodes = turb.GetNumNodes();

	// now we know the total number of nodes, so allocate accordingly
	inflows.resize(totalNodes * 3, 0.0);
	bladeNodePositions.resize(totalNodes * 3);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	GenerateInflowVelocities(bladeNodePositions, totalNodes, InflowSpeed, inflows);

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
		// The reaction forces structure
		FASTTurbineModel::NacelleReactionForces rf;

		//---------------------------------------------------------------------
		turb.SetNacelleStates(time, nstate, true);
		turb.GetBladeNodePositions(bladeNodePositions);
		turb.SetInflowVelocities(inflows);
		rf = turb.UpdateStates();
		//---------------------------------------------------------------------

		RenderBladeNodes(window, bladeNodePositions, Vector3d(nstate.position), 5.0, totalNodes);
		
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