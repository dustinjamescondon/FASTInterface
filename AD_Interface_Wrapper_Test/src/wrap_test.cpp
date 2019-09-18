#include "..\..\AeroDyn_Interface_Wrapper\src\AeroDyn_Interface_Wrapper.h"
#include "..\..\AeroDyn_Interface_Wrapper\eigen\Eigen\Dense"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>
#include "Driver.h"
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
	double simulationTime = 30.0; // the amount of time to be simulated (in seconds)
	double shaftSpeed = 1.183333233;     // in rads/sec
	const int NumTimes = 16;
	double time_array[] = { 0.0, 0.0001, 0.0006, 0.0031, 0.0156, 0.0781, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};            // the time-step anagolous to what ProteusDS would be taking
	//double time_array[] = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 };            // the time-step anagolous to what ProteusDS would be taking
	double bladePitch = 0.0;
	double inflowSpeed = 10.0;    // in metres/sec
	double fluidDensity = 1.225;
	double kinematicFluidVisc = 1.4639e-05;
	//-------------------------
	// Local variables
	double time = 0.0;

	DriverStates states;
	states.y.hubPos = Vector3d(0.0, 50.0, 50.0);
	states.y.hubOri = Vector3d(0.0, 0.0, 0.0);
	states.dydt.hubVel = Vector3d(0.0, 0.0, 0.0);

	// create an axis-angle angular velocity using the x basis in global coordinate system
	Matrix3d hubOriMatrix = EulerConstruct(states.y.hubOri);
	states.dydt.hubAngVel = hubOriMatrix.row(0) * shaftSpeed; 

	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;

	int totalNodes = 0;
	//--------------------------
	// Outputs from Aerodyn
	Vector3d force, moment;
	double power;
	double tsr;
	double turbineDiameter;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	//--------------------------
	// Initialization

	// Create instance of the wrapper class
	AeroDyn_Interface_Wrapper ad;

	try {
		ad.InitAerodyn("C:/Users/dusti/Documents/Work/PRIMED/inputfiles/ad_interface_example2.inp", fluidDensity, kinematicFluidVisc,
			states.y.hubPos.data(), states.y.hubOri.data(), states.dydt.hubVel.data(),
			states.dydt.hubAngVel.data(), bladePitch);
	}
	catch (ADInputFileNotFound& e) {
		std::cout << "Input file couldn't be found" << std::endl;
		std::cout << e.what();
		return 0;
	}
	catch (ADInputFileContents& e) {
		std::cout << "Input file has invalid contents" << std::endl;
		std::cout << e.what();
		return 0;
	}
	catch (ADError& e) {
		std::cout << "An error occured in AeroDyn" << std::endl;
		std::cout << e.what();
	}

	turbineDiameter = ad.GetTurbineDiameter();
	totalNodes = ad.GetNumNodes();

	// now we know the total number of nodes, so allocate accordingly
	inflows.resize(totalNodes * 3, 0.0);
	bladeNodePositions.resize(totalNodes * 3);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	GenerateInflowVelocities(bladeNodePositions, totalNodes, inflowSpeed, inflows);

	ad.InitInflows(inflows);
	
	double prev_time = 0.0f; // need this to calculate dt for the local UpdateHubMotion
	//-------------------------------
	// Simulation loop
	for (int i = 0; i < NumTimes; i++)
	{
		// visualization window event handling
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}
		double dt = time_array[i] - prev_time;
		prev_time = time_array[i];

		time = time_array[i];
		states = UpdateDriverStates(states, bladeNodePositions, inflows, ad, time , dt);

		RenderBladeNodes(window, bladeNodePositions, states.y.hubPos, 7.0, totalNodes);
	}

	return 0;
}

//-----------------------------
// Function definitions
//-----------------------------
