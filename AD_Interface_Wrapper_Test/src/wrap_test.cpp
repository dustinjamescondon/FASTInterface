#include "..\..\AeroDyn_Interface_Wrapper\src\FASTTurbine_Interface.h"
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
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
	static const double EndTime = 30.0;
	static const double dt = 0.01;
	static const int NSteps = (int)(EndTime / dt);

	static const double InflowSpeed = 20.0;    // in metres/sec
	static const double FluidDensity = 1.225;
	static const double KinematicFluidVisc = 1.4639e-05;
	static const double GearboxRatio = 97.0;
	static const double InitialRotorSpeed = 5.233896;
	static const double DriveTrainDamping = 6215000.0;
	static const double DriveTrainStiffness = 867637000.0;
	static const double RotorMOI = 115926.0;
	static const double GenMOI = 534.116;
	static const double LPFCornerFreq = 1.570796;

	//-------------------------
	// Local variables
	double time = 0.0;

	// create an axis-angle angular velocity using the x basis in global coordinate system
	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;

	int totalNodes = 0;
	//--------------------------
	// Outputs from Aerodyn
	Vector3d force, moment;
	double bladePitch = 0.0;
	double power;
	double tsr;
	double turbineDiameter;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	//--------------------------
	// Initialization

	// Create instance of the wrapper class
	FASTTurbineModel turb;

	turb.SetLPFCornerFreq(LPFCornerFreq);
	turb.SetRotorMassMOI(RotorMOI);
	turb.SetGenMassMOI(GenMOI);
	turb.SetGearboxRatio(GearboxRatio);
	turb.SetInitialRotorSpeed(InitialRotorSpeed);
	turb.SetDriveTrainDamping(DriveTrainDamping);
	turb.SetDriveTrainStiffness(DriveTrainStiffness);

	// Initialize the nacelle state - it will be constant for this simulation test
	FASTTurbineModel::NacelleState nstate;
	nstate.angularVel[0] = 0.0;
	nstate.angularVel[1] = 0.0;
	nstate.angularVel[2] = 0.0;

	nstate.eulerAngles[0] = nstate.eulerAngles[1] = 0.0;
	nstate.eulerAngles[2] = 0.5;
	nstate.position[0] = 0.0;
	nstate.position[1] = nstate.position[2] = 50.0;
	nstate.velocity[0] = nstate.velocity[1] = nstate.velocity[2] = 0.0;


	try {
		turb.InitAeroDyn("C:/Users/dusti/Documents/Work/PRIMED/inputfiles/ad_interface_example2.inp", FluidDensity, KinematicFluidVisc,
			nstate.position, nstate.eulerAngles, nstate.velocity,
			nstate.angularVel, bladePitch);
		
		turb.InitGenController("GenControllerData.csv");
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
			if (event.type == sf::Event::Closed)
				window.close();
		}

		FASTTurbineModel::NacelleReactionForces rf;

		// This sets AeroDyn's hub state inputs according to nstate at t + dt/2
		turb.K1(nstate, time, dt);
		turb.GetBladeNodePositions(bladeNodePositions);
		turb.SetInflowVelocities(inflows);
		rf = turb.UpdateAeroDynStates();

		turb.K2(nstate);
		turb.GetBladeNodePositions(bladeNodePositions);
		turb.SetInflowVelocities(inflows);
		rf = turb.UpdateAeroDynStates();

		turb.K3(nstate);
		turb.GetBladeNodePositions(bladeNodePositions);
		turb.SetInflowVelocities(inflows);
		rf = turb.UpdateAeroDynStates();

		turb.K4(nstate);

		// would be passing the nacelle state determined by the weighted average of the K values
		turb.K_Final(nstate);
		turb.GetBladeNodePositions_Final(bladeNodePositions);
		turb.SetInflowVelocities_Final(inflows);
		rf = turb.UpdateAeroDynStates_Final(); // Perminantely updates AeroDyn's states from t to t

		RenderBladeNodes(window, bladeNodePositions, Vector3d(nstate.position), 7.0, totalNodes);
		
		time += dt;
	}

	return 0;
}

//-----------------------------
// Function definitions
//-----------------------------
