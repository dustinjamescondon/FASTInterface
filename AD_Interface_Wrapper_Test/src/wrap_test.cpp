#include "..\..\AD_Interface_Wrapper(ver2)\src\PDS_OpenFAST_Wrapper.h"
#include "..\..\AD_Interface_Wrapper(ver2)\eigen\Eigen\SparseCore"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

using namespace Eigen;

const double epsilon = 1.0e-6;

void updateHubState(Vector_3D& hubPos, Vector_3D& hubOri, Vector_3D& hubVel, Vector_3D& hubRotVel, double dt);
void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed);
void renderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Vector_3D& hubPos, double pixelsPerMeter, int nNodes);
Matrix3d EulerConstruct(const Vector3d& eulerAngles);
Vector3d EulerExtract(const Matrix3d& rotationMatrix);


int main(int argc, char *argv[])
{	
	sf::RenderWindow window(sf::VideoMode(800, 800), "Node position test");

	//------------------------
	// Parameters
	double simulationTime = 8.0; // the amount of time to be simulated (in seconds)
	double shaftSpeed = 2.0;     // in rads/sec
	double dt = 0.03;            
	double bladePitch = 0.0;
	double inflowSpeed = 8.7;    // in meters/sec
	//-------------------------
	// Local variables
	int nSteps = (int)ceil(simulationTime / dt);
	double time = 0.0;

	double fluidDensity = 1.236;
	double kinematicFluidVisc = 1.4639e-05;
	Vector_3D hubPos(0.0, 50.0, 50.0);
	Vector_3D hubOri(0.0, 3.143 / 4.0, 0.0);
	Vector_3D hubVel(0.0, 0.0, 0.0);
	Vector_3D hubRotVel(shaftSpeed, 0.0, 0.0);

	int nBlades, nNodes;
	int totalNodes = 0;

	Vector_3D force, moment;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;
	//-------------------------
	// Initialization

	// Create instance of the wrapper class
	PDS_AD_Wrapper adWrapper;

	totalNodes = adWrapper.initHub("input/ad_driver_example.inp", fluidDensity, kinematicFluidVisc,
		hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch, &nBlades, &nNodes);

	// now we know the total number of nodes, so allocate accordingly
	inflows.resize(totalNodes * 3, 0.0);
	bladeNodePositions.resize(totalNodes * 3);

	// get hub positions from AD and then use then to find new inflows

	// But for this test, just have a constant inflow
	// create a steady flow situation in +x direction
	createInflows(inflows, totalNodes, inflowSpeed);

	adWrapper.initInflows(inflows);

	std::cout << "Simulating..." << std::endl;
	
	//-------------------------------
	// Simulation loop
	for (int i = 0; i <= nSteps; i++)
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}
		// this would be where ProteusDS would take its time step, updating the hub kinematics.
		time = i * dt;
		updateHubState(hubPos, hubOri, hubVel, hubRotVel, dt);

		// send Aerodyn the hub kinematics.
		adWrapper.updateHubState(time, hubPos, hubOri, hubVel, hubRotVel, shaftSpeed, bladePitch);

		// then would usually request the current node positions from Aerodyn by calling 
		// getBladeNodePos(...)
		// and then figure out what the inflows would be, but our inflow is just constant anyway,
		// so we just leave the inflow as constant.

		// then we call solve, which will make Aerodyn step forward and simulate up to 'time', and return the 
		// force moment at that time.
		adWrapper.simulate(inflows, force, moment, massMatrix, addedMassMatrix);

		adWrapper.getBladeNodePositions(bladeNodePositions);

		renderBladeNodes(window, bladeNodePositions, hubPos, 7.0, totalNodes);
	}

	return 0;
}

void renderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Vector_3D& hubPos, double pixelsPerMeter, int nNodes) 
{

	sf::CircleShape shape(1.0);

	wnd.clear();

	for (int i = 0; i < nNodes; i++)
	{
		Vector_3D nodePos(&bladeNodePositions[i * 3]);
		Vector_3D relNodePos = nodePos - hubPos; // relative to the center position of the hub
		Vector_3D scaledRelNodePos = relNodePos * pixelsPerMeter;
		Vector_3D scaledHubPos = hubPos * pixelsPerMeter;
		Vector_3D screenNodePos = scaledHubPos + scaledRelNodePos;
		
		// just take the y and z coordinates
		sf::Vector2f pos(screenNodePos.y(), screenNodePos.z());

		shape.setPosition(pos);
		wnd.draw(shape);
	}

	wnd.display();
}

void updateHubState(Vector_3D& hubPos, Vector_3D& hubOri, Vector_3D& hubVel, Vector_3D& hubRotVel, double dt)
{
	hubOri = hubOri + (hubRotVel * dt);
	hubPos = hubPos + (hubVel * dt);
}

void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed)
{
	for (int i = 0; i < totalNodes; i++)
	{
		inflows[i * 3] = inflowSpeed;
	}
}


Matrix3d EulerConstruct(const Vector3d& theta)
{
	double cx = cos(theta(1));
	double sx = sin(theta(1));

	double cy = cos(theta(2));
	double sy = sin(theta(2));

	double cz = cos(theta(3));
	double sz = sin(theta(3));

	Matrix3d result;
	result(1, 1) = cy * cz;
	result(2, 1) = -cy * sz;
	result(3, 1) = sy;
	result(1, 2) = cx * sz + sx * sy * cz;
	result(2, 2) = cx * cz - sx * sy * sz;
	result(3, 2) = -sx * cy;
	result(1, 3) = sx * sz - cx * sy * cz;
	result(2, 3) = sx * cz + cx * sy * sz;
	result(3, 3) = cx * cy;
	
	return result;
}


double sign(double a, double b)
{
	if (b > 0) return a;
	else return -a;
}
Vector3d EulerExtract(const Matrix3d& m)
{
	double cx, cy, cz, sx, sy, sz;
	Vector3d theta;

	cy = sqrt(pow(m(1, 1), 2) + pow(m(2, 1), 2));

	if (cy <= epsilon) {

		theta(2) = atan2(m(3, 1), cy);
		theta(3) = 0.0;
		theta(1) = atan2(m(2, 3), m(2, 2));
	}
	else {
		theta(3) = atan2(-m(2, 1), m(1, 1));
		cz = cos(theta(3));
		sz = sin(theta(3));

		if (cz < epsilon) {
			cy = sign(cy, -m(2, 1) / sz);
		}

		else {
			cy = sign(cy, m(1, 1) / cz);

		}
		theta(2) = atan2(m(3, 1), cy);

		cz = cos(theta(3));
		sz = sin(theta(3));

		cx = sz * m(1, 2) + cz * m(2, 2);
		sx = sz * m(1, 3) + cz * m(2, 3);

		theta(1) = atan2(sx, cx);
	}

	return theta;
}