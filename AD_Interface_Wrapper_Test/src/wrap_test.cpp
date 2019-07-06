#include "..\..\AD_Interface_Wrapper(ver2)\src\PDS_OpenFAST_Wrapper.h"
#include "..\..\AD_Interface_Wrapper(ver2)\eigen\Eigen\Dense"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

using namespace Eigen;

void updateHubState(Vector3d& hubPos, Vector3d& hubOri, Vector3d& hubVel, Vector3d& hubRotVel, double dt);
void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed);
void renderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Vector3d& hubPos, double pixelsPerMeter, int nNodes);
Matrix3d EulerConstruct(const Vector3d& eulerAngles);  // creates global to local rotation matrix
Vector3d EulerExtract(const Matrix3d& rotationMatrix); // get Euler angles from global to local rotation matrix
Vector3d axisAngleRotation(const Vector3d& v, const Vector3d& e, double theta); // v is vector to be rotated around e by theta radians (right-hand rule applies)

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
	double fluidDensity = 1.236;
	double kinematicFluidVisc = 1.4639e-05;
	//-------------------------
	// Local variables
	int nSteps = (int)ceil(simulationTime / dt);
	double time = 0.0;

	Vector3d hubPos(0.0, 50.0, 50.0);
	Vector3d hubOri(0.0, 0.0, 0.0);
	Matrix3d hubOriMatrix = EulerConstruct(hubOri);
	Vector3d hubVel(0.0, 0.0, 0.0);
	Vector3d hubRotVel = hubOriMatrix.row(0) * shaftSpeed;

	int nBlades, nNodes;
	int totalNodes = 0;

	Vector3d force, moment;
	double power;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	std::vector<double> inflows;
	std::vector<double> bladeNodePositions;
	//-------------------------
	// Initialization

	// Create instance of the wrapper class
	PDS_AD_Wrapper adWrapper;

	totalNodes = adWrapper.initHub("input/ad_driver_example.inp", fluidDensity, kinematicFluidVisc,
		&hubPos(0), &hubOri(0), &hubVel(0), &hubRotVel(0), shaftSpeed, bladePitch, &nBlades, &nNodes);

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
		adWrapper.updateHubState(time, hubPos.data(), hubOri.data(), hubVel.data(), hubRotVel.data(), shaftSpeed, bladePitch);

		// then would usually request the current node positions from Aerodyn by calling 
		// getBladeNodePos(...)
		// and then figure out what the inflows would be, but our inflow is just constant anyway,
		// so we just leave the inflow as constant.

		// then we call solve, which will make Aerodyn step forward and simulate up to 'time', and return the 
		// force, moment, and power at that time.
		adWrapper.simulate(inflows, force.data(), moment.data(), &power, massMatrix, addedMassMatrix);

		adWrapper.getBladeNodePositions(bladeNodePositions);

		renderBladeNodes(window, bladeNodePositions, hubPos, 7.0, totalNodes);
	}

	return 0;
}

void renderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Vector3d& hubPos, double pixelsPerMeter, int nNodes) 
{

	sf::CircleShape shape(1.0);

	wnd.clear();

	for (int i = 0; i < nNodes; i++)
	{
		Vector3d nodePos(&bladeNodePositions[i * 3]); // set the node position from a pointer to the beginning of an array
		Vector3d relNodePos = nodePos - hubPos; // relative to the center position of the hub
		Vector3d scaledRelNodePos = relNodePos * pixelsPerMeter;
		Vector3d scaledHubPos = hubPos * pixelsPerMeter;
		Vector3d screenNodePos = scaledHubPos + scaledRelNodePos;
		
		// just take the y and z coordinates
		sf::Vector2f pos(screenNodePos.y(), screenNodePos.z());

		shape.setPosition(pos);
		wnd.draw(shape);
	}

	wnd.display();
}

void updateHubState(Vector3d& hubPos, Vector3d& hubOri, Vector3d& hubVel, Vector3d& hubRotVel, double dt)
{
	// update the orientation using rotational velocity
	Matrix3d hubOriMatrix = EulerConstruct(hubOri);
	Vector3d basisX = hubOriMatrix.row(0);
	Vector3d basisY = hubOriMatrix.row(1);
	Vector3d basisZ = hubOriMatrix.row(2);

	// rotate each basis vector around the axis
	basisX = axisAngleRotation(basisX, hubRotVel.normalized(), hubRotVel.norm() * dt);
	basisY = axisAngleRotation(basisY, hubRotVel.normalized(), hubRotVel.norm() * dt);
	basisZ = axisAngleRotation(basisZ, hubRotVel.normalized(), hubRotVel.norm() * dt);

	hubOriMatrix.row(0) = basisX;
	hubOriMatrix.row(1) = basisY;
	hubOriMatrix.row(2) = basisZ;

	// get the Euler angles out of the new orientation matrix
	hubOri = EulerExtract(hubOriMatrix);

	// integrate position
	hubPos = hubPos + (hubVel * dt);
}

void createInflows(std::vector<double>& inflows, int totalNodes, double inflowSpeed)
{
	for (int i = 0; i < totalNodes; i++)
	{
		inflows[i * 3] = inflowSpeed;
	}
}

// taken from Aerodyn's subroutine of the same name
Matrix3d EulerConstruct(const Vector3d& theta)
{
	double cx = cos(theta(0));
	double sx = sin(theta(0));

	double cy = cos(theta(1));
	double sy = sin(theta(1));

	double cz = cos(theta(2));
	double sz = sin(theta(2));

	Matrix3d result;
	result(0, 0) = cy * cz;
	result(1, 0) = -cy * sz;
	result(2, 0) = sy;
	result(0, 1) = cx * sz + sx * sy * cz;
	result(1, 1) = cx * cz - sx * sy * sz;
	result(2, 1) = -sx * cy;
	result(0, 2) = sx * sz - cx * sy * cz;
	result(1, 2) = sx * cz + cx * sy * sz;
	result(2, 2) = cx * cy;
	
	return result;
}

// a build-in subroutine in FORTRAN. Multiply abs(a) by the sign of b
double sign(double a, double b)
{
	if (b > 0) return a;
	else return -a;
}

// taken from Aerodyn's subroutine of the same name
Vector3d EulerExtract(const Matrix3d& m)
{
	static const double epsilon = 1.0e-5;

	double cx, cy, cz, sx, sy, sz;
	Vector3d theta;

	cy = sqrt(pow(m(0, 0), 2) + pow(m(1, 0), 2));

	if (cy < epsilon) {

		theta(1) = atan2(m(2, 0), cy);
		theta(2) = 0.0;
		theta(0) = atan2(m(1, 2), m(1, 1));
	}
	else {
		theta(2) = atan2(-m(1, 0), m(0, 0));
		cz = cos(theta(2));
		sz = sin(theta(2));

		if (cz < epsilon) {
			cy = sign(cy, -m(1, 0) / sz);
		}

		else {
			cy = sign(cy, m(0, 0) / cz);

		}
		theta(1) = atan2(m(2, 0), cy);

		cz = cos(theta(2));
		sz = sin(theta(2));

		cx = sz * m(0, 1) + cz * m(1, 1);
		sx = sz * m(0, 2) + cz * m(1, 2);

		theta(0) = atan2(sx, cx);
	}

	return theta;
}

// v is vector to be rotated around e by theta radians (right-hand rule applies)
Vector3d axisAngleRotation(const Vector3d& v, const Vector3d& e, double theta)
{
	Vector3d result = cos(theta) * v + sin(theta) * e.cross(v) + (1 - cos(theta)) * e.dot(v) * e;

	return result;
}