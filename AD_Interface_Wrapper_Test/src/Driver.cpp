#include "Driver.h"
#include "..\..\AeroDyn_Interface_Wrapper\src\AeroDyn_Interface_Wrapper.h"


using namespace Eigen;

Vector3d RotateOrientation(Vector3d ori, Vector3d axis_angle) {
	// calculate current basis vectors for the hub's orientation matrix
	Matrix3d oriMatrix = EulerConstruct(ori);
	Vector3d basisX = oriMatrix.row(0);
	Vector3d basisY = oriMatrix.row(1);
	Vector3d basisZ = oriMatrix.row(2);

	Vector3d axis_angle_unit = axis_angle.normalized();
	double theta = axis_angle.norm();

	// integrate the rotational radians/sec and rotate each basis vector around the axis accordingly
	basisX = axisAngleRotation(basisX, axis_angle_unit, theta);
	basisY = axisAngleRotation(basisY, axis_angle_unit, theta);
	basisZ = axisAngleRotation(basisZ, axis_angle_unit, theta);

	oriMatrix.row(0) = basisX;
	oriMatrix.row(1) = basisY;
	oriMatrix.row(2) = basisZ;

	return EulerExtract(oriMatrix);
}



// Returns the dydt states at time + dt
States_dydt Calculate_dydt(DriverStates states, AeroDyn_Interface_Wrapper& ad, std::vector<double>& bladeNodePos,
	std::vector<double>& inflows, double time, double dt)
{
	// return early for debugging purposes
	return states.dydt;

	DriverStates result;

	// Update our states first, then send them as inputs for AD_Interface
	result.dydt.hubVel = states.dydt.hubVel;
	result.dydt.hubAngVel = states.dydt.hubAngVel;

	result.dy.dHubPos = dt * states.dydt.hubVel;
	result.dy.dHubAng = dt * states.dydt.hubAngVel;

	result.y.hubPos = states.y.hubPos + result.dy.dHubPos;
	result.y.hubOri = RotateOrientation(states.y.hubOri, states.dy.dHubAng);

	// Now update the turbine's states (fakely)
	ad.SetHubMotion(time + dt, result.y.hubPos.data(), result.y.hubOri.data(), result.dydt.hubVel.data(),
		result.dydt.hubAngVel.data(), 0.0, false);

	ad.GetBladeNodePositions(bladeNodePos, false);

	ad.SetInflowVelocities(inflows, false);

	Vector3d force, moment;
	double power, tsr;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	ad.UpdateStates(force.data(), moment.data(), &power, &tsr, massMatrix, addedMassMatrix,
		false);

	// would do some dynamics calculations here to update dydt states based on loads/moments

	return result.dydt;
}

States_dy CalcWeightedAvg(const DriverStates k[4])
{
	Vector3d dHubPos = (1.0 / 6.0) * (k[0].dy.dHubPos + (2.0 * k[1].dy.dHubPos) + (2.0 * k[2].dy.dHubPos)
		+ k[3].dy.dHubPos);

	Vector3d dHubAng = k[0].dy.dHubAng; // (1.0 / 6.0)* (k[0].dy.dHubAng + (2.0 * k[1].dy.dHubAng) + (2.0 * k[2].dy.dHubAng)
		//+ k[3].dy.dHubAng);

	States_dy result;
	result.dHubPos = dHubPos;
	result.dHubAng = dHubAng;

	return result;
}



// Performs Runge-Kutta integration on dydt states
DriverStates UpdateDriverStates(DriverStates states, std::vector<double>& bladeNodePos,
	std::vector<double>& inflows, AeroDyn_Interface_Wrapper& ad, double time, double dt)
{
	// Uncomment this if we just want to use Euler's method
	/*
	DriverStates r = states;
	r.y.hubPos += r.dydt.hubVel * dt;
	r.dy.dHubAng = r.dydt.hubAngVel * dt;
	r.y.hubOri = RotateOrientation(r.y.hubOri, r.dy.dHubAng);

	ad.UpdateHubMotion(time + dt, r.y.hubPos.data(), r.y.hubOri.data(), r.dydt.hubVel.data(),
		r.dydt.hubAngVel.data(), 0.0, true);

	ad.GetBladeNodePositions(bladeNodePos, true);

	Vector3d force, moment;
	double power, tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	ad.Simulate(inflows, force.data(), moment.data(), &power, &tsr, massMat, addedMassMat, true);

	return r;
	*/

	// Implements something similar to the Runge-Kutta method
	//----------------------------------------
	DriverStates k[4];
	DriverStates states_tmp;

	// k[0] - evaluate f(t,y)... the contents of states.dydt is the answer
	// step over h
	k[0] = EulerStep(states, bladeNodePos, inflows, ad, time, dt);

	// k[1] - evaluate f(t + dt/2, y + k1/2)
	states_tmp.y = states.y;
	states_tmp.dydt = states.dydt;
	states_tmp.dydt = Calculate_dydt(states_tmp, ad, bladeNodePos, inflows, time, 0.5 * dt);

	// and step over dt
	k[1] = EulerStep(states_tmp, bladeNodePos, inflows, ad, time, dt);

	// k[2] - evaluate f(t + dt/2, y + k2/2)
	states_tmp.y = states.y;
	states_tmp.dydt = states.dydt /* not sure */;
	states_tmp.dydt = Calculate_dydt(states_tmp, ad, bladeNodePos, inflows, time, 0.5 * dt);

	// and step over dt
	k[2] = EulerStep(states_tmp, bladeNodePos, inflows, ad, time, dt);

	// k[3] - evaluate f(t + dt, y + k3)
	states_tmp.y = states.y;
	states_tmp.dydt = states.dydt/* not sure */;
	states_tmp.dydt = Calculate_dydt(states_tmp, ad, bladeNodePos, inflows, time, dt);

	// and step over dt
	k[3] = EulerStep(states_tmp, bladeNodePos, inflows, ad, time, dt);

	// calculate the weighted average of the dy values	
	DriverStates  result;
	result.dy = CalcWeightedAvg(k);

	// Update driver states
	result.y.hubPos = states.y.hubPos + result.dy.dHubPos;
	result.dydt.hubAngVel = states.dydt.hubAngVel;
	result.y.hubOri = RotateOrientation(states.y.hubOri, result.dy.dHubAng);
	result.dydt.hubVel = states.dydt.hubVel;

	// Use them to update AeroDyn's turbine states
	ad.SetHubMotion(time + dt, result.y.hubPos.data(), result.y.hubOri.data(),
		result.dydt.hubVel.data(), result.dydt.hubAngVel.data(), 0.0, true);

	ad.GetBladeNodePositions(bladeNodePos, true);

	ad.SetInflowVelocities(inflows, true);

	Vector3d force, moment;
	double power, tsr;
	double massMat[6][6];
	double addedMassMat[6][6];

	ad.UpdateStates(force.data(), moment.data(), &power, &tsr, massMat, addedMassMat, true);

	return result;
}

DriverStates EulerStep(DriverStates states, std::vector<double>& bladeNodePos, std::vector<double>& inflows, AeroDyn_Interface_Wrapper& ad, double time, double dt)
{
	DriverStates result;

	// Update our states first, then send them as inputs for AD_Interface
	result.dydt.hubVel = states.dydt.hubVel;
	result.dydt.hubAngVel = states.dydt.hubAngVel;

	result.dy.dHubPos = dt * states.dydt.hubVel;
	result.dy.dHubAng = dt * states.dydt.hubAngVel;

	result.y.hubPos = states.y.hubPos + result.dy.dHubPos;
	result.y.hubOri = RotateOrientation(states.y.hubOri, result.dy.dHubAng);

	// Now update the turbine's states (fakely)
	ad.SetHubMotion(time + dt, result.y.hubPos.data(), result.y.hubOri.data(), result.dydt.hubVel.data(),
		result.dydt.hubAngVel.data(), 0.0, false);

	ad.GetBladeNodePositions(bladeNodePos, false);

	ad.SetInflowVelocities(inflows, false);

	Vector3d force, moment;
	double power, tsr;
	double massMatrix[6][6];
	double addedMassMatrix[6][6];

	ad.UpdateStates(force.data(), moment.data(), &power, &tsr, massMatrix, addedMassMatrix,
		false);

	// would do some dynamics calculations here to update the hub state variables based on loads/moments

	return result;
}

void GenerateInflowVelocities(const std::vector<double>& nodePositions, int totalNodes, double inflowSpeed, std::vector<double>& inflows)
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
// Implements Rodrigues' rotation formula
Vector3d axisAngleRotation(const Vector3d& v, const Vector3d& e, double theta)
{
	Vector3d result = cos(theta) * v + sin(theta) * e.cross(v) + (1 - cos(theta)) * e.dot(v) * e;

	return result;
}