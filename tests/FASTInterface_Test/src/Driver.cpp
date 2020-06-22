#include "Driver.h"


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