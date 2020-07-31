#include "Utilities.h"

using namespace Eigen;

// a build-in subroutine in FORTRAN. Multiply abs(a) by the sign of b
double sign(double a, double b)
{
	if (b > 0) return a;
	else return -a;
}

// Extracts the Euler angles that will generate the passed orientation matrix
// Note: Copied from openfast's FORTRAN subroutine of the same name
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

// Construct the orientation matrix from the passed Euler angles
// Note: Copied from openfast's FORTRAN subroutine of the same name
Matrix3d EulerConstruct(const Vector3d& theta)
{
	double cx = cos(theta.x());
	double sx = sin(theta.x());

	double cy = cos(theta.y());
	double sy = sin(theta.y());

	double cz = cos(theta.z());
	double sz = sin(theta.z());

	Matrix3d m;
	m(0, 0) = cy * cz;
	m(1, 0) = -cy * sz;
	m(2, 0) = sy;

	m(0, 1) = cx * sz + sx * sy*cz;
	m(1, 1) = cx * cz - sx * sy*sz;
	m(2, 1) = -sx * cy;

	m(0, 2) = sx * sz - cx * sy*cz;
	m(1, 2) = sx * cz + cx * sy*sz;
	m(2, 2) = cx * cy;

	return m;
}




Matrix3d InterpOrientation(double target_time, const Matrix3d& orient_1, double time_1,
	const Matrix3d& orient_2, double time_2)
{
	// if the times are nearly equal, then just return the second vector
	if (abs(time_2 - time_1) < Epsilon) {
		return orient_1;
	}
	// if the target time is very close to time_2
	else if (abs(time_2 - target_time) < Epsilon)
	{
		return orient_2;
	}
	// if the target time is very close to time_1
	else if (abs(time_1 - target_time) < Epsilon)
	{
		return orient_1;
	}

	return InterpOrientation_WithoutChecks(target_time, orient_1, time_1, orient_2, time_2);
}

Matrix3d InterpOrientation_WithoutChecks(double target_time, const Matrix3d& orient1, double time1,
	const Matrix3d& orient2, double time2)
{
	// normalize the target time, so 0 is at time1, while 1 is at time2
	double t = (target_time - time1) / (time2 - time1);
	Quaterniond q1(orient1);
	Quaterniond q2(orient2);

	Matrix3d result(q1.slerp(t, q2));
	return result;
}

// TODO create version without the checks, so that functions that bulk interpolate/extrapolate a bunch of vectors/orientations
// at one time can do the check themselves once for all of them
Vector3d InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2)
{
	// if the times are nearly equal, then just return the second vector
	if (abs(time_2 - time_1) < Epsilon) {
		return vect_2;
	}
	// if the target time is very close to time_2
	else if (abs(time_2 - target_time) < Epsilon)
	{
		return vect_2;
	}
	// if the target time is very close to time_1
	else if (abs(time_1 - target_time) < Epsilon)
	{
		return vect_1;
	}

	return InterpExtrapVector_WithoutChecks(target_time, vect_1, time_1, vect_2, time_2);
}

Vector3d InterpExtrapVector_WithoutChecks(double target_time, const Vector3d& vect_1, double time_1,
	const Vector3d& vect_2, double time_2) 
{
	Vector3d m = (vect_2 - vect_1) / (time_2 - time_1);

	return vect_1 + m * (target_time - time_1);
}

