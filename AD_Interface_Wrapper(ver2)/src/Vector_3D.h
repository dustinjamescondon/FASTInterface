#pragma once

// 3D vector implemented using a fixed array of length 3
class Vector_3D {
public:
	friend Vector_3D operator*(double, const Vector_3D&);
	friend Vector_3D operator*(const Vector_3D&, double);

	Vector_3D();
	Vector_3D(const Vector_3D&); // copy constructor
	Vector_3D(const double array[3]); // copy constructor for array of length 3 holding a vector
	Vector_3D(double x, double y, double z); // copy construct given the three values in the vector
	
	Vector_3D operator+(const Vector_3D& v) const;
	Vector_3D operator-(const Vector_3D& v) const;

	const double* getCArray() const;
	double* getCArray();

	double& x();
	double& y();
	double& z();

	const double& x() const;
	const double& y() const;
	const double& z() const;

private:
	double vec[3];
};

// We define this operator outside the class so that we can have the scalar (double) on the LHS as well as RHS.
Vector_3D operator*(double, const Vector_3D&);
Vector_3D operator*(const Vector_3D&, double);