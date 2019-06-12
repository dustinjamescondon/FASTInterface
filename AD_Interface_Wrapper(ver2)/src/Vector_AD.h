#pragma once

class Vector_AD {
public:
	friend Vector_AD operator*(double, const Vector_AD&);
	friend Vector_AD operator*(const Vector_AD&, double);

	Vector_AD();
	Vector_AD(const Vector_AD&); // copy constructor
	
	Vector_AD operator+(const Vector_AD& v) const;

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

Vector_AD operator*(double, const Vector_AD&);
Vector_AD operator*(const Vector_AD&, double);