#include "Vector_AD.h"

Vector_AD::Vector_AD()
{
}

Vector_AD::Vector_AD(const Vector_AD& v) 
{
	vec[0] = v.vec[0];
	vec[1] = v.vec[1];
	vec[2] = v.vec[2];
}

Vector_AD Vector_AD::operator+(const Vector_AD& v) const
{
	Vector_AD result;
	result.vec[0] = vec[0] + v.vec[0];
	result.vec[1] = vec[1] + v.vec[1];
	result.vec[2] = vec[2] + v.vec[2];

	return result;
}

const double* Vector_AD::getCArray() const {
	return vec;
}

double* Vector_AD::getCArray() {
	return vec;
}

double& Vector_AD::x() {
	return vec[0];
}

double& Vector_AD::y() {
	return vec[1];
}

double& Vector_AD::z() {
	return vec[2];
}

const double& Vector_AD::x() const { 
	return vec[0];
}

const double& Vector_AD::y() const { 
	return vec[1];
}

const double& Vector_AD::z() const { 
	return vec[2]; 
}


Vector_AD operator*(double s, const Vector_AD& v) 
{
	Vector_AD result;
	result.vec[0] = v.vec[0] * s;
	result.vec[1] = v.vec[1] * s;
	result.vec[2] = v.vec[2] * s;

	return result;
}

Vector_AD operator*(const Vector_AD& v, double s)
{
	Vector_AD result;
	result.vec[0] = v.vec[0] * s;
	result.vec[1] = v.vec[1] * s;
	result.vec[2] = v.vec[2] * s; 

	return result;
}