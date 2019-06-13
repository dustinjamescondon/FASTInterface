#include "Vector_3D.h"

Vector_3D::Vector_3D()
{
	vec[0] = 0.0;
	vec[1] = 0.0;
	vec[2] = 0.0;
}

Vector_3D::Vector_3D(const Vector_3D& v) 
{
	vec[0] = v.vec[0];
	vec[1] = v.vec[1];
	vec[2] = v.vec[2];
}

Vector_3D::Vector_3D(const double arr[3])
{
	vec[0] = arr[0];
	vec[1] = arr[1];
	vec[2] = arr[2];
}

Vector_3D::Vector_3D(double x, double y, double z)
{
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

Vector_3D Vector_3D::operator+(const Vector_3D& v) const
{
	Vector_3D result;
	result.vec[0] = vec[0] + v.vec[0];
	result.vec[1] = vec[1] + v.vec[1];
	result.vec[2] = vec[2] + v.vec[2];

	return result;
}

Vector_3D Vector_3D::operator-(const Vector_3D& v) const
{
	Vector_3D result;
	result.vec[0] = vec[0] - v.vec[0];
	result.vec[1] = vec[1] - v.vec[1];
	result.vec[2] = vec[2] - v.vec[2];

	return result;
}

const double* Vector_3D::getCArray() const {
	return vec;
}

double* Vector_3D::getCArray() {
	return vec;
}

double& Vector_3D::x() {
	return vec[0];
}

double& Vector_3D::y() {
	return vec[1];
}

double& Vector_3D::z() {
	return vec[2];
}

const double& Vector_3D::x() const { 
	return vec[0];
}

const double& Vector_3D::y() const { 
	return vec[1];
}

const double& Vector_3D::z() const { 
	return vec[2]; 
}


Vector_3D operator*(double s, const Vector_3D& v) 
{
	Vector_3D result;
	result.vec[0] = v.vec[0] * s;
	result.vec[1] = v.vec[1] * s;
	result.vec[2] = v.vec[2] * s;

	return result;
}

Vector_3D operator*(const Vector_3D& v, double s)
{
	Vector_3D result;
	result.vec[0] = v.vec[0] * s;
	result.vec[1] = v.vec[1] * s;
	result.vec[2] = v.vec[2] * s; 

	return result;
}