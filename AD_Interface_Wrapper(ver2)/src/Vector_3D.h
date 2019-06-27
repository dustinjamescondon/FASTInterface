#pragma once

// Note, this is defined in the project preprocessor section
#ifdef AD_INTERFACE_WRAPPER_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  


// 3D vector implemented using a fixed array of length 3
class Vector_3D {
public:

	DECLDIR Vector_3D();
	DECLDIR Vector_3D(const Vector_3D&); // copy constructor
	DECLDIR Vector_3D(const double array[3]); // copy constructor for array of length 3 holding a vector
	DECLDIR Vector_3D(double x, double y, double z); // copy construct given the three values in the vector
	
	DECLDIR Vector_3D operator+(const Vector_3D& v) const;
	DECLDIR Vector_3D operator-(const Vector_3D& v) const;
	DECLDIR Vector_3D operator*(double) const;

	DECLDIR const double* getCArray() const;
	DECLDIR double* getCArray();

	DECLDIR double& x();
	DECLDIR double & y();
	DECLDIR double& z();

	DECLDIR const double& x() const;
	DECLDIR const double& y() const;
	DECLDIR const double& z() const;

private:
	double vec[3];
};
