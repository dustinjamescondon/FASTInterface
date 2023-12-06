#pragma once

#include <Eigen/Dense>

const double Epsilon = 1.0e-06;

// Extracts the Euler angles that will generate the passed orientation matrix
// Note: Copied from openfast's FORTRAN subroutine of the same name
Eigen::Vector3d EulerExtract(const Eigen::Matrix3d& m);

// Construct the orientation matrix from the passed Euler angles
// Note: Copied from openfast's FORTRAN subroutine of the same name
Eigen::Matrix3d EulerConstruct(const Eigen::Vector3d& theta);

//
Eigen::Matrix3d InterpOrientation(double target_time, const Eigen::Matrix3d& orient_1, double time_1, const Eigen::Matrix3d& orient_2, double time_2);

Eigen::Matrix3d InterpOrientation_WithoutChecks(double target_time, const Eigen::Matrix3d& orient1, double time1, const Eigen::Matrix3d& orient2, double time2);

Eigen::Vector3d InterpExtrapVector(double target_time, const Eigen::Vector3d& vect_1, double time_1, const Eigen::Vector3d& vect_2, double time_2);

Eigen::Vector3d InterpExtrapVector_WithoutChecks(double target_time, const Eigen::Vector3d& vect_1, double time_1, const Eigen::Vector3d& vect_2, double time_2);