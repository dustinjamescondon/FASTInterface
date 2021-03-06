#pragma once
#include <Eigen\Dense>
#include "..\..\FASTInterface\src\AeroDyn_Interface_Wrapper.h"
#include <vector>

// states_pred that are a derivative of time_act which we will be integrating
struct States_dydt {
	Eigen::Vector3d hubVel, hubAngVel;
};

struct States_y {
	Eigen::Vector3d hubPos, hubOri;
};

struct States_dy {
	Eigen::Vector3d dHubPos, dHubAng; // the integrated axis-angle 
};

struct DriverStates {
	States_y     y;
	States_dy    dy;
	States_dydt  dydt;
};

//----------------------------
// Functions that mimic what ProteusDS would be doing

// Integrates hub velocity and rotational velocity using RK4
DriverStates UpdateDriverStates(DriverStates states, std::vector<double>& bladeNodePos,
	std::vector<double>& inflows, AeroDyn_Interface_Wrapper& ad, double time, double dt);

// Uses Euler method to update states_pred in time_act by dt, returning the results
DriverStates EulerStep(DriverStates states, std::vector<double>& bladeNodePos, std::vector<double>& inflows, AeroDyn_Interface_Wrapper& ad, double time, double dt);

void GenerateInflowVelocities(const std::vector<double>& nodePositions, int totalNodes, 
	double inflowSpeed, std::vector<double>& inflowVel, std::vector<double>& inflowAcc);

//----------------------------
// Function to display the blade nodes to visualize rotation



//----------------------------
// Some utility functions to help update the orientation for our small example

// Returns the global-to-local orientation matrix from the euler angles
// That is, R = R_z*R_y*R_x
Eigen::Matrix3d EulerConstruct(const Eigen::Vector3d& eulerAngles);  // creates global to local rotation matrix

// Returns the euler angles that generate the given rotation matrix
Eigen::Vector3d EulerExtract(const Eigen::Matrix3d& rotationMatrix); // get Euler angles from global to local rotation matrix

// Rotates v around e by theta radians (right-hand rule applies)
Eigen::Vector3d axisAngleRotation(const Eigen::Vector3d& v, const Eigen::Vector3d& e, double theta);
