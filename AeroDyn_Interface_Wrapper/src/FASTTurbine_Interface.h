#pragma once
#include "AeroDyn_Interface_Wrapper.h"
#include "DriveTrain.h"
#include "MasterController.h"
#include "BladedInterface.h"
#include <memory>
#include <Eigen/Dense>

using namespace Eigen;

// Note, this is defined in the project preprocessor section
#ifdef FASTTURBINE_INTERFACE_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

struct HubMotion;

// includes drivetrain model and external generator and pitch controller
class FASTTurbineModel
{
public:

	struct NacelleReactionForces {
		double force[3];
		double moment[3];
		double tsr;
		double power;
	};

	struct NacelleMotion {
		double position[3];
		double velocity[3];
		double eulerAngles[3];
		double angularVel[3];
	};

	DECLDIR FASTTurbineModel();

	DECLDIR ~FASTTurbineModel();

	// Must call this one first
	DECLDIR void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorVel);

	// Must call one of these second
	// Bladed-style controller initialization
	DECLDIR void InitControllers(const char* blade_dll_fname);
	// Input file specified generator and pitch controllers
	DECLDIR void InitControllers(const char* gen_csv, const char* pit_fname, double lpfCornerFreq);

	// Must call last
	DECLDIR void InitAeroDyn(
		const char* inputFilename,
		double fluidDensity,
		double kinematicFluidVisc,
		const NacelleMotion&);

	DECLDIR void InitInflows(const std::vector<double>&);

	// Pass the nacelle state at t + dt/2; begins process which will evenetually return temporary nacelle reaction forces at t + dt/2
	DECLDIR void K1(const NacelleMotion&, double time, double dt);
	// Pass the nacelle state at t + dt/2; begins process which will evenetually return temporary nacelle reaction forces at t + dt/2
	DECLDIR void K2(const NacelleMotion&);
	// Pass the nacelle state at t + dt  ; begins process which will evenetually return temporary nacelle reaction forces at t + dt
	DECLDIR void K3(const NacelleMotion&);
	// Pass the actual nacelle state at t + dt  
	DECLDIR void K4(const NacelleMotion&);

	// Call these after each call to a K_()
	DECLDIR void GetBladeNodePositions(std::vector<double>&);
	DECLDIR void SetInflowVelocities(const std::vector<double>&);
	DECLDIR NacelleReactionForces UpdateAeroDynStates();

	// Final update functions
	DECLDIR void K_Final(const NacelleMotion& s);
	DECLDIR void GetBladeNodePositions_Final(std::vector<double>&);
	DECLDIR void SetInflowVelocities_Final(const std::vector<double>&);
	DECLDIR NacelleReactionForces UpdateAeroDynStates_Final();

	DECLDIR int GetNumNodes() const;
	DECLDIR int GetNumBlades() const;
	DECLDIR double GetBladePitch() const;
	DECLDIR double GetGeneratorTorque() const;
	DECLDIR double GetGeneratorSpeed() const;
	DECLDIR double GetRotorSpeed() const;

private:
	// Pointer to implementation class to hide implemenation
	class PImp;
	std::unique_ptr<PImp> p_imp;

	// 
	/*
	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);
	NacelleReactionForces TransferReactionForces(const double force[3], const double moment[3],
		const Matrix3d& nacelleOrienation, const Matrix3d& hubOrientation);
	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation);

	double time, dt; // dt of the current round of calling k_(...) functions
	Matrix3d nacelleOrient, hubOrient;
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain				  drivetrain;
	DriveTrain::ModelStates   dt_resultStates;
	MasterController		  mcont;
	*/
};