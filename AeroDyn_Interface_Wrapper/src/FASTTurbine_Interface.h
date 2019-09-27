#pragma once
#include "AeroDyn_Interface_Wrapper.h"
#include "DriveTrain.h"
#include "GenController.h"
#include "LowPassFilter.h"
#include "PitchController.h"
#include "BladedInterface.h"

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
	};

	struct NacelleMotion {
		double position[3];
		double velocity[3];
		double eulerAngles[3];
		double angularVel[3];
	};

	DECLDIR FASTTurbineModel();

	DECLDIR ~FASTTurbineModel();

	DECLDIR void InitGenController();

	DECLDIR void InitAeroDyn(const char* inputFilename,
		double fluidDensity,
		double kinematicFluidVisc,
		const NacelleMotion&,
		double bladePitch);

	DECLDIR void InitPitchController(const char*);

	DECLDIR void InitInflows(const std::vector<double>&);

	DECLDIR void SetLPFCornerFreq(double);
	DECLDIR void SetDriveTrainDamping(double);
	DECLDIR void SetDriveTrainStiffness(double);
	DECLDIR void SetRotorMassMOI(double);
	DECLDIR void SetGenMassMOI(double);
	DECLDIR void SetGearboxRatio(double);
	DECLDIR void SetInitialRotorSpeed(double);

	// Pass the nacelle state at t + dt/2; returns temporary nacelle reaction forces at t + dt/2
	DECLDIR void K1(const NacelleMotion&, double time, double dt);
	// Pass the nacelle state at t + dt/2; returns temporary nacelle reaction forces at t + dt/2
	DECLDIR void K2(const NacelleMotion&);
	// Pass the nacelle state at t + dt  ; returns temporary nacelle reaction forces at t + dt
	DECLDIR void K3(const NacelleMotion&);
	// Pass the actual nacelle state at t + dt  
	DECLDIR void K4(const NacelleMotion&);

	// Call these after each call to a K_()
	DECLDIR void GetBladeNodePositions(std::vector<double>&);
	DECLDIR void SetInflowVelocities(const std::vector<double>&);
	DECLDIR NacelleReactionForces UpdateAeroDynStates();

	// Final update function (rename them because the names are horrible)
	DECLDIR void K_Final(const NacelleMotion& s);
	DECLDIR void GetBladeNodePositions_Final(std::vector<double>&);
	DECLDIR void SetInflowVelocities_Final(const std::vector<double>&);
	DECLDIR NacelleReactionForces UpdateAeroDynStates_Final();

	DECLDIR int GetNumNodes() const;

private:

	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);

	double time, dt; // dt of the current round of calling k_(...) functions
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain				  drivetrain;
	DriveTrain::ModelStates   dt_resultStates;
	GenController			  gencont;
	LowPassFilter			  genSpeedLPF;
	PitchController           pitchcont;
	BladedInterface           bladed;
};

