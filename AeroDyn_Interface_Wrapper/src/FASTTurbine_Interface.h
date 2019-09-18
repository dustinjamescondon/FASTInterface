#pragma once
#include "AeroDyn_Interface_Wrapper.h"
#include "DriveTrain.h"
#include "GenController.h"
#include "LowPassFilter.h"

// includes drivetrain model and external generator and pitch controller
class FASTTurbineModel
{
public:
	struct NacelleReactionForces {
		double force[3];
		double moment[3];
	};

	struct NacelleState {
		double position[3];
		double velocity[3];
		double eulerAngles[3];
		double angularVel[3];
	};

	FASTTurbineModel();

	void InitAeroDyn(const char* inputFilename,
		double fluidDensity,
		double kinematicFluidVisc,
		const double hubPosition[3],
		const double hubOrientation[3],
		const double hubVelocity[3],
		const double hubAngularVelocity[3],
		double bladePitch);

	void SetDriveTrainDamping(double);
	void SetDriveTrainStiffness(double);
	void SetRotorMassMOI(double);
	void SetGenMassMOI(double);

	// Pass the nacelle state at t + dt/2; returns temporary nacelle reaction forces at t + dt/2
	void K1(const NacelleState&, double time, double dt);
	// Pass the nacelle state at t + dt/2; returns temporary nacelle reaction forces at t + dt/2
	void K2(const NacelleState&);
	// Pass the nacelle state at t + dt  ; returns temporary nacelle reaction forces at t + dt
	void K3(const NacelleState&);
	// Pass the actual nacelle state at t + dt  
	void K4(const NacelleState&);

	// Call these after each call to a K_()
	void GetBladeNodePositions(std::vector<double>& inflows);
	void SetInflowVelocities(const std::vector<double>&);
	NacelleReactionForces UpdateAeroDynStates();

	// Final update function (rename them because the names are horrible)
	void K_Final(const NacelleState& s);
	void GetBladeNodePositions_Final(std::vector<double>&);
	void SetInflowVelocities_Final(const std::vector<double>&);
	NacelleReactionForces UpdateAeroDynStates_Final();

private:
	double time, dt; // dt of the current round of calling k_(...) functions
	AeroDyn_Interface_Wrapper aerodyn;
	DriveTrain				  drivetrain;
	DriveTrain::ModelStates dt_resultStates;
	GenController			  gencont;
	LowPassFilter			  genSpeedLPF;
	//PitchController         pitchcont;
};

