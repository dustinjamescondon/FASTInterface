#pragma once
#include "AeroDyn_Interface_Wrapper.h"
#include "DriveTrain.h"
#include "MasterController.h"
#include <Eigen/Dense>

/* This class adds a controller and drivetrain overtop of AeroDyn */
class AeroDynTurbine
{
public:
	struct HubMotion {
		Vector3d position, velocity, acceleration, angularVel, angularAcc;
		Matrix3d orientation;
	};

	struct HubAcc {
		Vector3d acceleration, angularAcc;
	};

	// Structure to hold the state of the nacelle
	struct NacelleMotion {
		Vector3d position;
		Vector3d velocity;
		Vector3d acceleration;
		Vector3d EulerAngles;
		Vector3d angularVel;
		Vector3d angularAcc;
	};

	// There are two structures that carry the same information. This on uses Eigen::Vector3d; the other, arrays
	struct NacelleReactionLoads_Vec {
		Vector3d force, moment;
		double tsr, power;
	};

	void InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch);
	void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed);
	void InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch);
	void InitAeroDyn(
		const std::string& inputFilename,
		const std::string& outputFilename,
		double timestep,
		int numBlades,
		double hubRadius,
		double precone,
		double fluidDensity,
		double kinematicFluidVisc,
		const Vector3d& nacellePos,
		const Vector3d& nacelleEulerAngles,
		const Vector3d& nacelleVel,
		const Vector3d& nacelleAcc,
		const Vector3d& nacelleAngularVel,
		const Vector3d& nacelleAngularAcc);
	void InitInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);
	void SetInputs_Nacelle(
		double time,
		const Vector3d& nacellePos,
		const Vector3d& nacelleEulerAngles,
		const Vector3d& nacelleVel,
		const Vector3d& nacelleAcc,
		const Vector3d& nacelleAngularVel,
		const Vector3d& nacelleAngularAcc,
		bool isRealStep = true);

	// Sets the inflow velocities at the \time passed to SetNacelleMotion(...)
	void SetInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);
	NacelleReactionLoads_Vec AdvanceStates();
	NacelleReactionLoads_Vec CalcOutput();

	NacelleReactionLoads_Vec GetNacelleReactionLoads() const { return nacelleReactionLoads;  }
	Vector3d GetNacelleReactionForce() const { return nacelleReactionLoads.force; }
	Vector3d GetNacelleReactionMoment() const { return nacelleReactionLoads.moment; }
	double GetPower() const { return nacelleReactionLoads.power; }
	double GetTSR() const { return nacelleReactionLoads.tsr; }

protected:
private:
	DriveTrain::ModelStates IntegrateDriveTrain_Euler(double time, const AeroDynTurbine::NacelleMotion&);
	DriveTrain::ModelStates IntegrateDriveTrain_RK4(double time, const AeroDynTurbine::NacelleMotion&, const std::vector<double>& inflowVel,
		const std::vector<double>& inflowAcc);

	NacelleReactionLoads_Vec UpdateAeroDynStates(bool isRealStep);
	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&);
	HubAcc CalculateHubAcc(const Vector3d& nacAcc, const Vector3d& nacRotAcc, const Matrix3d& nacOri, double rotorShaftAcc);

	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation);
	Matrix3d InterpExtrapOrientation(double target_time, const Matrix3d& orient_1, double time_1, const Matrix3d& orient_2, double time_2) const;
	Vector3d InterpExtrapVector(double target_time, const Vector3d& vect_1, double time_1, const Vector3d& vect_2, double time_2) const;

	//---------------------
	bool onRealStep;
	double time, targetTime;

	/* AeroDyn stuff */
	AeroDyn_Interface_Wrapper aerodyn;
	std::vector<double> inflowVel, inflowAcc; // holds the blade node inflows
	AeroDynTurbine::NacelleMotion nacelleMotion;
	NacelleReactionLoads_Vec nacelleReactionLoads; // Stores the most recent load results
	Matrix3d nacelleOrient, hubOrient;
	Vector3d nacelleForce, nacelleMoment;

	/* Drivetrain stuff */
	DriveTrain::ModelStates drivetrainStates_pred; // Holds drivetrain states on non-real state updates
	DriveTrain		      drivetrain;

	/* Controller stuff */
	MasterController	      mcont;
};