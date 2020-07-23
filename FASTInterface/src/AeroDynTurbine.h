#pragma once
#include "AeroDyn_Interface_Wrapper.h"
#include "DriveTrain.h"
#include "MasterController.h"
#include <Eigen/Dense>

/* This class coordinates AeroDyn, the drivetrain, and the controller. It also implements
the input-output solver for when added mass is enabled. */
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
		Matrix3d orientation;
		Vector3d angularVel;
		Vector3d angularAcc;
	};

	// There are two structures that carry the same information. This on uses Eigen::Vector3d; the other, arrays
	struct NacelleReactionLoads_Vec {
		Vector3d force, moment;
		double tsr, power;
	};

	AeroDynTurbine();

	void InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch);
	void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed);
	void InitControllers_BladedDLL(int numBlades, const std::string& bladed_dll_fname, double initialBladePitch);
	void InitAeroDyn(
		const std::string& inputFilename,
		const std::string& outputFilename,
		bool useAddedMass,
		double coeffAddedMass,
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

	void SetCallback_CalcOutput(std::function<void(const double*, const double*, double*, double*)> CalcOutput);

	void SetInputs_Nacelle(
		double time,
		const Vector3d& nacellePos,
		const Vector3d& nacelleEulerAngles,
		const Vector3d& nacelleVel,
		const Vector3d& nacelleAcc,
		const Vector3d& nacelleAngularVel,
		const Vector3d& nacelleAngularAcc,
		bool isTempUpdate);

	void SetInputs_NacelleAcc(
		const Vector3d& nacelleAcc,
		const Vector3d& nacelleRotationAcc);

	// Sets the inflow velocities at the \time_act passed to SetNacelleMotion(...)
	void SetInputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);

	// This will actually calculate and solve for the correct accelerations/loads and get the outputs
	// unlike the UpdateStates functions of AeroDyn and the drivetrain
	NacelleReactionLoads_Vec AdvanceStates();

	void GetBladeNodePositions(std::vector<double>& nodePositions) { aerodyn.GetBladeNodePositions(nodePositions); }
	AeroDyn_Interface_Wrapper::HubReactionLoads GetHubReactionLoads() const { return aerodyn.GetHubReactionLoads(); }
	NacelleReactionLoads_Vec GetNacelleReactionLoads() const { return nacelleReactionLoads; }
	Vector3d GetNacelleReactionForce() const { return nacelleReactionLoads.force; }
	Vector3d GetNacelleReactionMoment() const { return nacelleReactionLoads.moment; }
	double GetPower() const { return nacelleReactionLoads.power; }
	double GetTSR() const { return nacelleReactionLoads.tsr; }
	int GetNumNodes() const { return aerodyn.GetNumNodes(); }
	int GetNumBlades() const { return aerodyn.GetNumBlades(); }
	double GetTurbineDiameter() const { return aerodyn.GetTurbineDiameter();  }
	double GetBladePitch() const { return aerodyn.GetBladePitch(); }
	double GetGenTorque() const { return mcont.GetGeneratorTorqueCommand(); }
	double GetGenSpeed() const { return drivetrain.GetGenShaftSpeed(); }
	double GetRotorSpeed() const { return drivetrain.GetRotorShaftSpeed(); }
	DriveTrain::States GetRotorShaftState() const { return drivetrain.GetRotorStates(); }
	DriveTrain::States GetGenShaftState() const { return drivetrain.GetGenStates(); }

protected:
private:
	typedef Eigen::Vector<double,13> SerializedVector;

	enum { Y_DVR_NAC_ACC = 0, Y_DVR_NAC_ROTACC = 3, Y_AD_HUB_FORCE = 6, Y_AD_HUB_MOMENT = 9, Y_DT_ROTOR_ACC = 12 };    // outputs
	enum { U_DVR_NAC_FORCE = 0, U_DVR_NAC_MOMENT = 3, U_AD_HUB_ACC = 6, U_AD_HUB_ROTACC = 9, U_DT_ROTOR_TORQUE = 12 }; // inputs

	void SaveCurrentStates();
	void RestoreSavedStates();

	// Parameters are the initial input guesses
	NacelleReactionLoads_Vec CalcOutputs_And_SolveInputs();
	NacelleReactionLoads_Vec CalcOutputs_And_DeriveInputs();

	SerializedVector CalcResidual(const SerializedVector& y, const SerializedVector& u) const;

	NacelleReactionLoads_Vec UpdateAeroDynStates();
	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&) const;
	HubAcc CalculateHubAcc(const Vector3d& nacAcc, const Vector3d& nacRotAcc, const Matrix3d& nacOri, double rotorShaftAcc) const;
	Matrix3d CalculateNacelleOrientation(const Vector3d& nacelleEulerAngles) const;

	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation) const;
	Matrix3d InterpExtrapOrientation(double time, const Matrix3d& orient_1, double time_1, const Matrix3d& orient_2, double time_2) const;
	Vector3d InterpExtrapVector(double time, const Vector3d& vect_1, double time_1, const Vector3d& vect_2, double time_2) const;
	NacelleReactionLoads_Vec CalcNacelleReactionLoads();

	//---------------------
	double time_curr, time_next;
	bool onTempUpdate;
	bool useAddedMass;

	/* AeroDyn stuff */
	AeroDyn_Interface_Wrapper aerodyn;
	AeroDynTurbine::NacelleMotion nacelleMotion;
	NacelleReactionLoads_Vec nacelleReactionLoads; // Stores the most recent load results
	Vector3d nacelleForce, nacelleMoment;

	/* Drivetrain stuff */
	DriveTrain				drivetrain;

	/* Controller stuff */
	MasterController	      mcont;

	/* Caller stuff */
	std::function<void(const double*, const double*, double*, double*)> CalcOutput_callback;
};