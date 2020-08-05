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

	struct NacelleAccelerations {
		Vector3d acceleration, rotation_acceleration;
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

	inline void GetBladeNodePositions(std::vector<double>& nodePositions) { aerodyn.GetBladeNodePositions(nodePositions); }
	inline AeroDyn_Interface_Wrapper::HubReactionLoads GetHubReactionLoads() const { return aerodyn.GetHubReactionLoads(); }
	inline NacelleReactionLoads_Vec GetNacelleReactionLoads() const { return nacelleReactionLoads_at_global_time_next; }
	inline Vector3d GetNacelleReactionForce() const { return nacelleReactionLoads_at_global_time_next.force; } // TODO could also return nacAccels_at_dvr_time_next
	inline Vector3d GetNacelleReactionMoment() const { return nacelleReactionLoads_at_global_time_next.moment; } // ^
	inline Vector3d GetNacelleAcc() const { return nacelleMotion_at_global_time_next.acceleration;  }
	inline Vector3d GetNacelleAngularAcc() const { return nacelleMotion_at_global_time_next.acceleration; }
	inline double GetPower() const { return nacelleReactionLoads_at_global_time_next.power; }
	inline double GetTSR() const { return nacelleReactionLoads_at_global_time_next.tsr; }
	inline int GetNumNodes() const { return aerodyn.GetNumNodes(); }
	inline int GetNumBlades() const { return aerodyn.GetNumBlades(); }
	inline double GetTurbineDiameter() const { return aerodyn.GetTurbineDiameter();  }
	inline double GetBladePitch() const { return aerodyn.GetBladePitch(); }
	inline double GetGenTorque() const { return mcont.GetGeneratorTorqueCommand(); }
	inline double GetGenSpeed() const { return drivetrain.GetGenShaftSpeed(); }
	inline double GetRotorSpeed() const { return drivetrain.GetRotorShaftSpeed(); }
	inline DriveTrain::States GetRotorShaftState() const { return drivetrain.GetRotorStates(); }
	inline DriveTrain::States GetGenShaftState() const { return drivetrain.GetGenStates(); }

private:
	typedef Eigen::Vector<double,13> SerializedVector;

	enum { Y_DVR_NAC_ACC = 0, Y_DVR_NAC_ROTACC = 3, Y_AD_HUB_FORCE = 6, Y_AD_HUB_MOMENT = 9, Y_DT_ROTOR_ACC = 12 };    // outputs
	enum { U_DVR_NAC_FORCE = 0, U_DVR_NAC_MOMENT = 3, U_AD_HUB_ACC = 6, U_AD_HUB_ROTACC = 9, U_DT_ROTOR_TORQUE = 12 }; // inputs

	void SaveCurrentStates();
	void RestoreSavedStates();

	void SetInputs_Nacelle_At_Global_Time_Next(const NacelleMotion& nm);

	NacelleReactionLoads_Vec AdvanceStates_By_One_Global_Timestep();

	// Parameters are the initial input guesses
	NacelleReactionLoads_Vec CalcOutputs_And_SolveInputs();
	NacelleReactionLoads_Vec CalcOutputs_And_DeriveInputs();

	SerializedVector CalcResidual(const SerializedVector& y, const SerializedVector& u) const;

	HubMotion CalculateHubMotion(const NacelleMotion&, const DriveTrain::States&) const;
	HubAcc CalculateHubAcc(const Vector3d& nacAcc, const Vector3d& nacRotAcc, const Matrix3d& nacOri, double rotorShaftAcc) const;
	Matrix3d CalculateNacelleOrientation(const Vector3d& nacelleEulerAngles) const;

	NacelleMotion InterpolateNacelleMotion_AtNextGlobalTime() const;
	Vector3d TransformHubToNacelle(const Vector3d& v, const Matrix3d& nacelleOrienation, const Matrix3d& hubOrienation) const;

	NacelleReactionLoads_Vec CalcNacelleReactionLoads();

	//---------------------
	/* Common stuff */
	double global_time_curr, global_time_curr_saved; // the AD and DT's previous time
	double global_time_next, global_time_next_saved; // the AD and DT's most recent time
	double global_timestep; // the timestep for AeroDyn and Drivetrain
	bool onTempUpdate;
	bool useAddedMass;
	NacelleReactionLoads_Vec nacelleReactionLoads_at_global_time_next, nacelleReactionLoads_at_global_time_next_saved;
	NacelleReactionLoads_Vec nacelleReactionLoads_at_global_time_curr, nacelleReactionLoads_at_global_time_curr_saved;

	/* AeroDyn stuff */
	AeroDyn_Interface_Wrapper aerodyn;

	/* Drivetrain stuff */
	DriveTrain				drivetrain;

	/* Controller stuff */
	MasterController	      mcont;

	/* Driver program stuff (ProteusDS) */
	NacelleMotion nacelleMotion_at_global_time_next, nacelleMotion_at_global_time_next_saved;
	NacelleMotion nacelleMotion_at_global_time_curr, nacelleMotion_at_global_time_curr_saved;
	NacelleMotion nacelleMotion_at_dvr_time_next, nacelleMotion_at_dvr_time_next_saved;
	NacelleAccelerations nacAccels_at_dvr_time_curr, nacAccels_at_dvr_time_curr_saved; // these states exist at dvr_time_curr
	NacelleAccelerations nacAccels_at_dvr_time_next, nacAccels_at_dvr_time_next_saved; // save the previous result for interpolation

	double dvr_time_curr, dvr_time_curr_saved;
	double dvr_time_next, dvr_time_next_saved;
	// assumes the inputs (the first two params) exists at dvr_time_next, and returns its output (next two params) at dvr_time_next
	std::function<void(const double*, const double*, double*, double*)> CalcOutput_callback;
	// This is a wrapper around CalcOutput_callback with small extra functionality: It interps/extraps the outputs to the appropriate times
	NacelleAccelerations Dvr_CalcOutput(const Vector3d& nacelle_force, const Vector3d& nacelle_moment);
};