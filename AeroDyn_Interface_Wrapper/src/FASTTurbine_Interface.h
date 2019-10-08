#pragma once
#include <memory>
#include <vector>

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

	// Structure to hold the results from AeroDyn transformed from the hub coordinate system
	// to the nacelle coordinate system
	struct NacelleReactionForces {
		double force[3];
		double moment[3];
		double tsr;
		double power;
	};

	// Structure to hold the state of the nacelle
	struct NacelleMotion {
		double position[3];
		double velocity[3];
		double eulerAngles[3];
		double angularVel[3];
	};

	DECLDIR FASTTurbineModel();

	DECLDIR ~FASTTurbineModel();

	// This takes the place of initializing the drive train and controllers
	DECLDIR void InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch);

	// Must initialize the drivetrain first
	DECLDIR void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorVel);

	// Must initialize one of the controller types second
	// (Bladed-style DLL controller initialization)
	DECLDIR void InitControllers(const char* blade_dll_fname);

	// (Input file specified generator and pitch controllers)
	DECLDIR void InitControllers(const char* gen_csv, const char* pit_fname, double lpfCornerFreq);

	// Must initialize AeroDyn last. This is becaues AeroDyn's initialization requires the rotor speed 
	// from the drive train and blade pitch command from the controller.
	DECLDIR void InitAeroDyn(
		const char* inputFilename,
		double fluidDensity,
		double kinematicFluidVisc,
		const NacelleMotion&);

	DECLDIR void InitInflows(const std::vector<double>&);

	// Pass the nacelle state at t + dt/2; begins process which will eventually return temporary nacelle reaction forces at t + dt/2
	DECLDIR void Step1(const NacelleMotion&, double time, double dt);
	// Pass the nacelle state at t + dt/2; begins process which will eventually return temporary nacelle reaction forces at t + dt/2
	DECLDIR void Step2(const NacelleMotion&);
	// Pass the nacelle state at t + dt  ; begins process which will eventually return temporary nacelle reaction forces at t + dt
	DECLDIR void Step3(const NacelleMotion&);
	// Pass the actual nacelle state at t + dt  
	DECLDIR void Step4(const NacelleMotion&);

	// Call these after each call to a K 1, 2, 3, & 4
	DECLDIR void GetBladeNodePositions_Tmp(std::vector<double>&);
	DECLDIR void SetInflowVelocities_Tmp(const std::vector<double>&);
	DECLDIR NacelleReactionForces UpdateAeroDynStates_Tmp();

	// Final update functions
	// Pass the actual nacelle state at t + dt
	DECLDIR void CompleteStep(const NacelleMotion& s);
	// Sets the pass-by-reference parameters with the blade node positions
	DECLDIR void GetBladeNodePositions(std::vector<double>&);
	// Sets sets the actual inflow velocities at t + dt
	DECLDIR void SetInflowVelocities(const std::vector<double>&);
	// Permanently updates AeroDyn's states from t to t + dt, returning the reaction forces
	// and other results in the nacelle coordinate system
	DECLDIR NacelleReactionForces UpdateAeroDynStates();

	// Returns the total number of nodes 
	DECLDIR int GetNumNodes() const;
	// Returns the number of blades for the turbine loaded from the AeroDyn input file
	DECLDIR int GetNumBlades() const;
	// Returns the current blade pitch
	DECLDIR double GetBladePitch() const;
	// Returns the current generator torque
	DECLDIR double GetGeneratorTorque() const;
	// Returns the current generator shaft speed from the drive train
	DECLDIR double GetGeneratorSpeed() const;
	// Returns the current rotor shaft speed from the drive train
	DECLDIR double GetRotorSpeed() const;
	
	DECLDIR double GetAerodynamicTorque() const;
	DECLDIR void GetForce(double[3]) const;
	DECLDIR void GetMoment(double[3]) const;

private:
	// Pointer to implementation class to hide implementation
	class PImp;
	std::unique_ptr<PImp> p_imp;
};
