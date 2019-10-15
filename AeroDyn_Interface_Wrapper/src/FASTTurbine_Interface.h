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

// Includes drivetrain model and external generator and pitch controller
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
	// \bladed_dll_fname: the filename and path to the bladed-style DLL
	DECLDIR void InitControllers(const char* bladed_dll_fname);

	// (Input file specified generator and pitch controllers)
	// \gen_csv: filename and path to the csv data file that defines the generator controller
	// \pit_fname:  the filename for the pitch controller input file
	// \lpfCornerFreq: the constant for the low-pass filter 
	DECLDIR void InitControllers(const char* gen_csv, const char* pit_fname, double lpfCornerFreq);

	// Must initialize AeroDyn last. This is becaues AeroDyn's initialization requires the rotor speed 
	// from the drive train and blade pitch command from the controller.
	DECLDIR void InitAeroDyn(
		const char* inputFilename,
		double fluidDensity,
		double kinematicFluidVisc,
		const NacelleMotion&);

	DECLDIR void InitInflows(const std::vector<double>&);

	// Begin a simulation step up to time
	// \time: the time to update to
	// \nacelleMotion: the state of the nacelle at \time
	// \isRealStep: if true, the turbines states will be permanently updated to \time; if false, 
	//              the turbines states will only be temporarily updated to \time so the reaction forces
	//              at \time can be reported
	DECLDIR void SetNacelleStates(double time, const NacelleMotion&, bool isRealStep = true);

	// Sets the pass-by-reference parameters with the blade node positions
	DECLDIR void GetBladeNodePositions(std::vector<double>&);

	// Sets sets the inflow velocities at t + dt
	DECLDIR void SetInflowVelocities(const std::vector<double>&);

	// If isRealStep was true for SetNacelleStates, then this doesn't update states permanently
	DECLDIR NacelleReactionForces UpdateStates();

	// Returns these values based on the last call to UpdateStates
	DECLDIR double GetAerodynamicTorque() const;

	DECLDIR void GetForce(double[3]) const;

	DECLDIR void GetMoment(double[3]) const;

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

private:
	// Pointer to implementation class to hide implementation
	class PImp;
	std::unique_ptr<PImp> p_imp;
};