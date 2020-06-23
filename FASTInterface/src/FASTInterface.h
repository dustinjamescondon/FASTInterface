/*
Author: Dustin Condon
Date: Nov 2019
Description:
	This class models a turbine which uses AeroDyn for its calculation of reaction loads.
	It includes a drive train, generator controller, and pitch controller. The user of this 
	class must provide the nacelle position, velocity, orientation, and angular velocity, as well 
	as inflow velocities.

Initialization:
	Note there are three parts to initialization which must happen in the following order: 
	1) initialize the drive train,
	2) initialize the controllers, 
	3) finally initialize AeroDyn.
	
	There is a choice for which method of control to use: external Bladed-style DLL, or input file 
	specified generator and pitch controllers. The choice must be made when calling the initialization functions.
	If using Bladed-style DLL, use InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch);
	If using input file specified controllers, use InitControllers_InputFiles(const char*)

	There is also a choice for how to initialize the drive train: parameters describing the moment of inertias,
	stiffness, and damping of the drive train model; or the location of a definition file that defines these things
	within it.

	Also, you can initialize the simulation with a constant rotor speed and blade pitch. This takes the place
	of steps 1 and 2 (initializing the drive train and controllers): InitWithConstantRotorSpeedAndPitch(double, double)

Usage during simulation:
	1) SetNacelleMotion(double time, ..., bool isRealStep)
	2) GetBladeNodePositions(...)
	3) SetInflowVelocities(...)
	4) Simulate()

	If this process was done with isRealStep == true, then the turbine's states will be updated to "time".
	However, if isRealStep == false, then the turbine's states remain at the previous time.

	In any case, whether isRealStep is true or false, after UpdateStates has been called, the nacelle reaction
	forces at "time" are returned; also any call to GetForce, GetMoment, GetGeneratorSpeed, GetRotorSpeed, will
	return the value at "time" as well.
*/

#pragma once
#include <memory>
#include <vector>
#include <string>

// Note, this is defined in the project preprocessor section
#ifdef FASTINTERFACE_EXPORTS  
#define DECLDIR __declspec(dllexport)   
#else  
#define DECLDIR __declspec(dllimport)   
#endif  

// Includes AeroDyn, drivetrain model, and external generator and pitch controllers
class FASTInterface
{
public:

	// Structure to hold the results from AeroDyn transformed from the hub coordinate system
	// to the nacelle coordinate system
	struct NacelleReactionLoads {
		double force[3];
		double moment[3];
		double tsr;
		double power;
	};

	DECLDIR FASTInterface();

	DECLDIR ~FASTInterface();

	//-----------------------------------------------------
	// Initialization methods

	// This takes the place of initializing the drive train and controllers
	// \constantRotorSpeed: in rad/sec
	// \constantBladePitch: in rads
	DECLDIR void InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch);

	// Must initialize the drivetrain first
	// \rotorMOI: rotor mass moment of inertia about the rotor shaft
	// \genMOI: generator mass moment of inertia about the generator shaft
	// \stiffness: the stiffness coefficient of the drive train
	// \damping: the damping coefficient of the drive train
	// \gearboxRatio: represented as "1 : \gearboxRatio", where LHS is the rotor shaft speed, and RHS is generator shaft speed
	// \initialRotorSpeed: the initial rotation speed of the rotor shaft in rad/sec
	DECLDIR void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed);

	// Version of drivetrain initialization which uses an input file
	DECLDIR void InitDriveTrain(const std::string& drivetrainDefFile, double initRotorSpeed);

	// Must initialize one of the controller types second
	// (Bladed-style DLL controller initialization)
	// \bladed_dll_fname: the filename and path to the bladed-style DLL
	// \initialBladePitch: The initial blade pitch in radians
	DECLDIR void InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch);

	// Initialize the controllers by input file
	// \inputfile: filename and path to the data file that defines the generator and pitch controllers
	// \initBladePitch: The initial blade pitch in radians
	DECLDIR void InitControllers_InputFile(const std::string& inputfile, double initBladePitch);

	// Must initialize AeroDyn last. This is because AeroDyn's initialization requires the rotor speed 
	// from the drive train and blade pitch command from the controller.
	// \inputFilename: the filename of the AeroDyn input file
	// \fluidDensity: 
	// \kinematicFluidVisc: 
	// \nacellePos: the initial position of the nacelle
	// \nacelleEulerAngles: the initial orientation in Euler angles 
	// \nacelleVel: the initial nacelle velocity
	// \nacelleAngularVel: the initial nacelle angular velocity of the nacelle represented as axis-angle vector
	DECLDIR void InitAeroDyn(
		const std::string& inputFilename,
		const std::string& outputFilename,
		double timestep,
		int numBlades,
		double hubRadius,
		double precone,
		double fluidDensity,
		double kinematicFluidVisc,
		const double nacellePos[3],
		const double nacelleEulerAngles[3],
		const double nacelleVel[3],
		const double nacelleAcc[3],
		const double nacelleAngularVel[3],
		const double nacelleAngularAcc[3]);


	DECLDIR void InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);
	//-----------------------------------------------------------

	// This will simulate from the last simulation time up until the time passed to SetNacelleStates(...)
	// and return the nacelle reaction loads at that time.
	// If isRealStep was false for SetNacelleStates, then this doesn't update states permanently.
	// If false, then this does update states perminantely.
	DECLDIR NacelleReactionLoads Simulate();

	// Returns the blade node postion at the \time which was passed to SetNacelleMotion(...)
	// Sets the pass-by-reference parameters with the blade node positions. Assumes enough space has been allocated.
	DECLDIR void GetBladeNodePositions(std::vector<double>&);

	DECLDIR double GetAerodynamicTorque() const;

	// Returns the reaction force on the nacelle
	DECLDIR void GetNacelleForce(double[3]) const;

	// Returns the moment of the nacelle
	DECLDIR void GetNacelleMoment(double[3]) const;

	// Returns the tip speed ratio
	DECLDIR double GetTSR() const;

	// Get reaction force at hub in hub coordinate system
	DECLDIR void GetHubForce(double[3]) const;

	// Get moment of hub in hub coordinate system
	DECLDIR void GetHubMoment(double[3]) const;

	// Returns the total number of nodes 
	DECLDIR int GetNumNodes() const;

	// Returns the number of blades for the turbine loaded from the AeroDyn input file
	DECLDIR int GetNumBlades() const;

	DECLDIR double GetTurbineDiameter() const;

	// Returns the current blade pitch
	DECLDIR double GetBladePitch() const;

	// Returns the current generator torque
	DECLDIR double GetGeneratorTorque() const;

	// Returns the current generator shaft speed from the drive train
	DECLDIR double GetGeneratorSpeed() const;

	// Returns the current rotor shaft speed from the drive train
	DECLDIR double GetRotorSpeed() const;

	// Returns the current angular displacement of the rotor shaft 
	DECLDIR double GetRotorAngularDisp() const;

	// Returns the current angular displacement of the generator shaft
	DECLDIR double GetGeneratorAngularDisp() const;

	// Begin an update to simulation states to bring them to \time
	// \time: the time to update to
	// \nacellePos: the position of the nacelle
	// \nacelleEulerAngles: the orientation in Euler angles 
	// \nacelleVel: the nacelle velocity
	// \nacelleAngularVel: the nacelle angular velocity of the nacelle represented as axis-angle vector	
	// \isRealStep: if true, the turbines states will be permanently updated to \time; if false, 
	//              the turbines states will only be temporarily updated to \time. In both cases
	//				the reaction forces at /time are reported
	DECLDIR void SetNacelleStates(
		double time,
		const double nacellePos[3],
		const double nacelleEulerAngles[3],
		const double nacelleVel[3],
		const double nacelleAcc[3],
		const double nacelleAngularVel[3],
		const double nacelleAngularAcc[3],
		bool isRealStep = true);

	// Sets the inflow velocities at the \time passed to SetNacelleMotion(...)
	DECLDIR void SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);

private:
	// Pointer to implementation class to hide implementation
	class PImp;
	std::unique_ptr<PImp> p_imp;
};
