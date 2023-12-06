#pragma once
#include <functional>
#include <memory>
#include <vector>
#include <string>

// Note, this is defined in the project preprocessor section
//#ifdef FASTINTERFACE_EXPORTS  
//#define DECLDIR __declspec(dllexport)   
//#else  
//#define DECLDIR __declspec(dllimport)   
//#endif  
#define DECLDIR 

/*!
 * @author Dustin Condon
 * @date Aug 2020

 * @brief **The main class** used by ProteusDS that models a turbine with a two-mass drivetrain and controller.
 *
 * This class models a turbine which uses AeroDyn for its calculation of reaction loads.
 * It includes a drive train, generator controller, and pitch controller. The user of this
 * class must provide the nacelle position, velocity, acclerations, orientation, angular velocity, angular acceleration,
 * inflow velocities, and inflow accelerations.
 * 
 * This class is just a wrapper for AeroDynTurbine; however this class removes the need for Eigen with this function arguments and return values. Therefore
 * the user of this class doesn't need to include Eigen.
 *
 * Initialization:
 * ---------------
 * Note there are three parts to initialization which must happen in the following order:
 * 1. initialize the drive train,
 * 2. initialize the controllers,
 * 3. finally initialize AeroDyn.
 *
 * The only controller option existing right now is a Bladed-style DLL.
 * Use InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch);
 * 
 * Also, you can initialize the simulation with a constant rotor speed and blade pitch. This takes the place
 * of steps 1 and 2 (initializing the drive train and controllers): InitWithConstantRotorSpeedAndPitch(double, double)
 *
 * Usage order during simulation loop:
 * ----------------------------------
 *  1. SetNacelleStates(double time, ..., bool isTempUpdate)
 *	2. GetBladeNodePositions(...)
 *	3. SetInflows(...)
 *	4. AdvanceStates()
 *
 * If this process was done with isTempUpdate == false, then the turbine's states_pred will be updated to **time**.
 * However, if isTempUpdate == true, then the turbine's states remain at the previous time.
 * @see AeroDyn_Interface_Wrapper for more details about how its implemented via state saving.
 * 
 * In any case, whether isTempUpdate is true or false, after UpdateStates has been called, the nacelle reaction
 * forces at time are returned.
*/
class FASTInterface
{
public:

	/*! 
	 * Structure to hold the results from AeroDyn transformed from the hub coordinate system
	 * to the nacelle coordinate system
	 */
	struct NacelleReactionLoads {
		double force[3];
		double moment[3];
		double tsr;
		double power;
	};

	struct NacelleAccelerations {
		double nacelleAcc[3];
		double nacelleRotationAcc[3];
	};

	struct TurbineOutput {
		double nacelleForce[3];
		double nacelleMoment[3];
	};

	DECLDIR FASTInterface();

	DECLDIR ~FASTInterface();

	//-----------------------------------------------------
	// Initialization methods
	//.....................................................
	/*!
	* This takes the place of initializing the drive train and controllers.
	* @param constantRotorSpeed: in rad/sec
	* @param constantBladePitch: in rads
	*/
	DECLDIR void InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch);
	/*!
	* Must initialize the drivetrain first
	* @param rotorMOI: rotor mass moment of inertia about the rotor shaft
	* @param genMOI: generator mass moment of inertia about the generator shaft
	* @param stiffness: the stiffness coefficient of the drive train
	* @param damping: the damping coefficient of the drive train
	* @param gearboxRatio: represented as "1 : \gearboxRatio", where LHS is the rotor shaft speed, and RHS is generator shaft speed
	* @param initialRotorSpeed: the initial rotation speed of the rotor shaft in rad/sec
	*/
	DECLDIR void InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed);

	/*!
	* Must initialize one of the controller types second
	* (Bladed-style DLL controller initialization)
	* \bladed_dll_fname: the filename and path to the bladed-style DLL
	* \initialBladePitch: The initial blade pitch in radians
	*/
	DECLDIR void InitControllers_BladedDLL(int numBlades, const std::string& bladed_dll_fname, double initialBladePitch);

	/*!
	 * Must initialize AeroDyn last. This is because AeroDyn's initialization requires the rotor speed 
	 * from the drive train and blade pitch command from the controller.
	 * @param inputFilename the filename of the AeroDyn input file
	 * @param outputFilename the filename where the resulting AeroDyn output file should go
	 * @param fluidDensity density of the surrounding fluid (kg/m^3)
	 * @param kinematicFluidVisc  
	 * @param nacellePos the initial position of the nacelle
	 * @param nacelleEulerAngles the initial orientation in Euler angles 
	 * @param nacelleVel the initial nacelle velocity
	 * @param nacelleAngularVel the initial nacelle angular velocity of the nacelle represented as axis-angle vector
	 */
	DECLDIR void InitAeroDyn(
		const std::string& inputFilename,
		const std::string& outputFilename,
		bool useAddedMass,
		double coeffAddedMass,
		double timestep,
		int numBlades,
		double hubRadius,
		double precone,
		const double nacellePos[3],
		const double nacelleEulerAngles[3],
		const double nacelleVel[3],
		const double nacelleAcc[3],
		const double nacelleAngularVel[3],
		const double nacelleAngularAcc[3]);


	DECLDIR void InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);
	//-----------------------------------------------------------
	// Get methods
	//...........................................................
	/*!
	* Returns the blade node postion at the \time_act which was passed to SetNacelleMotion(...)
	* Sets the pass-by-reference parameters with the blade node positions. Assumes enough space has been allocated.
	*/
	DECLDIR void GetBladeNodePositions(std::vector<double>&);

	DECLDIR double GetAerodynamicTorque() const;

	/*! Returns the reaction force on the nacelle in the nacelle coordinate system */
	DECLDIR void GetNacelleForce(double[3]) const;

	/*! Returns the moment of the nacelle in the nacelle coordinate system */
	DECLDIR void GetNacelleMoment(double[3]) const;

	/*! When using added mass, this returns the solved-for nacelle linear acceleration;
	 * when not using added mass, this just returns the linear acceleration passed to SetNacelleStates
	*/
	DECLDIR void GetNacelleAcc(double[3]) const;

	/*! When using added mass, this returns the solved-for nacelle angular acceleration
	 * when not using added mass, this just returns the angular acceleration passed to SetNacelleStates(...)
	 */
	DECLDIR void GetNacelleAngularAcc(double[3]) const;

	/*! Returns the tip speed ratio */
	DECLDIR double GetTSR() const;

	/*! Get reaction force at hub in hub coordinate system */
	DECLDIR void GetHubForce(double[3]) const;

	/*! Get moment of hub in hub coordinate system */
	DECLDIR void GetHubMoment(double[3]) const;

	/*! Returns the total number of nodes */
	DECLDIR int GetNumNodes() const;

	/*! Returns the number of blades for the turbine loaded from the AeroDyn input file */
	DECLDIR int GetNumBlades() const;

	DECLDIR double GetTurbineDiameter() const;

	/*! Returns the current blade pitch */
	DECLDIR double GetBladePitch() const;

	/*! Returns the current generator torque */
	DECLDIR double GetGeneratorTorque() const;

	/*! Returns the current generator shaft speed from the drive train */
	DECLDIR double GetGeneratorSpeed() const;

	/*! Returns the current rotor shaft speed from the drive train */
	DECLDIR double GetRotorSpeed() const;

	/*! Returns the current angular displacement of the rotor shaft */
	DECLDIR double GetRotorAngularDisp() const;

	/*! Returns the current angular displacement of the generator shaft */
	DECLDIR double GetGeneratorAngularDisp() const;

	//-----------------------------------------------------------------
	// Set methods
	//.................................................................
	/*!
	* @brief Begin an update to simulation states_pred to bring them to time_act
	* @param time: the time to update to
	* @param nacellePos: the position of the nacelle
	* @param nacelleEulerAngles: the orientation in Euler angles 
	* @param nacelleVel: the nacelle linear velocity
	* @param nacelleAcc: the nacelle linear acceleration
	* @param nacelleAngularVel: the nacelle angular velocity as an axis-angle vector
	* @param nacelleAngularAcc: the nacelle angular acceleration as an axis-angle vector
	* @param isTempUpdate: if false, the turbines states_pred will be permanently updated to time_act; if true, 
	*                     the turbines states_pred will only be temporarily updated to time_act. In both cases
	*			          the reaction forces at time_act are reported
	*/
	DECLDIR void SetNacelleStates(
		double time,
		const double nacellePos[3],
		const double nacelleEulerAngles[3],
		const double nacelleVel[3],
		const double nacelleAcc[3],
		const double nacelleAngularVel[3],
		const double nacelleAngularAcc[3],
		bool isTempUpdate = false);
	/*!
	 * @brief Sets the inflow velocities and accelerations at the time passed to SetNacelleStates
	 *
	 * @param inflowVel: the serialized inflow velocity vectors for each node
	 * @param inflowAcc: the serialized inflow acceleration vectors for each node (not used when added mass effects are disabled)
	 */
	DECLDIR void SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);

	/*!
	 * @brief Sets the **CalcAccelerations** function that will be called within the direct feed through solver when added
	 * mass is enabled. **Note:** this doesn't need to be set if added mass is not enabled.
	 *
	 * @param CalcAccelerations A function pointer that takes in the nacelle force and moment (first two params),
	 *        and returns nacelle linear and angular accelerations (last two parameters).
	 */
	DECLDIR void SetCalcOutputCallback(std::function<void(const double*, const double*, double*, double*)> CalcAccelerations);

	//-----------------------------------------------------------------
	// Other methods
	//.................................................................

	/*!
	* This will simulate from the last simulation time up until the time_act passed to SetNacelleStates(...)
	* and return the nacelle reaction loads at that time_act.
	* If isTempStep was true for SetNacelleStates, then this doesn't update states_pred permanently;
	* if false, then this does update states_pred permanentely.
	*/
	DECLDIR NacelleReactionLoads AdvanceStates();

private:
	/*! Pointer to implementation class to hide implementation */
	class PImp;
	std::unique_ptr<PImp> p_imp;
};
