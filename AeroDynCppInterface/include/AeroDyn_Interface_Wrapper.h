#pragma once

#include <vector>
#include "FASTInterfaceExceptions.h"
#include "Eigen/SparseCore"

using Eigen::Matrix3d;
using Eigen::Vector3d;
/*!
 * @author Dustin Condon
 * @date Aug 2020

 * @brief This is a wrapper for AeroDyn_Interface, which is written in FORTRAN. Its main purpose is to provide Aerodyn with inputs,
 * for which it can return reaction loads on the hub. It also changes from ProteusDS' z-positive-down coordinate
 * system to AeroDyn's z-positive-up coordinate system.
 * 
 * This interface has two features:
 * 
 * 1.The interface allows one to save all of AeroDyn's states and restore them by calling SaveCurrentStates and RestoreSavedStates, respectively.
 * This is used to take temporary updates when the driver program is calling this AeroDyn interface during the intermediate steps
 * of its explicit integrator. The process is covered in the Fall 2019 work term report, and is refered to as "modified loose coupling".
 * The implementation on AeroDyn_Interface's end has since changed to this "save and restore states" method since that report, but the basic idea
 * of the method is the same.
 * 
 * 2.The other feature is that the interface doesn't overwrite the existing states when UpdateStates is called. Instead it keeps two states, "current" and "predicted". When
 * UpdateStates is called, the "current" states are integrated, and the result is saved in "predicted", so "current" remains unchanged. This allows
 * for this module to be coordinate with others using the same predictor-corrector method used in OpenFAST. So UpdateStates can be called multiple times
 * using different inputs until the "predicted" states have been corrected enough times. Then the "predicted" states can be copied into the "current" states
 * by calling CopyStates_Pred_to_Curr.
 * 
 * Example:
 * --------
 * 
 * The expected order of function calls is follows (this example is without using any corrector steps)
 * @code
 * //----------- initialization --------------
 * InitAerodyn(...);           // passing driver input file name, hub kinematics, and blade pitch; receiving the total number of nodes used
 * GetBladeNodePositions();    // receiving the positions of the blade nodes in global coordinate system
 * InitInputs_Inflow(...);     // passing the inflows at time_act=0
 * //----------- simulation loop -------------
 * Set_Inputs_Hub(...);        // passing hubstate at time_act = 0
 * GetBladeNodePositions();    // ...
 * Set_Inputs_Inflow(...);     // ...
 * UpdateStates();             // passing the inflow at time_act = 0
 * CopyStates_Pred_to_Curr();  // ...
 * Advance_InputWindow();      // ... 
 * //---next timestep---
 * Set_Inputs_Hub(...);        // passing hubstate at time_act = t_1
 * GetBladeNodePositions();    // ...
 * Set_Inputs_Inflow(...);     //
 * UpdateStates();             // passing the inflow at time_act = t_1
 * CopyState_Pred_to_Curr();   //
 * Advance_InputWindow();      //
 *                             // etc...
 * @endcode
*/
class AeroDyn_Interface_Wrapper {
public:

	struct HubReactionLoads {
		Vector3d force;
		Vector3d moment;
		double power;
		double tsr;
	};

	AeroDyn_Interface_Wrapper();

	~AeroDyn_Interface_Wrapper();

	/*! @brief Initializes AeroDyn by loading input files and setting initial hub state 
	 * 
	 * @param[in] inputFilename filename (including path) of the AeroDyn input file
	 * @param[in] outputFilename root name of the output file to be generated
	 * @param[in] timestep the timestep that AeroDyn will take (sec)
	 * @param[in] numBlades the number of blades to use in the simulation
	 * @param[in] hubRadius the distance from hub center to blade root
	 * @param[in] precone the precone angle of the blades (radians)
	 * @param[in] have AeroDyn include added mass effects 
	 * @param[in] coefficient of added mass (only used when useAddedMass is true)
	 * @param[in] hubPosition global position of the hub center in ProteusDS's coordinate system
	 * @param[in] hubOrientation 3x3 orientation matrix of the hub
	 * @param[in] hubVelocity (metres/sec)
	 * @param[in] hubAcceleration (metres/sec^2)
	 * @param[in] hubRotationalVel axis-angle form in global coordinate system
	 * @param[in] hubRotationalAcc axis-angle form in global coordinate system	
	 * @param[in] bladePitch radians
	*/
	void InitAerodyn(
		const char* inputFilename,		
		const char* outputFilename,		
		double timestep,                 
		int numBlades,                   
		double hubRadius,                
		double precone,                  
		bool useAddedMass,               
		double coeffAddedMass,           
		const Vector3d& hubPosition,	 
		const Matrix3d& hubOrientation,  
		const Vector3d& hubVel,	         
		const Vector3d& hubAcc,		     
		const Vector3d& hubRotationalVel,
		const Vector3d& hubRotationalAcc,
		double bladePitch);              

	/*!----------------------------------------------------------------------------------------------
	@brief Initializes the inflows. Note, inflow velocities are in global coordinate system

	* The format expected is (in global coordinate system)
	* inflows[0] inflow velocity in x direction at node 0
	*     ...[1] inflow velocity in y direction at node 0
	*     ...[2] inflow velocity in z direction at node 0
	*     ...[3] inflow velocity in x direction at node 1
	*	  ... etc

	* @param inflowVel: a serialized vector of the inflow velocity at each blade node
	* @param inflowAcc: a serialized vector of the inflow acceleration at each blade node (not used when added mass is not enabled),
	*                   so could just pass the inflowVel vector twice in that case.

	* @todo maybe make version that doesn't require the acceleration for when added mass isn't enabled
	*/
	void InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);


	void SaveCurrentStates();

	void RestoreSavedStates();

	/*! @brief Updates the hub motion variables, changing the positions of the nodes accordingly.
	 * 
	 * **Note:** call this before calling GetBladeNodePositions because this will inform where the node 
	 * positions are.
	 * @param time The time the inputs describe (sec)
	 * @param hubPosition position of the hub in global coordinate system (meters)
	 * @param hubOrientation orientation matrix in ProteusDS' z-positive-down coordinate system
	 * @param hubVel velocity of the hub in ProteusDS' global coordinate system (meters/sec)
	 * @param hubAcc acceleration of the hub in ProteusDS' global coordinate system (m/sec^2)
	 * @param hubAngularVel angular velocity of the hub in ProteusDS' global coordinate system (axis-angle)
	 * @param hubAngularAccangular angular velocity of the hub in ProteusDS' global coordinate system (axis-angle)
	 * @param bladePitch (radians)
	 * @warning the **time** parameter should always be time_curr + timestep. Any interpolation/extrapolation should be done outside this
	 *  function.
	 */
	void Set_Inputs_Hub(
		double time,
		const Vector3d& hubPosition,
		const Matrix3d& hubOrientation,
		const Vector3d& hubVel,
		const Vector3d& hubAcc,    
		const Vector3d& hubAngularVel,
		const Vector3d& hubAngularAcc,
		double bladePitch);

	/*! 
	 * @brief Sets the acceleration inputs only.

	 * The expected use of this function is to perturb the components indivually in order to calculate the partial derivatives of the residual function
	 * w.r.t. acceleration inputs, which is a composite function of CalcOutput function 
	 */
	void Set_Inputs_HubAcceleration(
		const Vector3d& hubAcc,
		const Vector3d& hubAngularAcc);

	/*!------------------------------------------------------------------------------------
	* @brief Sets the flow velocity and acceleration at the next time-step. Note, inflow velocities are in global coordinate system

	* The format expected is (in global coordinate system)
	* inflows[0] inflow velocity in x direction at node 0
	*     ...[1] inflow velocity in y direction at node 0
	*     ...[2] inflow velocity in z direction at node 0
	*     ...[3] inflow velocity in x direction at node 1
	*	  ... etc
	* @param inflowVel: a serialized vector of the inflow velocity at each blade node
	* @param inflowAcc: a serialized vector of the inflow acceleration at each blade node; not used when added mass is not enabled,
	*                   so could just pass the inflowVel vector twice in that case.

	* @todo maybe make version that doesn't require the acceleration for when added mass isn't enabled
	*/
	void Set_Inputs_Inflow(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc);

	/*!
	 * @brief Calculates the hub reaction loads 
	 *
	 * **Note:** Should be called after either:
	 * 1. Updating the states, or
	 * 2. Changing the inputs at global_time_next
	 */
	HubReactionLoads CalcOutput();

	/*!
	 * @brief Returns the current outputs
	 * **Note:** the values returned by this are part of what is saved when SaveCurrentStates is called, so if this is 
	 * called after calling RestoreSavedStates, then the outputs will be from those saved states.
	*/
	HubReactionLoads GetOutput() const;

	/*!
	 * @brief Cycles the two inputs: curr_input=next_input
	 */
	void Advance_InputWindow();

	/*!
	* @brief Returns the acceleration input at the next timestep
	* 
	* **Note:** used in the input solver in the coordinating class
	* 
	* @return acceleration at next timestep of the hub in the global coordinate system.
	*/
	Vector3d GetInput_HubAcc() const { return input.hubAcc; }
	Vector3d GetInput_HubRotAcc() const { return input.hubRotAcc; }
	Matrix3d GetInput_HubOrient() const { return input.hubOrient; }

	/*!
	* @brief 
	* Once updateHubState has been called, we call this to get where those hub kinematics put the blade nodes
	* 
	* The format expected is the same as initInflows. So indices 0,1,2 correspond to the x,y,z
	* position of node 0; and inflow indices 0,1,2 represent the x,y,z inflow velocity for that
	* node.
	*/
	void GetBladeNodePositions(
		std::vector<double>& bladeNodePositions);

	/*! @brief Returns the total number of nodes (call after InitAerodyn) */
	int GetNumNodes() const;

	/*! @brief Returns the number of blades (call after InitAerodyn) */
	int GetNumBlades() const;

	/*! @brief Returns the diameter of the turbine (call after InitAeroDyn) */
	double GetTurbineDiameter() const;
	
	/*! @brief Updates from the current states at time_curr to time_next, and stores the new states in the predicted states (in the FORTRAN layer) */
	void UpdateStates();

	/*! @brief Wrapper to the FORTRAN subroutine of the same name (see AeroDyn_Interface project) */
	void CopyStates_Pred_to_Curr();

	/*! @brief Prints to the output file the one-line results from the last call to CalcOutput */
	void PrintOutputLine();
	
	HubReactionLoads GetHubReactionLoads() const;

	/*! @brief Returns the TSR of the current outputs */
	double GetTSR() const;

	/*! @brief Returns the torque resulting from the current outputs */
	double GetTorque() const;

	/*! @brief Returns the force resulting from the current outputs */
	Vector3d GetForce() const;

	/*! @brief Returns the moment resulting from the current outputs */
	Vector3d GetMoment() const;

	/*! @brief Returns the last actual pitch value */
	double GetBladePitch() const;

private:

	struct Input {
		Vector3d hubPos, hubVel, hubRotVel, hubAcc, hubRotAcc;
		Matrix3d hubOrient;
		double bladePitch;
	};

	/*! @brief Transforms a vector expressed in a z-positive-down coordinate system, to a z-positive-up coordinate system */
	Vector3d Transform_PDStoAD(const Vector3d& v) const;

	/*! @brief Transforms a vector expressed in a z-positive-up coordinate system, to a z-positive-down coordinate system */
	Vector3d Transform_ADtoPDS(const Vector3d& v) const;

	/*! @brief Transforms the local to global orientation matrix from z-down to z-up coordinate system */
	Matrix3d TransformOrientation(const Matrix3d& orientation) const;

	/*! updates aerodynInflows with transformed pdsInflows */
	void SaveInflows(const std::vector<double>& pdsInflowVel, const std::vector<double>& pdsInflowAcc);

	/*! the turbine instance pointer for the current instance of the class (points to a memory location allocated by the
	 * layer of the project; the data pointed to is a FORTRAN type) 
	 * @see FORTRAN project AeroDyn_Interface */
	void* simulationInstance;

	//! Saved reaction loads from last call to UpdateStates
	HubReactionLoads hubReactionLoads, hubReactionLoads_saved; 

	Input input, input_saved;
	std::vector<double> aerodynInflowVel, aerodynInflowVel_saved; //! holds the flow velocities in AeroDyn's coordinate system 
	std::vector<double> aerodynInflowAcc, aerodynInflowAcc_saved; //! holds the flow accelerations in AeroDyn's coordinate system 


	double turbineDiameter;
	double pitch, pitch_saved;
	int totalNodes;
	int nBlades;
	int nNodes; /*! number of nodes per blade */
};
