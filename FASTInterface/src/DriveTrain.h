#pragma once

/*!
 * @author Dustin Condon
 * @date Aug 2020

 * @brief This class implements a two-mass drivetrain model with a particular generator and rotor mass
 * moment of inertia, shaft stiffness and damping, and gearbox ratio.
 * 
 * The inputs to this simulation module are the rotor and generator torque. The outputs are the shaft displacements, speeds, and accelerations.
 * This has been written to have the same general structure as an openfast module, so that it can be coordinate with them using openfast's 
 * predictor-corrector method. The key parts of how the drivetrain is implemented is that 
 * 1. Advance states integrate the states held in states_curr, integrates them, and assigns the result to states_pred. Then advance states can be called multiple times
 *    without actually changing the states_curr.
 * 2. Advance states uses the inputs at input[CURR] and input[NEXT], i.e. it uses inputs at its current timestep, and inputs at the next timestep to do its calculation,
 *    as is the case with the openfast modules.
*/
class DriveTrain {
public:

	struct States
	{
		double acc, vel, theta;
	};

	// By model I mean all the states_pred of the model: rotor and generator shafts
	struct ModelStates {
		States rotor, gen;
	};

	DriveTrain();
	DriveTrain(double constantRotorSpeed);
	DriveTrain(double initialRotorSpeed, double gearboxratio);

	void Init(double constantRotorSpeed);
	void Init(double initialRotorSpeed, double gearboxRatio, double dampingCoeff, double stiffnessCoeff,
		double rotorMassMOI, double generatorMassMOI);

	//! @brief This saves the current state of the turbine
	void SaveCurrentStates();
	//! @brief This restores the saved state of the turbine
	void RestoreSavedStates();

	//! @brief Uses inputs and calculates new states, storing them in the predicted states 
	void UpdateStates();

	//! @brief Moves the predicted states into the current states
	void CopyStates_Pred_to_Curr();

	void AdvanceInputWindow();

	void SetInputs(double time, double rotorTorque, double generatorTorque);

	// @brief calculates the accelerations and returns states_pred of both shafts
	ModelStates CalcOutput();

	void SetInitialRotorSpeed(double);
	void SetInitialGenSpeed(double);
	void SetDampingCoeff(double);
	void SetStiffnessCoeff(double);
	void SetGenMassMomentOfInertia(double);
	void SetRotorMassMomentOfInertia(double);
	void SetGearboxRatio(double);

	ModelStates GetStates() const;

	double GetInput_RotorTorque() const;
	double GetInput_GenTorque() const;

	States GetRotorStates() const;
	double GetRotorShaftSpeed() const;  // radians/sec
	double GetRotorShaftTheta() const;  // radians
	double GetRotorShaftAcc()   const;  // radians/sec^2

	States GetGenStates() const;
	double GetGenShaftSpeed() const;    // radians/sec
	double GetGenShaftTheta() const;    // radians
	double GetGenShaftAcc()   const;    // radians/sec^2

private:
	//! @brief maps NEXT and CURR to array indices for the time and inputs
	enum {NEXT=0,CURR=1};

	struct Input {
		double rotorTorque, genTorque;
	};

	//! @brief Dynamic is using ODEs; Constant is just using a constant rotor speed
	enum Mode { DYNAMIC, CONSTANT };
	Mode mode;

	ModelStates CalcOutput(const ModelStates& s, const Input& u) const;

	double gearbox_ratio;
	double damping_coeff, stiffness_coeff;
	double gen_moi, rotor_moi;              //! moment of inertia for generator mass and rotor mass
	ModelStates states_pred, states_curr, saved_states;
	double time, saved_time; //! The time that the states_pred are currently at
	double inputTime[2], saved_inputTime[2];
	Input input[2], saved_input[2];
};
