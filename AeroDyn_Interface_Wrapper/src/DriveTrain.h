#pragma once

/* Author: Dustin Condon
   
   File: DriveTrain.h

   Description:
   

 */


class DriveTrain {
public:

	struct States
	{
		double acc, vel, theta;
	};

	// By model I mean all the states of the model: rotor and generator shafts
	struct ModelStates {
		States rotor, gen;
	};

	DriveTrain();
	DriveTrain(double initialRotorSpeed, double gearboxratio);

	// updates states from previous time to "time" using results of previous calls to the RK4 step functions
	ModelStates UpdateStates();

	// Updates states from previous time to "time" using RK4 method without using results from RK4 step functions
	// Assumes that the torques are constant for the whole time-step
	ModelStates UpdateStates(double dt, double rotor_torque, double generator_torque);

	// Pass torques at current time; returns temporary states at time + dt/2
	ModelStates K1(double dt, double rotorTorque, double genTorque);

	// Pass torques at time + dt/2 ; returns temporary states at time + dt/2
	ModelStates K2(double dt, double rotorTorque, double genTorque);

	// Pass torques at time + dt/2 ; returns temporary state at time + dt
	ModelStates K3(double dt, double rotorTorque, double genTorque);

	// Pass torques at time + dt   ; returns temporary states at time + dt
	ModelStates K4(double dt, double rotorTorque, double genTorque);  

	void SetInitialRotorSpeed(double);
	void SetInitialGenSpeed(double);
	void SetDampingCoeff(double);
	void SetStiffnessCoeff(double);
	void SetGenMassMomentOfInertia(double);
	void SetRotorMassMomentOfInertia(double);
	void SetGearboxRatio(double);

	ModelStates GetModelStates() const;

	States GetRotorStates() const;
	double GetRotorShaftSpeed() const;  // radians/sec
	double GetRotorShaftTheta() const;  // radians
	double GetRotorShaftAcc()   const;  // radians/sec^2

	States GetGenStates() const;
	double GetGenShaftSpeed() const;    // radians/sec
	double GetGenShaftTheta() const;    // radians
	double GetGenShaftAcc()   const;    // radians/sec^2

private:

	struct ModelStateDeriv {
	  double rotorAcc;
	  double genAcc;
	};

	// By D I mean the different between the value at t and t + dt for some dt
	// Holds the increment in each value for the final RK4 average
	struct DStates {
		double dVel, dTheta;
	};

	// By model I mean all the states of the model: rotor and generator shaft
	struct ModelDStates {
		DStates gen, rotor;
	};

	// returns the accelerations of both shafts given the states
	ModelStateDeriv CalcAccelerations(const ModelStates& states, double torque_rotor, double torque_gen) const;
	
	// The core code of each K function with parameters for their differences
	ModelStates K(int i, double dt, double dState_coeff, const ModelStates& _states, double rotor_torque, double gen_torque);
	
	// Calculates the weighted average of the results of the four K function calls
	ModelDStates CalcWeightedAverage(const ModelDStates s[4]) const;

	// Save the states calculated after each K_() call
	ModelDStates dStates_k[4];
	ModelStates result_k[4];

	double gearbox_ratio;
	double damping_coeff, stiffness_coeff;
	double gen_moi, rotor_moi;              // moment of inertia for generator mass and rotor mass
	ModelStates states;
	double time;
};
