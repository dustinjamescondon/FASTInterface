#pragma once

#include "CSVSpecFunction.h"
#include "FASTInterfaceExceptions.h"

class PitchController
{
public:
	PitchController(const char* gainSchedulingFile, double initial_pitch);
	PitchController();

	void LoadGainSchedulingFile(const char* filename, double initial_pitch);

	double Calculate(double time, double genSpeed);
	double GetLastPitchCommand() const;

	void SetProportionGain(double);
	void SetIntegralGain(double);
	void SetTargetGenSpeed(double);

private:
	bool engaged;
	double lastTime;
	double prevTime;              // holds the previous time passed to calculate(...)
	double bias;				  // should be the value of the pitch angle before controller was turned on
	double targetGenSpeed;
	double lastPitchCommand;      // the current blade pitch
	double p_gain;				  // proportion gain
	double i_gain;				  // integral gain
	CSVSpecFunction p_gain_coeff; // proportion gain coefficient, which is a function of blade pitch

	double integral_term;
};