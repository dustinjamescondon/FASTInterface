#include "PitchController.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

PitchController::PitchController(const char* filename, double initial_pitch)
	: p_gain_coeff()
{
	LoadGainSchedulingFile(filename, initial_pitch);

	lastTime = 0.0;
}

PitchController::PitchController()
	: p_gain_coeff()
{
	prevTime = bias = p_gain = i_gain = 0.0;
	integral_term = lastTime = 0;
}

void PitchController::LoadGainSchedulingFile(const char* filename, double initialPitch)
{
	bias = initialPitch;
	p_gain_coeff.LoadCSVFile(filename);
}

// Calculates the suggested output of our pitch variable in order for our process variable get closer
// to the set point.
double PitchController::Calculate(double time, double genSpeed)
{
	double dt = time - lastTime;

	// get p_gain_coeff from lookup table
	double p_gain_coeff_value = p_gain_coeff.F(genSpeed);

	// calculate the error of our input
	double error = targetGenSpeed - genSpeed;

	// update the integral term using the integral of our error over 0 to dt
	integral_term += dt * error;

	// calculate and return the new suggested pitch angle
	double pitchCommand = bias + (error * p_gain * p_gain_coeff_value) + (i_gain * integral_term);

	lastPitchCommand = pitchCommand;

	lastTime = time;

	return pitchCommand;
}

double PitchController::GetLastPitchCommand() const 
{
	return lastPitchCommand;
}


void PitchController::SetProportionGain(double x)
{
	p_gain = x;
}

void PitchController::SetIntegralGain(double x)
{
	i_gain = x;
}