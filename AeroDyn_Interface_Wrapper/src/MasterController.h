#pragma once

/*
Description:

This class abstracts the generator and pitch controller. Because we want to have the option of interfacing with a
Bladed-style DLL or defining the controller behaviors by CSV files and other parameters contained in an input file,
we have this high-level class which provides a common interface for both options. */

#include "PitchController.h"
#include "GenController.h"
#include "BladedInterface.h"
#include "LowPassFilter.h"

class MasterController
{
public:
	MasterController();
	MasterController(double bladePitch);
	MasterController(const char* bladed_dll_fname);
	MasterController(const char* gen_csv, const char* pitch_param_csv, double cornerFreq, double initGenSpeed);

	// Initialize the controller with a constant rotor speed and blade pitch
	void Init(double bladePitch);

	// Initialization function for the controllers when using a Bladed-style DLL
	void Init(const char* bladed_dll_fname);

	// Initialization function for the controllers when using parameters from CSV and parameter files
	void Init(const char* gen_csv, const char* pitch_params, double cornerFreq, double initGenSpeed);

	void UpdateController(double time, double genSpeed, double currBladePitch);

	double GetGeneratorTorqueCommand() const;
	double GetBladePitchCommand() const;
private:
	enum ControllerMode { BLADED_DLL, CSV_TABLES, CONSTANT_SPEED };
	ControllerMode controlMode;

	double constantBladePitch;
	LowPassFilter genSpeedLPF;
	PitchController pitcont;
	GenController   gencont;

	BladedInterface bladedcont;
};