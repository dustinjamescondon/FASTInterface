/*
Description:

This class abstracts the generator and pitch controller. Because we want to have the option of interfacing with a
Bladed-style DLL or just specifying a constant rotor speed, we have this high-level class which provides a common 
interface for both options. */


#pragma once
#include "BladedInterface.h"


class MasterController
{
public:
	MasterController();

	// Initialize the controller with a constant rotor speed and blade pitch
	void Init(double bladePitch);

	// Initialization function for the controllers when using a Bladed-style DLL
	void Init_BladedDLL(int numBlades, const char* bladed_dll_fname, double initBladePitch);

	void UpdateController(double time, double genSpeed, double currBladePitch);

	double GetGeneratorTorqueCommand() const;
	double GetBladePitchCommand() const;

private:
	enum ControllerMode { BLADED_DLL, CONSTANT_SPEED };
	ControllerMode controlMode;

	double bladePitchCommand;

	BladedInterface bladedcont;
};