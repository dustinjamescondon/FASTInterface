/*
File: GenController.h

Author: Dustin Condon
		dustincondon@uvic.ca

Description:
	This class representing a generator controller simply reads from a file containing a lookup table
	that maps a generator shaft speed to a generator torque. The file's first line should just be the 
	column labels (it is ignored), and the remaining lines are delimiter-seperated values. The first column is 
	the generator speed; the second column is the generator torque.
*/

#pragma once

#include "CSVSpecFunction.h"
#include "FASTTurbineExceptions.h"

class GenController
{
public:

	GenController(const char* filename);
	GenController() noexcept;

	void ReadCSVFile(const char* filename);

	double GetTorque(double rotosSpeed) const;
	double GetMaxRatedTorque() const;

private:

	CSVSpecFunction torqueFunc; // function of rotor speed
	double maxRatedTorque;
};