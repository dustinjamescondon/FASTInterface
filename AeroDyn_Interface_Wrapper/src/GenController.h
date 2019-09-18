#pragma once

#include "CSVSpecFunction.h"

class GenController
{
public:
	GenController(const char* filename);
	GenController() noexcept;

	void loadCSVFile(const char* filename);

	double getTorque(double rotorSpeed) const;
	double getMaxRatedTorque() const noexcept;

private:
	CSVSpecFunction torqueFunc; // function of rotor speed
	double maxRatedTorque;
};