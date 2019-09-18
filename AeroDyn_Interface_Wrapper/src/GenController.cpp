#include "GenController.h"

GenController::GenController(const char* filename) : torqueFunc(filename)
{
	loadCSVFile(filename);
}

GenController::GenController() noexcept
{
	maxRatedTorque = 0.0;
}

void GenController::loadCSVFile(const char* filename)
{
	torqueFunc.loadCSVFile(filename);

	// the max rated torque is the last entry in the CSV table, so use the biggest rotor speed
	// entry to lookup the last torque.
	maxRatedTorque = torqueFunc.F(torqueFunc.getMaxSpecX());
}


double GenController::getTorque(double rotorSpeed) const
{
	return torqueFunc.F(rotorSpeed);
}

double GenController::getMaxRatedTorque() const noexcept
{
	return maxRatedTorque;
}
