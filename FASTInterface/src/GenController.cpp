#include "GenController.h"
#include <math.h>

GenController::GenController(const char* filename) : torqueFunc(filename)
{
	ReadCSVFile(filename);
}

GenController::GenController() noexcept 
{
	maxRatedTorque = 0.0;
}

void GenController::ReadCSVFile(const char* filename)
{
	torqueFunc.LoadCSVFile(filename);

	// the max rated torque is the last entry in the CSV table, so use the biggest rotor speed
	// entry to lookup the last torque.
	maxRatedTorque = torqueFunc.F(torqueFunc.GetMaxSpecX());
}

double GenController::GetTorque(double genSpeed) const
{
	return torqueFunc.F(genSpeed);
}

double GenController::GetMaxRatedTorque() const 
{
	return maxRatedTorque;
}
