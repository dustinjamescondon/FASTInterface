#pragma once

#include "CSVSpecFunction.h"

class GenController
{
public:

	GenController(const char* filename);
	GenController() noexcept;

	void LoadCSVFile(const char* filename);

	double GetTorque_CSV(double rotosSpeed) const;
	double GetTorque(double rotorSpeed) const;
	double GetMaxRatedTorque() const noexcept;

private:
	const double VS_RtGnSp = 121.6805;
	const double VS_RtPwr    = 5296610.0;
	const double VS_CtInSp  = 70.16224;
	const double VS_Rgn2Sp  = 91.21091;
	const double VS_Rgn2K = 2.332287;
	const double VS_MaxTq = 47402.91;
	const double VS_SlPc = 10.0;
	const double VS_SySp;    // need to calculate
	const double VS_Slope15; // ^
	const double VS_Slope25; // ^
	const double VS_TrGnSp;  // ^

	CSVSpecFunction torqueFunc; // function of rotor speed
	double maxRatedTorque;
};