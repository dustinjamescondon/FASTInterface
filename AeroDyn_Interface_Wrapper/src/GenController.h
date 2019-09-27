#pragma once

#include "InputFile.h"
#include "CSVSpecFunction.h"

class GenController : private InputFile
{
public:

	GenController(const char* filename);
	GenController() noexcept;

	void ReadCSVFile(const char* filename);
	void ReadParameters(const char* fname);

	double GetTorque_CSV(double rotosSpeed) const;
	double GetTorque(double rotorSpeed) const;
	double GetMaxRatedTorque() const noexcept;

private:
	double VS_RtGnSp = 121.6805;
	double VS_RtPwr    = 5296610.0;
	double VS_CtInSp  = 70.16224;
	double VS_Rgn2Sp  = 91.21091;
	double VS_Rgn2K = 2.332287;
	double VS_MaxTq = 47402.91;
	double VS_SlPc = 10.0;
	double VS_SySp;    // need to calculate
	double VS_Slope15; // ^
	double VS_Slope25; // ^
	double VS_TrGnSp;  // ^

	CSVSpecFunction torqueFunc; // function of rotor speed
	double maxRatedTorque;
};