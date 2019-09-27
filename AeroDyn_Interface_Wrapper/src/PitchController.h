#pragma once

#include <float.h>
#include "InputFile.h"

class PitchController : private InputFile
{
public:
	struct Parameters{
		double RefSpd, MaxPit, MinPit, MaxRat, DT, KI, KK, KP;
	};

	PitchController();

	// Reads the integral gain, proportion gain, and gain scheduling csv function the given file
	PitchController(const char*);        

	// 
	PitchController(const Parameters &);

	void ReadParameters(const char* fname);

	void SetParameters(const Parameters&);
	// Doesn't update the persistent variables perminantely
	double GetPitch() const;
	// Does update the persistent variables perminantely
	double CalcPitch(double time, double GenSpeedF);

private:
	double MIN(double, double) const;
	double MAX(double, double) const;

	const double OnePlusEpsilon = 1.0 + FLT_EPSILON;

	double ElapTime;
	double DT = 0.000125;
	double GK;
	double KI = 0.008068634;
	double KK = 0.1099965;
	double KP = 0.01882681;

	double PitRate;
	double PitCom;
	double PitComP, PitComI, PitComT;
	double RefSpd = 122.9096;
	double MaxPit = 1.570796;
	double MinPit = 0.0;
	double MaxRat = 0.1396263;

	double IntSpdErr;
	double SpdErr;

	double LastTime;
	double LastBlPitch;
};

