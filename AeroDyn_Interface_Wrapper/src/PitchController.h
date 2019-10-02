#pragma once

#include "CSVSpecFunction.h"
#include "ControllerExceptions.h"

class PitchController
{
public:
	PitchController(const char* filename, double initial_pitch);
	PitchController();

	void LoadInputFile(const char* filename, double initial_pitch);

	double Calculate(double time, double genSpeed);
	double GetLastPitchCommand() const;

private:
	// Reads in the value for p_gain, i_gain, and the function defining p_gain_coeff from a CSV table
	// contained within the file.
	void ReadInputFile(const char*);

	// Reads the next non-comment line of the input file, makes sure it matches the label,
	// and if so return the value. Throws an exception if the label is incorrect, or the 
	// value is invalidly formatted.
	double ReadDouble(std::ifstream&, const char* label) const;

	// 
	std::string GetNextNonCommentLine(std::ifstream& fin) const;

	// Checks if the string can be converted to a double (doesn't check for scientific notation)
	bool isNumber(const std::string& s) const;

	bool engaged;
	double lastTime;
	double prevTime;              // holds the previous time passed to calculate(...)
	double bias;				  // should be the value of the pitch angle before controller was turned on
	double targetGenSpeed;
	double lastPitchCommand;      // the current blade pitch
	double p_gain;				  // proportion gain
	double i_gain;				  // integral gain
	CSVSpecFunction p_gain_coeff; // proportion gain coefficient, which is a function of blade pitch

	double integral_term;
};



/*
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
*/

