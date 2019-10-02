#include "PitchController.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

PitchController::PitchController(const char* filename, double initial_pitch)
	: p_gain_coeff()
{
	LoadInputFile(filename, initial_pitch);

	lastTime = 0.0;
}

PitchController::PitchController()
	: p_gain_coeff()
{
	prevTime = bias = p_gain = i_gain = 0.0;
	integral_term = lastTime = 0;
}

void PitchController::LoadInputFile(const char* filename, double initialPitch)
{
	bias = initialPitch;
	ReadInputFile(filename);
}

// Calculates the suggested output of our pitch variable in order for our process variable get closer
// to the set point.
double PitchController::Calculate(double time, double genSpeed)
{
	double dt = time - lastTime;

	// get p_gain_coeff from lookup table
	double p_gain_coeff_value = p_gain_coeff.F(genSpeed);

	// calculate the error of our input
	double error = targetGenSpeed - genSpeed;

	// update the integral term using the integral of our error over 0 to dt
	integral_term += dt * error;

	// calculate and return the new suggested pitch angle
	double pitchCommand = bias + (error * p_gain * p_gain_coeff_value) + (i_gain * integral_term);

	lastPitchCommand = pitchCommand;

	lastTime = time;

	return pitchCommand;
}

double PitchController::GetLastPitchCommand() const 
{
	return lastPitchCommand;
}


// Ignores lines beginning with '!'
void PitchController::ReadInputFile(const char* filename)
{
	std::ifstream fin(filename);

	if (!fin.is_open()) {
		std::string errMsg = std::string("Couldn't open PI Controller input file: ") + std::string(filename);
		throw FileNotFoundException(errMsg);
	}

	// Throws exception if input file doesn't contain this entry at this point
	targetGenSpeed = ReadDouble(fin, "TargetGenSpeed");

	// Throws exception if input file doesn't contain this entry at this point
	p_gain = ReadDouble(fin, "ProportionGain");

	// Throws exception if input file doesn't contain this entry at this point
	i_gain = ReadDouble(fin, "IntegralGain");

	// get and ignore the label portion of the CSV table for proportion gain coefficent
	GetNextNonCommentLine(fin);

	// Now we should be at the first line of the CSV part of the file.
	// Load the CSV table that specifies the proportion gain coefficient function
	p_gain_coeff.readCSVFile(fin);

	fin.close();
}

// Reads the next non-comment line, converts it to a number, 
// and assigns it to the passed-by-reference parameter, value. If the label doesn't match
// what in the input file at this point, or the read-in value is not a valid 
// number, then it returns false; but otherwise returns true.
double PitchController::ReadDouble(std::ifstream& fin, const char* label) const
{
	std::string line = GetNextNonCommentLine(fin);

	std::stringstream linestream(line);
	std::string token;

	// get LHS of assignment
	std::getline(linestream, token, '=');
	if (token != label) {
		std::string errMsg = std::string("Expected ") + std::string(label) +
			std::string(" in input file but got ") + std::string(token);
		throw FileContentsException(errMsg);
	}

	// get RHS of assignment
	std::getline(linestream, token);
	if (!isNumber(token)) {
		std::string errMsg = ("Expected a decimal number but got ") + std::string(token);
		throw FileContentsException(errMsg);
	}

	return std::stod(token.c_str());
}

std::string PitchController::GetNextNonCommentLine(std::ifstream& fin) const
{
	// throw away every comment line until we reach a non-comment line, and then return it
	std::string line;
	while (std::getline(fin, line, '\n')) {
		if (line.front() != '!')
			break;
	}

	return line;
}

// Note, will return false for numbers written in scientific notation, e.g.  "1.0e-06"
bool PitchController::isNumber(const std::string& s) const
{
	bool decimalFound = false;
	for (std::string::const_iterator it = s.begin(); it != s.end(); it++) {
		// if this character is a decimal
		if ((*it) == '.') {
			// if a decimal has already been found, then there are two decimals
			// so this is not a valid number
			if (decimalFound) {
				return false;
			}
			decimalFound = true;
		}

		// otherwise, if this character isn't a valid digit
		else if (!std::isdigit(*it)) {
			return false;
		}
	}
	return true;
}


/*
PitchController::PitchController()
{
	// 
	ElapTime = GK = PitRate = PitCom = 0.0;
	PitComP = PitComI = PitComT = LastBlPitch = 0.0;
	IntSpdErr = SpdErr = LastTime = 0.0;
}

PitchController::PitchController(const char* fname) : InputFile()
{
	// Set variables to zero
	ElapTime = GK = PitRate = PitCom = 0.0;
	PitComP = PitComI = PitComT = LastBlPitch = 0.0;
	IntSpdErr = SpdErr = LastTime = 0.0;

	ReadParameters(fname);
}

// Should do some checking
void PitchController::SetParameters(const Parameters& p)
{
	DT = p.DT;
	KI = p.KI;
	KK = p.KK;
	KP = p.KP;
	RefSpd = p.RefSpd;
	MaxPit = p.MaxPit;
	MinPit = p.MinPit;
	MaxRat = p.MaxRat;
}

void PitchController::ReadParameters(const char* fname) 
{
	LoadFile(fname);
	DT = ReadDouble("DT");
	KI = ReadDouble("KI");
	KK = ReadDouble("KK");
	KP = ReadDouble("KP");
	RefSpd = ReadDouble("RefSpd");
	MaxPit = ReadDouble("MaxPit");
	MinPit = ReadDouble("MinPit");
	MaxRat = ReadDouble("MaxRat");
	CloseFile();
}

// Code adapted from NREL's DISCON.f90 for 5MW OC3 turbine
// TODO implement pitch desaturation
double PitchController::CalcPitch(double Time, double GenSpeedF)
{
	double ElapTime = Time - LastTime;

	if ((Time * OnePlusEpsilon - LastTime) >= DT) {

		GK = 1.0 / (1.0 + PitCom / KK);

		SpdErr = GenSpeedF - RefSpd;
		IntSpdErr = IntSpdErr + SpdErr * ElapTime;
		IntSpdErr = MIN(MAX(IntSpdErr, MinPit / (GK * KI)), MaxPit / (GK*KI));

		PitComP = GK * KP * SpdErr;
		PitComI = GK * KI * IntSpdErr;

		PitComT = PitComP + PitComI;
		PitComT = MIN(MAX(PitComT, MinPit), MaxPit);

		PitRate = (PitComT - LastBlPitch) / ElapTime;
		PitRate = MIN(MAX(PitRate, -MaxRat), MaxRat);

		PitCom = LastBlPitch + PitRate * ElapTime;
		PitCom = MIN(MAX(PitCom, MinPit), MaxPit);

		LastTime = Time;
		LastBlPitch = PitCom;
	}
	
	return PitCom;
}

double PitchController::GetPitch() const
{
	return LastBlPitch;
}

double PitchController::MIN(double a, double b) const
{
	if (a < b)
		return a;
	else
		return b;
}

double PitchController::MAX(double a, double b) const
{
	if (a > b)
		return a;
	else
		return b;
}

*/