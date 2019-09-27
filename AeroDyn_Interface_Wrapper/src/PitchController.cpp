#include "PitchController.h"

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