#include "MasterController.h"

MasterController::MasterController()
{

}

MasterController::MasterController(const char* bladed_dll_fname)
{
	Init(bladed_dll_fname);
}

MasterController::MasterController(const char* gen_csv_fname, const char* pitch_param_csv_fname, double cornerFreq, double initGenSpeed) 
{
	Init(gen_csv_fname, pitch_param_csv_fname, cornerFreq, initGenSpeed);
}

void MasterController::Init(const char* fname)
{
	controlMode = BLADED_DLL;

	bladedcont.Init(fname);
}

void MasterController::Init(const char* gen_csv, const char* pit_param, double cf, double initSpeed)
{
	controlMode = CSV_TABLES;

	gencont.ReadCSVFile(gen_csv);
	pitcont.LoadInputFile(pit_param, 0.0f);
	genSpeedLPF.InitFilterVal(initSpeed);
	genSpeedLPF.SetCornerFreq(cf);
}

void MasterController::UpdateController(double time, double genSpeed, double currBladePitch)
{
	if (controlMode == BLADED_DLL)
	{
		bladedcont.UpdateController(time, currBladePitch, currBladePitch, currBladePitch, genSpeed, 0.0);
	}
	else
	{
		// Update the Low-Pass Filter
		double genSpeedF = genSpeedLPF.UpdateEstimate(time, genSpeed);

		// Update the pitch controller 
		pitcont.Calculate(time, genSpeedF);

		// Don't need to update the gen controller because it doesn't have any internal states
		// to update; it's just a lookup table
	}
}

double MasterController::GetBladePitchCommand() const
{
	if (controlMode == BLADED_DLL)
	{
		return bladedcont.GetBlPitchCommand();
	}
	else
	{
		return pitcont.GetLastPitchCommand();
	}
}

double MasterController::GetGeneratorTorqueCommand() const
{
	if (controlMode == BLADED_DLL)
	{
		return bladedcont.GetGenTorqueCommand();
	}
	else
	{
		return gencont.GetTorque(genSpeedLPF.GetCurrEstimatedValue());
	}
}
