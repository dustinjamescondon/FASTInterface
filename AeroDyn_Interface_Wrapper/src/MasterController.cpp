#include "MasterController.h"

MasterController::MasterController()
{

}

MasterController::MasterController(double bladePitch)
{
	Init(bladePitch);
}

MasterController::MasterController(const char* bladed_dll_fname)
{
	Init(bladed_dll_fname);
}

MasterController::MasterController(const char* gen_csv_fname, const char* pitch_param_csv_fname, double cornerFreq, double initGenSpeed) 
{
	Init(gen_csv_fname, pitch_param_csv_fname, cornerFreq, initGenSpeed);
}

void MasterController::Init(double bladePitch)
{
	controlMode = CONSTANT_SPEED;
	
	constantBladePitch = bladePitch;
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
	double genSpeedF;
	switch (controlMode)
	{
	case BLADED_DLL:
		bladedcont.UpdateController(time, currBladePitch, currBladePitch, currBladePitch, genSpeed, 0.0);
		break;
	case CSV_TABLES:
		// Update the Low-Pass Filter
		genSpeedF = genSpeedLPF.UpdateEstimate(time, genSpeed);

		// Update the pitch controller 
		pitcont.Calculate(time, genSpeedF);

		// Don't need to update the gen controller because it doesn't have any internal states
		// to update; it's just a lookup table
		break;
	default:
		// Nothing to do in this case
		break;
	}
}

double MasterController::GetBladePitchCommand() const
{
	switch (controlMode) {
	case BLADED_DLL:
		return bladedcont.GetBlPitchCommand();
		break;
	case CSV_TABLES:
		return pitcont.GetLastPitchCommand();
		break;
	default:
		return constantBladePitch;
	}
}

double MasterController::GetGeneratorTorqueCommand() const
{
	switch (controlMode)
	{
	case BLADED_DLL:
		return bladedcont.GetGenTorqueCommand();
		break;
	case CSV_TABLES:
		return gencont.GetTorque(genSpeedLPF.GetCurrEstimatedValue());
		break;
	default:
		return 0.0;
	}
}
