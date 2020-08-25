#include "MasterController.h"

MasterController::MasterController()
{

}


void MasterController::Init(double bladePitch)
{
	controlMode = CONSTANT_SPEED;
	
	bladePitchCommand = bladePitch;
}

void MasterController::Init_BladedDLL(int numBlades, const char* fname, double initBladePitch)
{
	controlMode = BLADED_DLL;
	bladePitchCommand = initBladePitch;

	bladedcont.Init(numBlades, fname);
}

void MasterController::UpdateController(double time, double genSpeed, double currBladePitch)
{
	double genSpeedF;
	switch (controlMode)
	{
	case BLADED_DLL:
		bladedcont.UpdateController(time, currBladePitch, currBladePitch, currBladePitch, genSpeed, 0.0);
		bladePitchCommand = bladedcont.GetBlPitchCommand();
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
		return bladePitchCommand;
		break;
	default:
		return bladePitchCommand;
	}
}

double MasterController::GetGeneratorTorqueCommand() const
{
	switch (controlMode)
	{
	case BLADED_DLL:
		return bladedcont.GetGenTorqueCommand();
		break;
	default:
		return 0.0;
	}
}
