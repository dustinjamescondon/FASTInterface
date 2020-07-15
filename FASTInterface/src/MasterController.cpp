#include "MasterController.h"

MasterController::MasterController()
{

}


void MasterController::Init(double bladePitch)
{
	controlMode = CONSTANT_SPEED;
	
	bladePitchCommand = bladePitch;
}

void MasterController::Init_BladedDLL(const char* fname, double initBladePitch)
{
	controlMode = BLADED_DLL;
	bladePitchCommand = initBladePitch;

	bladedcont.Init(fname);
}

void MasterController::Init_InputFile(const char* inputfile_fname, double initGenSpeed, double initRotorSpeed)
{
	// Set the mode to reflect how we're using the controller
	controlMode = INPUT_FILE;

	// Load the main input file
	inputfile.Load(inputfile_fname);

	// Read the entry for the generator controller table filename
	std::string genTable = inputfile.ReadString("GeneratorTorqueTable");

	// Read the entry for the pitch gain scheduling table filename
	std::string pitchTable = inputfile.ReadString("PitchGainSchedulingTable");

	// Read the entry for the corner frequency of the low pass filter
	double cornerFreq = inputfile.ReadDouble("LPFCornerFreqency");

	// Read the pitch controller's proportion gain
	double proportionGain = inputfile.ReadDouble("PitchController_ProportionGain");

	// Read the pitch controller's integral gain
	double integralGain = inputfile.ReadDouble("PitchController_IntegralGain");

	// Done reading from file, so close it
	inputfile.Close();


	// Use the read values to set parameters
	gencont.ReadCSVFile(genTable.c_str());
	pitcont.LoadGainSchedulingFile(pitchTable.c_str(), 0.0f);
	pitcont.SetProportionGain(proportionGain);
	pitcont.SetIntegralGain(integralGain);
	genSpeedLPF.InitFilterVal(initRotorSpeed);
	genSpeedLPF.SetCornerFreq(cornerFreq);
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
	case INPUT_FILE:
		// Update the Low-Pass Filter
		genSpeedF = genSpeedLPF.UpdateEstimate(time, genSpeed);

		// Update the pitch controller 
		pitcont.Calculate(time, genSpeedF);

		bladePitchCommand = pitcont.GetLastPitchCommand();

		// Don't need to update the gen controller because it doesn't have any internal states_pred
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
		return bladePitchCommand;
		break;
	case INPUT_FILE:
		return pitcont.GetLastPitchCommand();
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
	case INPUT_FILE:
		return gencont.GetTorque(genSpeedLPF.GetCurrEstimatedValue());
		break;
	default:
		return 0.0;
	}
}
