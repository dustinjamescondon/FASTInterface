#include "BladedInterface.h"
#include <iostream>
#include "FASTTurbineExceptions.h"

BladedInterface::BladedInterface() 
{
	iStatus = 0;
	bladePitchCommand = 0.0;
	genTorqueCommand = 0.0;
}

BladedInterface::~BladedInterface()
{
	if(hInstance)
		FreeLibrary(hInstance);
}


void BladedInterface::Init(const char* fname)
{
	// Link to the DLL
	hInstance = LoadLibraryA(fname);

	if (!hInstance) {
		// Throw exception
		std::string errMsg;
		errMsg = "Could not load DLL file, " + std::string(fname);
		throw FileNotFoundException(errMsg.c_str());
	}

	// Get function address
	DISCON = (f_ptr)GetProcAddress(hInstance, "DISCON");

	if (!DISCON) {
		// Throw exception
		throw FileContentsException("Could not load the DISCON procedure from the controller DLL");
	}
}

float BladedInterface::GetBlPitchCommand() const
{
	return bladePitchCommand;
}

float BladedInterface::GetGenTorqueCommand() const
{
	return genTorqueCommand;
}

void BladedInterface::UpdateController(double time, float BlPitch1, float BlPitch2, float BlPitch3, float GenSp, float HorWindV)
{
	// Set up the inputs in the swap array
	swap[SwapIndex::Time] = (float)time;
	swap[SwapIndex::BlPitch1] = BlPitch1;
	swap[SwapIndex::BlPitch2] = BlPitch2;
	swap[SwapIndex::BlPitch3] = BlPitch3;
	swap[SwapIndex::GenSp] = GenSp;
	swap[SwapIndex::HorWindV] = HorWindV;
	swap[SwapIndex::IStatus] = (float)iStatus;

	fail = 0;
	// Call the DLL
	DISCON(swap, &fail, accINFILE.c_str(), avcOUTNAME.c_str(), avcMSG.c_str());

	// Change this variable to 1 to signify we are no longer on the first call to the DLL
	iStatus = 1;

	// Get output from swap array
	bladePitchCommand = swap[SwapIndex::PitCom];
	genTorqueCommand = swap[SwapIndex::GenTrqDem];
}