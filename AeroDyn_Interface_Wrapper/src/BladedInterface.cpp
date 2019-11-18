#include "BladedInterface.h"
#include <iostream>
#include "FASTTurbineExceptions.h"

BladedInterface::BladedInterface() 
{
	// This is passed to the Bladed-style DLL to signify this is the first call to it
	iStatus = 0;

	// Initialize the controller commands to zero. The DLL should actually initialize these
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
	
	// If we couldn't load it
	if (!hInstance) {
		// Throw exception
		std::string errMsg;
		errMsg = "Could not load DLL file, " + std::string(fname);
		throw FileNotFoundException(errMsg.c_str());
	}

	vf_ptr TEST = (vf_ptr)GetProcAddress(hInstance, "TEST");

	// Get function address
	DISCON = (f_ptr)GetProcAddress(hInstance, "DISCON");

	// If we couldn't find the function DISCON
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

// Bladed-style DLL's communicate with the caller via a parameter called Swap. It is an array of floats which
// which is used to hold both input to the DLL and output from the DLL. The DLL expects the array to be allocated 
// by the caller, so each instance of this class contains a swap array of a max length, which it passes to the 
// DLL's primary function.
// Currently this is just the most basic interface with the Bladed-style DLL
void BladedInterface::UpdateController(double time, float blPitch1, float blPitch2, float blPitch3, float genSp, float horWindV)
{
	// Set up the inputs in the swap array
	swap[SwapIndex::Time] = (float)time;
	swap[SwapIndex::BlPitch1] = blPitch1;
	swap[SwapIndex::BlPitch2] = blPitch2;
	swap[SwapIndex::BlPitch3] = blPitch3;
	swap[SwapIndex::GenSp] = genSp;
	swap[SwapIndex::HorWindV] = horWindV;
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