#include "BladedInterface.h"
#include <iostream>

BladedInterface::BladedInterface()
{

}

BladedInterface::~BladedInterface()
{
	FreeLibrary(hInstance);
}


void BladedInterface::Init(const char* fname, int numBlades)
{
	
	numBl = numBlades;

	// Link to the DLL
	hInstance = LoadLibraryA(fname);

	if (!hInstance)
	{
		std::cout << "Couldn't read file" << std::endl;
		exit(1);
	}

	DISCON = (f_ptr)GetProcAddress(hInstance, "DISCON");

	if (!DISCON) {
		std::cout << "Couldn't find function" << std::endl;
	}
}

float BladedInterface::GetBlPitchDemand() const
{
	return blPitchDmd;
}

float BladedInterface::GetGenTorqueDemand() const
{
	return genTrqDmd;
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
	swap[SwapIndex::NumBl] = (float)numBl;

	fail = 0;
	// Call the DLL
	DISCON(swap, &fail, accINFILE.c_str(), avcOUTNAME.c_str(), avcMSG.c_str());

	// Get output from swap array
	blPitchDmd = swap[SwapIndex::PitCom];
	genTrqDmd = swap[SwapIndex::GenTrqDem];
}