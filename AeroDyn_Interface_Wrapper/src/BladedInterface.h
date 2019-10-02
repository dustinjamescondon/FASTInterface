#pragma once
#include <windows.h>
#include <string>

class BladedInterface
{
public:

	BladedInterface();
	~BladedInterface();

	void Init(const char* fname);

	void UpdateController(double time, float BlPitch1, float BlPitch2, float BlPitch3, float GenSp, float HorWindV);
	float GetBlPitchCommand() const;
	float GetGenTorqueCommand() const;

private:
	typedef void(*f_ptr)(float*, int*, const char*, const char*, const char*);

	// number of elements in the swap array
	static const int NumSwap = 100; 
	float swap[NumSwap];
	int fail;
	int iStatus;
	std::string accINFILE, avcOUTNAME, avcMSG;

	// Results from DLL
	float bladePitchCommand;
	float genTorqueCommand;

	HINSTANCE hInstance;
	f_ptr DISCON;

	enum SwapIndex : int {
		NumBl = 60, BlPitch1 = 3, BlPitch2 = 32, BlPitch3 = 33, GenSp = 19, HorWindV = 26,
		Time = 1, IStatus = 0, PitchOverride = 54, PitCom1 = 41, PitCom2 = 42, PitCom3 = 43, 
		PitCom = 44, GenContactStatus = 55, GenTrqDem = 46
	};

};