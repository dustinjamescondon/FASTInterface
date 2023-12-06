/*!
 * @author Dustin Condon
 * @date August 2020
 * 
 * @brief This class provides an interface with the Bladed-style DLL. The interface was built by referencing 
 *        the Bladed-style DLL of NREL's OC3 5 MW turbine.
 * 	   
 * The interface allows the user of the class to pass
 * the needed values to the DLL, and recieve the results. Inside each DLL there is a pitch controller,
 * generator controller, and a low-pass filter on the generator shaft speed.
 * 
 * Note that (as far as I have seen), the Bladed-style DLL are written to be loosely coupled with the turbine 
 * model; therefore, this interface assumes that. It may be possible to write a tightly coupled controller DLL,
 * however - in which case this interface should be built upon to allow for it.
*/


#pragma once
#include <windows.h>
#include <string>

class BladedInterface
{
public:

	BladedInterface();
	~BladedInterface();

	void Init(int numBlades, const char* fname);

	void UpdateController(double time, float blPitch1, float blPitch2, float blPitch3, float genSp, float horWindV);
	float GetBlPitchCommand() const;
	float GetGenTorqueCommand() const;

private:
	typedef void(__cdecl *f_ptr)(float*, int*, const char*, const char*, const char*);
	typedef void(__cdecl* vf_ptr)();

	// number of elements in the swap array
	static const int NumSwap = 100; 
	float swap[NumSwap];
	int fail;
	int iStatus;
	std::string accINFILE, avcOUTNAME, avcMSG;
	int numBlades;

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