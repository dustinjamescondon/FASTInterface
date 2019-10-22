#include "GenController.h"
#include <math.h>

GenController::GenController(const char* filename) : torqueFunc(filename), 
VS_SySp(VS_RtGnSp / (1.0 + 0.01 * VS_SlPc)),
VS_Slope15((VS_Rgn2K* VS_Rgn2Sp* VS_Rgn2Sp) / (VS_Rgn2Sp - VS_CtInSp)),
VS_Slope25((VS_RtPwr / VS_RtGnSp) / (VS_RtGnSp - VS_SySp)),
VS_TrGnSp((VS_Slope25 - sqrt(VS_Slope25 * (VS_Slope25 - 4.0 * VS_Rgn2K * VS_SySp))) / (2.0 * VS_Rgn2K))
{
	ReadCSVFile(filename);
}

GenController::GenController() noexcept :
	VS_SySp(VS_RtGnSp / (1.0 + 0.01 * VS_SlPc)),
	VS_Slope15((VS_Rgn2K* VS_Rgn2Sp* VS_Rgn2Sp) / (VS_Rgn2Sp - VS_CtInSp)),
	VS_Slope25((VS_RtPwr / VS_RtGnSp) / (VS_RtGnSp - VS_SySp)),
	VS_TrGnSp((VS_Slope25 - sqrt(VS_Slope25 * (VS_Slope25 - 4.0 * VS_Rgn2K * VS_SySp))) / (2.0 * VS_Rgn2K))
{
	maxRatedTorque = 0.0;
}

void GenController::ReadCSVFile(const char* filename)
{
	torqueFunc.LoadCSVFile(filename);

	// the max rated torque is the last entry in the CSV table, so use the biggest rotor speed
	// entry to lookup the last torque.
	maxRatedTorque = torqueFunc.F(torqueFunc.GetMaxSpecX());
}

void GenController::ReadParameters(const char* fname)
{
	Load(fname);
	VS_RtGnSp = ReadDouble("VS_RtGenSp");


}

double GenController::GetTorque(double genSpeed) const
{
	double genTrq = 0.0;
	if (genSpeed >= VS_RtGnSp) {
		genTrq = VS_RtPwr / genSpeed;
	} 
	else if (genSpeed <= VS_CtInSp) {
		genTrq = 0.0;
	}
	else if (genSpeed < VS_Rgn2Sp) {
		genTrq = VS_Slope15 * (genSpeed - VS_CtInSp);
	}
	else if (genSpeed < VS_TrGnSp) {
		genTrq = VS_Rgn2K * genSpeed * genSpeed;
	} 
	else {
		genTrq = VS_Slope25 * (genSpeed - VS_SySp);
	}

	if (genTrq > VS_MaxTq) {
		genTrq = VS_MaxTq;
	}

	return genTrq;
}

double GenController::GetTorque_CSV(double genSpeed) const
{
	return torqueFunc.F(genSpeed);
}

double GenController::GetMaxRatedTorque() const noexcept
{
	return maxRatedTorque;
}
