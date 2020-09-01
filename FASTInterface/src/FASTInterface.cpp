#include "FASTInterface.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
#include "AeroDynTurbine.h"
#include <Eigen/Dense>
#include <fstream>

// Used for representing vectors and matrices
using namespace Eigen;

//! @brief The implementation class for the turbine model, just make it inherit from AeroDynTurbineClass
class FASTInterface::PImp : public AeroDynTurbine
{
};

FASTInterface::FASTInterface() :
	p_imp(new PImp)
{

}

FASTInterface::~FASTInterface()
{

}

//-----------------------------------------------------
// Initialization methods

void FASTInterface::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->InitWithConstantRotorSpeedAndPitch(constantRotorSpeed, constantBladePitch);
}

void FASTInterface::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initRotorSpeed)
{
	p_imp->InitDriveTrain(rotorMOI, genMOI, stiffness, damping, gearboxRatio, initRotorSpeed);
 }

void FASTInterface::InitControllers_BladedDLL(int numBlades, const std::string& bladed_dll_fname, double initialBladePitch)
{
	p_imp->InitControllers_BladedDLL(numBlades, bladed_dll_fname, initialBladePitch);
}

void FASTInterface::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
	bool useAddedMass,
	double coeffAddedMass,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
	const double nacellePos[3],
	const double nacelleEulerAngles[3],
	const double nacelleVel[3],
	const double nacelleAcc[3],
	const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3])
{
	Vector3d nacellePos_v(nacellePos);
	Vector3d nacelleEuler_v(nacelleEulerAngles);
	Vector3d nacelleVel_v(nacelleVel);
	Vector3d nacelleAcc_v(nacelleAcc);
	Vector3d nacelleAngVel_v(nacelleAngularVel);
	Vector3d nacelleAngAcc_v(nacelleAngularAcc);

	p_imp->InitAeroDyn(inputFilename, outputFilename, useAddedMass, coeffAddedMass, timestep, numBlades, hubRadius,
		precone, nacellePos_v, nacelleEuler_v,
		nacelleVel_v, nacelleAcc_v, nacelleAngVel_v, nacelleAngAcc_v);
}


void FASTInterface::InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->InitInputs_Inflow(inflowVel, inflowAcc);
}

FASTInterface::NacelleReactionLoads FASTInterface::AdvanceStates()
{
	NacelleReactionLoads r;
	
	AeroDynTurbine::NacelleReactionLoads_Vec t = p_imp->AdvanceStates();

	memcpy(r.force, t.force.data(), 3 * sizeof(double));
	memcpy(r.moment, t.moment.data(), 3 * sizeof(double));
	r.power = t.power;
	r.tsr = t.tsr;

	return r;
}

void FASTInterface::GetBladeNodePositions(std::vector<double>& nodePos_out)
{
	p_imp->GetBladeNodePositions(nodePos_out);
}

double FASTInterface::GetAerodynamicTorque() const
{
	return p_imp->GetHubReactionLoads().moment.x();
}

void FASTInterface::GetNacelleForce(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleReactionForce().data(), 3 * sizeof(double));
}

void FASTInterface::GetNacelleMoment(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleReactionMoment().data(), 3 * sizeof(double));
}

void FASTInterface::GetNacelleAcc(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleAcc().data(), 3 * sizeof(double));
}

void FASTInterface::GetNacelleAngularAcc(double out[3]) const
{
	memcpy(out, p_imp->GetNacelleAngularAcc().data(), 3 * sizeof(double));
}

double FASTInterface::GetTSR() const
{
	return p_imp->GetTSR();
}

void FASTInterface::GetHubForce(double out[3]) const
{
	memcpy(out, p_imp->GetHubReactionLoads().force.data(), 3 * sizeof(double));
}

void FASTInterface::GetHubMoment(double out[3]) const
{
	memcpy(out, p_imp->GetHubReactionLoads().moment.data(), 3 * sizeof(double));
}

int FASTInterface::GetNumNodes() const
{
	return p_imp->GetNumNodes();
}

int FASTInterface::GetNumBlades() const
{
	return p_imp->GetNumBlades();
}

double FASTInterface::GetTurbineDiameter() const
{
	return p_imp->GetTurbineDiameter();
}

double FASTInterface::GetBladePitch() const
{
	return p_imp->GetBladePitch();
}

double FASTInterface::GetGeneratorTorque() const
{
	return p_imp->GetGenTorque();
}

double FASTInterface::GetGeneratorSpeed() const
{
	return p_imp->GetGenSpeed();
}

double FASTInterface::GetRotorSpeed() const
{
	return p_imp->GetRotorSpeed();
}

double FASTInterface::GetRotorAngularDisp() const
{
	return p_imp->GetRotorShaftState().theta;
}

double FASTInterface::GetGeneratorAngularDisp() const
{
	return p_imp->GetGenShaftState().theta;
}

void FASTInterface::SetNacelleStates(
	double time,
	const double nacellePos[3],
	const double nacelleEulerAngles[3],
	const double nacelleVel[3],
	const double nacelleAcc[3],
	const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3],
	bool isTempUpdate)
{
	using namespace Eigen;
	
	p_imp->SetInputs_Nacelle(time, Vector3d(nacellePos), Vector3d(nacelleEulerAngles), Vector3d(nacelleVel),
		Vector3d(nacelleAcc), Vector3d(nacelleAngularVel), Vector3d(nacelleAngularAcc), isTempUpdate);
}

void FASTInterface::SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->SetInputs_Inflow(inflowVel, inflowAcc);
}

void FASTInterface::SetCalcOutputCallback(std::function<void(const double*, const double*, double*, double*)> calcOutput) 
{
	p_imp->SetCallback_CalcOutput(calcOutput);
}
