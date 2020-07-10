#include "FASTInterface.h"
#include "DriveTrain.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "MasterController.h"
#include "AeroDynTurbine.h"
#include <Eigen/Dense>
#include "InputFile.h"
#include <fstream>

// If this is defined, then output files will be written to which log activity
// To silence any logging, just comment out this define preprocessor statement
// #define LOG_ACTIVITY

// Used for representing vectors and matrices
using namespace Eigen;

// Takes Euler angles and returns global to local rotation matrix
Matrix3d EulerConstruct(const Vector3d& theta);
// Takes global to local rotation matrix and returns Euler angles
Vector3d EulerExtract(const Matrix3d& m);

// The implementation class for the turbine model. Anything that would be in the private section of 
// the turbine class has been placed here.
class FASTInterface::PImp : public AeroDynTurbine
{
public:
	PImp();

	Vector<double, 12> CalcResidual(const Vector<double, 12>& y, const Vector<double, 12>& u);

	std::function<void(const double*, const double*, double*, double*)> CalcOutput_callback;

#ifdef LOG_ACTIVITY
	//-----------------------------------------------
	// Debug
	std::ofstream fout_input;
	std::ofstream fout_output;
	std::ofstream fout_funcCalls;

	void WriteHeader() {
		fout_input << "time\torient_roll\torient_pitch\torient_yaw\tang_vel_x\tang_vel_y\tang_vel_z" << std::endl;
		fout_output << "time\tnacforce_x\tnacforce_y\tnacforce_z\tnacmoment_x\tnacmoment_y\tnacmoment_z" << std::endl;
	}
	void LogInput(double time, FASTInterface::PImp::NacelleMotion nm) {
		fout_input << time << '\t' << nm.EulerAngles[0] << '\t' << nm.EulerAngles[1] << '\t' << nm.EulerAngles[2] << '\t' << nm.angularVel[0] << '\t' << nm.angularVel[1] << '\t'
			<< nm.angularVel[2] << '\t';
	}
	void LogOutput(double time, FASTInterface::NacelleReactionLoads rf) {
		fout_output << time << '\t' << rf.force[0] << '\t' << rf.force[1] << '\t' << rf.force[2] << '\t' << rf.moment[0] << '\t'
			<< rf.moment[1] << '\t' << rf.moment[2] << std::endl;
	}
	void LogInflows(double x) {
		fout_input << x << '\t' << std::endl;
	}
#endif
};

FASTInterface::PImp::PImp() : AeroDynTurbine()
{

}

FASTInterface::FASTInterface() : p_imp(new PImp)
{

}

FASTInterface::~FASTInterface() = default;


void FASTInterface::InitWithConstantRotorSpeedAndPitch(double constantRotorSpeed, double constantBladePitch)
{
	p_imp->InitWithConstantRotorSpeedAndPitch(constantBladePitch, constantBladePitch);

	// Debug
#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitWithiConstantRotorSpeedAndPitch( " << constantRotorSpeed << ", " << constantBladePitch << " );" << std::endl;
#endif
}

// Sets all the necessary parameters for the drive train in order. Note, the gearbox ratio must be 
// set before the initial rotor speed.
void FASTInterface::InitDriveTrain(double rotorMOI, double genMOI, double stiffness, double damping, double gearboxRatio, double initialRotorVel)
{
	p_imp->InitDriveTrain(rotorMOI, genMOI, stiffness, damping, gearboxRatio, initialRotorVel);

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitDriveTrain( " << rotorMOI << ", " << genMOI << ", " << stiffness << ", " << damping << ", " << gearboxRatio << ", " << initialRotorVel << " );" << std::endl;
#endif
}

void FASTInterface::InitControllers_BladedDLL(const std::string& bladed_dll_fname, double initialBladePitch)
{
	InitControllers_BladedDLL(bladed_dll_fname, initialBladePitch);
#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitControllers_BladedDLL( " << bladed_dll_fname << " );" << std::endl;
#endif
}

// Note, this must be called after the drive train and controllers have been initialized because it 
// requires information gathered from both: rotor speed and blade pitch command.
void FASTInterface::InitAeroDyn(
	const std::string& inputFilename,
	const std::string& outputFilename,
	double timestep,
	int numBlades,
	double hubRadius,
	double precone,
	double fluidDensity,
	double kinematicFluidVisc,
	const double nacellePos[3],
	const double nacelleEulerAngles[3],
	const double nacelleVel[3],
	const double nacelleAcc[3],
	const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3])
{
	// Convert the arrays to Vector3d to pass to the actual AeroDynTurbine object
	Vector3d pos(nacellePos);
	Vector3d EulerAngles(nacelleEulerAngles);
	Vector3d vel(nacelleVel);
	Vector3d acc(nacelleAcc);
	Vector3d angularVel(nacelleAngularVel);
	Vector3d angularAcc(nacelleAngularAcc);

	p_imp->InitAeroDyn(inputFilename, outputFilename, timestep, numBlades, hubRadius, precone, fluidDensity, kinematicFluidVisc,
		pos, EulerAngles, vel, acc, angularVel, angularAcc);

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitAeroDyn( );" << std::endl;
#endif
}

void FASTInterface::InitInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->InitInputs_Inflow(inflowVel, inflowAcc);

#ifdef LOG_ACTIVITY
	p_imp->fout_funcCalls << "InitInflows();" << std::endl;
#endif
}

// Begin updating the model's state by first setting the nacelle states at point in future
void FASTInterface::SetNacelleStates(double time, const double nacellePos[3], const double nacelleEulerAngles[3],
	const double nacelleVel[3], const double nacelleAcc[3], const double nacelleAngularVel[3],
	const double nacelleAngularAcc[3], bool isRealStep)
{
	// Convert the arrays to Vector3d to pass to the actual AeroDynTurbine object
	Vector3d pos(nacellePos);
	Vector3d EulerAngles(nacelleEulerAngles);
	Vector3d vel(nacelleVel);
	Vector3d acc(nacelleAcc);
	Vector3d angularVel(nacelleAngularVel);
	Vector3d angularAcc(nacelleAngularAcc);

	p_imp->SetInputs_Nacelle(time, pos, EulerAngles, vel, acc, angularVel, angularAcc, isRealStep);
}




void FASTInterface::GetBladeNodePositions(std::vector<double>& p)
{
	p_imp->GetBladeNodePositions(p, false);
}

void FASTInterface::SetInflows(const std::vector<double>& inflowVel, const std::vector<double>& inflowAcc)
{
	p_imp->SetInputs_Inflow(inflowVel, inflowAcc);
}

void FASTInterface::SetCalcOutputCallback(std::function<void(const double*, const double*, double*, double*)> calcOutput)
{
	p_imp->CalcOutput_callback = calcOutput;
}

// Q: What is the drivetrain state when this should be called??
// A: 
// Note: Uses pointer arithmetic for now just to save on code length
FASTInterface::PDSAccOutputs FASTInterface::SolveForInputs_And_Outputs(const double nacelleAcc[3], const double nacelleRotationAcc[3])
{
	// hard-code here for now
	const int solver_iterations = 2;
	const double perturb = 0.01;

	// Setup serialized vector of inputs
	Vector<double,12> u;
	// Setup serialized vector of outputs
	Vector<double,12> y;

	// Use the function arguments as the initial inputs to start off the 
	// rootfinding process
	u.segment(6, 3) = Vector3d(nacelleAcc);
	u.segment(9, 3) = Vector3d(nacelleRotationAcc);

	p_imp->Set_Inputs_NacelleAcceleration(u.segment(6,3), u.segment(9,3), false);
	Vector3d hubForce, hubMoment;

	p_imp->aerodyn.CalcOutput(hubForce, hubMoment, false);

	// Transfer from hub coordinate system to nacelle
	Vector3d nacForce = p_imp->TransformHubToNacelle(hubForce,p_imp->nacelleOrient, p_imp->hubOrient);
	Vector3d nacMoment = p_imp->TransformHubToNacelle(hubMoment, p_imp->nacelleOrient, p_imp->hubOrient);

	u.segment(0, 3) = nacForce;
	u.segment(3, 3) = nacMoment;

	int k = 0;
	while (true) {
		//------------------------------------------------------
		// Calculate the outputs
		//------------------------------------------------------
		p_imp->CalcOutput_callback(u.data() + 0, u.data() + 3, 
								   y.data() + 0, y.data() + 3);

		p_imp->aerodyn.Set_Inputs_HubAcceleration(
			Vector3d(u.data() + 6), 
			Vector3d(u.data() + 9), 
			false);

		// calculate the outputs from AeroDyn
		Vector3d hubForce, hubMoment;
		p_imp->aerodyn.CalcOutput(
			hubForce, 
			hubMoment, 
			false);

		// Pack into the output vector
		y.segment(6, 3) = hubForce;
		y.segment(9, 3) = hubMoment;
		//------------------------------------------------------

		if (k >= solver_iterations)
			break;

		// At what point do we transfer the outputs from one to the 
		// inputs of the other? We do this in the residual function

		//------------------------------------------------------
		// Perturb inputs, calculate partial derivatives, and fill 
		// in the jacobian matrix
		//------------------------------------------------------
		Matrix<double, 12, 12> jacobian;
		Vector<double, 12> u_residual = p_imp->CalcResidual(y, u);
		
		// PDS first
		for (int i = 0; i < 6; i++) {
			Vector<double, 12> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			p_imp->CalcOutput_callback(u_perturb.data() + 0, u_perturb.data() + 3,
				y_perturb.data() + 0, y_perturb.data() + 3);

			Vector<double, 12> u_perturb_residual = p_imp->CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			Vector<double, 12> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}

		// AD second
		for (int i = 6; i < 12; i++) {
			Vector<double, 12> u_perturb, y_perturb;

			// perturb the i'th input
			u_perturb = u;
			u_perturb(i) += perturb;

			// then calculate the corresponding output
			p_imp->Set_Inputs_NacelleAcceleration(
				Vector3d(u_perturb.data() + 6),
				Vector3d(u_perturb.data() + 9),
				false);

			p_imp->CalcOutput(
				nacelleForce,
				nacelleMoment,
				false);

			y_perturb.segment(6, 3) = nacelleForce;
			y_perturb.segment(9, 3) = nacelleMoment;

			Vector<double, 12> u_perturb_residual = p_imp->CalcResidual(y_perturb, u_perturb);
			// Numerically calculate the partial derivative of wrt this input
			Vector<double, 12> residual_func_partial_u = (u_perturb_residual - u_residual) / perturb;
			// Add this entry into the jacobian as a column
			jacobian.col(i) = residual_func_partial_u;
		}
		//------------------------------------------------------
		// Do one step of Newton-Raphson (actually Secant method)
		//------------------------------------------------------

	}
	PDSAccOutputs result;
	Vector3d acc(0.0, 0.0, 0.0);
	Vector3d rotAcc(0.0, 0.0, 0.0);
	memcpy(result.nacelleAcc, acc.data(), 3 * sizeof(double));
	memcpy(result.nacelleRotationAcc, rotAcc.data(), 3 * sizeof(double));
	return result;
}

/*
Transfers the outputs from one module to the inputs of the other, then these
resulting inputs are subtracted from the inputs that generated the outputs
*/
Vector<double, 12> FASTInterface::PImp::CalcResidual(const Vector<double, 12>& y, const Vector<double, 12>& u)
{
	//----------------------------------------------------------------
	// Derive the inputs of one module from the outputs of the other
	//................................................................
	/* first start with deriving AD's inputs from ProteusDS's outputs */
	Vector3d nacelleAcc(y.data() + 0);
	Vector3d nacelleRotationAcc(y.data() + 3);
	Vector3d hubRotationAcc = nacelleRotationAcc + (drivetrain.GetGenShaftAcc() * hubOrient.col(0));

	Vector3d hubAcc = nacelleAcc;

	/* second derive PDS's inputs from AD's outputs */
	Vector3d hubForce(y.data() + 6);
	Vector3d hubMoment(y.data() + 9);

	Vector3d nacelleForce = TransformHubToNacelle(
		hubForce, 
		nacelleOrient, 
		hubOrient);
	Vector3d nacelleMoment = TransformHubToNacelle(
		hubMoment,
		nacelleOrient,
		hubOrient);
	//------------------------------------------------------
	// Pack these results in a vector (TODO use .segment(i,n) for these instead)
	//......................................................
	Vector<double, 12> u_derived;
	memcpy(u_derived.data() + 0, nacelleForce.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 3, nacelleMoment.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 6, hubAcc.data(), 3 * sizeof(double));
	memcpy(u_derived.data() + 9, hubRotationAcc.data(), 3 * sizeof(double));

	/* Subtract the derived inputs from the original ones */
	return u - u_derived;
}

FASTInterface::NacelleReactionLoads FASTInterface::AdvanceStates()
{
	auto nrl_vec = p_imp->AdvanceStates();

	NacelleReactionLoads nrl;
	memcpy(nrl.force, nrl_vec.force.data(), 3 * sizeof(double));
	memcpy(nrl.moment, nrl_vec.moment.data(), 3 * sizeof(double));
	nrl.power = nrl_vec.power;
	nrl.tsr = nrl_vec.tsr;
	
	return nrl;
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
	return p_imp->mcont.GetGeneratorTorqueCommand();
}

double FASTInterface::GetGeneratorSpeed() const
{
	return p_imp->drivetrain.GetGenShaftSpeed();
}

double FASTInterface::GetRotorSpeed() const
{
	return p_imp->drivetrain.GetRotorShaftSpeed();
}

double FASTInterface::GetRotorAngularDisp() const
{
	return p_imp->drivetrainStates_pred.rotor.theta;
}

double FASTInterface::GetGeneratorAngularDisp() const
{
	return p_imp->GetGeneratorAngularDisp();
}

// All three of GetForce, GetMoment, and GetAerodynamicTorque get their values from member
// variables in AeroDyn_Interface. The value returned is the last value 
// returned by AeroDyn, whether it be from a real step or a temporary step for the RK4 
// integration.

double FASTInterface::GetAerodynamicTorque() const
{
	return p_imp->aerodyn.GetTorque();
}

void FASTInterface::GetNacelleForce(double v[3]) const
{
	memcpy(v, p_imp->nacelleForce.data(), 3 * sizeof(double));
}

// The x component of the moment is equal to the aerodynamic torque
void FASTInterface::GetNacelleMoment(double v[3]) const
{
	v[0] = p_imp->mcont.GetGeneratorTorqueCommand();
	v[1] = p_imp->nacelleMoment.y();
	v[2] = p_imp->nacelleMoment.z();
}

double FASTInterface::GetTSR() const
{
	return p_imp->aerodyn.GetTSR();
}

void FASTInterface::GetHubForce(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetForce().data(), 3 * sizeof(double));
}

void FASTInterface::GetHubMoment(double v[3]) const
{
	memcpy(v, p_imp->aerodyn.GetMoment().data(), 3 * sizeof(double));
}

