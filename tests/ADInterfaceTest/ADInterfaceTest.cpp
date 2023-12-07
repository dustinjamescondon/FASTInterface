#include "pch.h"
#include "CppUnitTest.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "Eigen/Sparse"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace Eigen;

namespace ADInterfaceTest
{
	void GenerateInflowVelocities(int totalNodes,
		double inflowSpeed, std::vector<double>& inflowVel, std::vector<double>& inflowAcc);

	TEST_CLASS(ADInterfaceTest)
	{
	public:
		
		TEST_METHOD(Initialize)
		{
			AeroDyn_Interface_Wrapper ad_interface;
			double dt = 0.05;
			int numBlades = 3;
			double hubRadius = 1;
			double preconeAngle = 0;
			bool useAddedMass = false;
			double coeffOfAddedMass = 0;
			auto hubPos = Vector3d::Zero();
			auto hubOri = Matrix3d::Identity();
			auto hubVel = Vector3d::Zero();
			auto hubAcc = Vector3d::Zero();
			auto hubRotVel = Vector3d::Zero();
			auto hubRotAcc = Vector3d::Zero();
			double bladePitch = 0.0;

			ad_interface.InitAerodyn("../../../tests/resources/5MW_OC4Semi_WSt_WavesWN/NRELOffshrBsline5MW_OC3Hywind_AeroDyn15.dat",
				"output",
				dt, numBlades, hubRadius,
				preconeAngle, 
				useAddedMass,
				coeffOfAddedMass,
				hubPos,
				hubOri,
				hubVel,
				hubAcc,
				hubRotVel,
				hubRotAcc,
				bladePitch
			);

			Assert::AreNotEqual(0, ad_interface.GetNumNodes());
		}

		TEST_METHOD(InitializeAndInitInflows)
		{
			AeroDyn_Interface_Wrapper ad_interface;
			double dt = 0.05;
			int numBlades = 3;
			double hubRadius = 1;
			double preconeAngle = 0;
			bool useAddedMass = false;
			double coeffOfAddedMass = 0;
			auto hubPos = Vector3d::Zero();
			auto hubOri = Matrix3d::Identity();
			auto hubVel = Vector3d::Zero();
			auto hubAcc = Vector3d::Zero();
			auto hubRotVel = Vector3d::Zero();
			auto hubRotAcc = Vector3d::Zero();
			double bladePitch = 0.0;

			ad_interface.InitAerodyn("../../../tests/resources/5MW_OC4Semi_WSt_WavesWN/NRELOffshrBsline5MW_OC3Hywind_AeroDyn15.dat",
				"output",
				dt, numBlades, hubRadius,
				preconeAngle,
				useAddedMass,
				coeffOfAddedMass,
				hubPos,
				hubOri,
				hubVel,
				hubAcc,
				hubRotVel,
				hubRotAcc,
				bladePitch
			);

			Assert::AreNotEqual(0, ad_interface.GetNumNodes());
		
			auto num_nodes = ad_interface.GetNumNodes();
			std::vector<double> velocities(3 * num_nodes);
			std::vector<double> accelerations(3 * num_nodes);
			GenerateInflowVelocities(num_nodes, 5.0, velocities, accelerations);

			ad_interface.InitInflows(velocities, accelerations);
			ad_interface.CalcOutput();
			Vector3d hub_reaction_load = ad_interface.GetHubReactionLoads().force;
			Assert::AreNotEqual(0.0, hub_reaction_load.x());
		}
	};

	// Assumes the inflow acceleration is zero
	void GenerateInflowVelocities(int totalNodes,
		double inflowSpeed, std::vector<double>& inflowVel, std::vector<double>& inflowAcc)
	{

		for (int i = 0; i < totalNodes; i++)
		{
			inflowVel[i * 3 + 0] = inflowSpeed;
			inflowVel[i * 3 + 1] = 0.0;
			inflowVel[i * 3 + 2] = 0.0;

			// For this test, just assume the inflows aren't accelerating
			inflowAcc[i * 3 + 0] = 0.0;
			inflowAcc[i * 3 + 1] = 0.0;
			inflowAcc[i * 3 + 2] = 0.0;
		}
	}
}
