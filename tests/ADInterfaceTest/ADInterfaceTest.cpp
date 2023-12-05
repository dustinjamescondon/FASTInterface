#include "pch.h"
#include "CppUnitTest.h"
#include "FASTInterface.h"
#include "AeroDyn_Interface_Wrapper.h"
#include "Eigen/Sparse"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace Eigen;

namespace ADInterfaceTest
{
	TEST_CLASS(ADInterfaceTest)
	{
	public:
		
		TEST_METHOD(Initializes)
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
	};
}
