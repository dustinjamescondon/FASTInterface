#include <DriveTrain.h>

int main()
{
	double const_rotor_speed = 10;

	DriveTrain drivetrain;
	drivetrain.Init(const_rotor_speed);

	return 0;
}