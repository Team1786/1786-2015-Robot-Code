#include "WPILib.h"

class Robot : public IterativeRobot
{
private:
	RobotDrive drivetrain;
	Joystick driveStick;

public:
	Robot():
		drivetrain(0, 1),
		driveStick(1)
	{

	}

	void TeleopInit()
	{
		printf("Starting Teleop mode");
		drivetrain.SetExpiration(2);  //set the timeout for the watchdog
		drivetrain.SetSafetyEnabled(true);  //enable watchdog
	}

	void TeleopPeriodic()
	{
		drivetrain.MecanumDrive_Cartesian(driveStick.GetX(), driveStick.GetY(), driveStick.GetTwist());
	}
};

START_ROBOT_CLASS(Robot);
