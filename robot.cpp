#include "WPILib.h"

class Robot : public IterativeRobot
{
private:
	RobotDrive drivetrain;
	Joystick driveStick, lifterStick;
	CANTalon frontLeft, frontRight,
	         rearLeft, rearRight;

public:
	Robot():
		frontLeft(0), frontRight(1),
		rearLeft(2), rearRight(3),
		drivetrain(frontLeft, rearLeft,
		           frontRight, rearRight),
		driveStick(0), lifterStick(1)
	{

	}

	void DisabledInit()
	{
		drivetrain.SetSafetyEnabled(false);  //disable watchdog
	}

	void TeleopInit()
	{
		printf("Starting Teleop mode");
		drivetrain.SetExpiration(2);  //set the timeout for the watchdog
		drivetrain.SetSafetyEnabled(true);  //enable watchdog
	}

	void TeleopPeriodic()
	{
		//Driving
		float throttleScale = ((1 - driveStick.GetThrottle()) / 2);
		drivetrain.MecanumDrive_Cartesian(driveStick.GetX()*throttleScale, driveStick.GetY()*throttleScale, driveStick.GetTwist()*throttleScale*driveStick.GetRawButton(2));
	}
};

START_ROBOT_CLASS(Robot);
