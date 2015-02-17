#include "WPILib.h"
#include <cmath>

#define RAMP_RATE 0.05

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
		static float last[3];
		static double scaled[3];
		float throttleScale = ((1 - driveStick.GetThrottle()) / 2);
		scaled[0] = driveStick.GetX()*throttleScale;
		scaled[1] = driveStick.GetY()*throttleScale;
		scaled[2] = driveStick.GetTwist()*throttleScale*driveStick.GetRawButton(2);		

		last[0] = (std::abs(last[0] - scaled[0]) < RAMP_RATE ? scaled[0] :
		           scaled[0] > last[0] ? last[0] + RAMP_RATE :
		           scaled[0] < last[0] ? last[0] - RAMP_RATE : last[0]);
		last[1] = (std::abs(last[0] - scaled[1]) < RAMP_RATE ? scaled[1] :
		           scaled[1] > last[1] ? last[1] + RAMP_RATE :
		           scaled[1] < last[1] ? last[1] - RAMP_RATE : last[1]);
		last[2] = (std::abs(last[0] - scaled[2]) < RAMP_RATE ? scaled[2] :
		           scaled[2] > last[2] ? last[2] + RAMP_RATE :
		           scaled[2] < last[2] ? last[2] - RAMP_RATE : last[2]);
		drivetrain.MecanumDrive_Cartesian(scaled[0], last[1], last[2]);
	}
};

START_ROBOT_CLASS(Robot);
