#include "WPILib.h"
#include <iostream>
#include <vector>

class Robot : public IterativeRobot
{
private:
	RobotDrive drivetrain;
	Joystick driveStick, lifterStick;
	CANTalon frontLeft, frontRight,
	         rearLeft, rearRight;
	DigitalInput winchTension;
	CANTalon winch;
	std::vector<DigitalInput*> winchLimits;
	DigitalInput a,b,c,d,e,f,g;

public:
	Robot():
		frontLeft(0), frontRight(1),
		rearLeft(2), rearRight(3),
		drivetrain(frontLeft, rearLeft,
		           frontRight, rearRight),
		driveStick(0), lifterStick(1),
		winchTension(0),
		winch(4),
		a(1), b(2), c(3), d(4), e(5), f(6), g(7)
	{
		winchLimits.push_back(&a);
		winchLimits.push_back(&b);
		winchLimits.push_back(&c);
		winchLimits.push_back(&d);
		winchLimits.push_back(&e);
		winchLimits.push_back(&f);
		winchLimits.push_back(&g);
	}

	void DisabledPeriodic()
	{
		int ii=0;
		for(ii=0;ii<6;ii++)
		{
			std::cout << " limit " << ii << ":" << winchLimits[ii]->Get();
		}
		std::cout << std::endl;
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
		std::cout << "Winch tension limit switch: " << winchTension.Get() << std::endl;	// this reads 0 when the switch is closed, because of the way the roborio is wired internally.	
		winch.Set(lifterStick.GetY());
	}
};

START_ROBOT_CLASS(Robot);
