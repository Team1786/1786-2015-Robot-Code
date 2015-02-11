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

	bool getLimit(int num)
	{
		//invert everything other than top and bottom because the roboRIO reports open as 1
		//top and bottom are not inverted becuase they they are NC (whereas the rest are NO) 
		return (num == 0 || num == 5) ? winchLimits[num]->Get() : !winchLimits[num]->Get();
	}

	void updateWinch(int t)
	{
		static int target=-1;
		static int lastLimit=-1;

		//iterate through all of the limits, and save the last one
		for(int ii=0;ii<=5;ii++)
		{
			if(getLimit(ii)) lastLimit=ii; 
		}
		std::cout << "level: " << lastLimit << std::endl;

		//check if we have a new target
		if(t > -1) target=t;

		//clear target if -2 passed (for disable)
		if(t == -2) target=-1;

		//check if we have hit an end stop
		if(getLimit(0) || getLimit(5))
		{
			winch.Set(0);
			std::cout << "Winch limit hit, stopping" << std::endl;
		}

		//if the stick is not being used, and we have a target, turn on the motor
		else if(abs(lifterStick.GetY()) < 0.05 && target != -1)
		{
			if(getLimit(target))
				//if the target is above the lastLimit, go up (-1), else go down (1)
				winch.Set(target>lastLimit?-1:1);
			else
			{
				winch.Set(0);
				target=-1; //clear the target
			}
		}
		else
		{
			winch.Set(lifterStick.GetY());
			target=-1; //clear the target
		}
	}

	void DisabledInit()
	{
		//clear winch target
		updateWinch(-2);
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
		int winchButton=-1;
		for(int jj=7;jj<=12;jj++)
		{
			if(lifterStick.GetRawButton(jj)) winchButton=(jj-6);
		}
		updateWinch(winchButton);
	}
};

START_ROBOT_CLASS(Robot);
