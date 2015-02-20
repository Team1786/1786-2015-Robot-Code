#include "WPILib.h"
#include <iostream>
#include <vector>
#include <cmath>

#define AUTO_TARGET 10 //TODO: get a real value for this

#define RAMP_RATE 0.05

class Robot : public IterativeRobot
{
private:
	RobotDrive drivetrain;
	Joystick driveStick, lifterStick;
	CANTalon frontLeft, frontRight,
	         rearLeft, rearRight;
	DigitalInput winchTension;
	CANTalon winch, gripper;
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
		winch(4), gripper(5),
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

	bool updateWinch(int t)
	{
		static int target=-1;
		static int lastLimit=-1;
		float out=0;

		//iterate through all of the limits, and save the last one
		for(int ii=0;ii<=5;ii++)
		{
			if(getLimit(ii)) lastLimit=ii; 
		}

		//check if we have a new target
		if(t > -1) target=t;
		//clear target if -2 passed (for disable)
		if(t == -2) target=-1;
		//check for down
		if(t == -3) target=-3;

		//if the stick is not being used, and we have a target, turn on the motor
		if(abs(lifterStick.GetY()) < 0.01 && target != -1)
		{
			if(target == -3)
			{
				if(winchTension.Get()) //check for tension
					out = 1; //go down
				else
				{
					out = 0;
					target = -1;
				}
			}
			else if(!getLimit(target))
				//if the target is above the lastLimit, go up (-1), else go down (1)
				out=target<lastLimit?1:-1;
			else
			{
				out=0;
				target=-1; //clear the target
			}
		}
		else
		{
			out=lifterStick.GetY();
			target=-1; //clear the target
		}

		//check if we have hit an end stop
		if(getLimit(0) || getLimit(5))
		{
			if((getLimit(0) && (out < 0))  || (getLimit(5) && (out > 0)))
			{
				winch.Set(out);
				std::cout << "Winch limit hit, limiting movement" << std::endl;
			}
			else
			{
				winch.Set(0);
				std::cout << "Winch limit hit, stopping" << std::endl;
			}
		}
		else
			winch.Set(out);
		return (target == -1);
	}

	void DisabledInit()
	{
		drivetrain.SetSafetyEnabled(false);  //disable watchdog
		//clear winch target
		updateWinch(-2);
	}

	void AutonomousPeriodic()
	{
		static Timer t;
		static short stage=0;
		switch(stage)
		{
		case 0:
			//TODO: grip grippers
			stage += 1;
			break;
		case 1:
			stage += updateWinch(1);
			break;
		case 2:
			if(!t.Get())
				t.Start();
			//check if we are at our destination distance
			if(t.Get() < AUTO_TARGET)
			{
				drivetrain.MecanumDrive_Cartesian(0, -0.2, 0); //drive forwards
				std::cout << "time: " << t.Get() << std::endl;
			}
			else
			{
				drivetrain.MecanumDrive_Cartesian(0, 0, 0);
				stage += 1;
			}
			break;
		case 3:
			stage += updateWinch(0);
			break;
		case 4:
			gripper.Set(-1);
			break;
		}
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
		float gripperScale = ((1 - lifterStick.GetThrottle()) / 2);

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

		int winchButton=-1;
		for(int jj=7;jj<=12;jj++)
		{
			if(lifterStick.GetRawButton(jj)) winchButton=(jj-7);
		}
		winchButton = lifterStick.GetRawButton(1) ? -3 : winchButton;
		updateWinch(winchButton);
		gripper.Set((-(lifterStick.GetPOV() == 90) + (lifterStick.GetPOV() == 270))*gripperScale);
	}
};

START_ROBOT_CLASS(Robot);
