#include "WPILib.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <cmath>

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
		std::cout << "target: " << target << " t:" << t << std::endl;

		//check if we have hit an end stop
		if(lastLimit == 0 || lastLimit == 5)
		{
			winch.Set(((getLimit(0) && lifterStick.GetY() < 0) || (getLimit(5) && lifterStick.GetY() > 0)) ? lifterStick.GetY() : 0);
			target=-1;
			std::cout << "Winch limit hit, stopping" << std::endl;
		}

		//if the stick is not being used, and we have a target, turn on the motor
		else if(abs(lifterStick.GetY()) < 0.05 && target != -1)
		{
			if(!getLimit(target))
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

	void DisabledPeriodic()
	{
		LogData();
	}
	void DisabledInit()
	{
		drivetrain.SetSafetyEnabled(false);  //disable watchdog
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
		
		//Winching
		int winchButton=-1;
		for(int jj=7;jj<=12;jj++)
		{
			if(lifterStick.GetRawButton(jj)) winchButton=(jj-7);
		}
		updateWinch(winchButton);
		gripper.Set((-(lifterStick.GetPOV() == 90) + (lifterStick.GetPOV() == 270))*gripperScale);

		//Data logging
		LogData();
	}

	void LogData()
	{
		static PowerDistributionPanel pdp;	// preparing to read from the pdp
		static DriverStation* ds = DriverStation::GetInstance();
		static std::vector<CANTalon*> motors;

		static std::ofstream log;
		timeval tm;

		if (!log.is_open())
		{
			// writing to /home/lvuser/logs/[unixtime].log
			log.open("/home/lvuser/logs/" + std::to_string(time(0)) +".csv");
			log << "Time\tpdpInput voltage\tpdpTemperature\tpdpTotal Current\t";
			for (int ii = 0; ii < 16; ii++)
			{
				log << "pdpChannel " << ii << " current\t";
			}

			log << "FrontLeft Bus Voltage\tFrontLeft Output Current\tFrontLeft Output Voltage\tFrontLeft Temperature";
			motors.push_back(&frontLeft);
			log << "\tFrontRight Bus Voltage\tFrontRight Output Current\tFrontRight Output Voltage\tFrontRight Temperature";
			motors.push_back(&frontRight);
			log << "\tRearLeft Bus Voltage\tRearLeft Output Current\tRearLeft Output Voltage\tRearLeft Temperature";
			motors.push_back(&rearLeft);
			log << "\tRearRight Bus Voltage\tRearRight Output Current\tRearRight Output Voltage\tRearRight Temperature";
			motors.push_back(&rearRight);
			log << "\tWinch Bus Voltage\tWinch Output Current\tWinch Output Voltage\tWinch Temperature";
			motors.push_back(&winch);
			log << "\tGripper Bus Voltage\tGripper Output Current\tGripper Output Voltage\tGripper Temperature";
			motors.push_back(&gripper);

			log << "\tJoystick X\tJoystick Y\tJoystick Twist";
			log << "\tWinch Tension";
			for(int ii = 0; ii <= 5; ii++)
			{
				log << "\t Winch Limit " << ii;
			}
			log << std::endl;
		}
		gettimeofday(&tm, NULL);
		log << time(0) << '.' << std::setfill('0') << std::setw(3) << tm.tv_usec/1000;
		// Some general information
		log << "\t" << pdp.GetVoltage();
		log << "\t" << pdp.GetTemperature();
		log << "\t" << pdp.GetTotalCurrent();
		// current on each channel
		for (int ii = 0; ii < 16; ii++)
		{
			log << "\t" << pdp.GetCurrent(ii);
		}
		
		//Talon Data
		for(int ii = 0; ii < motors.size(); ii++)
		{
			log << "\t" << motors[ii]->GetBusVoltage();
			log << "\t" << motors[ii]->GetOutputVoltage();
			log << "\t" << motors[ii]->GetOutputCurrent();
			log << "\t" << motors[ii]->GetTemperature();
		}

		//control data
		log << "\t" << driveStick.GetX();
		log << "\t" << driveStick.GetY();
		log << "\t" << driveStick.GetTwist();
		log << std::endl;
		
		//Winch Limits
		log << "\t" << winchTension.Get();
		for(int ii = 0; ii <= 5; ii++)
		{
			log << "\t" << getLimit(ii);
		}

		//DriverStation Data
		log << "\t" << ds->GetAlliance();
		log << "\t" << ds->GetLocation();
		log << "\t" << ds->GetMatchTime();
		log << "\t" << ds->IsFMSAttached();
		log << "\t" << ds->IsSysBrownedOut();
	}
};

START_ROBOT_CLASS(Robot);
