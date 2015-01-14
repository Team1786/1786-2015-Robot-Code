#include "WPILib.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

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
		//Driving
		drivetrain.MecanumDrive_Cartesian(driveStick.GetX(), driveStick.GetY(), driveStick.GetTwist());
		
		//Data logging
		LogData();
	}
	
	void LogData()
	{
		// write to /home/lvuser/logs/[unixtime].log
		std::cout << time(0) << std::endl;
		
		std::ofstream log;
		std::ostringstream convert;
		convert << time(0);
		std::string logPath = "/home/lvuser/logs/" + convert.str() + ".log";
		log.open(logPath);//setting up log file
		
		PowerDistributionPanel pdp;	
		log << "Input voltage: " << pdp.GetVoltage();
		log << "\nTemperature: " << pdp.GetTemperature();
		log <<"\nTotal Current: " << pdp.GetTotalCurrent() << "\n";
		for (int i = 0; i < 16; i++)
		{
			log << "Channel " << i << "current: " << pdp.GetCurrent(i) << "\n";
		}	
		log.close();	
	}
};

START_ROBOT_CLASS(Robot);
