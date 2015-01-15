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
		//std::cout << time(0) << std::endl;
		// writing to /home/lvuser/logs/[unixtime].log
		std::ofstream log;
		std::ostringstream convert;  
		time_t now = time(0);
		convert << now;//I'd rather stream it through like this than actually think about.
		std::string logPath = "/home/lvuser/logs/" + convert.str() + ".csv";
		log.open(logPath);//finally actually opening the log file
		
		log << now << ",";
		PowerDistributionPanel pdp;	//preparing to read from the pdp
		// Some general information
		log << "Input voltage," << pdp.GetVoltage();
		log << ",Temperature," << pdp.GetTemperature();
		log << ",Total Current," << pdp.GetTotalCurrent();
		//current on each channel
		for (int i = 0; i < 16; i++)
		{
			log << ",Channel " << i << "current," << pdp.GetCurrent(i);
		}	
		log.close();	
	}
};

START_ROBOT_CLASS(Robot);
