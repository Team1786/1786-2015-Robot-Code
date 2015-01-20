#include "WPILib.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>

class Robot : public IterativeRobot
{
private:
	RobotDrive drivetrain;
	Joystick driveStick;
	CANTalon frontLeft, frontRight,
	         rearLeft, rearRight;

public:
	Robot():
		frontLeft(0), frontRight(1),
		rearLeft(2), rearRight(3),
		drivetrain(frontLeft, rearLeft,
		           frontRight, rearRight),
		driveStick(1)
	{

	}

	void DisabledPeriodic()
	{
		LogData();
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
		timeval tm;
		static std::ofstream log;
		if (!log.is_open())
		{
			// writing to /home/lvuser/logs/[unixtime].log
			log.open("/home/lvuser/logs/" + std::to_string(time(0)) +".csv");
			log << "Time\tpdpInput voltage\tpdpTemperature\tpdpTotal Current\t";
			for (int i = 0; i < 16; i++)
			{
				log << "pdpChannel " << i << " current\t";
			}
			log << "\tFrontLeft Bus Voltage\tFrontLeft Output Current\tFrontLeft Output Voltage\tFrontLeft Temperature";
			log << "\tFrontRight Bus Voltage\tFrontRight Output Current\tFrontRight Output Voltage\tFrontRight Temperature";
			log << "\tRearLeft Bus Voltage\tRearLeft Output Current\tRearLeft Output Voltage\tRearLeft Temperature";
			log << "\tRearRight Bus Voltage\tRearRight Output Current\tRearRight Output Voltage\tRearRight Temperature";
			log << std::endl;
		}
		gettimeofday(&tm, NULL);
		log << time(0) << '.' << std::setfill('0') << std::setw(3) << tm.tv_usec/1000;
		PowerDistributionPanel pdp;	// preparing to read from the pdp
		// Some general information
		log << "\t" << pdp.GetVoltage();
		log << "\t" << pdp.GetTemperature();
		log << "\t" << pdp.GetTotalCurrent();
		// current on each channel
		for (int i = 0; i < 16; i++)
		{
			log << "\t" << pdp.GetCurrent(i);
		}
		//Talon Data
		log << "\t" <<frontLeft.GetBusVoltage();
		log << "\t" <<frontLeft.GetOutputVoltage();
		log << "\t" <<frontLeft.GetOutputCurrent();
		log << "\t" <<frontLeft.GetTemperature();
		log << "\t" <<frontRight.GetBusVoltage();
		log << "\t" <<frontRight.GetOutputVoltage();
		log << "\t" <<frontRight.GetOutputCurrent();
		log << "\t" <<frontRight.GetTemperature();
		log << "\t" <<rearLeft.GetBusVoltage();
		log << "\t" <<rearLeft.GetOutputVoltage();
		log << "\t" <<rearLeft.GetOutputCurrent();
		log << "\t" <<rearLeft.GetTemperature();
		log << "\t" <<rearRight.GetBusVoltage();
		log << "\t" <<rearRight.GetOutputVoltage();
		log << "\t" <<rearRight.GetOutputCurrent();
		log << "\t" <<rearRight.GetTemperature();
		log << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
