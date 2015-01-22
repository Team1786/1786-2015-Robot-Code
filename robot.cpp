#include "WPILib.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <Timer.h>

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
	
	void AutonomousPeriodic()
	{
		//testing things
		frontLeft.Set(1);
		waiting();
		/*leep(2000);
		frontLeft.Set(-1);
		sleep(2000);
		frontLeft.Set(0);
		frontRight.Set(1);
		sleep(2000);
		frontRight.Set(-1);
		sleep(2000);
		frontRight.Set(0);
		//rear wheels
		rearLeft.Set(1);
		sleep(2000);
		rearLeft.Set(-1);
		sleep(2000);
		rearLeft.Set(0);
		rearRight.Set(1);
		sleep(2000);
		rearRight.Set(-1);
		sleep(2000);
		rearRight.Set(0);*/
	}
	
	void waiting()
	{
			Timer thing = Timer();
			thing.Start();
			while (!thing.HasPeriodPassed(2)) {}
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
			log << "FrontLeft Bus Voltage\tFrontLeft Output Current\tFrontLeft Output Voltage\tFrontLeft Temperature";
			log << "\tFrontRight Bus Voltage\tFrontRight Output Current\tFrontRight Output Voltage\tFrontRight Temperature";
			log << "\tRearLeft Bus Voltage\tRearLeft Output Current\tRearLeft Output Voltage\tRearLeft Temperature";
			log << "\tRearRight Bus Voltage\tRearRight Output Current\tRearRight Output Voltage\tRearRight Temperature";
			log << "\tJoystic X\tJoystic Y\tJoystick Twist";
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
		
		//control data
		log << "\t" << driveStick.GetX();
		log << "\t" << driveStick.GetY();
		log << "\t" << driveStick.GetTwist();
		log << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
