#include "WPILib.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <cmath>

#define AUTO_TARGET 2.2 //TODO: get a real value for this

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
	bool IgnoreLimits;
	short stage=0;

	bool getLimit(int num)
	{
		//invert everything other than top and bottom because the roboRIO reports open as 1
		//top and bottom are not inverted becuase they they are NC (whereas the rest are NO)
		return (num == 0 || num == 5) ? winchLimits[num]->Get() : !winchLimits[num]->Get();
	}

	bool updateWinch(int t)
	{
		static int target = -1;
		static int lastLimit = -1;
		float out = 0;

		//iterate through all of the limits, and save the last one
		for(int ii = 0;ii <= 5;ii++)
		{
			if(getLimit(ii)) lastLimit = ii;
		}

		//check if we have a new target
		if(t > -1) target = t;
		//clear target if -2 passed (for disable)
		if(t == -2) target = -1;
		//check for down
		if(t == -3) target = -3;

		//if the stick is not being used, and we have a target, turn on the motor
		if(std::abs(lifterStick.GetY()) < 0.2 && target != -1)
		{
			if(target == -3)
			{
				if(winchTension.Get() && !getLimit(0)) //check for tension
					out = 1; //go down
				else
				{
					out = 0;
					target = -1;
				}
			}
			else if(!getLimit(target))
				//if the target is above the lastLimit, go up (-1), else go down (1)
				out = target < lastLimit ? 1 : -1;
			else
			{
				out = 0;
				target = -1; //clear the target
			}
		}
		else
		{
			out = lifterStick.GetY();
			target = -1; //clear the target
		}

		//check if we have hit an end stop
		if(getLimit(0) || getLimit(5) && !IgnoreLimits)
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

	void updateDashboard()
	{
		for(int ii = 0;ii <= 5;ii++)
		{
			SmartDashboard::PutBoolean("Winch " + std::to_string(ii), getLimit(ii));
		}
		SmartDashboard::PutBoolean("Winch Tension", winchTension.Get());
		IgnoreLimits = SmartDashboard::GetBoolean("Ignore Limits", false);
		SmartDashboard::PutBoolean("Ignore Limits", false);
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
			log.open("/media/sda1/logs/" + std::to_string(time(0)) +".csv");
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
			log << "\tAlliance\tLocation\tMatch Time\tFMS Attached\tBrowned Out";
			log << "\tTestStage";
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

		//Test stage
		log << "\t" << stage;
		log << std::endl;
	}

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
	void DisabledInit()
	{
		stage=0;
		drivetrain.SetSafetyEnabled(false);  //disable watchdog
		//clear winch target
		updateWinch(-2);
	}

	void DisabledPeriodic()
	{
		updateDashboard();
		LogData();
	}

	void AutonomousPeriodic()
	{
		static Timer t;
		static short stage = 0;
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
				drivetrain.MecanumDrive_Cartesian(0, -0.5, 0); //drive forwards
				std::cout << "time: " << t.Get() << std::endl;
			}
			else
			{
				drivetrain.MecanumDrive_Cartesian(0, 0, 0);
				stage += 1;
			}
			break;
		case 3:
			stage += updateWinch(-3);
			break;
		case 4:
			gripper.Set(-1);
			break;
		}
		updateDashboard();
		LogData();
	}

	void TestPeriodic()
	{
		static Timer t;
		static short oldStage = -1;
		int timePerAction = 5;
		float testSpeed = 0.3;

		if(oldStage != stage)
		{
			if(driveStick.GetRawButton(1))
			{
				oldStage = stage;
				if(!t.Get()) t.Start();
			}
			else SmartDashboard::PutString("Test Status", "Press driveStick trigger to continue...");
		}
		else
		{
			switch(stage)
			{
			case 0:
				//drive forwards for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(0, -testSpeed, 0);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 1:
				//drive backwards for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(0, testSpeed, 0);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 2:
				//drive left for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(-testSpeed, 0, 0);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 3:
				//drive right for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(testSpeed, 0, 0);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 4:
				//rotate left (CC) for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(0, 0, -testSpeed);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 5:
				//rotate right (C) for 5 seconds
				if(t.Get() < timePerAction) drivetrain.MecanumDrive_Cartesian(0, 0, testSpeed);
				else
				{
					drivetrain.MecanumDrive_Cartesian(0, 0, 0);
					t.Reset();
					stage++;
				}
				break;
			case 6:
				//winch down to level 0
				stage += updateWinch(0);
				break;
			case 7:
				//winch up to level 1
				stage += updateWinch(1);
				break;
			case 8:
				//winch up to level 2
				stage += updateWinch(2);
				break;
			case 9:
				//winch up to level 3
				stage += updateWinch(3);
				break;
			case 10:
				//winch up to level 4
				stage += updateWinch(4);
				break;
			case 11:
				//winch up to level 5
				stage += updateWinch(5);
				break;
			case 12:
				//test winch set down
				stage += updateWinch(-3);
				break;
			case 13:
				//winch down to level 0
				stage += updateWinch(0);
				break;
			case 14:
				//grip to inner limit
				if(gripper.GetForwardLimitOK()) gripper.Set(testSpeed);
				else
				{
					gripper.Set(0);
					stage++;
				}
				break;
			case 15:
				//grip to inner limit
				if(gripper.GetReverseLimitOK()) gripper.Set(-testSpeed);
				else
				{
					gripper.Set(0);
					stage++;
				}
				break;
			default:
				drivetrain.MecanumDrive_Cartesian(0, 0, 0);
				winch.Set(0);
				gripper.Set(0);
				break;
			}
		}

		updateDashboard();
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
		static double scaled[3];
		float throttleScale = ((1 - driveStick.GetThrottle()) / 2);
		float gripperScale = ((1 - lifterStick.GetThrottle()) / 2);

		scaled[0] = (driveStick.GetX() + -(driveStick.GetPOV() == 90) + (driveStick.GetPOV() == 270)) * throttleScale;
		scaled[1] = driveStick.GetY() * throttleScale;
		scaled[2] = driveStick.GetTwist() * throttleScale * driveStick.GetRawButton(2) * 0.8;

		drivetrain.MecanumDrive_Cartesian(scaled[0], scaled[1], scaled[2]);

		//Winching
		int winchButton = -1;
		for(int jj = 7;jj <= 12;jj++)
		{
			if(lifterStick.GetRawButton(jj)) winchButton = (jj-7);
		}
		if(lifterStick.GetRawButton(3)) winchButton = 2;
		winchButton = lifterStick.GetRawButton(1) ? -3 : winchButton;
		updateWinch(winchButton);
		gripper.Set((-(lifterStick.GetPOV() == 90) + (lifterStick.GetPOV() == 270)) * gripperScale);

		updateDashboard();
		//Data logging
		LogData();
	}
};

START_ROBOT_CLASS(Robot);
