#include "WPILib.h"/**
 * Simplest program to drive a robot with mecanum drive using a single Logitech
 * Extreme 3D Pro joystick and 4 drive motors connected as follows:
 *   - Digital Sidecar 1:
 *     - PWM 1 - Connected to front left drive motor
 *     - PWM 2 - Connected to rear left drive motor
 *     - PWM 3 - Connected to front right drive motor
 *     - PWM 4 - Connected to rear right drive motor
 */class MecanumDefaultCode : public IterativeRobot
{
	RobotDrive *m_robotDrive;		// RobotDrive object using CAN for drive motors
	CANTalon leftFront, leftBack, rightFront, rightBack; // Aforementioned motors
	Joystick *m_driveStick;			// Joystick object on USB port 1 (mecanum drive)public:
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
	MecanumDefaultCode(void)
	{
		// Create a RobotDrive object using CAN drive motors
		m_robotDrive = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
		// Define joystick being used at USB port #1 on the Drivers Station
		m_driveStick = new Joystick(1);
		// Twist is on Axis 3 for the Extreme 3D Pro
		m_driveStick->SetAxisChannel(Joystick::kTwistAxis, 3);
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void)
	{
		// Driving
		m_robotDrive->MecanumDrive_Cartesian(m_driveStick->GetX(), m_driveStick->GetY(), m_driveStick->GetTwist());
	}
};
START_ROBOT_CLASS(MecanumDefaultCode);
