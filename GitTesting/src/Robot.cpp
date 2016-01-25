#include "WPILib.h"
#include <iostream>
#include <AnalogGyro.h>

class Robot: public IterativeRobot
{
	AnalogGyro gyro;
public:
	Robot():
	gyro(0)
	{}

	void RobotInit()
	{
		gyro.Calibrate();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
//Send help
		//Plz
	}

	void TeleopPeriodic()
	{
		std::cout<<"\ngyro angle =";
		std::cout<<gyro.GetAngle();

		Wait(0.5);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)
