#include "WPILib.h"
#include <iostream>
#include <AnalogGyro.h>


const float SMOOTH_DRIVE_GAIN		= 0.5;
const float SMOOTH_DRIVE_DEADZONE	= 0.01;

float drivePowerLeft	= 0;
float drivePowerRight	= 0;

class Robot: public IterativeRobot
{
	RobotDrive Robotc;
	Joystick driveStick;
	CANTalon driveLeftFront;
	CANTalon driveRightFront;
	CANTalon driveLeftBack;
	CANTalon driveRightBack;
	AnalogGyro gyro;

	JoystickButton JoyR;
	JoystickButton JoyL;
	DigitalOutput AR;
	DigitalOutput AL;
	DoubleSolenoid *Piston = new DoubleSolenoid(0, 1);

public:
	Robot():
		Robotc(1, 2, 3, 4),
		driveStick(0),
		driveLeftFront(1),
		driveRightFront(2),
		driveLeftBack(3),
		driveRightBack(4),
		gyro(0),

		JoyR(&driveStick,5),
		JoyL(&driveStick,4),
		AR(0),
		AL(1)
	{}
private:



	LiveWindow *lw = LiveWindow::GetInstance();

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
		Piston->Set(DoubleSolenoid::Value::kOff);
	}

	void AutonomousPeriodic()
	{
		std::cout<<"\ngyro angle =";
		std::cout<<gyro.GetAngle();
		TankDrive(gyro.GetAngle()/90,0);

//		Piston->Set(DoubleSolenoid::Value::kForward);
//		Wait(3);
//		Piston->Set(DoubleSolenoid::Value::kReverse);
//		Wait(3);
//		std::cout << "Piston ";

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		if ( driveStick.GetTrigger() )	SmoothTankDrive( gyro.GetAngle()/90, driveStick.GetRawAxis(1) );
		else							SmoothTankDrive( driveStick.GetRawAxis(0), driveStick.GetRawAxis(1) );
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void Drive( float left , float right )
	{
		// set left motors
		driveLeftFront.Set	(left);
		driveLeftBack.Set	(left);
		// set right motors (inverted)
		driveRightFront.Set	(-right);
		driveRightBack.Set	(-right);
	}

	void SmoothDrive( float left , float right )
	{
		if( abs( left ) <= SMOOTH_DRIVE_DEADZONE && abs( right ) <= SMOOTH_DRIVE_DEADZONE )
		{
			drivePowerLeft	= 0;
			drivePowerRight	= 0;
		}
		else
		{
			drivePowerLeft	+= SMOOTH_DRIVE_GAIN * (	left	- 	drivePowerLeft	);
			drivePowerRight	+= SMOOTH_DRIVE_GAIN * (	right	- 	drivePowerRight	);
		}
		Drive( drivePowerLeft , drivePowerRight );
	}

	void TankDrive( float x , float y )
	{
		Drive( -y - x , -y + x );
	}

	void SmoothTankDrive( float x , float y )
	{
		SmoothDrive( -y - x , -y + x );
	}
};

START_ROBOT_CLASS(Robot)
