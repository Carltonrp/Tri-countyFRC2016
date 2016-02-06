#include <iostream>
#include <cmath>
#include <unistd.h>
#include "WPILib.h"
#include "Timer.h"

const double	SMOOTH_DRIVE_P_GAIN		=	0.5;
const double	DRIVE_DEADZONE			=	0.05;
const double	DRIVE_X_TOLERANCE		=	0.05;
const double	ANGLE_TOLERANCE			=	0.1;

const double	TURN_P_GAIN				=	1;
const double	TURN_I_GAIN				=	0.5;
const double	TURN_D_GAIN				=	6;
const double	TURN_K					=	0.001;

double	speedLeft						=	0;
double	speedRight						=	0;

bool	driveStraight					=	false;

double	turnP							=	0;
double	turnI							=	0;
double	turnD							=	0;
int		turnInterval					=	0;

double	drivePower						=	0;
double	drivePowerLeft					=	0;
double	drivePowerRight					=	0;
double	turnPower						=	0;
double	targetAngle						=	0;

int		autoDriveState					=	0;
double	speed							=	0;
double	distance						=	0;
bool	tracking						=	false;

double times;

double accelX;
double accelY;
double accelZ;

class Robot: public IterativeRobot
{
	LiveWindow *lw = LiveWindow::GetInstance();

	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";

	const std::string autoNameCustom0 = "Auto0";
	const std::string autoNameCustom1 = "Auto1";
	const std::string autoNameCustom2 = "Auto2";
	const std::string autoNameCustom3 = "Auto3";
	const std::string autoNameCustom4 = "Auto4";

	std::string autoSelected;

	Timer timer;
	RobotDrive Robotc;
	Joystick driveStick;
	JoystickButton driveThumb;
	JoystickButton driveThumbLU;
	JoystickButton driveThumbRU;
	JoystickButton driveThumbLD;
	JoystickButton driveThumbRD;
	CANTalon driveLeft;
	CANTalon driveRight;
	CANTalon arm;
	CANTalon throwLow;
	CANTalon throwHigh;
	AnalogGyro gyro;
	ADXL345_I2C accel;

	JoystickButton JoyL;
	JoystickButton JoyR;

	Relay *AR = new Relay(0);
	Relay *AL = new Relay(1);
	DoubleSolenoid *Piston = new DoubleSolenoid(0, 1);

	const char *JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = {"java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };

public:
	Robot():
		Robotc(1, 2),
		driveStick(0),
		driveThumb( &driveStick , 2 ),
		driveThumbLU( &driveStick , 5 ),
		driveThumbRU( &driveStick , 6 ),
		driveThumbLD( &driveStick , 3 ),
		driveThumbRD( &driveStick , 4 ),
		driveLeft(1),
		driveRight(2),
		arm(4),
		throwHigh(3),
		throwLow(5),
		gyro(0),
		JoyL(&driveStick,4),
		JoyR(&driveStick,5),
		chooser(),
		accel(I2C::Port::kOnboard)
	{}




	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom0, (void*)&autoNameCustom0);
		chooser->AddObject(autoNameCustom1, (void*)&autoNameCustom1);
		chooser->AddObject(autoNameCustom2, (void*)&autoNameCustom2);
		chooser->AddObject(autoNameCustom3, (void*)&autoNameCustom3);
		chooser->AddObject(autoNameCustom4, (void*)&autoNameCustom4);

		SmartDashboard::PutData("Auto Modes", chooser);

		AR->Set(Relay::Value::kOff);
		AL->Set(Relay::Value::kOff);

		if (fork() == 0)						//Creating process for grip and testing if it fails
		{
			if (execv(JAVA, GRIP_ARGS) == -1)
			{
				perror("Error running GRIP");
		    }
		}

		timer.Stop();
		timer.Reset();

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
		AR->Set(Relay::Value::kOn);
		AL->Set(Relay::Value::kOff);

		autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		if(autoSelected == autoNameCustom0)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom1)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom2)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom3)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom4)
		{
			KillAll();
		}
		else
		{
			KillAll();
		}
		timer.Start();
		Wait(0.1);
		gyro.Calibrate();
	}

	void AutonomousPeriodic()
	{
		times = timer.Get();
		if(autoSelected == autoNameCustom0)
		{
			std::cout<<"\nSpeed:\t";
			std::cout<<speed;
			std::cout<<"\tDistance:\t";
			std::cout<<distance;
			AutoDrive( 10 , 0.5 );
		}
		else if(autoSelected == autoNameCustom1)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom2)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom3)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom4)
		{
			KillAll();
		}
		else
		{
				//Default Auto goes here
//			std::cout<<"/n Time =";
//			std::cout<<timer.Get();
//
//			if (times <= 5)
//			{
//				driveLeft.Set(0.2);
//				driveRight.Set(-0.2);
//			}
//			else if ((times <= 10) && (times > 5))
//			{
//				driveLeft.Set(-0.2);
//				driveRight.Set(0.2);
//			}
//			else
//			{
//				timer.Reset();
//				timer.Start();
//			}

			accelX = accel.GetX();
			accelY = accel.GetY();
			accelZ = accel.GetZ();

//			std::cout<<"/n AccelX =";
//			std::cout<<accelX;
			std::cout<<"\n AccelY = ";
			std::cout<<accelY;
//			std::cout<<"/n AccelZ =";
//			std::cout<<accelZ;
			std::cout<<"\n speed = ";
			std::cout<<speed;
			std::cout<<"\n dist = ";
			std::cout<<distance;

			KillAll();
		}

//		TankDrive(gyro.GetAngle()/90,0);

		auto grip = NetworkTable::GetTable("grip");

		        /* Get published values from GRIP using NetworkTables */
		auto areas = grip->GetNumberArray("targets/area", llvm::ArrayRef<double>());

		for (auto area : areas)
		{
			std::cout << "Got contour with area=" << area << std::endl;
		}

//		Piston->Set(DoubleSolenoid::Value::kForward);
//		Wait(3);
//		Piston->Set(DoubleSolenoid::Value::kReverse);
//		Wait(3);
//		std::cout << "Piston ";
//
//		std::cout<< "\nangle = ";
//		std::cout<< gyro.GetAngle();
//		std::cout<< "\trate = ";
//		std::cout<< gyro.GetRate();

		TrackAccel();	// must be called at the end of the periodic loop
	}

	void TeleopInit()
	{
		AL->Set(Relay::Value::kOn);
		AR->Set(Relay::Value::kOff);
		gyro.Calibrate();
	}

	void TeleopPeriodic()
	{
		std::cout<< "\nangle = ";
		std::cout<< gyro.GetAngle();
//		std::cout<< "\tP = ";
//		std::cout<< (int) turnP;
//		std::cout<< "\tI = ";
//		std::cout<< (int) turnI;
//		std::cout<< "\tD = ";
//		std::cout<< (int) turnD;

		// Stop all movement and reset PID controls
		if ( driveThumb.Get() )
		{
			KillDrive();
		}
		else
		{
			TankDrive( driveStick.GetRawAxis(0) , driveStick.GetRawAxis(1) );
		}

		if ( driveStick.GetPOV() != -1 ) {
			targetAngle = ModAngle( -driveStick.GetPOV() );
		}

		if (driveThumbLU.Get())
		{
			arm.Set(0.5);
		}
		else if (driveThumbRU.Get())
		{
			arm.Set(-0.5);
		}
		else
		{
			arm.Set(0);
		}
		if (driveThumbLD.Get())
		{
			throwHigh.Set((driveStick.GetRawAxis(3)+1)/2);
			throwLow.Set((driveStick.GetRawAxis(3)+1)/2);
		}
		else if (driveThumbRD.Get())
		{
			throwHigh.Set(-(driveStick.GetRawAxis(3)+1)/2);
			throwLow.Set(-(driveStick.GetRawAxis(3)+1)/2);
		}
		else
		{
			throwHigh.Set(0);
			throwLow.Set(0);
		}

		TrackAccel();	// must be called at the end of the periodic loop

	}

	void TestPeriodic()
	{
		lw->Run();
	}

	/* Kill Functions */

	/*
	 * Unconditionally stop all motors and reset control variables.
	 */
	void	KillAll ()
	{
		driveLeft.Set	(	0	);
		driveRight.Set	(	0	);
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();
	}

	/*
	 * Unconditionally stop all drive motors and reset drive control variables.
	 */
	void	KillDrive ()
	{
		driveLeft.Set	(	0	);
		driveRight.Set	(	0	);
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();

		turnPower		=	0;
		TurnPIDReset();

	}

	/* DRIVE FUNCTIONS */

	void	Drive	( double _left , double _right )
	{
		// set left motors
		driveLeft.Set	( -_left );
		// set right motors
		driveRight.Set	( _right );
	}

	void	SmoothDrive	( double _left , double _right )
	{
		if( fabs( _left ) <= DRIVE_DEADZONE && fabs( _right ) <= DRIVE_DEADZONE )
		{
			drivePowerLeft	= 0;
			drivePowerRight	= 0;
		}
		else
		{
			drivePowerLeft	+= SMOOTH_DRIVE_P_GAIN * (	_left	- 	drivePowerLeft	);
			drivePowerRight	+= SMOOTH_DRIVE_P_GAIN * (	_right	- 	drivePowerRight	);
		}
		Drive( drivePowerLeft , drivePowerRight );
	}

	void	TankDrive	( double _x , double _y )
	{
		Drive( _y - _x , _y + _x );
	}

	void	SmoothTankDrive	( double _x , double _y )
	{
		SmoothDrive( _y - _x , _y + _x );
	}

	void	KeepAngle	( double _targetAngle , double _drive )
	{
		// calculate angle deviation
		double _currentAngleDeviation = AngularDifference( GetAngle() , _targetAngle );

		// increment interval
		turnInterval++;

		// calculate PID
		turnP	=	_currentAngleDeviation;
		turnI	+=	_currentAngleDeviation;
		turnD	=	-gyro.GetRate();

		// calculate turn power
		turnPower	=	TURN_K * ( TURN_P_GAIN * turnP + TURN_I_GAIN * turnI + TURN_D_GAIN * turnD );

		// limit turnPower to [-1,+1]
		if		( turnPower > +1 )	turnPower	=	+1;
		else if	( turnPower < -1 )	turnPower	=	-1;

		// calculate drive power
		drivePower	+=	SMOOTH_DRIVE_P_GAIN * (	_drive	- 	drivePower	);

		// drive the robot
		TankDrive( turnPower , drivePower );
	}

	void	SpecialTankDrive	( double _x , double _y )
	{
		if ( fabs( _x ) <= DRIVE_X_TOLERANCE || driveStick.GetTrigger() )
		{
			std::cout<<"\nSTRAIGHT DRIVE!";
			if ( !driveStraight )
			{
				gyro.Reset();
				TurnPIDReset();
			}
			if ( fabs( _x ) <= DRIVE_DEADZONE && fabs( _y ) <= DRIVE_DEADZONE )
			{
				driveStraight = false;
				Drive( 0 , 0 );
			}
			else
			{

			driveStraight = true;
			KeepAngle( 0 , _y );

				driveStraight = true;
				KeepAngle( 0 , _y );
			}
		}
		else
		{
			std::cout<<"\nSMOOTH DRIVE!";
			driveStraight = false;
			SmoothDrive( _y - _x , _y + _x );
		}
	}

	/*
	 * Automatically drive straight for a specified distance.
	 */
	bool	AutoDrive	( double _targetDistance , double _speed = 0.5 )
	{
		if ( autoDriveState == 0 )
		{
			gyro.Reset();
			TurnPIDReset();
			DistanceReset();
			autoDriveState = 1;
		}
		if ( autoDriveState == 1 )
		{
			TrackAccel();
			double	distanceRemaining	=	_targetDistance - distance;
			//
			if ( distanceRemaining < 0 )
			{
				// if facing correct angle,
				if ( fabs( GetAngle() ) <= ANGLE_TOLERANCE )
				{
					// drive straight
					KeepAngle( 0 , _speed );
				}
				else
				{
					// stop to correct angle
					KeepAngle( 0 , 0 );
				}
			}
			else if ( distanceRemaining > 0 )
			{
				// if facing correct angle,
				if ( fabs( GetAngle() ) <= ANGLE_TOLERANCE )
				{
					// drive straight
					KeepAngle( 0 , _speed );
				}
				else
				{
					// stop to correct angle
					KeepAngle( 0 , 0 );
				}
			}
			else
			{
				autoDriveState = 2;
			}
		}
		if ( autoDriveState == 2 )
		{
			// if not facing the correct angle
			if ( fabs( GetAngle() ) > ANGLE_TOLERANCE )
			{
				// turn to face the correct angle
				KeepAngle( 0 , 0 );
			}
			else
			{
				autoDriveState = -1;
			}
		}
		if ( autoDriveState == -1 )
		{
			autoDriveState = 0;
			return true;
		}
		else return false;
	}

	/* GYRO FUNCTIONS */

	double	ModAngle	( double angle )
	{
		angle = angle - 360 * floorf( ( angle - 180 ) / 360 ) - 360;
		if ( angle == -180 ) angle = 180;
		return angle;
	}

	double	GetAngle	()
	{
		return ModAngle( gyro.GetAngle() );
	}

	double	AngularDifference	( double left , double right )
	{
		std::cout<<"\n";
		std::cout<<left;
		std::cout<<"-";
		std::cout<<right;
		std::cout<<"=";
		std::cout<<ModAngle( left - right );
		return ModAngle( left - right );
	}

	void	TurnPIDReset	()
	{
		turnP			=	0;
		turnI			=	0;
		turnD			=	0;
		turnInterval	=	0;
	}

	/* ACCELEROMETER FUNCTIONS */
	void	DistanceReset	()
	{
		distance	=	0;
	}

	void	TrackAccel	()
	{
		if ( tracking )
		{
			speed		+=	timer.Get() * accel.GetY();
			distance	+=	timer.Get() * speed;
			timer.Reset();
		}
		tracking = true;
	}
};

START_ROBOT_CLASS(Robot)
